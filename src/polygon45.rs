// SPDX-License-Identifier: Apache-2.0 AND BSL-1.0
//! Sets of polygons with 45 degree edges: merged, formed into polygons with holes, and sliced
//! vertically into trapezoids — what the reference network does with an octilinear special wire
//! or an octilinear pin (upstream 5f0b2d8fb02, 1d4cb901b1b).
//!
//! Derived from Boost.Polygon (Copyright 2008 Intel Corporation; Boost Software License 1.0,
//! http://www.boost.org/LICENSE_1_0.txt): `polygon_45_set_data` (insert, `clean`), the unary OR of
//! `boolean_op_45::Scan45`, and `polygon_45_formation`'s `Polygon45Formation` and
//! `Polygon45Tiling`. The algorithms are kept step for step, because their OUTPUT is observable
//! beyond the region they describe: which vertex a polygon starts at, the order of the
//! trapezoids, and collinear 45 degree vertices the formation keeps — the network places nodes on
//! them.
//!
//! Each C++ iterator into an ordered container is an id into an arena here, and each ordered
//! container a list of ids kept in its comparator's order (the scanline comparators read the
//! current x, so the order is the container's own, not a key's). An insert of an element the
//! container already holds an equal of is dropped, as `std::set::insert` drops it.
//!
//! ⚠️ Not transcribed: boost's fallback when two 45 degree edges cross between grid points
//! (it scales the set by 2, merges and snaps back). That input is refused with boost's message.

// The count loops index the reference's four-slot arrays exactly as its `for(int i ...)` do, so
// the case analysis reads line for line against it.
#![allow(clippy::needless_range_loop)]

use std::collections::{BTreeSet, VecDeque};

/// A point `(x, y)`.
pub type Pt = (i32, i32);

/// The boost message for two edges that cross off the integer grid.
pub const PRECISION: &str = "GTL 45 Boolean error, precision insufficient to represent edge intersection coordinate value.";

/// A polygon with holes, each ring as its points in order (`polygon_45_with_holes_data`).
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct PolygonWithHoles {
    pub outer: Vec<Pt>,
    pub holes: Vec<Vec<Pt>>,
}

/// `Vertex45Compact`: a point and the change of the count across each of its four edge
/// directions — index `rise + 1` for the edges leaving it to the right (rise −1, 0, +1) and 3
/// for the vertical one.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Vertex45Compact {
    pub pt: Pt,
    pub count: [i32; 4],
}

impl Vertex45Compact {
    fn is_45(&self) -> bool {
        self.count[0] != 0 || self.count[2] != 0
    }
}

/// `polygon_45_set_data<int>`.
#[derive(Debug, Clone)]
pub struct Polygon45Set {
    data: Vec<Vertex45Compact>,
    dirty: bool,
    unsorted: bool,
    is_manhattan: bool,
}

impl Default for Polygon45Set {
    fn default() -> Self {
        Polygon45Set { data: Vec::new(), dirty: false, unsorted: false, is_manhattan: true }
    }
}

/// `characterizeEdge45`: the count index an edge from `p1` to `p2` changes, and by how much.
fn characterize_edge_45(p1: Pt, p2: Pt) -> (usize, i32) {
    if p1.0 == p2.0 {
        return (3, -1);
    }
    let first = if p1.1 == p2.1 {
        1
    } else if p1.0 < p2.0 {
        if p1.1 < p2.1 {
            2
        } else {
            0
        }
    } else if p1.1 < p2.1 {
        0
    } else {
        2
    };
    (first, 1)
}

/// The winding `insert` gives a ring: `point_sequence_area` (exact here, a `long double` there,
/// both exact on 32-bit coordinates) below zero is counterclockwise. `true` = clockwise (`LOW`).
fn winding_is_clockwise(pts: &[Pt]) -> bool {
    let Some(&first) = pts.first() else { return true };
    let yb = first.1 as i128;
    // twice the area: the /2 of each term is exact in the reference's floating type
    let mut area2: i128 = 0;
    let mut prev = first;
    for &p in &pts[1..] {
        let (x1, x2) = (prev.0 as i128, p.0 as i128);
        if x1 != x2 {
            area2 += (x2 - x1) * ((p.1 as i128 - yb) + (prev.1 as i128 - yb));
        }
        prev = p;
    }
    if first != prev {
        let (x1, x2) = (prev.0 as i128, first.0 as i128);
        area2 += (x2 - x1) * ((first.1 as i128 - yb) + (prev.1 as i128 - yb));
    }
    area2 >= 0
}

impl Polygon45Set {
    pub fn new() -> Polygon45Set {
        Polygon45Set::default()
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    fn insert_clean(&mut self, v: Vertex45Compact) {
        if v.is_45() {
            self.is_manhattan = false;
        }
        self.data.push(v);
    }

    /// `insert_vertex_half_edge_45_pair`: the vertex `p2` between the edges `p1 → p2 → p3`.
    fn insert_vertex_half_edge_45_pair(&mut self, p1: Pt, p2: Pt, p3: Pt, clockwise: bool) {
        let multiplier = if clockwise { -1 } else { 1 };
        let mut v = Vertex45Compact { pt: p2, count: [0; 4] };
        let (i, c) = characterize_edge_45(p1, p2);
        v.count[i] += c * -multiplier;
        let (i, c) = characterize_edge_45(p2, p3);
        v.count[i] += c * multiplier;
        self.data.push(v);
        if v.is_45() {
            self.is_manhattan = false;
        }
    }

    /// `insert_vertex_sequence`: a ring, duplicate points skipped, closed whether or not its last
    /// point repeats its first.
    fn insert_vertex_sequence(&mut self, pts: &[Pt], mut clockwise: bool, is_hole: bool) {
        if is_hole {
            clockwise = !clockwise;
        }
        let mut it = pts.iter().copied();
        let Some(first) = it.next() else { return };
        let second = loop {
            match it.next() {
                None => return,
                Some(p) if p == first => continue,
                Some(p) => break p,
            }
        };
        let (mut prev, mut prev_prev) = (second, first);
        for p in it {
            if p == prev {
                continue;
            }
            self.insert_vertex_half_edge_45_pair(prev_prev, prev, p, clockwise);
            prev_prev = prev;
            prev = p;
        }
        if prev != first {
            self.insert_vertex_half_edge_45_pair(prev_prev, prev, first, clockwise);
            self.insert_vertex_half_edge_45_pair(prev, first, second, clockwise);
        } else {
            self.insert_vertex_half_edge_45_pair(prev_prev, first, second, clockwise);
        }
        self.dirty = true;
        self.unsorted = true;
    }

    /// `insert(polygon_45_data)`.
    pub fn insert_polygon(&mut self, pts: &[Pt]) {
        self.insert_vertex_sequence(pts, winding_is_clockwise(pts), false);
    }

    /// `insert(polygon_45_with_holes_data)`: the outer ring with its winding, each hole with its
    /// own, as holes.
    pub fn insert_polygon_with_holes(&mut self, p: &PolygonWithHoles) {
        self.insert_vertex_sequence(&p.outer, winding_is_clockwise(&p.outer), false);
        for h in &p.holes {
            self.insert_vertex_sequence(h, winding_is_clockwise(h), true);
        }
    }

    /// `insert(polygon_45_set_data)`: its data appended as it stands.
    pub fn insert_set(&mut self, other: &Polygon45Set) {
        self.data.extend_from_slice(&other.data);
        self.dirty = true;
        self.unsorted = true;
        if !other.is_manhattan {
            self.is_manhattan = false;
        }
    }

    /// `insert(polygon_90_set_data)` of a HORIZONTAL set holding these rectangles (each inserted
    /// as odb's `toPolygon90`, whose corner vertices are the rectangle's `(y, (x, ±1))`): each
    /// vertex becomes one with the vertical and the horizontal count.
    pub fn insert_rects_90(&mut self, rects: &[(i32, i32, i32, i32)]) {
        for &(xl, yl, xh, yh) in rects {
            for (x, y, c) in [(xl, yl, 1), (xh, yl, -1), (xl, yh, -1), (xh, yh, 1)] {
                let mut v = Vertex45Compact { pt: (x, y), count: [0; 4] };
                v.count[3] = c;
                v.count[1] = c;
                self.insert_clean(v);
            }
        }
        self.dirty = true;
        self.unsorted = true;
    }

    /// The vertex data as the set holds it (`begin()`..`end()`).
    pub fn data(&self) -> &[Vertex45Compact] {
        &self.data
    }

    fn sort(&mut self) {
        if self.unsorted {
            // `polygon_sort` by point only; equal points are summed wherever they are read, so
            // their order is not a value
            self.data.sort_by_key(|v| v.pt);
            self.unsorted = false;
        }
    }

    /// `clean`: sorted, then the OR of everything inserted (`applyAdaptiveUnary_<0>`).
    pub fn clean(&mut self) -> Result<(), String> {
        self.sort();
        if self.dirty {
            self.apply_adaptive_unary_or()?;
            self.dirty = false;
        }
        Ok(())
    }

    fn apply_adaptive_unary_or(&mut self) -> Result<(), String> {
        if self.is_manhattan {
            // the vertical edges as a VERTICAL polygon_90 set, sorted and OR-ed
            let mut l90: Vec<(i32, (i32, i32))> = self.data.iter().filter(|v| v.count[3] != 0).map(|v| (v.pt.0, (v.pt.1, v.count[3]))).collect();
            l90.sort();
            vyges_drt::polygon90::apply_boolean_or(&mut l90);
            self.data = l90
                .into_iter()
                .map(|(x, (y, c))| {
                    let mut v = Vertex45Compact { pt: (x, y), count: [0; 4] };
                    v.count[3] = c;
                    v.count[1] = c;
                    v
                })
                .collect();
            self.is_manhattan = true;
        } else {
            self.sort();
            let (data, manhattan) = apply_unary_45_or(&self.data)?;
            self.data = data;
            self.is_manhattan = manhattan;
        }
        self.dirty = false;
        self.unsorted = false;
        Ok(())
    }

    /// `get_polygons_with_holes`.
    pub fn polygons_with_holes(&mut self) -> Result<Vec<PolygonWithHoles>, String> {
        self.clean()?;
        Formation::new(false).scan(&self.data)
    }

    /// `get_trapezoids` (slicing VERTICAL): polygons of three or four vertices, each closed.
    pub fn trapezoids(&mut self) -> Result<Vec<Vec<Pt>>, String> {
        self.clean()?;
        Tiling::new().scan(&self.data)
    }
}

// ------------------------------------------------------------------------------------------------
// boolean_op_45: the unary OR scan (`Scan45<Count1, unary_op_45_output_functor<0>>`)
// ------------------------------------------------------------------------------------------------

/// `Vertex45`: a point, the rise of the edge leaving it to the right, a count.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Vertex45 {
    pt: Pt,
    rise: i32,
    count: i32,
}

impl Vertex45 {
    fn eval_at_x(&self, x: i32) -> i32 {
        self.pt.1 + self.rise * (x - self.pt.0)
    }
}

/// `Scan45ElementT<Count1>`.
#[derive(Debug, Clone, Copy)]
struct Scan45Element {
    x: i32,
    y: i32,
    rise: i32,
    count: i32,
}

impl Scan45Element {
    fn eval_at_x(&self, x: i32) -> i32 {
        self.y + self.rise * (x - self.x)
    }
}

/// `lessScan45Element` and `lessVertex45`: by the height at `x`, then by rise — reversed just
/// before `x`.
fn less_at(y1: i32, r1: i32, y2: i32, r2: i32, just_before: bool) -> bool {
    if y1 < y2 {
        return true;
    }
    if y1 == y2 {
        return if just_before { r1 > r2 } else { r1 < r2 };
    }
    false
}

type Scan45Vertex = (Pt, [i32; 4]);

/// `applyLogic<0>(Count1, Count1)`: −1 where the OR turns off, +1 where it turns on.
fn edge_type(c1: i32, c2: i32) -> i32 {
    let (l1, l2) = (c1 > 0, c2 > 0);
    if l1 && !l2 {
        -1
    } else if !l1 && l2 {
        1
    } else {
        0
    }
}

/// `unary_op_45_output_functor<0>`: `low` = the `LOW` end.
fn emit(out: &mut Vec<Vertex45>, c1: i32, c2: i32, pt: Pt, rise: i32, low: bool) {
    let e = edge_type(c1, c2);
    if e != 0 {
        out.push(Vertex45 { pt, rise, count: e * if low { -1 } else { 1 } });
    }
}

struct Scan45 {
    elems: Vec<Scan45Element>,
    /// `scanData_`: element ids in the comparator's order.
    order: Vec<usize>,
    cross_queue: BTreeSet<Pt>,
    cross_vector: Vec<Scan45Vertex>,
    x: i32,
    just_before: bool,
}

impl Scan45 {
    fn new() -> Scan45 {
        Scan45 { elems: Vec::new(), order: Vec::new(), cross_queue: BTreeSet::new(), cross_vector: Vec::new(), x: i32::MIN, just_before: false }
    }

    fn less(&self, a: &Scan45Element, b: &Scan45Element) -> bool {
        less_at(a.eval_at_x(self.x), a.rise, b.eval_at_x(self.x), b.rise, self.just_before)
    }

    fn pos(&self, id: usize) -> usize {
        self.order.iter().position(|&i| i == id).expect("live scan element")
    }

    fn lower_bound(&self, key: &Scan45Element) -> usize {
        self.order.iter().position(|&i| !self.less(&self.elems[i], key)).unwrap_or(self.order.len())
    }

    /// `lookUp_`: the first element not below `y` (from rise 1 just before `x`, −1 after).
    fn look_up(&self, y: i32) -> usize {
        let key = Scan45Element { x: self.x, y, rise: -1 + 2 * self.just_before as i32, count: 0 };
        self.lower_bound(&key)
    }

    fn eval(&self, pos: usize) -> i32 {
        self.elems[self.order[pos]].eval_at_x(self.x)
    }

    /// `scanData_.insert`: dropped (the existing one returned) when an equal element is there.
    fn insert(&mut self, e: Scan45Element) -> usize {
        let p = self.lower_bound(&e);
        if p < self.order.len() && !self.less(&e, &self.elems[self.order[p]]) {
            return self.order[p];
        }
        self.elems.push(e);
        let id = self.elems.len() - 1;
        self.order.insert(p, id);
        id
    }

    fn erase(&mut self, id: usize) {
        let p = self.pos(id);
        self.order.remove(p);
    }

    /// `scan`: every input stop, the crossings scheduled between them processed at their own x.
    fn scan(&mut self, out: &mut Vec<Vertex45>, input: &[Scan45Vertex]) -> Result<(), String> {
        let mut begin = 0;
        while begin < input.len() {
            let mut next_x = input[begin].0 .0;
            if !self.cross_vector.is_empty() && self.cross_vector[0].0 .0 < next_x {
                next_x = self.cross_vector[0].0 .0;
            }
            if next_x != self.x {
                if let Some(&first) = self.cross_queue.iter().next() {
                    if first.0 < next_x {
                        next_x = next_x.min(first.0);
                    }
                }
                self.just_before = true;
                self.x = next_x;
                self.advance(out)?;
                self.just_before = false;
                if !self.cross_vector.is_empty() && next_x == input[begin].0 .0 {
                    begin = self.merge_cross(input, begin);
                }
                let cv = std::mem::take(&mut self.cross_vector);
                self.process_event(out, &cv, 0)?;
            } else {
                begin = self.process_event(out, input, begin)?;
            }
        }
        Ok(())
    }

    /// `advance_`: the crossings queued at the current x.
    fn advance(&mut self, out: &mut Vec<Vertex45>) -> Result<(), String> {
        let mut erase_vec: Vec<usize> = Vec::new();
        while let Some(&cp) = self.cross_queue.iter().next() {
            if cp.0 != self.x {
                break;
            }
            self.cross_queue.remove(&cp);
            let mut vertex: Scan45Vertex = (cp, [0; 4]);
            let mut low = self.look_up(cp.1);
            if low == self.order.len() || self.eval(low) != cp.1 {
                continue;
            }
            let mut count_below = 0;
            let mut search_down = low;
            while search_down != 0 && self.eval(search_down) == cp.1 {
                search_down -= 1;
                count_below = self.elems[self.order[search_down]].count;
            }
            let mut count = [count_below; 4];
            let mut erase_itrs: Vec<usize> = Vec::new();
            while low < self.order.len() && self.eval(low) == cp.1 {
                let e = self.elems[self.order[low]];
                for index in (0..=(e.rise + 1) as usize).rev() {
                    count[index] = e.count;
                }
                erase_itrs.push(self.order[low]);
                low += 1;
            }
            if erase_itrs.len() > 3 {
                return Err("polygon_45 scan: more than three edges at a crossing".into());
            }
            if erase_itrs.len() == 1 {
                self.find_cross(erase_itrs[0])?;
                continue;
            }
            let mut current = count_below;
            for &id in &erase_itrs {
                let e = self.elems[id];
                emit(out, current, e.count, cp, e.rise, true);
                current = e.count;
            }
            erase_vec.extend_from_slice(&erase_itrs);
            vertex.1[2] = count[2] - count_below;
            vertex.1[1] = count[1] - count[2];
            vertex.1[0] = count[0] - count[1];
            self.cross_vector.push(vertex);
        }
        let mut search_vec: Vec<usize> = Vec::new();
        for &id in &erase_vec {
            let p = self.pos(id);
            if p != 0 {
                let s = self.order[p - 1];
                if search_vec.last() != Some(&s) {
                    search_vec.push(s);
                }
            }
            self.erase(id);
        }
        for &s in &search_vec {
            self.find_cross(s)?;
        }
        Ok(())
    }

    /// `mergeCross_`: the crossings merged by y into the input stop at the current x.
    fn merge_cross(&mut self, input: &[Scan45Vertex], mut begin: usize) -> usize {
        let vec = std::mem::take(&mut self.cross_vector);
        let mut merge_end = begin;
        while merge_end < input.len() && input[merge_end].0 .0 == self.x {
            merge_end += 1;
        }
        for v in vec {
            while begin != merge_end && input[begin].0 .1 < v.0 .1 {
                self.cross_vector.push(input[begin]);
                begin += 1;
            }
            self.cross_vector.push(v);
        }
        while begin != merge_end {
            self.cross_vector.push(input[begin]);
            begin += 1;
        }
        begin
    }

    /// `processEvent_`: the input vertices at the current x, from `begin`; returns where it
    /// stopped.
    fn process_event(&mut self, out: &mut Vec<Vertex45>, input: &[Scan45Vertex], mut begin: usize) -> Result<usize, String> {
        let mut vertical_count = 0i32;
        let mut prev_point: Pt = (0, 0);
        // `prevIter`: an element id, None = end
        let mut prev_iter: Option<usize> = None;
        while begin < input.len() && input[begin].0 .0 == self.x {
            let mut vertex = input[begin];
            let fake = vertical_count != 0 && prev_iter.is_some_and(|id| self.elems[id].eval_at_x(self.x) < vertex.0 .1);
            if fake {
                let id = prev_iter.expect("checked by `fake`");
                vertex = ((self.x, self.elems[id].eval_at_x(self.x)), [0; 4]);
            } else {
                begin += 1;
                while begin < input.len() && input[begin].0 .0 == self.x && input[begin].0 .1 == vertex.0 .1 {
                    for i in 0..4 {
                        vertex.1[i] += input[begin].1[i];
                    }
                    begin += 1;
                }
            }
            let mut current = vertical_count;
            for i in 0..3 {
                current += vertex.1[i];
                vertex.1[i] = current;
            }
            let mut count_below = 0;
            let mut low = self.look_up(vertex.0 .1);
            if low != 0 {
                count_below = self.elems[self.order[low - 1]].count;
            }
            let mut count_at = [count_below - vertical_count; 4];
            if vertical_count != 0 {
                emit(out, count_below - vertical_count, count_below, prev_point, 2, false);
                emit(out, count_below - vertical_count, count_below, vertex.0, 2, true);
            }
            let mut current = count_below - vertical_count;
            while low < self.order.len() && self.eval(low) == vertex.0 .1 {
                let e = self.elems[self.order[low]];
                for i in (e.rise + 1) as usize..3 {
                    count_at[i] = e.count;
                }
                if (e.x, e.y) != vertex.0 {
                    emit(out, current, e.count, vertex.0, e.rise, true);
                }
                current = e.count;
                let next = if low + 1 < self.order.len() { Some(self.order[low + 1]) } else { None };
                self.order.remove(low);
                if let Some(n) = next {
                    self.find_cross(n)?;
                }
                // `lowIter = nextIter`: the next element's position after the erase (find_cross
                // inserts nothing into the scan)
                low = match next {
                    Some(n) => self.pos(n),
                    None => self.order.len(),
                };
            }
            vertical_count += vertex.1[3];
            prev_point = vertex.0;
            prev_iter = if low < self.order.len() { Some(self.order[low]) } else { None };
            for i in 0..4 {
                vertex.1[i] += count_at[i];
            }
            for i in 0..3 {
                if vertex.1[i] != count_below {
                    let id = self.insert(Scan45Element { x: vertex.0 .0, y: vertex.0 .1, rise: i as i32 - 1, count: vertex.1[i] });
                    self.find_cross(id)?;
                    emit(out, count_below, vertex.1[i], vertex.0, i as i32 - 1, false);
                }
                count_below = vertex.1[i];
            }
        }
        Ok(begin)
    }

    /// `scheduleCross0_`: `a` horizontal.
    fn schedule_cross0(&mut self, a: usize, b: usize) {
        let y1 = self.elems[a].eval_at_x(self.x) as i64;
        let y2 = self.elems[b].eval_at_x(self.x) as i64;
        let delta = (y1 - y2).abs();
        if delta + self.x as i64 <= i32::MAX as i64 {
            self.cross_queue.insert((self.x + delta as i32, y1 as i32));
        }
    }

    /// `scheduleCross1_`: neither horizontal, `a` above `b`.
    fn schedule_cross1(&mut self, a: usize, b: usize) -> Result<(), String> {
        let y1 = self.elems[a].eval_at_x(self.x) as i64;
        let y2 = self.elems[b].eval_at_x(self.x) as i64;
        let delta = y1 - y2;
        let max = i32::MAX as i64;
        if (delta & 1) == 1 {
            if delta == 1 {
                return Err(PRECISION.into());
            }
            let h = (y1 - y2) / 2;
            if h + self.x as i64 <= max && h + y2 <= max {
                self.cross_queue.insert((self.x + h as i32, (y2 + h) as i32));
                self.cross_queue.insert((self.x + h as i32, (y2 + h) as i32 + 1));
            }
        } else {
            let h = (y1 - y2) / 2;
            if h + self.x as i64 <= max && h + y2 <= max {
                self.cross_queue.insert((self.x + h as i32, (y2 + h) as i32));
            }
        }
        Ok(())
    }

    /// `findCross_`: the crossings of an element with its neighbours.
    fn find_cross(&mut self, id: usize) -> Result<(), String> {
        let p = self.pos(id);
        let rise = self.elems[id].rise;
        if p != 0 && rise < 1 {
            let below = self.order[p - 1];
            let br = self.elems[below].rise;
            if rise == 0 {
                if br == 1 {
                    self.schedule_cross0(id, below);
                }
            } else if br == 1 {
                self.schedule_cross1(id, below)?;
            } else if br == 0 {
                self.schedule_cross0(below, id);
            }
        }
        if p + 1 < self.order.len() && rise > -1 {
            let above = self.order[p + 1];
            let ar = self.elems[above].rise;
            if rise == 0 {
                if ar == -1 {
                    self.schedule_cross0(id, above);
                }
            } else if ar == -1 {
                self.schedule_cross1(above, id)?;
            } else if ar == 0 {
                self.schedule_cross0(above, id);
            }
        }
        Ok(())
    }
}

/// `applyUnary45OpOnVectors<int, 0>`: the sorted vertex data scanned stop by stop; the output
/// summed per point, a point two events cancel dropped. Returns the data and whether it is
/// Manhattan.
fn apply_unary_45_or(lvalue: &[Vertex45Compact]) -> Result<(Vec<Vertex45Compact>, bool), String> {
    let mut manhattan = true;
    let mut scan = Scan45::new();
    let mut event_out: Vec<Vertex45> = Vec::new();
    let mut event_in: Vec<Scan45Vertex> = Vec::new();
    let mut result: Vec<Vertex45Compact> = Vec::new();
    let mut x = i32::MAX;
    let flush = |event_out: &mut Vec<Vertex45>, result: &mut Vec<Vertex45Compact>, manhattan: &mut bool| {
        // `polygon_sort` by (x, y, rise); equal ones are summed below
        event_out.sort_by_key(|v| (v.pt, v.rise));
        let mut pt_count = 0usize;
        for e in event_out.iter() {
            if result.last().is_some_and(|b| b.pt == e.pt) {
                result.last_mut().expect("checked by is_some_and").count[(e.rise + 1) as usize] += e.count;
                pt_count += 1;
            } else {
                if let Some(b) = result.last() {
                    if b.is_45() {
                        *manhattan = false;
                    }
                    if pt_count == 2 && b.count == [0; 4] {
                        result.pop();
                    }
                }
                let mut v = Vertex45Compact { pt: e.pt, count: [0; 4] };
                v.count[(e.rise + 1) as usize] = e.count;
                result.push(v);
                pt_count = 1;
            }
        }
        if pt_count == 2 && result.last().is_some_and(|b| b.count == [0; 4]) {
            result.pop();
        }
        event_out.clear();
    };
    for v in lvalue {
        if v.pt.0 != x {
            scan.scan(&mut event_out, &event_in)?;
            flush(&mut event_out, &mut result, &mut manhattan);
            event_in.clear();
            x = v.pt.0;
        }
        event_in.push((v.pt, v.count));
    }
    scan.scan(&mut event_out, &event_in)?;
    flush(&mut event_out, &mut result, &mut manhattan);
    if result.last().is_some_and(|b| b.is_45()) {
        manhattan = false;
    }
    Ok((result, manhattan))
}

// ------------------------------------------------------------------------------------------------
// polygon_45_formation: active tails, formation, tiling
// ------------------------------------------------------------------------------------------------

/// `ActiveTail45`: one end of a chain of points; `tail` its polyline (shared by both ends).
#[derive(Debug, Clone)]
struct Tail {
    tail: Option<usize>,
    other: Option<usize>,
    holes: VecDeque<usize>,
    head: bool,
}

#[derive(Default)]
struct Tails {
    tails: Vec<Tail>,
    lines: Vec<VecDeque<Pt>>,
}

impl Tails {
    fn new_line(&mut self, p: Pt) -> usize {
        self.lines.push(VecDeque::from([p]));
        self.lines.len() - 1
    }

    fn push_tail(&mut self, t: Tail) -> usize {
        self.tails.push(t);
        self.tails.len() - 1
    }

    /// `pushPoint`: at the head the front, else the back; a point continuing a horizontal or
    /// vertical run replaces the run's end.
    fn push_point(&mut self, t: usize, p: Pt) {
        let head = self.tails[t].head;
        let line = &mut self.lines[self.tails[t].tail.expect("a tail with a polyline")];
        if line.len() < 2 {
            if head {
                line.push_front(p);
            } else {
                line.push_back(p);
            }
            return;
        }
        let (end, next) = if head { (0, 1) } else { (line.len() - 1, line.len() - 2) };
        let (first, second) = (line[end], line[next]);
        if (second.1 == p.1 && first.1 == p.1) || (second.0 == p.0 && first.0 == p.0) {
            line[end] = p;
        } else if head {
            line.push_front(p);
        } else {
            line.push_back(p);
        }
    }

    /// `copyHoles`: `that`'s holes moved to the end of `this`'s.
    fn copy_holes(&mut self, this: usize, that: usize) {
        if this == that {
            return;
        }
        let moved = std::mem::take(&mut self.tails[that].holes);
        self.tails[this].holes.extend(moved);
    }

    /// `addHole`.
    fn add_hole(&mut self, this: usize, hole: usize) {
        self.tails[this].holes.push_back(hole);
        self.copy_holes(this, hole);
        let other = self.tails[hole].other.expect("a hole's other end");
        self.copy_holes(this, other);
    }

    /// `join`: this chain and `tail`'s become one, `this` and `tail` dropping out.
    fn join(&mut self, this: usize, tail: usize) -> bool {
        if Some(tail) == self.tails[this].other || self.tails[tail].head == self.tails[this].head || self.tails[this].tail.is_none() {
            return false;
        }
        let this_other = self.tails[this].other.expect("other end");
        let tail_other = self.tails[tail].other.expect("other end");
        if !self.tails[this_other].head {
            self.copy_holes(this_other, tail);
            self.copy_holes(this_other, this);
        } else {
            self.copy_holes(tail_other, this);
            self.copy_holes(tail_other, tail);
        }
        let (mut t1, mut t2) = (self.tails[this].tail.expect("checked above"), self.tails[tail].tail.expect("a live tail has a polyline"));
        if self.tails[this].head {
            std::mem::swap(&mut t1, &mut t2);
        }
        let moved = std::mem::take(&mut self.lines[t2]);
        self.lines[t1].extend(moved);
        self.tails[this_other].tail = Some(t1);
        self.tails[tail_other].tail = Some(t1);
        self.tails[this_other].other = Some(tail_other);
        self.tails[tail_other].other = Some(this_other);
        self.tails[this].tail = None;
        self.tails[tail].tail = None;
        self.tails[tail].other = None;
        self.tails[this].other = None;
        true
    }

    /// `createActiveTail45sAsPair` without fractured holes.
    fn create_pair(&mut self, p: Pt, solid: bool, phole: Option<usize>) -> (usize, usize) {
        let line = self.new_line(p);
        let at1 = self.push_tail(Tail { tail: Some(line), other: None, holes: VecDeque::new(), head: solid });
        let at2 = self.push_tail(Tail { tail: Some(line), other: Some(at1), holes: VecDeque::new(), head: !solid });
        self.tails[at1].other = Some(at2);
        if let Some(h) = phole {
            self.add_hole(at2, h);
        }
        (at1, at2)
    }

    fn ring(&self, t: usize) -> Vec<Pt> {
        self.lines[self.tails[t].tail.expect("a closed figure's polyline")].iter().copied().collect()
    }

    /// `joinChains`: closing a figure writes a solid one out (with its holes) or returns the
    /// hole; otherwise the chains are joined.
    fn join_chains(&mut self, p: Pt, at1: usize, at2: usize, solid: bool, out: &mut Output) -> Option<usize> {
        if self.tails[at1].other == Some(at2) {
            self.push_point(at1, p);
            self.push_point(at2, p);
            if solid {
                let other = self.tails[at1].other.expect("at1 is at2's other end");
                self.copy_holes(at1, other);
                let outer = self.ring(at1);
                match out {
                    Output::WithHoles(v) => {
                        let holes = self.tails[at1].holes.iter().map(|&h| self.ring(h)).collect();
                        v.push(PolygonWithHoles { outer, holes });
                    }
                    Output::Plain(v) => v.push(outer),
                }
                return None;
            }
            return Some(at1);
        }
        self.push_point(at1, p);
        self.join(at1, at2);
        None
    }
}

enum Output {
    WithHoles(Vec<PolygonWithHoles>),
    Plain(Vec<Vec<Pt>>),
}

/// The scanline of the formation and the tiling: `std::map<Vertex45, ActiveTail45*,
/// lessVertex45>`, as (key, tail) entries in an arena and their ids in the comparator's order.
struct ScanMap {
    entries: Vec<(Vertex45, usize)>,
    order: Vec<usize>,
    x: i32,
    just_before: bool,
}

impl ScanMap {
    fn new() -> ScanMap {
        ScanMap { entries: Vec::new(), order: Vec::new(), x: i32::MIN, just_before: false }
    }

    fn less(&self, a: &Vertex45, b: &Vertex45) -> bool {
        less_at(a.eval_at_x(self.x), a.rise, b.eval_at_x(self.x), b.rise, self.just_before)
    }

    fn look_up(&self, y: i32) -> usize {
        let key = Vertex45 { pt: (self.x, y), rise: -1 + 2 * self.just_before as i32, count: 0 };
        self.order.iter().position(|&i| !self.less(&self.entries[i].0, &key)).unwrap_or(self.order.len())
    }

    fn key(&self, pos: usize) -> Vertex45 {
        self.entries[self.order[pos]].0
    }

    fn eval(&self, pos: usize) -> i32 {
        self.key(pos).eval_at_x(self.x)
    }

    fn insert(&mut self, k: Vertex45, t: usize) {
        let p = self.order.iter().position(|&i| !self.less(&self.entries[i].0, &k)).unwrap_or(self.order.len());
        if p < self.order.len() && !self.less(&k, &self.entries[self.order[p]].0) {
            return;
        }
        self.entries.push((k, t));
        self.order.insert(p, self.entries.len() - 1);
    }

    fn erase(&mut self, id: usize) {
        let p = self.order.iter().position(|&i| i == id).expect("live scan entry");
        self.order.remove(p);
    }
}

/// `Polygon45Formation` (holes kept, not fractured).
struct Formation {
    map: ScanMap,
    t: Tails,
}

impl Formation {
    fn new(_fracture_holes: bool) -> Formation {
        Formation { map: ScanMap::new(), t: Tails::default() }
    }

    fn scan(mut self, input: &[Vertex45Compact]) -> Result<Vec<PolygonWithHoles>, String> {
        let mut out = Output::WithHoles(Vec::new());
        let mut begin = 0;
        while begin < input.len() {
            self.map.x = input[begin].pt.0;
            begin = self.process_event(&mut out, input, begin)?;
        }
        match out {
            Output::WithHoles(v) => Ok(v),
            Output::Plain(_) => unreachable!(),
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn process_point(&mut self, out: &mut Output, elements: &mut Vec<(Vertex45, usize)>, p: Pt, counts: &mut [i32; 4], tails: &mut [Option<usize>; 4], incoming: &mut [i32; 4]) -> (i32, Option<usize>) {
        let t = &mut self.t;
        let mut ret: Option<usize> = None;
        let mut ret_count = 0;
        // case 1: closing solid corners
        for i in 0..3 {
            if counts[i] == -1 {
                for j in i + 1..4 {
                    if counts[j] != 0 {
                        if counts[j] == 1 {
                            t.join_chains(p, tails[i].expect("a nonzero count at the point has its tail"), tails[j].expect("a nonzero count at the point has its tail"), true, out);
                            counts[i] = 0;
                            counts[j] = 0;
                            tails[i] = None;
                            tails[j] = None;
                        }
                        break;
                    }
                }
            }
        }
        // case 2: incoming pairs that open a solid
        for i in 0..3 {
            if incoming[i] == 1 {
                for j in i + 1..4 {
                    if incoming[j] != 0 {
                        if incoming[j] == -1 {
                            let (a, b) = t.create_pair(p, true, None);
                            if j == 3 {
                                ret = Some(a);
                                ret_count = 1;
                            } else {
                                elements.push((Vertex45 { pt: p, rise: j as i32 - 1, count: -1 }, a));
                            }
                            elements.push((Vertex45 { pt: p, rise: i as i32 - 1, count: 1 }, b));
                            incoming[i] = 0;
                            incoming[j] = 0;
                        }
                        break;
                    }
                }
            }
        }
        // case 3: pass through, solid on top
        for i in 0..4 {
            if counts[i] != 0 {
                if counts[i] == 1 {
                    for j in (0..4).rev() {
                        if incoming[j] != 0 {
                            if incoming[j] == 1 {
                                let ti = tails[i].expect("a nonzero count at the point has its tail");
                                t.push_point(ti, p);
                                if j == 3 {
                                    ret = Some(ti);
                                    ret_count = -1;
                                } else {
                                    elements.push((Vertex45 { pt: p, rise: j as i32 - 1, count: incoming[j] }, ti));
                                }
                                tails[i] = None;
                                counts[i] = 0;
                                incoming[j] = 0;
                            }
                            break;
                        }
                    }
                }
                break;
            }
        }
        // case 4: pass through, solid on the bottom
        for i in (0..4).rev() {
            if counts[i] != 0 {
                if counts[i] == -1 {
                    for j in 0..4 {
                        if incoming[j] != 0 {
                            if incoming[j] == -1 {
                                let ti = tails[i].expect("a nonzero count at the point has its tail");
                                t.push_point(ti, p);
                                if j == 3 {
                                    ret = Some(ti);
                                    ret_count = 1;
                                } else {
                                    elements.push((Vertex45 { pt: p, rise: j as i32 - 1, count: incoming[j] }, ti));
                                }
                                tails[i] = None;
                                counts[i] = 0;
                                incoming[j] = 0;
                            }
                            break;
                        }
                    }
                }
                break;
            }
        }
        // case 5: the end of a hole
        for i in 0..3 {
            if counts[i] != 0 {
                for j in i + 1..4 {
                    if counts[j] != 0 {
                        ret = t.join_chains(p, tails[i].expect("a nonzero count at the point has its tail"), tails[j].expect("a nonzero count at the point has its tail"), false, out);
                        tails[i] = None;
                        tails[j] = None;
                        counts[i] = 0;
                        counts[j] = 0;
                        break;
                    }
                }
                break;
            }
        }
        // case 6: the beginning of a hole
        for i in 0..3 {
            if incoming[i] != 0 {
                for j in i + 1..4 {
                    if incoming[j] != 0 {
                        let holep = if counts[3] == 0 { tails[3] } else { None };
                        let (a, b) = t.create_pair(p, false, holep);
                        if j == 3 {
                            ret = Some(a);
                            ret_count = -1;
                        } else {
                            elements.push((Vertex45 { pt: p, rise: j as i32 - 1, count: incoming[j] }, a));
                        }
                        elements.push((Vertex45 { pt: p, rise: i as i32 - 1, count: incoming[i] }, b));
                        incoming[i] = 0;
                        incoming[j] = 0;
                        break;
                    }
                }
                break;
            }
        }
        (ret_count, ret)
    }

    fn process_event(&mut self, out: &mut Output, input: &[Vertex45Compact], begin: usize) -> Result<usize, String> {
        self.map.just_before = true;
        let x = self.map.x;
        let mut vertical_tail: Option<usize> = None;
        let mut vertical_count = 0;
        let mut cur = begin;
        let mut element_ids: Vec<usize> = Vec::new();
        let mut elements: Vec<(Vertex45, usize)> = Vec::new();
        while cur < input.len() && input[cur].pt.0 == x {
            let y = input[cur].pt.1;
            let mut iter = self.map.look_up(y);
            let mut counts = [0i32; 4];
            let mut tails: [Option<usize>; 4] = [None, None, None, vertical_tail];
            while iter < self.map.order.len() && self.map.eval(iter) == y {
                let id = self.map.order[iter];
                element_ids.push(id);
                let (k, tl) = self.map.entries[id];
                let index = (k.rise + 1) as usize;
                counts[index] = k.count;
                tails[index] = Some(tl);
                iter += 1;
            }
            let mut incoming = [0i32; 4];
            loop {
                for i in 0..4 {
                    incoming[i] += input[cur].count[i];
                }
                cur += 1;
                if !(cur < input.len() && input[cur].pt.1 == y && input[cur].pt.0 == x) {
                    break;
                }
            }
            if vertical_tail.is_some() {
                counts[3] = -vertical_count;
            }
            incoming[3] *= -1;
            for i in 0..4 {
                incoming[i] += counts[i];
            }
            let (vc, vt) = self.process_point(out, &mut elements, (x, y), &mut counts, &mut tails, &mut incoming);
            vertical_count = vc;
            vertical_tail = vt;
            if let Some(vt) = vertical_tail {
                if vertical_count == 0 {
                    // a hole out of the point just processed: it goes to the edge above
                    let above_ok = iter < self.map.order.len();
                    let handle = cur == input.len() || input[cur].pt.0 != x || (above_ok && input[cur].pt.1 >= self.map.eval(iter));
                    if handle {
                        if !above_ok {
                            return Err("polygon_45 formation: a hole with no edge above it".into());
                        }
                        let above = self.map.entries[self.map.order[iter]].1;
                        self.t.add_hole(above, vt);
                        vertical_tail = None;
                    }
                }
            }
        }
        for id in element_ids {
            self.map.erase(id);
        }
        self.map.just_before = false;
        for (k, t) in elements {
            self.map.insert(k, t);
        }
        Ok(cur)
    }
}

/// `Polygon45Tiling`: the set sliced vertically into trapezoids.
struct Tiling {
    map: ScanMap,
    t: Tails,
}

impl Tiling {
    fn new() -> Tiling {
        Tiling { map: ScanMap::new(), t: Tails::default() }
    }

    fn scan(mut self, input: &[Vertex45Compact]) -> Result<Vec<Vec<Pt>>, String> {
        let mut out = Output::Plain(Vec::new());
        let mut begin = 0;
        while begin < input.len() {
            self.map.x = input[begin].pt.0;
            begin = self.process_event(&mut out, input, begin)?;
        }
        match out {
            Output::Plain(v) => Ok(v),
            Output::WithHoles(_) => unreachable!(),
        }
    }

    /// `getVerticalPair_`: a vertical cut at the edge below.
    fn get_vertical_pair(&mut self, vp: &mut (Option<usize>, Option<usize>), previter: usize) -> Result<(), String> {
        if previter >= self.map.order.len() {
            return Err("polygon_45 tiling: no edge below a vertical cut".into());
        }
        let id = self.map.order[previter];
        let iter_tail = self.map.entries[id].1;
        let prev_point = (self.map.x, self.map.eval(previter));
        self.t.push_point(iter_tail, prev_point);
        let (a, b) = self.t.create_pair(prev_point, true, None);
        vp.0 = Some(iter_tail);
        vp.1 = Some(a);
        self.map.entries[id].1 = b;
        Ok(())
    }

    #[allow(clippy::too_many_arguments)]
    fn process_point(
        &mut self,
        out: &mut Output,
        elements: &mut Vec<(Vertex45, usize)>,
        vp: &mut (Option<usize>, Option<usize>),
        previter: usize,
        p: Pt,
        counts: &mut [i32; 4],
        tails: &mut [Option<usize>; 4],
        incoming: &mut [i32; 4],
    ) -> Result<(i32, Option<usize>), String> {
        let mut ret: Option<usize> = None;
        let mut vp_out: (Option<usize>, Option<usize>) = (None, None);
        let mut ret_count = 0;
        // case 1
        for i in 0..3 {
            if counts[i] == -1 {
                for j in i + 1..4 {
                    if counts[j] != 0 {
                        if counts[j] == 1 {
                            self.t.join_chains(p, tails[i].expect("a nonzero count at the point has its tail"), tails[j].expect("a nonzero count at the point has its tail"), true, out);
                            counts[i] = 0;
                            counts[j] = 0;
                            tails[i] = None;
                            tails[j] = None;
                        }
                        break;
                    }
                }
            }
        }
        // case 2
        for i in 0..3 {
            if incoming[i] == 1 {
                for j in i + 1..4 {
                    if incoming[j] != 0 {
                        if incoming[j] == -1 {
                            let (a, b) = self.t.create_pair(p, true, None);
                            if j == 3 {
                                ret = Some(a);
                                ret_count = 1;
                            } else {
                                elements.push((Vertex45 { pt: p, rise: j as i32 - 1, count: -1 }, a));
                            }
                            elements.push((Vertex45 { pt: p, rise: i as i32 - 1, count: 1 }, b));
                            incoming[i] = 0;
                            incoming[j] = 0;
                        }
                        break;
                    }
                }
            }
        }
        // case 3: pass through, solid on top
        for i in 0..4 {
            if counts[i] != 0 {
                if counts[i] == 1 {
                    for j in (0..4).rev() {
                        if incoming[j] != 0 {
                            if incoming[j] == 1 {
                                let ti = tails[i].expect("a nonzero count at the point has its tail");
                                // (removing this guard changes no output: the vertical tail
                                // takes another point at the same x before it is read, which
                                // replaces this one — an equivalent mutant, 2026-11-04)
                                if i != 3 {
                                    self.t.push_point(ti, p);
                                }
                                if j == 3 {
                                    ret = Some(ti);
                                    ret_count = -1;
                                } else {
                                    vp_out.0 = Some(ti);
                                    let (a, b) = self.t.create_pair(p, true, None);
                                    vp_out.1 = Some(a);
                                    elements.push((Vertex45 { pt: p, rise: j as i32 - 1, count: incoming[j] }, b));
                                }
                                tails[i] = None;
                                counts[i] = 0;
                                incoming[j] = 0;
                            }
                            break;
                        }
                    }
                }
                break;
            }
        }
        // case 4: pass through, solid on the bottom
        for i in (0..4).rev() {
            if counts[i] != 0 {
                if counts[i] == -1 {
                    for j in 0..4 {
                        if incoming[j] != 0 {
                            if incoming[j] == -1 {
                                let ti = tails[i].expect("a nonzero count at the point has its tail");
                                if i == 3 {
                                    if j == 3 {
                                        ret = Some(ti);
                                        ret_count = 1;
                                    } else {
                                        self.t.push_point(ti, p);
                                        elements.push((Vertex45 { pt: p, rise: j as i32 - 1, count: incoming[j] }, ti));
                                    }
                                } else if j == 3 {
                                    if vp.0.is_none() {
                                        self.get_vertical_pair(vp, previter)?;
                                    }
                                    self.t.join_chains(p, ti, vp.0.expect("the vertical pair is made just before it is used"), true, out);
                                    ret = vp.1;
                                    ret_count = 1;
                                } else {
                                    if vp.0.is_none() {
                                        self.get_vertical_pair(vp, previter)?;
                                    }
                                    self.t.join_chains(p, ti, vp.0.expect("the vertical pair is made just before it is used"), true, out);
                                    self.t.push_point(vp.1.expect("the vertical pair is made just before it is used"), p);
                                    elements.push((Vertex45 { pt: p, rise: j as i32 - 1, count: incoming[j] }, vp.1.expect("the vertical pair is made just before it is used")));
                                }
                                tails[i] = None;
                                counts[i] = 0;
                                incoming[j] = 0;
                            }
                            break;
                        }
                    }
                }
                break;
            }
        }
        // case 5: the end of a hole
        for i in 0..3 {
            if counts[i] != 0 {
                for j in i + 1..4 {
                    if counts[j] != 0 {
                        let ti = tails[i].expect("a nonzero count at the point has its tail");
                        self.t.push_point(ti, p);
                        vp_out.0 = Some(ti);
                        if j == 3 {
                            vp_out.1 = tails[j];
                        } else {
                            if vp.0.is_none() {
                                self.get_vertical_pair(vp, previter)?;
                            }
                            self.t.join_chains(p, tails[j].expect("a nonzero count at the point has its tail"), vp.0.expect("the vertical pair is made just before it is used"), true, out);
                            vp_out.1 = vp.1;
                        }
                        tails[i] = None;
                        tails[j] = None;
                        counts[i] = 0;
                        counts[j] = 0;
                        break;
                    }
                }
                break;
            }
        }
        // case 6: the beginning of a hole
        for i in 0..3 {
            if incoming[i] != 0 {
                for j in i + 1..4 {
                    if incoming[j] != 0 {
                        if vp.0.is_none() {
                            self.get_vertical_pair(vp, previter)?;
                        }
                        self.t.push_point(vp.1.expect("the vertical pair is made just before it is used"), p);
                        if j == 3 {
                            ret = vp.0;
                            ret_count = -1;
                        } else {
                            let (a, b) = self.t.create_pair(p, true, None);
                            elements.push((Vertex45 { pt: p, rise: j as i32 - 1, count: incoming[j] }, b));
                            vp_out.1 = Some(a);
                            vp_out.0 = vp.0;
                        }
                        elements.push((Vertex45 { pt: p, rise: i as i32 - 1, count: incoming[i] }, vp.1.expect("the vertical pair is made just before it is used")));
                        incoming[i] = 0;
                        incoming[j] = 0;
                        break;
                    }
                }
                break;
            }
        }
        *vp = vp_out;
        Ok((ret_count, ret))
    }

    fn process_event(&mut self, out: &mut Output, input: &[Vertex45Compact], begin: usize) -> Result<usize, String> {
        self.map.just_before = true;
        let x = self.map.x;
        let mut vertical_tail: Option<usize> = None;
        let mut vp: (Option<usize>, Option<usize>) = (None, None);
        let mut vertical_count = 0;
        let mut cur = begin;
        let mut element_ids: Vec<usize> = Vec::new();
        let mut elements: Vec<(Vertex45, usize)> = Vec::new();
        while cur < input.len() && input[cur].pt.0 == x {
            let y = input[cur].pt.1;
            let mut iter = self.map.look_up(y);
            let mut counts = [0i32; 4];
            let mut tails: [Option<usize>; 4] = [None, None, None, vertical_tail];
            let mut previter = iter;
            if previter < self.map.order.len() && self.map.eval(previter) >= y && previter != 0 {
                previter -= 1;
            }
            while iter < self.map.order.len() && self.map.eval(iter) == y {
                let id = self.map.order[iter];
                element_ids.push(id);
                let (k, tl) = self.map.entries[id];
                let index = (k.rise + 1) as usize;
                counts[index] = k.count;
                tails[index] = Some(tl);
                iter += 1;
            }
            let mut incoming = [0i32; 4];
            loop {
                for i in 0..4 {
                    incoming[i] += input[cur].count[i];
                }
                cur += 1;
                if !(cur < input.len() && input[cur].pt.1 == y && input[cur].pt.0 == x) {
                    break;
                }
            }
            if vertical_tail.is_some() {
                counts[3] = -vertical_count;
            }
            incoming[3] *= -1;
            for i in 0..4 {
                incoming[i] += counts[i];
            }
            let (vc, vt) = self.process_point(out, &mut elements, &mut vp, previter, (x, y), &mut counts, &mut tails, &mut incoming)?;
            vertical_count = vc;
            vertical_tail = vt;
            if let (Some(first), Some(second)) = vp {
                if iter < self.map.order.len() && (cur == input.len() || input[cur].pt.0 != x || input[cur].pt.1 > self.map.eval(iter)) {
                    // splice the vertical pair into the edge above
                    let id = self.map.order[iter];
                    let tail_above = self.map.entries[id].1;
                    let point = (x, self.map.eval(iter));
                    self.t.push_point(second, point);
                    self.t.join_chains(point, tail_above, first, true, out);
                    self.map.entries[id].1 = second;
                    vp = (None, None);
                }
            }
        }
        for id in element_ids {
            self.map.erase(id);
        }
        self.map.just_before = false;
        for (k, t) in elements {
            self.map.insert(k, t);
        }
        Ok(cur)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // The bump octagon of the reference's check_power_grid_short_iterms (pin bump1/VDD on
    // metal3), as the oracle (instruments/psm/poly45-probe/boost.cc) slices and merges it.
    const BUMP: [Pt; 9] = [(13000, 11000), (13000, 11800), (13600, 12400), (14400, 12400), (15000, 11800), (15000, 11000), (14400, 10400), (13600, 10400), (13000, 11000)];

    #[test]
    fn an_octagon_slices_into_three_trapezoids_in_scan_order_each_starting_top_right() {
        let mut s = Polygon45Set::new();
        s.insert_polygon(&BUMP);
        let t = s.trapezoids().unwrap();
        assert_eq!(
            t,
            vec![
                vec![(13600, 12400), (13000, 11800), (13000, 11000), (13600, 10400), (13600, 12400)],
                vec![(14400, 12400), (13600, 12400), (13600, 10400), (14400, 10400), (14400, 12400)],
                vec![(15000, 11800), (14400, 12400), (14400, 10400), (15000, 11000), (15000, 11800)],
            ]
        );
    }

    #[test]
    fn an_octagon_merged_with_a_strap_is_one_polygon_and_five_pieces() {
        let mut pin = Polygon45Set::new();
        pin.insert_polygon(&BUMP);
        let mut layer = Polygon45Set::new();
        layer.insert_set(&pin);
        layer.insert_rects_90(&[(11000, 11230, 16000, 11570)]);
        let polys = layer.polygons_with_holes().unwrap();
        assert_eq!(polys.len(), 1);
        assert_eq!(
            polys[0].outer,
            vec![
                (16000, 11570),
                (15000, 11570),
                (15000, 11800),
                (14400, 12400),
                (13600, 12400),
                (13000, 11800),
                (13000, 11570),
                (11000, 11570),
                (11000, 11230),
                (13000, 11230),
                (13000, 11000),
                (13600, 10400),
                (14400, 10400),
                (15000, 11000),
                (15000, 11230),
                (16000, 11230),
                (16000, 11570)
            ]
        );
        assert!(polys[0].holes.is_empty());
        let mut one = Polygon45Set::new();
        one.insert_polygon_with_holes(&polys[0]);
        let t = one.trapezoids().unwrap();
        assert_eq!(t.len(), 5);
        assert_eq!(t[0], vec![(13000, 11570), (11000, 11570), (11000, 11230), (13000, 11230), (13000, 11570)]);
        assert_eq!(t[4], vec![(16000, 11570), (15000, 11570), (15000, 11230), (16000, 11230), (16000, 11570)]);
    }
}
