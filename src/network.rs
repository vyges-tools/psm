// SPDX-License-Identifier: Apache-2.0
//! The resistive network of one supply net: nodes on every layer, the shapes they sit on, and the
//! connections between them.
//!
//! [`Network::build`] is the construction sequence and nothing else; each stage is one function,
//! in the order it runs and named for what it does, so a per-stage dump lines up with it.
//!
//! Container order is behaviour here:
//! - per-layer maps are ordered by the layer's database id ([`LayerKey`]), never its name;
//! - node order is (layer number, x, y, kind, terminal id) — [`Network::compare`];
//! - stable sorts keep equal nodes in the order they were made, and the first of a run of nodes
//!   at one point is the one that survives, so the order nodes are MADE in matters too.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, HashMap, HashSet};

use vyges_drt::polygon90::Polygon90Set;

use crate::polygon45::{Polygon45Set, PolygonWithHoles, Pt};
use vyges_drt::rtree::{PackedRTree, StdLib};

use crate::input::{Dir, Geom, ITerm, NetInput, Rect, SBox, Tech, ViaShape};

/// The coarsest node pitch, in microns, and the pitch multiple that caps it per layer.
const MIN_NODE_PITCH_UM: f64 = 10.0;
const MIN_NODE_PITCH_MULTIPLIER: i32 = 10;

/// Called after each construction stage that the reference can dump, with that stage's name.
pub type DumpFn<'a> = Box<dyn FnMut(&str, &Network) + 'a>;

pub type NodeId = usize;
pub type ConnId = usize;

/// A per-layer map key: ordered by the layer's database id, carrying its index into the tech.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct LayerKey {
    pub id: u32,
    pub li: usize,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NodeKind {
    /// A plain node, or a terminal node (the centre of a pin shape): they order and print alike.
    Node,
    /// An instance terminal's base node; `id` is the terminal's.
    ITerm { iterm: usize, id: i32 },
    /// A block pin's node; `id` is the pin's.
    BPin { id: i32, disconnect: bool },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Node {
    pub pt: (i32, i32),
    pub li: usize,
    pub kind: NodeKind,
    /// The pin shape of a terminal, iterm or bpin node.
    pub shape: Option<Rect>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConnKind {
    Layer { length: i32, width: i32 },
    Via { cuts: i32 },
    Term,
    Bridge,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Conn {
    pub n0: Option<NodeId>,
    pub n1: Option<NodeId>,
    pub kind: ConnKind,
}

/// A piece of a layer's metal: a `RectShape` (`polygon` none), or a `PolygonShape`, a trapezoid
/// with 45 degree edges whose `rect` is only its bounding box.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Shape {
    pub rect: Rect,
    pub li: usize,
    pub id: usize,
    /// The trapezoid's points, corrected (clockwise, closed) as `odb::Polygon` holds them.
    pub polygon: Option<Vec<Pt>>,
}

/// Why a build stopped: an error the reference reports too, or a case this build refuses.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BuildError {
    Error(String),
    Refused(String),
}

/// The length and width of the metal joining two nodes of a shape.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ConnectionSize {
    pub length: i32,
    pub width: i32,
}

impl Shape {
    /// True when the point is in the shape or on its boundary (`boost::geometry::covered_by`
    /// for a polygon).
    pub fn contains(&self, p: (i32, i32)) -> bool {
        match &self.polygon {
            None => rect_holds(&self.rect, p),
            Some(pts) => covered_by(p, pts),
        }
    }

    /// `getConnectionSize`: a rect measures along the longer axis and across the shape; a
    /// polygon along the line between the points and across the shape square to it, through
    /// their middle.
    pub fn connection_size(&self, p0: (i32, i32), p1: (i32, i32)) -> ConnectionSize {
        let Some(pts) = &self.polygon else {
            let (len_x, len_y) = ((p1.0 - p0.0).abs(), (p1.1 - p0.1).abs());
            if len_x > len_y {
                return ConnectionSize { length: len_x, width: self.rect.dy() };
            }
            return ConnectionSize { length: len_y, width: self.rect.dx() };
        };
        let dx = f64::from(p1.0 - p0.0);
        let dy = f64::from(p1.1 - p0.1);
        let length = dx.hypot(dy);
        if length == 0.0 {
            return ConnectionSize { length: 0, width: self.rect.dx() };
        }
        let mx = f64::from(p0.0 + p1.0) / 2.0;
        let my = f64::from(p0.1 + p1.1) / 2.0;
        let (t_min, t_max) = clip_line(pts, mx, my, -dy / length, dx / length);
        ConnectionSize { length: lround(length), width: 1.max(lround(t_max - t_min)) }
    }

    /// `getFillerPoint`: where the filler node for `p` goes. A rect takes `p`; a polygon the
    /// middle of the line across it through `p` (square to the step), none when it misses.
    pub fn filler_point(&self, p: (i32, i32), along_x: bool) -> Option<(i32, i32)> {
        let Some(pts) = &self.polygon else { return Some(p) };
        let (dx, dy) = if along_x { (0.0, 1.0) } else { (1.0, 0.0) };
        let (t_min, t_max) = clip_line(pts, f64::from(p.0), f64::from(p.1), dx, dy);
        if t_min > t_max {
            return None;
        }
        let middle = lround((t_min + t_max) / 2.0);
        Some(if along_x { (p.0, p.1 + middle) } else { (p.0 + middle, p.1) })
    }
}

/// `std::lround` to an int, as the reference casts it.
fn lround(v: f64) -> i32 {
    v.round() as i32
}

/// `clipLine`: the range of t over which `p + t * dir` lies in the convex polygon (clockwise,
/// closed: the inside of an edge is to its right), empty (first above second) when it misses.
fn clip_line(pts: &[Pt], px: f64, py: f64, dir_x: f64, dir_y: f64) -> (f64, f64) {
    let mut t_min = f64::NEG_INFINITY;
    let mut t_max = f64::INFINITY;
    for i in 0..pts.len().saturating_sub(1) {
        let edge_x = f64::from(pts[i + 1].0 - pts[i].0);
        let edge_y = f64::from(pts[i + 1].1 - pts[i].1);
        let offset = edge_y * (px - f64::from(pts[i].0)) - edge_x * (py - f64::from(pts[i].1));
        let rate = edge_y * dir_x - edge_x * dir_y;
        if rate == 0.0 {
            if offset < 0.0 {
                return (1.0, 0.0);
            }
            continue;
        }
        let t = -offset / rate;
        if rate > 0.0 {
            t_min = t_min.max(t);
        } else {
            t_max = t_max.min(t);
        }
    }
    (t_min, t_max)
}

/// The closed rectangle holds the point (`Rect::intersects(Point)`).
fn rect_holds(r: &Rect, p: (i32, i32)) -> bool {
    p.0 >= r.xl && p.0 <= r.xh && p.1 >= r.yl && p.1 <= r.yh
}

/// `boost::geometry::covered_by(point, ring)` on integer points: on the boundary, or inside by
/// the crossing count. Exact; the ring's orientation does not change the answer.
fn covered_by(p: (i32, i32), pts: &[Pt]) -> bool {
    let (px, py) = (i64::from(p.0), i64::from(p.1));
    let mut inside = false;
    for w in pts.windows(2) {
        let (ax, ay) = (i64::from(w[0].0), i64::from(w[0].1));
        let (bx, by) = (i64::from(w[1].0), i64::from(w[1].1));
        let cross = (bx - ax) * (py - ay) - (by - ay) * (px - ax);
        if cross == 0 && px >= ax.min(bx) && px <= ax.max(bx) && py >= ay.min(by) && py <= ay.max(by) {
            return true;
        }
        if (ay > py) != (by > py) {
            // the edge crosses the horizontal through p: on which side of p
            let lhs = (px - ax) * (by - ay);
            let rhs = (bx - ax) * (py - ay);
            if (by > ay && lhs < rhs) || (by < ay && lhs > rhs) {
                inside = !inside;
            }
        }
    }
    inside
}

/// `boost::geometry::correct` on odb's ring (clockwise, closed), as `odb::Polygon` stores its
/// points: closed on its first point, then reversed when it runs counterclockwise.
fn correct_ring(mut pts: Vec<Pt>) -> Vec<Pt> {
    if pts.len() <= 2 {
        return pts;
    }
    if pts.first() != pts.last() {
        let first = pts[0];
        pts.push(first);
    }
    let mut twice_ccw: i128 = 0;
    for w in pts.windows(2) {
        twice_ccw += i128::from(w[0].0) * i128::from(w[1].1) - i128::from(w[1].0) * i128::from(w[0].1);
    }
    if twice_ccw > 0 {
        pts.reverse();
    }
    pts
}

/// `odb::Polygon::getEnclosingRect`.
fn enclosing_rect(pts: &[Pt]) -> Rect {
    let mut r = Rect { xl: i32::MAX, yl: i32::MAX, xh: i32::MIN, yh: i32::MIN };
    for p in pts {
        r.xl = r.xl.min(p.0);
        r.yl = r.yl.min(p.1);
        r.xh = r.xh.max(p.0);
        r.yh = r.yh.max(p.1);
    }
    r
}

/// `odb::Polygon::isRect`: at least four points, and every edge (the closing one included) on a
/// side of the enclosing rect.
fn is_rect(pts: &[Pt]) -> bool {
    if pts.len() < 4 {
        return false;
    }
    let r = enclosing_rect(pts);
    for i in 0..pts.len() {
        let (p0, p1) = (pts[i], pts[(i + 1) % pts.len()]);
        let horizontal = p0.1 == p1.1 && (p0.1 == r.yl || p0.1 == r.yh);
        let vertical = p0.0 == p1.0 && (p0.0 == r.xl || p0.0 == r.xh);
        if !horizontal && !vertical {
            return false;
        }
    }
    true
}

/// `getSharedEdgeCenters`: the middle of every vertical edge overlap of two closed point lists,
/// in the first list's then the second's edge order.
fn shared_edge_centers(p0: &[Pt], p1: &[Pt]) -> Vec<Pt> {
    let mut out = Vec::new();
    for i in 0..p0.len().saturating_sub(1) {
        let (s0, e0) = (p0[i], p0[i + 1]);
        for j in 0..p1.len().saturating_sub(1) {
            let (s1, e1) = (p1[j], p1[j + 1]);
            if s0.0 != e0.0 || s1.0 != e1.0 || s0.0 != s1.0 {
                continue;
            }
            let low = s0.1.min(e0.1).max(s1.1.min(e1.1));
            let high = s0.1.max(e0.1).min(s1.1.max(e1.1));
            if low < high {
                out.push((s0.0, (low + high) / 2));
            }
        }
    }
    out
}

/// A layer's rectilinear shapes as the reference's `BoostPolygon90Set` holds them: the set, and
/// every rectangle put in it, in order (what a 45-degree set reads when it takes the layer).
#[derive(Debug, Clone, Default)]
struct Shapes90 {
    set: Polygon90Set,
    rects: Vec<Rect>,
}

impl Shapes90 {
    fn insert_rect(&mut self, r: Rect) {
        self.set.insert_rect(r);
        self.rects.push(r);
    }
}

/// A trapezoid of a 45-degree set: `Err` (a refusal) for boost's precision error.
fn refused(e: String) -> BuildError {
    BuildError::Refused(format!("a 45-degree layer boost refuses ({e})"))
}

/// What the build reports besides the network.
#[derive(Debug, Default)]
pub struct BuildLog {
    /// No node at all: the net is empty, and nothing after generation ran.
    pub empty: bool,
}

pub struct Network<'a> {
    pub tech: &'a Tech,
    pub floorplanning: bool,
    min_node_pitch: HashMap<usize, i32>,
    pub arena: Vec<Node>,
    pub nodes: BTreeMap<LayerKey, Vec<NodeId>>,
    pub iterm_nodes: Vec<NodeId>,
    pub bpin_nodes: Vec<NodeId>,
    pub shapes: BTreeMap<LayerKey, Vec<Shape>>,
    pub conns: Vec<Conn>,
    pub connections: Vec<ConnId>,
    dump: Option<DumpFn<'a>>,
}

type ConnMap = HashMap<NodeId, Vec<ConnId>>;

fn intersects(a: &Rect, b: &Rect) -> bool {
    b.xh >= a.xl && b.xl <= a.xh && b.yh >= a.yl && b.yl <= a.yh
}

/// Strictly inside.
fn overlaps(r: &Rect, p: (i32, i32)) -> bool {
    p.0 > r.xl && p.0 < r.xh && p.1 > r.yl && p.1 < r.yh
}

fn center(r: &Rect) -> (i32, i32) {
    ((r.xl + r.xh) / 2, (r.yl + r.yh) / 2)
}

fn max_dxdy(r: &Rect) -> i32 {
    r.dx().max(r.dy())
}

/// The standard library the reference is built against: the Bazel build of the pin (4155ac3,
/// clang with libc++) — the CMake build before it was GCC with libstdc++.
pub const REF_STDLIB: StdLib = StdLib::LibCxx;

/// An R-tree packed as the reference packs one: its standard library's `nth_element` permutes
/// the packing's slices and its heap orders a nearest query's pending branches, so two nodes at
/// the same distance are told apart exactly as there ([`REF_STDLIB`]).
pub fn ref_tree<T>(values: Vec<(Rect, T)>) -> PackedRTree<T> {
    PackedRTree::new_in(values, REF_STDLIB)
}

fn point_rect(p: (i32, i32)) -> Rect {
    Rect { xl: p.0, yl: p.1, xh: p.0, yh: p.1 }
}

fn swap_xy(r: &Rect) -> Rect {
    Rect { xl: r.yl, yl: r.xl, xh: r.yh, yh: r.xh }
}

/// `get_rectangles` with the slicing a layer gets: VERTICAL slices on a horizontal (or
/// undirected) layer, HORIZONTAL on a vertical one. The set is horizontal, so vertical slicing is
/// the horizontal slicing of the set with x and y swapped, each rectangle swapped back.
fn layer_rectangles(set: &mut Polygon90Set, dir: Dir) -> Vec<Rect> {
    match dir {
        Dir::Vertical => set.rectangles(),
        Dir::None | Dir::Horizontal => {
            let mut swapped = Polygon90Set::new();
            for r in set.rectangles() {
                swapped.insert_rect(swap_xy(&r));
            }
            swapped.rectangles().iter().map(swap_xy).collect()
        }
    }
}

impl<'a> Network<'a> {
    /// The construction: `None` when the net has no nodes at all (the caller reports it empty).
    pub fn build(tech: &'a Tech, net: &NetInput, floorplanning: bool, dump: Option<DumpFn<'a>>) -> Result<(Network<'a>, BuildLog), BuildError> {
        if !net.supply {
            return Err(BuildError::Error(format!("{} is not a supply net.", net.name)));
        }
        let mut n = Network {
            tech,
            floorplanning,
            min_node_pitch: HashMap::new(),
            arena: Vec::new(),
            nodes: BTreeMap::new(),
            iterm_nodes: Vec::new(),
            bpin_nodes: Vec::new(),
            shapes: BTreeMap::new(),
            conns: Vec::new(),
            connections: Vec::new(),
            dump,
        };
        n.init_minimum_node_pitch();
        let log = n.construct(net)?;
        Ok((n, log))
    }

    fn construct(&mut self, net: &NetInput) -> Result<BuildLog, BuildError> {
        self.generate_routing_layer_shapes_and_nodes(net)?;
        self.generate_cut_layer_nodes(net);
        if !self.has_nodes() {
            return Ok(BuildLog { empty: true });
        }
        self.generate_top_layer_filler_nodes();
        self.sort_nodes();
        self.dump_nodes("intital_nodes");
        self.cleanup_nodes();
        self.cleanup_connections();
        self.dump_nodes("after_node_cleanup");
        self.connect_layer_nodes();
        self.sort_connections();
        self.dump_nodes("after_layer_connections");
        self.cleanup_connections();
        self.sort_connections();
        self.dump_nodes("after_connection_cleanup");
        Ok(BuildLog::default())
    }

    // ---- small helpers ------------------------------------------------------------------------

    fn key(&self, li: usize) -> LayerKey {
        LayerKey { id: self.tech.layers[li].id, li }
    }

    fn li(&self, number: i32) -> usize {
        self.tech.by_number(number).expect("a layer the tech has")
    }

    /// The pitch map read the way the reference reads it: a layer it has no entry for reads 0.
    fn pitch(&self, li: usize) -> i32 {
        self.min_node_pitch.get(&li).copied().unwrap_or(0)
    }

    fn new_node(&mut self, pt: (i32, i32), li: usize, kind: NodeKind, shape: Option<Rect>) -> NodeId {
        self.arena.push(Node { pt, li, kind, shape });
        self.arena.len() - 1
    }

    fn terminal(&mut self, shape: Rect, li: usize) -> NodeId {
        self.new_node(center(&shape), li, NodeKind::Node, Some(shape))
    }

    fn push_node(&mut self, id: NodeId) {
        let k = self.key(self.arena[id].li);
        self.nodes.entry(k).or_default().push(id);
    }

    /// Node order: layer number, x, y, kind (node < iterm < bpin), then the terminal's id.
    pub fn compare(&self, a: NodeId, b: NodeId) -> Ordering {
        let (na, nb) = (&self.arena[a], &self.arena[b]);
        let rank = |k: &NodeKind| match k {
            NodeKind::Node => (0, 0),
            NodeKind::ITerm { id, .. } => (2, *id),
            NodeKind::BPin { id, .. } => (3, *id),
        };
        self.tech.layers[na.li]
            .number
            .cmp(&self.tech.layers[nb.li].number)
            .then(na.pt.0.cmp(&nb.pt.0))
            .then(na.pt.1.cmp(&nb.pt.1))
            .then(rank(&na.kind).cmp(&rank(&nb.kind)))
    }

    /// `compare` with an absent node: anything is greater than nothing.
    fn compare_opt(&self, a: NodeId, b: Option<NodeId>) -> Ordering {
        b.map_or(Ordering::Greater, |b| self.compare(a, b))
    }

    fn new_conn(&mut self, n0: NodeId, n1: NodeId, kind: ConnKind) -> ConnId {
        let mut c = Conn { n0: Some(n0), n1: Some(n1), kind };
        self.ensure_node_order(&mut c);
        self.conns.push(c);
        self.conns.len() - 1
    }

    /// The lower node first.
    fn ensure_node_order(&self, c: &mut Conn) {
        if let Some(a) = c.n0 {
            if self.compare_opt(a, c.n1) == Ordering::Greater {
                std::mem::swap(&mut c.n0, &mut c.n1);
            }
        }
    }

    /// Two connections in connection order.
    pub fn conn_order(&self, a: ConnId, b: ConnId) -> Ordering {
        let (x, y) = (&self.conns[a], &self.conns[b]);
        if self.conn_less(x, y) {
            Ordering::Less
        } else if self.conn_less(y, x) {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }

    /// Connection order: node0 then node1, an absent node first.
    fn conn_less(&self, a: &Conn, b: &Conn) -> bool {
        match (a.n0, b.n0) {
            (None, Some(_)) => return true,
            (Some(_), None) => return false,
            (Some(x), Some(y)) => match self.compare(x, y) {
                Ordering::Equal => {}
                o => return o == Ordering::Less,
            },
            (None, None) => {}
        }
        match (a.n1, b.n1) {
            (None, Some(_)) => true,
            (Some(x), Some(y)) => self.compare(x, y) == Ordering::Less,
            _ => false,
        }
    }

    /// Each routing layer's minimum node pitch: the coarse pitch in microns, capped at a multiple
    /// of the layer's own pitch (at least 1).
    fn init_minimum_node_pitch(&mut self) {
        let min_pitch = (f64::from(self.tech.dbus) * MIN_NODE_PITCH_UM) as i32;
        for (li, l) in self.tech.layers.iter().enumerate() {
            if l.routing_level != 0 {
                self.min_node_pitch.insert(li, min_pitch.min(MIN_NODE_PITCH_MULTIPLIER * l.pitch.max(1)));
            }
        }
    }

    fn has_nodes(&self) -> bool {
        self.nodes.values().any(|v| !v.is_empty()) || !self.iterm_nodes.is_empty() || !self.bpin_nodes.is_empty()
    }

    // ---- shapes ----------------------------------------------------------------------------------

    /// A box's metal on routing layers, per layer: a via's enclosures (its cut is skipped), or
    /// the box itself.
    fn polygons_from_box(&self, g: &Geom) -> BTreeMap<LayerKey, Vec<Rect>> {
        let mut out: BTreeMap<LayerKey, Vec<Rect>> = BTreeMap::new();
        if g.is_via {
            for s in &g.via_shapes {
                let li = self.li(s.layer);
                if self.tech.layers[li].routing_level != 0 {
                    out.entry(self.key(li)).or_default().push(s.rect);
                }
            }
        } else if let Some(l) = g.layer {
            let li = self.li(l);
            if self.tech.layers[li].routing_level != 0 {
                out.entry(self.key(li)).or_default().push(g.rect);
            }
        }
        out
    }

    /// A special wire's metal: an octilinear box goes in whole to its layer's 45-degree set (its
    /// box would be all of its bounding box), every other box to the layer's rectilinear set.
    fn polygons_from_swire(&self, wire: &[SBox], into: &mut BTreeMap<LayerKey, Shapes90>, into_45: &mut BTreeMap<LayerKey, Polygon45Set>) {
        for b in wire {
            let g = match b {
                SBox::Oct { layer, points, .. } => {
                    let li = self.li(*layer);
                    if self.tech.layers[li].routing_level != 0 {
                        into_45.entry(self.key(li)).or_default().insert_polygon(points);
                    }
                    continue;
                }
                SBox::Wire { layer, rect } => Geom { is_via: false, layer: Some(*layer), rect: *rect, via_shapes: vec![] },
                SBox::Via { rect, shapes, .. } => Geom { is_via: true, layer: None, rect: *rect, via_shapes: shapes.clone() },
            };
            for (k, rs) in self.polygons_from_box(&g) {
                let set = into.entry(k).or_default();
                for r in rs {
                    set.insert_rect(r);
                }
            }
        }
    }

    /// A terminal's pins in order: (its boxes, its polygons). Without pins read, all its boxes
    /// are one pin.
    fn mpin_runs(it: &ITerm) -> Vec<(&[Geom], &[crate::input::PinPolygon])> {
        if it.mpins.is_empty() {
            return vec![(&it.geoms[..], &[])];
        }
        let mut out = Vec::new();
        let mut at = 0;
        for m in &it.mpins {
            out.push((&it.geoms[at..at + m.boxes], &m.polygons[..]));
            at += m.boxes;
        }
        out
    }

    /// Instance terminals: a base node per terminal at its pins' average point, on the side its
    /// pins are on, and a terminal node per pin shape linked to it. Floorplanning skips any
    /// instance not FIXED; if none was skipped, floorplanning is turned off.
    ///
    /// Per pin, a polygon with 45 degree edges goes to its layer's 45-degree set itself (odb's
    /// boxes for it reach outside of it, so they are skipped), with a terminal node at the
    /// centre of each of its trapezoids; then the pin's other boxes as before.
    fn polygons_from_iterms(&mut self, net: &NetInput, into: &mut BTreeMap<LayerKey, Shapes90>, into_45: &mut BTreeMap<LayerKey, Polygon45Set>) -> Result<(), BuildError> {
        let front = self.tech.front_base.map(|n| self.li(n));
        let back = self.tech.back_base.map(|n| self.li(n));
        let mut floorplan_asserted = false;
        for (ti, it) in net.iterms.iter().enumerate() {
            if !it.placed {
                continue;
            }
            if self.floorplanning && !it.fixed {
                floorplan_asserted = true;
                continue;
            }
            if it.bbox_inverted {
                continue;
            }
            let (mut has_front, mut has_back) = (false, false);
            for g in &it.geoms {
                if let Some(l) = g.layer {
                    if self.tech.layers[self.li(l)].backside {
                        has_back = true;
                    } else {
                        has_front = true;
                    }
                }
            }
            let mut base_layer = front;
            if !has_front && has_back && back.is_some() {
                base_layer = back;
            }
            let primary_is_back = base_layer == back;
            let base_li = base_layer.expect("a front-side routing layer");
            let base = self.new_node(it.avg, base_li, NodeKind::ITerm { iterm: ti, id: it.id }, Some(point_rect(it.avg)));
            let mut has_routing_term = false;
            for (boxes, polygons) in Self::mpin_runs(it) {
            let mut polygon_boxes: BTreeSet<usize> = BTreeSet::new();
            for poly in polygons {
                let li = self.li(poly.layer);
                let rectilinear = !poly.points.windows(2).any(|w| w[0].0 != w[1].0 && w[0].1 != w[1].1);
                if self.tech.layers[li].routing_level == 0 || rectilinear {
                    continue;
                }
                polygon_boxes.extend(poly.boxes.iter().copied());
                let mut pin_set = Polygon45Set::new();
                pin_set.insert_polygon(&poly.points);
                into_45.entry(self.key(li)).or_default().insert_set(&pin_set);
                // a trapezoid of a vertical slicing holds the centre of its box
                for piece in pin_set.trapezoids().map_err(refused)? {
                    has_routing_term = true;
                    let t = self.terminal(enclosing_rect(&piece), li);
                    self.link_terminal(base, t, primary_is_back, it.bridge);
                    self.push_node(t);
                }
            }
            for (gi, g) in boxes.iter().enumerate() {
                if polygon_boxes.contains(&gi) {
                    continue;
                }
                let pin_shapes = self.polygons_from_box(g);
                if pin_shapes.is_empty() {
                    continue;
                }
                for (k, rs) in &pin_shapes {
                    let set = into.entry(*k).or_default();
                    for r in rs {
                        set.insert_rect(*r);
                    }
                }
                if g.is_via {
                    for (k, rs) in pin_shapes {
                        let mut set = Polygon90Set::new();
                        rs.into_iter().for_each(|r| set.insert_rect(r));
                        for r in set.rectangles() {
                            has_routing_term = true;
                            let t = self.terminal(r, k.li);
                            self.link_terminal(base, t, primary_is_back, it.bridge);
                            self.push_node(t);
                        }
                    }
                } else {
                    let li = self.li(g.layer.expect("a pin box's layer"));
                    has_routing_term = true;
                    let t = self.terminal(g.rect, li);
                    self.link_terminal(base, t, primary_is_back, it.bridge);
                    self.push_node(t);
                }
            }
            }
            if has_routing_term {
                self.iterm_nodes.push(base);
            }
        }
        if self.floorplanning && !floorplan_asserted {
            self.floorplanning = false;
        }
        Ok(())
    }

    /// A terminal on the base node's side joins it; one across the boundary joins only through a
    /// bridge cell, and is otherwise left for the layer graph to reach (or report open).
    fn link_terminal(&mut self, base: NodeId, term: NodeId, primary_is_back: bool, bridge: bool) {
        let term_is_back = self.tech.layers[self.arena[term].li].backside;
        let c = if term_is_back == primary_is_back {
            self.new_conn(base, term, ConnKind::Term)
        } else if bridge {
            self.new_conn(base, term, ConnKind::Bridge)
        } else {
            return;
        };
        self.connections.push(c);
    }

    /// Block pins: every non-via box gets a terminal node and a pin node, joined.
    fn polygons_from_bterms(&mut self, net: &NetInput, into: &mut BTreeMap<LayerKey, Shapes90>) {
        for bt in &net.bterms {
            for pin in &bt.pins {
                for b in &pin.boxes {
                    for (k, rs) in self.polygons_from_box(&b.geom) {
                        let set = into.entry(k).or_default();
                        for r in rs {
                            set.insert_rect(r);
                        }
                    }
                    if b.geom.is_via {
                        continue;
                    }
                    let li = self.li(b.geom.layer.expect("a pin box's layer"));
                    let rect = b.geom.rect;
                    let term = self.terminal(rect, li);
                    let disconnect = pin.disconnect || b.disconnect;
                    let pin_node = self.new_node(center(&rect), li, NodeKind::BPin { id: pin.id, disconnect }, Some(rect));
                    let c = self.new_conn(term, pin_node, ConnKind::Term);
                    self.connections.push(c);
                    self.push_node(term);
                    self.bpin_nodes.push(pin_node);
                }
            }
        }
    }

    /// One merged polygon of a layer, sliced into rectangles: each rectangle is a shape, with a
    /// node at its centre and one at the centre of its overlap with every LATER rectangle.
    fn process_polygon_to_rectangles(&mut self, li: usize, mut poly: Polygon90Set, new_shapes: &mut Vec<Shape>, new_nodes: &mut Vec<NodeId>) {
        let rects = layer_rectangles(&mut poly, self.tech.layers[li].dir);
        for (i, rect) in rects.iter().enumerate() {
            let mut pts: BTreeSet<(i32, i32)> = BTreeSet::new();
            for other in &rects[i + 1..] {
                if intersects(other, rect) {
                    let x = Rect { xl: rect.xl.max(other.xl), yl: rect.yl.max(other.yl), xh: rect.xh.min(other.xh), yh: rect.yh.min(other.yh) };
                    pts.insert(center(&x));
                }
            }
            pts.insert(center(rect));
            for pt in pts {
                new_nodes.push(self.new_node(pt, li, NodeKind::Node, None));
            }
            // (The reference also collects the terminals each rectangle overlaps; nothing reads them.)
            new_shapes.push(Shape { rect: *rect, li, id: 0, polygon: None });
        }
    }

    /// One merged polygon of a 45-degree layer, sliced into trapezoids: a piece that is a
    /// rectangle is a rect shape, the others keep their polygon. Each piece gets a node at the
    /// centre of its box and one at the middle of each vertical edge it shares with a LATER
    /// piece whose box it touches.
    fn process_polygon_to_trapezoids(&mut self, li: usize, polygon: &PolygonWithHoles, new_shapes: &mut Vec<Shape>, new_nodes: &mut Vec<NodeId>) -> Result<(), BuildError> {
        let mut set = Polygon45Set::new();
        set.insert_polygon_with_holes(polygon);
        let mut shapes = Vec::new();
        let mut shape_points = Vec::new();
        for trapezoid in set.trapezoids().map_err(refused)? {
            let mut pts = trapezoid;
            if !pts.is_empty() && pts.first() != pts.last() {
                let first = pts[0];
                pts.push(first);
            }
            let piece = correct_ring(pts);
            let rect = enclosing_rect(&piece);
            let poly = if is_rect(&piece) { None } else { Some(piece.clone()) };
            shapes.push(Shape { rect, li, id: 0, polygon: poly });
            shape_points.push(piece);
        }
        for i in 0..shapes.len() {
            let mut pts: BTreeSet<(i32, i32)> = BTreeSet::new();
            pts.insert(center(&shapes[i].rect));
            for j in i + 1..shapes.len() {
                if !intersects(&shapes[i].rect, &shapes[j].rect) {
                    continue;
                }
                pts.extend(shared_edge_centers(&shape_points[i], &shape_points[j]));
            }
            for pt in pts {
                new_nodes.push(self.new_node(pt, li, NodeKind::Node, None));
            }
            // (The reference also collects the terminals each piece holds; nothing reads them.)
        }
        new_shapes.extend(shapes);
        Ok(())
    }

    fn generate_routing_layer_shapes_and_nodes(&mut self, net: &NetInput) -> Result<(), BuildError> {
        let mut shapes_by_layer: BTreeMap<LayerKey, Shapes90> = BTreeMap::new();
        let mut shapes_45_by_layer: BTreeMap<LayerKey, Polygon45Set> = BTreeMap::new();
        for wire in &net.swires {
            self.polygons_from_swire(wire, &mut shapes_by_layer, &mut shapes_45_by_layer);
        }
        self.polygons_from_iterms(net, &mut shapes_by_layer, &mut shapes_45_by_layer)?;
        self.polygons_from_bterms(net, &mut shapes_by_layer);
        // A layer with shapes with 45 degree edges is merged with them and sliced into
        // trapezoids rather than rectangles.
        let mut all_poly_45_shapes = Vec::new();
        for (k, mut shapes_45) in shapes_45_by_layer {
            if let Some(s90) = shapes_by_layer.remove(&k) {
                let rects: Vec<(i32, i32, i32, i32)> = s90.rects.iter().map(|r| (r.xl, r.yl, r.xh, r.yh)).collect();
                shapes_45.insert_rects_90(&rects);
            }
            for p in shapes_45.polygons_with_holes().map_err(refused)? {
                all_poly_45_shapes.push((k.li, p));
            }
        }
        let mut all_poly_shapes = Vec::new();
        for (k, mut s90) in shapes_by_layer {
            for p in s90.set.polygons() {
                all_poly_shapes.push((k.li, p));
            }
        }
        let (mut poly_shapes, mut poly_nodes) = (Vec::new(), Vec::new());
        for (li, p) in all_poly_shapes {
            self.process_polygon_to_rectangles(li, p, &mut poly_shapes, &mut poly_nodes);
        }
        for (li, p) in &all_poly_45_shapes {
            self.process_polygon_to_trapezoids(*li, p, &mut poly_shapes, &mut poly_nodes)?;
        }
        for id in poly_nodes {
            self.push_node(id);
        }
        for s in poly_shapes {
            let k = self.key(s.li);
            self.shapes.entry(k).or_default().push(s);
        }
        self.sort_shapes();
        let mut id = 0;
        for shapes in self.shapes.values_mut() {
            for s in shapes {
                s.id = id;
                id += 1;
            }
        }
        Ok(())
    }

    // ---- vias ----------------------------------------------------------------------------------

    /// How many cuts one cut shape counts for: the first valid cut class its size matches (either
    /// way round; a class without a length matches on width alone), else 1.
    fn effective_cuts(&self, s: &ViaShape) -> i32 {
        let l = &self.tech.layers[self.li(s.layer)];
        for c in &l.cut_classes {
            if !c.cuts_valid {
                continue;
            }
            let (dx, dy) = (s.rect.dx(), s.rect.dy());
            if dx == c.width && (!c.length_valid || dy == c.length) {
                return c.num_cuts;
            }
            if dy == c.width && (!c.length_valid || dx == c.length) {
                return c.num_cuts;
            }
        }
        1
    }

    /// A via: one pair of nodes (top, bottom) at the via's origin carrying every cut, when the via
    /// is smaller than the finer of its two layers' node pitches (or always, floorplanning); else
    /// a pair per cut at the cut's centre.
    fn generate_cut_nodes_for_sbox(&mut self, b: &SBox, single_via: bool, new_nodes: &mut Vec<NodeId>, new_conns: &mut Vec<ConnId>) {
        let SBox::Via { rect, xy, top, bottom, shapes } = b else { return };
        let cuts: Vec<ViaShape> = shapes.iter().filter(|s| self.tech.layers[self.li(s.layer)].routing_level == 0).copied().collect();
        let (top, bottom) = (self.li(*top), self.li(*bottom));
        let min_pitch = self.pitch(bottom).min(self.pitch(top));
        let use_single_via = self.floorplanning || max_dxdy(rect) < min_pitch;
        if single_via || use_single_via {
            let b = self.new_node(*xy, bottom, NodeKind::Node, None);
            let t = self.new_node(*xy, top, NodeKind::Node, None);
            let n: i32 = cuts.iter().map(|s| self.effective_cuts(s)).sum();
            new_conns.push(self.new_conn(b, t, ConnKind::Via { cuts: n }));
            new_nodes.push(t);
            new_nodes.push(b);
        } else {
            for s in &cuts {
                let c = center(&s.rect);
                let b = self.new_node(c, bottom, NodeKind::Node, None);
                let t = self.new_node(c, top, NodeKind::Node, None);
                let n = self.effective_cuts(s);
                new_conns.push(self.new_conn(b, t, ConnKind::Via { cuts: n }));
                new_nodes.push(t);
                new_nodes.push(b);
            }
        }
    }

    fn generate_cut_layer_nodes(&mut self, net: &NetInput) {
        let boxes: Vec<&SBox> = net.swires.iter().flatten().filter(|b| matches!(b, SBox::Via { .. })).collect();
        let (mut loop_nodes, mut loop_conns) = (Vec::new(), Vec::new());
        for b in boxes {
            self.generate_cut_nodes_for_sbox(b, false, &mut loop_nodes, &mut loop_conns);
        }
        let mut via_nodes: BTreeMap<LayerKey, Vec<NodeId>> = BTreeMap::new();
        for id in loop_nodes {
            via_nodes.entry(self.key(self.arena[id].li)).or_default().push(id);
        }
        self.connections.extend(loop_conns);
        for (k, ids) in via_nodes {
            self.nodes.entry(k).or_default().extend(ids);
        }
    }

    // ---- filler, sorting -------------------------------------------------------------------------

    /// The top layer: the first layer that has nodes if it is a back-side layer, else the last.
    pub fn top_layer(&self) -> Option<LayerKey> {
        let first = *self.nodes.keys().next()?;
        if self.tech.layers[first.li].backside {
            return Some(first);
        }
        self.nodes.keys().next_back().copied()
    }

    /// Top-layer shapes get a node every pitch along their length (only when solving, not when
    /// floorplanning): where a voltage source may land.
    fn generate_top_layer_filler_nodes(&mut self) {
        if self.floorplanning {
            return;
        }
        let Some(top) = self.top_layer() else { return };
        let max_distance = self.pitch(top.li);
        let shapes = self.shapes.entry(top).or_default().clone();
        for s in shapes {
            for id in self.create_filler_nodes(&s, max_distance) {
                self.push_node(id);
            }
        }
    }

    fn create_filler_nodes(&mut self, s: &Shape, max_distance: i32) -> Vec<NodeId> {
        let r = s.rect;
        let radius = max_distance / 2;
        let (delta, mut start) = if r.dx() > r.dy() {
            ((max_distance, 0), (r.xl + radius, (r.yl + r.yh) / 2))
        } else {
            ((0, max_distance), ((r.xl + r.xh) / 2, r.yl + radius))
        };
        let mut out = Vec::new();
        while overlaps(&r, start) {
            if let Some(pt) = s.filler_point(start, delta.0 != 0) {
                out.push(self.new_node(pt, s.li, NodeKind::Node, None));
            }
            start = (start.0 + delta.0, start.1 + delta.1);
        }
        out
    }

    fn sort_shapes(&mut self) {
        for shapes in self.shapes.values_mut() {
            shapes.sort_by_key(|s| (s.rect.xl, s.rect.yl, s.rect.xh, s.rect.yh));
        }
    }

    fn sort_nodes(&mut self) {
        let mut nodes = std::mem::take(&mut self.nodes);
        for v in nodes.values_mut() {
            v.sort_by(|a, b| self.compare(*a, *b));
        }
        self.nodes = nodes;
    }

    fn sort_connections(&mut self) {
        let mut cs = std::mem::take(&mut self.connections);
        cs.sort_by(|a, b| {
            if self.conn_less(&self.conns[*a], &self.conns[*b]) {
                Ordering::Less
            } else if self.conn_less(&self.conns[*b], &self.conns[*a]) {
                Ordering::Greater
            } else {
                Ordering::Equal
            }
        });
        self.connections = cs;
    }

    // ---- node cleanup ----------------------------------------------------------------------------

    fn connection_map(&self) -> ConnMap {
        let mut m: ConnMap = HashMap::new();
        for &c in &self.connections {
            let conn = &self.conns[c];
            m.entry(conn.n0.unwrap_or(usize::MAX)).or_default().push(c);
            m.entry(conn.n1.unwrap_or(usize::MAX)).or_default().push(c);
        }
        m
    }

    /// Hand every connection of `remove` to `keep`.
    fn copy(&mut self, keep: NodeId, remove: NodeId, map: &mut ConnMap) {
        let moved = std::mem::take(map.entry(remove).or_default());
        for &c in &moved {
            let conn = &mut self.conns[c];
            if conn.n0 == Some(remove) {
                conn.n0 = Some(keep);
            }
            if conn.n1 == Some(remove) {
                conn.n1 = Some(keep);
            }
        }
        map.entry(keep).or_default().extend(moved);
    }

    /// Drop `removes` from a layer's nodes (order kept), detaching their connections.
    fn remove_nodes(&mut self, removes: &HashSet<NodeId>, k: LayerKey, map: &ConnMap) {
        if removes.is_empty() {
            return;
        }
        for &n in removes {
            if let Some(cs) = map.get(&n) {
                for &c in cs {
                    let conn = &mut self.conns[c];
                    if conn.n0 == Some(n) {
                        conn.n0 = None;
                    }
                    if conn.n1 == Some(n) {
                        conn.n1 = None;
                    }
                }
            }
        }
        if let Some(v) = self.nodes.get_mut(&k) {
            v.retain(|n| !removes.contains(n));
        }
    }

    /// Nodes at the same point on a layer (adjacent once sorted): the first one takes the rest.
    fn cleanup_overlapping_nodes(&mut self, map: &mut ConnMap) {
        let keys: Vec<LayerKey> = self.nodes.keys().copied().collect();
        for k in keys {
            let v = self.nodes[&k].clone();
            if v.is_empty() {
                continue;
            }
            let mut removes = HashSet::new();
            let mut prev = v[0];
            for &n in &v[1..] {
                if self.arena[n].pt == self.arena[prev].pt {
                    self.copy(prev, n, map);
                    removes.insert(n);
                } else {
                    prev = n;
                }
            }
            self.remove_nodes(&removes, k, map);
        }
    }

    /// Nodes on more than one shape of their layer: never merged away.
    fn shared_shape_nodes(&self) -> HashSet<NodeId> {
        let mut shared = HashSet::new();
        for (k, nodes) in &self.nodes {
            let shapes = self.shapes.get(k).map(Vec::as_slice).unwrap_or(&[]);
            let tree = ref_tree(shapes.iter().enumerate().map(|(i, s)| (s.rect, i)).collect());
            for &n in nodes {
                // the box of a shape with 45 degree edges can hold a node only in the shape next to it
                let pt = self.arena[n].pt;
                let count = tree.query(&point_rect(pt)).into_iter().filter(|e| shapes[e.1].contains(pt)).count();
                if count > 1 {
                    shared.insert(n);
                }
            }
        }
        shared
    }

    /// The nodes of a layer touching `r`, in node order (one per distinct node).
    fn nodes_on(&self, tree: &PackedRTree<NodeId>, r: &Rect) -> Vec<NodeId> {
        let mut v: Vec<NodeId> = tree.query(r).into_iter().map(|e| e.1).collect();
        v.sort_by(|a, b| self.compare(*a, *b));
        v.dedup_by(|a, b| self.compare(*a, *b) == Ordering::Equal);
        v
    }

    /// `getNodes`: the nodes of a layer in a shape (its box can hold nodes of the shapes next to
    /// it), in node order.
    fn shape_nodes(&self, tree: &PackedRTree<NodeId>, s: &Shape) -> Vec<NodeId> {
        let mut v = self.nodes_on(tree, &s.rect);
        v.retain(|&n| s.contains(self.arena[n].pt));
        v
    }

    fn node_tree(&self, k: LayerKey) -> PackedRTree<NodeId> {
        let ids = self.nodes.get(&k).map(Vec::as_slice).unwrap_or(&[]);
        ref_tree(ids.iter().map(|&n| (point_rect(self.arena[n].pt), n)).collect())
    }

    /// Per shape, nodes closer than half the node pitch merge into one: shared nodes first, then
    /// those on the shape's centre lines, then the rest, each taking every unused node of the
    /// shape within the radius. A node merged away by one shape can still take others on the
    /// next; it is removed all the same.
    fn merge_nodes(&mut self, map: &mut ConnMap) {
        let shared = self.shared_shape_nodes();
        let mut removes: HashSet<NodeId> = HashSet::new();
        let keys: Vec<LayerKey> = self.shapes.keys().copied().collect();
        for k in keys {
            let tree = self.node_tree(k);
            let shapes = self.shapes[&k].clone();
            for s in &shapes {
                let min_distance = if self.floorplanning { 2 * max_dxdy(&s.rect) } else { self.pitch(s.li) };
                for n in self.shape_cleanup_nodes(s, min_distance, &tree, &shared, map) {
                    removes.insert(n);
                }
            }
        }
        let mut by_layer: BTreeMap<LayerKey, HashSet<NodeId>> = BTreeMap::new();
        for n in removes {
            by_layer.entry(self.key(self.arena[n].li)).or_default().insert(n);
        }
        for (k, set) in by_layer {
            self.remove_nodes(&set, k, map);
        }
    }

    fn shape_cleanup_nodes(&mut self, s: &Shape, min_distance: i32, tree: &PackedRTree<NodeId>, shared: &HashSet<NodeId>, map: &mut ConnMap) -> HashSet<NodeId> {
        let sorted = self.shape_nodes(tree, s);
        let c = center(&s.rect);
        let (mut center_nodes, mut other_nodes, mut shape_shared) = (Vec::new(), Vec::new(), Vec::new());
        for &n in &sorted {
            let p = self.arena[n].pt;
            if p.0 == c.0 || p.1 == c.1 {
                center_nodes.push(n);
            } else {
                other_nodes.push(n);
            }
        }
        // The search tree holds the shape's unshared nodes, each with a `used` flag.
        let mut data = Vec::new();
        for &n in &sorted {
            if shared.contains(&n) {
                shape_shared.push(n);
            } else {
                data.push(n);
            }
        }
        let search = ref_tree(data.iter().enumerate().map(|(i, &n)| (point_rect(self.arena[n].pt), i)).collect());
        let mut used = vec![false; data.len()];
        let mut remove = HashSet::new();
        let radius = min_distance / 2;
        for group in [shape_shared, center_nodes, other_nodes] {
            for n in group {
                if remove.contains(&n) {
                    continue;
                }
                let p = self.arena[n].pt;
                let check = Rect { xl: p.0 - radius, yl: p.1 - radius, xh: p.0 + radius, yh: p.1 + radius };
                let mut merge = Vec::new();
                for e in search.query(&check) {
                    let i = e.1;
                    if !used[i] && data[i] != n {
                        merge.push(data[i]);
                        used[i] = true;
                    }
                }
                for m in merge {
                    self.copy(n, m, map);
                    remove.insert(m);
                }
            }
        }
        remove
    }

    fn cleanup_nodes(&mut self) {
        let mut map = self.connection_map();
        self.cleanup_overlapping_nodes(&mut map);
        self.merge_nodes(&mut map);
    }

    // ---- connection cleanup ----------------------------------------------------------------------

    fn is_valid(c: &Conn) -> bool {
        match c.kind {
            ConnKind::Layer { length, width } => width > 0 && length > 0,
            ConnKind::Via { cuts } => cuts > 0,
            ConnKind::Term | ConnKind::Bridge => true,
        }
    }

    /// Drop `removes`; when anything was dropped, every survivor puts its lower node first again.
    fn remove_connections(&mut self, removes: &HashSet<ConnId>) {
        if removes.is_empty() {
            return;
        }
        self.connections.retain(|c| !removes.contains(c));
        for i in 0..self.connections.len() {
            let id = self.connections[i];
            let mut c = self.conns[id];
            self.ensure_node_order(&mut c);
            self.conns[id] = c;
        }
    }

    fn cleanup_invalid_connections(&mut self) {
        let removes: HashSet<ConnId> = self
            .connections
            .iter()
            .copied()
            .filter(|&c| {
                let conn = &self.conns[c];
                conn.n0.is_none() || conn.n1.is_none() || conn.n0 == conn.n1 || !Self::is_valid(conn)
            })
            .collect();
        self.remove_connections(&removes);
    }

    /// Connections between the same two nodes (as stored) become one: parallel layer connections
    /// add their widths, via connections their cuts.
    fn cleanup_duplicate_connections(&mut self) {
        let mut groups: HashMap<(Option<NodeId>, Option<NodeId>), Vec<ConnId>> = HashMap::new();
        for &c in &self.connections {
            groups.entry((self.conns[c].n0, self.conns[c].n1)).or_default().push(c);
        }
        let mut removes = HashSet::new();
        for (_, cs) in groups {
            let Some((&keep, rest)) = cs.split_first() else { continue };
            for &r in rest {
                let other = self.conns[r].kind;
                match (&mut self.conns[keep].kind, other) {
                    (ConnKind::Layer { width, .. }, ConnKind::Layer { width: w, .. }) => *width += w,
                    (ConnKind::Via { cuts }, ConnKind::Via { cuts: c }) => *cuts += c,
                    _ => {}
                }
                removes.insert(r);
            }
        }
        self.remove_connections(&removes);
    }

    fn cleanup_connections(&mut self) {
        self.cleanup_invalid_connections();
        self.cleanup_duplicate_connections();
    }

    // ---- layer connections -----------------------------------------------------------------------

    /// Per shape, every node joins the nearest of the shape's nodes not yet visited, visiting in
    /// node order: a chain along the shape.
    fn connect_layer_nodes(&mut self) {
        let keys: Vec<LayerKey> = self.shapes.keys().copied().collect();
        for k in keys {
            let layer_tree = self.node_tree(k);
            let shapes = self.shapes[&k].clone();
            for s in &shapes {
                for c in self.shape_connect_nodes(s, &layer_tree) {
                    self.connections.push(c);
                }
            }
        }
    }

    fn shape_connect_nodes(&mut self, s: &Shape, layer_tree: &PackedRTree<NodeId>) -> Vec<ConnId> {
        let sorted = self.shape_nodes(layer_tree, s);
        let tree = ref_tree(sorted.iter().map(|&n| (point_rect(self.arena[n].pt), n)).collect());
        let mut used: HashSet<NodeId> = HashSet::new();
        let mut out = Vec::new();
        for &n in &sorted {
            let pt = self.arena[n].pt;
            used.insert(n);
            let Some(&(_, other)) = tree.nearest_one(pt, |v| !used.contains(v)) else { continue };
            let size = s.connection_size(pt, self.arena[other].pt);
            out.push(self.new_conn(n, other, ConnKind::Layer { length: size.length, width: size.width }));
        }
        out
    }

    // ---- reports ---------------------------------------------------------------------------------

    fn dump_nodes(&mut self, name: &str) {
        if let Some(mut f) = self.dump.take() {
            f(name, self);
            self.dump = Some(f);
        }
    }

    /// A node as the dumps print it.
    pub fn describe(&self, n: NodeId) -> String {
        let node = &self.arena[n];
        let dbus = f64::from(self.tech.dbus);
        let kind = match node.kind {
            NodeKind::Node => "Node",
            NodeKind::ITerm { .. } => "ITerm Node",
            NodeKind::BPin { .. } => "BPin Node",
        };
        format!("{kind}: ({:.4}, {:.4}) on {}", f64::from(node.pt.0) / dbus, f64::from(node.pt.1) / dbus, self.tech.layers[node.li].name)
    }

    /// A node as the solve's dumps print it: an instance terminal's node adds the terminal's name.
    pub fn describe_full(&self, n: NodeId, net: &NetInput) -> String {
        match self.arena[n].kind {
            NodeKind::ITerm { iterm, .. } => format!("{}: {}", self.describe(n), net.iterms[iterm].name),
            _ => self.describe(n),
        }
    }

    /// Every layer node, numbered in map order: one dump file's lines.
    pub fn dump_lines(&self) -> Vec<String> {
        self.nodes.values().flatten().enumerate().map(|(i, &n)| format!("{i}: {}", self.describe(n))).collect()
    }

    /// The construction statistics, as reported.
    pub fn stats_lines(&self) -> Vec<String> {
        let mut out = Vec::new();
        let mut count = 0;
        for (k, s) in &self.shapes {
            count += s.len();
            out.push(format!("Shapes on {}: {}", self.tech.layers[k.li].name, s.len()));
        }
        out.push(format!("Total shapes: {count}"));
        count = 0;
        for (k, n) in &self.nodes {
            count += n.len();
            out.push(format!("Nodes on {}: {}", self.tech.layers[k.li].name, n.len()));
        }
        out.push(format!("Total nodes: {count}"));
        out.push(format!("Total connections: {}", self.connections.len()));
        out
    }
}

#[cfg(test)]
mod tests {
    // The reference's trees follow ITS standard library (the Bazel pin: libc++): among nodes
    // tied for nearest, the one libc++'s packing and heap give — here libstdc++'s differs, so
    // the choice is the library's, not the geometry's.
    #[test]
    fn a_nearest_tie_is_broken_as_the_reference_library_breaks_it() {
        use vyges_drt::rtree::{PackedRTree, StdLib};
        let ring = [(3, 4), (4, 3), (5, 0), (4, -3), (3, -4), (0, -5), (-3, -4), (-4, -3), (-5, 0), (-4, 3), (-3, 4), (0, 5)];
        let mut pts: Vec<(i32, i32)> = Vec::new();
        for k in 0..40 {
            pts.push((100 + 7 * k, 50 - 3 * k));
            pts.push((-90 - 5 * k, 30 + 11 * k));
        }
        pts.extend(ring);
        let values = |pts: &[(i32, i32)]| pts.iter().enumerate().map(|(i, &p)| (point_rect(p), i)).collect::<Vec<_>>();
        let pick = |t: &PackedRTree<usize>| t.nearest_one((0, 0), |_| true).map(|v| v.1);
        let ours = pick(&ref_tree(values(&pts)));
        assert_eq!(ours, pick(&PackedRTree::new_in(values(&pts), StdLib::LibCxx)));
        assert_ne!(ours, pick(&PackedRTree::new_in(values(&pts), StdLib::LibStdCxx)));
    }

    use super::*;
    use crate::input::{CutClass, Layer};

    fn layer(id: u32, number: i32, name: &str, routing_level: i32, dir: Dir) -> Layer {
        Layer { id, number, name: name.into(), routing_level, pitch: 200, dir, backside: false, upper: None, cut_classes: vec![] }
    }

    /// m1 (horizontal), v1 (cut), m2 (vertical); 1000 units per micron, node pitch 2000.
    fn tech() -> Tech {
        Tech {
            dbus: 1000,
            front_base: Some(0),
            back_base: None,
            layers: vec![layer(1, 0, "m1", 1, Dir::Horizontal), layer(2, 1, "v1", 0, Dir::None), layer(3, 2, "m2", 2, Dir::Vertical)],
        }
    }

    fn r(xl: i32, yl: i32, xh: i32, yh: i32) -> Rect {
        Rect { xl, yl, xh, yh }
    }

    fn net(swires: Vec<Vec<SBox>>) -> NetInput {
        NetInput { name: "VDD".into(), supply: true, swires, iterms: vec![], bterms: vec![] }
    }

    fn wire(layer: i32, rect: Rect) -> SBox {
        SBox::Wire { layer, rect }
    }

    fn build(t: &Tech, n: &NetInput) -> (Network<'static>, BuildLog) {
        let t: &'static Tech = Box::leak(Box::new(t.clone()));
        Network::build(t, n, false, None).unwrap()
    }

    // Upstream 5f0b2d8fb02 + 1d4cb901b1b (check_power_grid_short_iterms' bump): a layer with a
    // 45-degree shape merges its rectilinear metal into it and is sliced into trapezoids, not
    // rectangles. An octagon on a strap gives 5 pieces, the octagon's two slanted thirds held as
    // polygons. A polygon measures a connection across itself square to it, through its middle
    // (width 1400 at x 13300, not its box's 600), and puts a filler on its own middle line.
    #[test]
    fn an_octagon_on_a_strap_is_sliced_into_trapezoids() {
        let t = tech();
        let oct = vec![(13000, 11000), (13000, 11800), (13600, 12400), (14400, 12400), (15000, 11800), (15000, 11000), (14400, 10400), (13600, 10400), (13000, 11000)];
        let n = net(vec![vec![
            SBox::Oct { layer: 0, rect: r(13000, 10400, 15000, 12400), points: oct },
            wire(0, r(11000, 11230, 16000, 11570)),
        ]]);
        let (nw, _) = build(&t, &n);
        let shapes: Vec<(Rect, bool)> = nw.shapes.values().flatten().map(|s| (s.rect, s.polygon.is_some())).collect();
        assert_eq!(
            shapes,
            vec![
                (r(11000, 11230, 13000, 11570), false),
                (r(13000, 10400, 13600, 12400), true),
                (r(13600, 10400, 14400, 12400), false),
                (r(14400, 10400, 15000, 12400), true),
                (r(15000, 11230, 16000, 11570), false),
            ]
        );
        let slanted = &nw.shapes.values().flatten().nth(1).expect("five shapes");
        assert_eq!(slanted.connection_size((13000, 11400), (13600, 11400)), ConnectionSize { length: 600, width: 1400 });
        assert_eq!(slanted.filler_point((13300, 11800), false), Some((13300, 11800)));
        assert!(slanted.contains((13300, 12100)) && !slanted.contains((13100, 12100)));
    }

    // Upstream 1d4cb901b1b (`Shape::getNodes`, `getSharedShapeNodes`): the box of a polygon piece
    // can hold nodes that are not in it, here the centre of a rect in the corner of a diamond's
    // left half. They belong to that piece only when `contains` (covered_by) says so: no layer
    // connection joins the rect to the diamond, and the rect's node is on one shape, not shared.
    #[test]
    fn a_node_in_a_polygon_piece_box_but_outside_it_is_not_its_node() {
        let t = tech();
        let diamond = vec![(0, 1000), (1000, 2000), (2000, 1000), (1000, 0), (0, 1000)];
        let n = net(vec![vec![
            SBox::Oct { layer: 0, rect: r(0, 0, 2000, 2000), points: diamond },
            wire(0, r(100, 1700, 300, 1900)),
        ]]);
        let (nw, _) = build(&t, &n);
        assert!(nw.shapes.values().flatten().any(|s| s.polygon.is_some() && intersects(&s.rect, &r(200, 1800, 200, 1800))));
        for &c in &nw.connections {
            let conn = nw.conns[c];
            if let (ConnKind::Layer { .. }, Some(a), Some(b)) = (conn.kind, conn.n0, conn.n1) {
                let (pa, pb) = (nw.arena[a].pt, nw.arena[b].pt);
                assert!(nw.shapes.values().flatten().any(|s| s.contains(pa) && s.contains(pb)), "{pa:?}-{pb:?} joins two shapes");
            }
        }
        let on_m1: Vec<(i32, i32)> = nw.nodes.values().flatten().map(|&id| nw.arena[id].pt).collect();
        assert!(on_m1.contains(&(200, 1800)), "the rect's centre is not merged into the diamond: {on_m1:?}");
        let shared = nw.shared_shape_nodes();
        assert!(shared.iter().all(|&id| nw.arena[id].pt != (200, 1800)), "the rect's centre is on one shape");
    }

    // A horizontal layer is sliced VERTICALLY: an L of a long bar and a stub above it is the stub's
    // column plus the bar either side, three shapes, not the bar plus the stub.
    #[test]
    fn a_horizontal_layer_is_sliced_vertically() {
        let t = tech();
        let n = net(vec![vec![wire(0, r(0, 0, 10000, 100)), wire(0, r(4000, 100, 4100, 3000))]]);
        let (nw, _) = build(&t, &n);
        let shapes: Vec<Rect> = nw.shapes.values().flatten().map(|s| s.rect).collect();
        assert_eq!(shapes, vec![r(0, 0, 4000, 100), r(4000, 0, 4100, 3000), r(4100, 0, 10000, 100)]);
    }

    // Shapes take ids in layer-map order (by layer id) after each layer is sorted by rectangle.
    #[test]
    fn shapes_are_sorted_and_numbered_per_layer() {
        let t = tech();
        let n = net(vec![vec![wire(2, r(0, 0, 100, 5000)), wire(0, r(300, 0, 5000, 100)), wire(0, r(-5000, 0, -300, 100))]]);
        let (nw, _) = build(&t, &n);
        let got: Vec<(i32, usize)> = nw.shapes.values().flatten().map(|s| (s.rect.xl, s.id)).collect();
        assert_eq!(got, vec![(-5000, 0), (300, 1), (0, 2)]);
    }

    // A non-supply net is refused.
    #[test]
    fn a_signal_net_is_refused() {
        let t: &'static Tech = Box::leak(Box::new(tech()));
        let mut n = net(vec![]);
        n.supply = false;
        assert_eq!(Network::build(t, &n, false, None).err().unwrap(), BuildError::Error("VDD is not a supply net.".into()));
    }

    // A net with no shapes and no terminals is empty: nothing after generation runs.
    #[test]
    fn a_net_without_nodes_is_empty() {
        let (nw, log) = build(&tech(), &net(vec![]));
        assert!(log.empty);
        assert!(nw.nodes.is_empty() && nw.connections.is_empty());
    }

    /// The cut-layer stage alone, on an empty network.
    fn cut_nodes(t: Tech, n: &NetInput) -> Network<'static> {
        let t: &'static Tech = Box::leak(Box::new(t));
        let mut nw = Network {
            tech: t,
            floorplanning: false,
            min_node_pitch: HashMap::new(),
            arena: vec![],
            nodes: BTreeMap::new(),
            iterm_nodes: vec![],
            bpin_nodes: vec![],
            shapes: BTreeMap::new(),
            conns: vec![],
            connections: vec![],
            dump: None,
        };
        nw.init_minimum_node_pitch();
        nw.generate_cut_layer_nodes(n);
        nw
    }

    // A via smaller than the finer node pitch of its two layers is one node pair at the via's
    // origin carrying all its cuts; a cut matching a cut class counts its class's cuts.
    #[test]
    fn a_small_via_is_one_node_pair_with_every_cut() {
        let mut t = tech();
        t.layers[1].cut_classes = vec![CutClass { cuts_valid: true, width: 50, length_valid: true, length: 100, num_cuts: 2 }];
        let shapes = vec![
            ViaShape { layer: 0, rect: r(0, 0, 400, 200) },
            ViaShape { layer: 1, rect: r(50, 50, 150, 100) },
            ViaShape { layer: 1, rect: r(250, 50, 300, 150) },
            ViaShape { layer: 2, rect: r(0, 0, 400, 200) },
        ];
        let via = SBox::Via { rect: r(0, 0, 400, 200), xy: (200, 100), top: 2, bottom: 0, shapes };
        let n = net(vec![vec![via]]);
        let nw = cut_nodes(t, &n);
        let pts: Vec<((i32, i32), usize)> = nw.nodes.values().flatten().map(|&i| (nw.arena[i].pt, nw.arena[i].li)).collect();
        assert_eq!(pts, vec![((200, 100), 0), ((200, 100), 2)]);
        // 50 x 100 matches the class (2 cuts) either way round; 50 x 100 rotated too.
        assert_eq!(nw.conns[0].kind, ConnKind::Via { cuts: 4 });
    }

    // The single-via test is STRICT: a via whose longer side equals the finer node pitch (2000
    // here) gets a node pair per cut, at each cut's centre; one unit shorter is one pair.
    #[test]
    fn a_via_as_long_as_the_node_pitch_is_split_per_cut() {
        let via = |len: i32| {
            let shapes = vec![
                ViaShape { layer: 0, rect: r(0, 0, len, 200) },
                ViaShape { layer: 1, rect: r(50, 50, 150, 150) },
                ViaShape { layer: 1, rect: r(1800, 50, 1900, 150) },
                ViaShape { layer: 2, rect: r(0, 0, len, 200) },
            ];
            net(vec![vec![SBox::Via { rect: r(0, 0, len, 200), xy: (1000, 100), top: 2, bottom: 0, shapes }]])
        };
        let at = |nw: &Network| -> Vec<(i32, i32)> { nw.conns.iter().map(|c| nw.arena[c.n0.unwrap()].pt).collect() };
        assert_eq!(at(&cut_nodes(tech(), &via(2000))), vec![(100, 100), (1850, 100)]);
        assert_eq!(at(&cut_nodes(tech(), &via(1999))), vec![(1000, 100)]);
    }

    // Top-layer filler nodes: one every node pitch along the shape's long side, starting half a
    // pitch in, while the point is STRICTLY inside.
    #[test]
    fn filler_nodes_run_along_the_long_side_strictly_inside() {
        let t = tech();
        let n = net(vec![vec![wire(2, r(0, 0, 100, 6000))]]);
        let (nw, _) = build(&t, &n);
        // m2's pitch 200 caps the node pitch at 2000: fillers at y 1000, 3000, 5000 — then merged
        // with the centre node (3000) and each other only within a radius of 1000.
        let s = Shape { rect: r(0, 0, 100, 6000), li: 2, id: 0, polygon: None };
        let mut nw = nw;
        let made: Vec<(i32, i32)> = nw.create_filler_nodes(&s, 2000).iter().map(|&i| nw.arena[i].pt).collect();
        assert_eq!(made, vec![(50, 1000), (50, 3000), (50, 5000)]);
        assert!(nw.create_filler_nodes(&Shape { rect: r(0, 0, 100, 2000), li: 2, id: 0, polygon: None }, 2000).iter().all(|&i| nw.arena[i].pt.1 < 2000));
    }

    // Each shape's nodes are chained: every node joins the nearest node of the shape not yet
    // visited, in node order; a node's length is along the longer axis and its width the shape's
    // other side.
    #[test]
    fn shape_nodes_are_chained_to_the_nearest_unvisited() {
        let t = tech();
        let n = net(vec![vec![wire(0, r(0, 0, 30000, 100))]]);
        let (nw, _) = build(&t, &n);
        let mut lens: Vec<i32> = nw.connections.iter().map(|&c| match nw.conns[c].kind {
            ConnKind::Layer { length, width } => {
                assert_eq!(width, 100);
                length
            }
            k => panic!("{k:?}"),
        }).collect();
        lens.sort();
        let nodes = nw.nodes.values().flatten().count();
        assert_eq!(lens.len(), nodes - 1);
    }
}
