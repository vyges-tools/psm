// SPDX-License-Identifier: Apache-2.0
//! The short check: every other object's metal laid against the checked net's shapes.
//!
//! Walked in the order the check walks it — every other net (its block pins, special wires,
//! routing), then every instance (its pins, then its master's obstructions), then the block's
//! obstructions, then fill — and each overlap reported in that order, by its bounding box.

use std::collections::{BTreeMap, HashMap};

use vyges_drt::polygon90::Polygon90Set;
use vyges_drt::rtree::PackedRTree;

use crate::network::ref_tree;

use crate::input::{Rect, Stream};
use crate::network::Network;

/// The check stops recording after this many shorts.
pub const MAX_SHORT_ENTRIES: usize = 10000;

/// A box of a pin, wire or obstruction; a via's shapes on every layer (cut included).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BoxGeom {
    pub is_via: bool,
    pub layer: Option<i32>,
    pub rect: Rect,
    pub via: Vec<(Option<i32>, Rect)>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BPinGeom {
    pub placed: bool,
    pub fixed: bool,
    pub boxes: Vec<BoxGeom>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OtherNet {
    pub name: String,
    pub bterms: Vec<(String, Vec<BPinGeom>)>,
    /// Special-wire boxes, with whether each is octilinear.
    pub sboxes: Vec<(bool, BoxGeom)>,
    pub wire: Vec<BoxGeom>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ITermGeom {
    pub name: String,
    pub on_net: bool,
    pub net: Option<String>,
    pub sig: i64,
    pub mterm: usize,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct InstGeom {
    pub name: String,
    pub placed: bool,
    pub fixed: bool,
    pub master: usize,
    pub orient: i64,
    pub offset: (i32, i32),
    pub iterms: Vec<ITermGeom>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct MasterGeom {
    pub boundary: Rect,
    pub mterms: Vec<TermGeom>,
    pub obstructions: Vec<BoxGeom>,
    pub obstruction_polygons: Vec<PolygonGeom>,
}

/// A master polygon as written, on its layer.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PolygonGeom {
    pub layer: Option<i32>,
    pub pts: Vec<(i32, i32)>,
}

/// A master terminal: its pins' boxes and polygons.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TermGeom {
    pub boxes: Vec<BoxGeom>,
    pub polygons: Vec<PolygonGeom>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BlockObstruction {
    pub except_pg: bool,
    pub slot: bool,
    pub fill: bool,
    pub layer: Option<i32>,
    pub rect: Rect,
    pub inst: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FillGeom {
    pub layer: Option<i32>,
    pub rect: Rect,
    pub mask: i64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ShortInputs {
    pub sig: i64,
    pub nets: Vec<OtherNet>,
    pub insts: Vec<InstGeom>,
    pub masters: Vec<MasterGeom>,
    pub obstructions: Vec<BlockObstruction>,
    pub fills: Vec<FillGeom>,
}

fn polygon_geom(s: &mut Stream) -> Result<PolygonGeom, String> {
    let layer = s.layer()?;
    let n = s.count()?;
    let pts = (0..n).map(|_| Ok((s.int()?, s.int()?))).collect::<Result<_, String>>()?;
    Ok(PolygonGeom { layer, pts })
}

fn box_geom(s: &mut Stream) -> Result<BoxGeom, String> {
    let is_via = s.flag()?;
    let layer = s.layer()?;
    let rect = s.rect()?;
    let n = s.count()?;
    let via = (0..n).map(|_| Ok((s.layer()?, s.rect()?))).collect::<Result<_, String>>()?;
    Ok(BoxGeom { is_via, layer, rect, via })
}

/// The stream from `psm_short_inputs`, with `names` from `psm_short_names`.
pub fn decode_short_inputs(v: &[i64], names: &[String]) -> Result<ShortInputs, String> {
    let mut s = Stream::new(v);
    let name = |i: i64| -> Result<String, String> {
        usize::try_from(i).ok().and_then(|i| names.get(i)).cloned().ok_or_else(|| format!("name index {i}"))
    };
    let opt_name = |i: i64| -> Result<Option<String>, String> { if i < 0 { Ok(None) } else { name(i).map(Some) } };
    let sig = s.next()?;
    let nnets = s.count()?;
    let mut nets = Vec::with_capacity(nnets);
    for _ in 0..nnets {
        let net = name(s.next()?)?;
        let nbt = s.count()?;
        let mut bterms = Vec::with_capacity(nbt);
        for _ in 0..nbt {
            let bt = name(s.next()?)?;
            let np = s.count()?;
            let mut pins = Vec::with_capacity(np);
            for _ in 0..np {
                let placed = s.flag()?;
                let fixed = s.flag()?;
                let nb = s.count()?;
                let boxes = (0..nb).map(|_| box_geom(&mut s)).collect::<Result<_, String>>()?;
                pins.push(BPinGeom { placed, fixed, boxes });
            }
            bterms.push((bt, pins));
        }
        let nsb = s.count()?;
        let sboxes = (0..nsb).map(|_| Ok((s.flag()?, box_geom(&mut s)?))).collect::<Result<_, String>>()?;
        let nw = s.count()?;
        let wire = (0..nw).map(|_| box_geom(&mut s)).collect::<Result<_, String>>()?;
        nets.push(OtherNet { name: net, bterms, sboxes, wire });
    }
    let ninst = s.count()?;
    let mut insts = Vec::with_capacity(ninst);
    for _ in 0..ninst {
        let iname = name(s.next()?)?;
        let placed = s.flag()?;
        let fixed = s.flag()?;
        let master = s.count()?;
        let orient = s.next()?;
        let offset = (s.int()?, s.int()?);
        let nit = s.count()?;
        let mut iterms = Vec::with_capacity(nit);
        for _ in 0..nit {
            let tname = name(s.next()?)?;
            let on_net = s.flag()?;
            let net = opt_name(s.next()?)?;
            let sig = s.next()?;
            let mterm = s.count()?;
            iterms.push(ITermGeom { name: tname, on_net, net, sig, mterm });
        }
        insts.push(InstGeom { name: iname, placed, fixed, master, orient, offset, iterms });
    }
    let nm = s.count()?;
    let mut masters = Vec::with_capacity(nm);
    for _ in 0..nm {
        let boundary = s.rect()?;
        let nmt = s.count()?;
        let mut mterms = Vec::with_capacity(nmt);
        for _ in 0..nmt {
            let nb = s.count()?;
            let boxes = (0..nb).map(|_| box_geom(&mut s)).collect::<Result<_, String>>()?;
            let np = s.count()?;
            let polygons = (0..np).map(|_| polygon_geom(&mut s)).collect::<Result<_, String>>()?;
            mterms.push(TermGeom { boxes, polygons });
        }
        let no = s.count()?;
        let obstructions = (0..no).map(|_| box_geom(&mut s)).collect::<Result<_, String>>()?;
        let np = s.count()?;
        let obstruction_polygons = (0..np).map(|_| polygon_geom(&mut s)).collect::<Result<_, String>>()?;
        masters.push(MasterGeom { boundary, mterms, obstructions, obstruction_polygons });
    }
    let nobs = s.count()?;
    let mut obstructions = Vec::with_capacity(nobs);
    for _ in 0..nobs {
        let except_pg = s.flag()?;
        let slot = s.flag()?;
        let fill = s.flag()?;
        let layer = s.layer()?;
        let rect = s.rect()?;
        let inst = opt_name(s.next()?)?;
        obstructions.push(BlockObstruction { except_pg, slot, fill, layer, rect, inst });
    }
    let nf = s.count()?;
    let fills = (0..nf).map(|_| Ok(FillGeom { layer: s.layer()?, rect: s.rect()?, mask: s.next()? })).collect::<Result<_, String>>()?;
    s.done()?;
    Ok(ShortInputs { sig, nets, insts, masters, obstructions, fills })
}

// ---- regions (sets of Manhattan rectangles) -------------------------------------------------------

fn positive(r: &Rect) -> bool {
    r.xl < r.xh && r.yl < r.yh
}

fn intersect(a: &Rect, b: &Rect) -> Rect {
    Rect { xl: a.xl.max(b.xl), yl: a.yl.max(b.yl), xh: a.xh.min(b.xh), yh: a.yh.min(b.yh) }
}

/// Strict overlap: sharing an edge or a corner is not overlapping.
fn overlaps(a: &Rect, b: &Rect) -> bool {
    b.xh > a.xl && b.xl < a.xh && b.yh > a.yl && b.yl < a.yh
}

fn bbox(rs: &[Rect]) -> Rect {
    rs.iter().skip(1).fold(rs[0], |b, r| Rect { xl: b.xl.min(r.xl), yl: b.yl.min(r.yl), xh: b.xh.max(r.xh), yh: b.yh.max(r.yh) })
}

/// A region as disjoint slices.
fn region(rs: &[Rect]) -> Vec<Rect> {
    let mut s = Polygon90Set::new();
    rs.iter().for_each(|r| s.insert_rect(*r));
    s.rectangles()
}

fn clip(rs: &[Rect], c: &Rect) -> Vec<Rect> {
    rs.iter().map(|r| intersect(r, c)).filter(positive).collect()
}

/// `a` without `b`: the cells of the two regions' grid inside `a` and not inside `b`.
fn minus(a: &[Rect], b: &[Rect]) -> Vec<Rect> {
    let mut xs: Vec<i32> = a.iter().chain(b).flat_map(|r| [r.xl, r.xh]).collect();
    let mut ys: Vec<i32> = a.iter().chain(b).flat_map(|r| [r.yl, r.yh]).collect();
    xs.sort_unstable();
    xs.dedup();
    ys.sort_unstable();
    ys.dedup();
    let inside = |rs: &[Rect], x: i32, y: i32| rs.iter().any(|r| r.xl <= x && x < r.xh && r.yl <= y && y < r.yh);
    let mut out = Vec::new();
    for wx in xs.windows(2) {
        for wy in ys.windows(2) {
            if inside(a, wx[0], wy[0]) && !inside(b, wx[0], wy[0]) {
                out.push(Rect { xl: wx[0], yl: wy[0], xh: wx[1], yh: wy[1] });
            }
        }
    }
    region(&out)
}

/// The connected pieces of a region in the order the general polygon set returns them: by the
/// piece's right x, then the lowest y of its edge on that line (its scan runs along x and emits a
/// piece as it closes it). Pinned against boost by `instruments/psm/polyset-order`.
pub fn pieces(rs: &[Rect]) -> Vec<Vec<Rect>> {
    let mut s = Polygon90Set::new();
    rs.iter().for_each(|r| s.insert_rect(*r));
    let mut out: Vec<((i32, i32), Vec<Rect>)> = s
        .polygons()
        .into_iter()
        .map(|mut p| {
            let pr = p.rectangles();
            let b = bbox(&pr);
            let right_lo = pr.iter().filter(|r| r.xh == b.xh).map(|r| r.yl).min().expect("a slice");
            ((b.xh, right_lo), pr)
        })
        .collect();
    out.sort_by_key(|p| p.0);
    out.into_iter().map(|p| p.1).collect()
}

/// An instance placement of a point: orientation (R0 R90 R180 R270 MY MYR90 MX MXR90), then the
/// offset.
fn place(orient: i64, off: (i32, i32), (x, y): (i32, i32)) -> (i32, i32) {
    let (x, y) = match orient {
        1 => (-y, x),
        2 => (-x, -y),
        3 => (y, -x),
        4 => (-x, y),
        5 => (-y, -x),
        6 => (x, -y),
        7 => (y, x),
        _ => (x, y),
    };
    (x + off.0, y + off.1)
}

fn transform(orient: i64, off: (i32, i32), r: &Rect) -> Rect {
    let (a, b) = (place(orient, off, (r.xl, r.yl)), place(orient, off, (r.xh, r.yh)));
    Rect { xl: a.0.min(b.0), yl: a.1.min(b.1), xh: a.0.max(b.0), yh: a.1.max(b.1) }
}

// ---- polygons ------------------------------------------------------------------------------------

/// A polygon whose every edge is horizontal or vertical, as disjoint slices; `None` otherwise.
fn manhattan_rects(pts: &[(i32, i32)]) -> Option<Vec<Rect>> {
    let n = pts.len();
    if (0..n).any(|i| {
        let (a, b) = (pts[i], pts[(i + 1) % n]);
        a.0 != b.0 && a.1 != b.1
    }) {
        return None;
    }
    let mut xs: Vec<i32> = pts.iter().map(|p| p.0).collect();
    let mut ys: Vec<i32> = pts.iter().map(|p| p.1).collect();
    xs.sort_unstable();
    xs.dedup();
    ys.sort_unstable();
    ys.dedup();
    // A cell is inside when its centre is: crossings of a ray to +x, on doubled coordinates.
    let inside = |cx: i64, cy: i64| {
        let mut odd = false;
        for i in 0..n {
            let (a, b) = (pts[i], pts[(i + 1) % n]);
            if a.0 != b.0 {
                continue;
            }
            let (lo, hi) = (2 * i64::from(a.1.min(b.1)), 2 * i64::from(a.1.max(b.1)));
            if lo < cy && cy < hi && 2 * i64::from(a.0) > cx {
                odd = !odd;
            }
        }
        odd
    };
    let mut cells = Vec::new();
    for wx in xs.windows(2) {
        for wy in ys.windows(2) {
            if inside(i64::from(wx[0]) + i64::from(wx[1]), i64::from(wy[0]) + i64::from(wy[1])) {
                cells.push(Rect { xl: wx[0], yl: wy[0], xh: wx[1], yh: wy[1] });
            }
        }
    }
    Some(region(&cells))
}

fn convex(pts: &[(i32, i32)]) -> bool {
    let n = pts.len();
    let mut sign = 0i64;
    for i in 0..n {
        let (a, b, c) = (pts[i], pts[(i + 1) % n], pts[(i + 2) % n]);
        let cross = i64::from(b.0 - a.0) * i64::from(c.1 - b.1) - i64::from(b.1 - a.1) * i64::from(c.0 - b.0);
        if cross != 0 {
            if sign != 0 && cross.signum() != sign {
                return false;
            }
            sign = cross.signum();
        }
    }
    n >= 3
}

/// A convex polygon cut to a rectangle: the bounding box of what is left, when it has area, and
/// whether what is left is that box. `Err` when a corner of the cut does not land on the grid (a
/// slope other than 45 degrees).
fn clip_convex(pts: &[(i32, i32)], r: &Rect) -> Result<Option<(Rect, bool)>, String> {
    let mut poly: Vec<(f64, f64)> = pts.iter().map(|p| (f64::from(p.0), f64::from(p.1))).collect();
    let planes: [(usize, f64, bool); 4] = [(0, f64::from(r.xl), true), (0, f64::from(r.xh), false), (1, f64::from(r.yl), true), (1, f64::from(r.yh), false)];
    for (axis, v, keep_above) in planes {
        let coord = |p: (f64, f64)| if axis == 0 { p.0 } else { p.1 };
        let ins = |p: (f64, f64)| if keep_above { coord(p) >= v } else { coord(p) <= v };
        let mut out = Vec::new();
        for i in 0..poly.len() {
            let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
            if ins(a) {
                out.push(a);
            }
            if ins(a) != ins(b) {
                let t = (v - coord(a)) / (coord(b) - coord(a));
                out.push((a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1)));
            }
        }
        poly = out;
        if poly.is_empty() {
            return Ok(None);
        }
    }
    let area: f64 = (0..poly.len()).map(|i| {
        let (a, b) = (poly[i], poly[(i + 1) % poly.len()]);
        a.0 * b.1 - b.0 * a.1
    }).sum();
    if area.abs() == 0.0 {
        return Ok(None);
    }
    if poly.iter().any(|p| p.0.fract() != 0.0 || p.1.fract() != 0.0) {
        return Err("a polygon edge that is not at 45 degrees".to_string());
    }
    let xs = poly.iter().map(|p| p.0 as i32);
    let ys = poly.iter().map(|p| p.1 as i32);
    let b = Rect { xl: xs.clone().min().unwrap(), yl: ys.clone().min().unwrap(), xh: xs.max().unwrap(), yh: ys.max().unwrap() };
    let boxed = area.abs() / 2.0 == f64::from(b.dx()) * f64::from(b.dy());
    Ok(Some((b, boxed)))
}

/// One extracted piece of a master's layer: Manhattan slices, or a single convex polygon.
#[derive(Debug, Clone, PartialEq, Eq)]
enum Piece {
    Rects(Vec<Rect>),
    Convex(Vec<(i32, i32)>),
}

impl Piece {
    fn placed(&self, orient: i64, off: (i32, i32)) -> Piece {
        match self {
            Piece::Rects(rs) => Piece::Rects(rs.iter().map(|r| transform(orient, off, r)).collect()),
            Piece::Convex(pts) => Piece::Convex(pts.iter().map(|p| place(orient, off, *p)).collect()),
        }
    }
}

fn bloat(r: &Rect, d: i32) -> Rect {
    Rect { xl: r.xl - d, yl: r.yl - d, xh: r.xh + d, yh: r.yh + d }
}

// ---- the check -----------------------------------------------------------------------------------

/// A database object a short's marker names besides the checked net.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Source {
    Net(String),
    BTerm(String),
    ITerm(String),
    Inst(String),
    /// By its position among the block's obstructions.
    Obstruction(usize),
}

/// What a short is to: the words the report uses and the objects the marker names.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Other {
    pub to: String,
    pub sources: Vec<Source>,
}

/// One short: the layer, the overlap's bounding box (`rect` when the overlap IS that box), and what
/// the net is shorted to.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Short {
    pub li: usize,
    pub bbox: Rect,
    pub rect: bool,
    pub to: String,
    pub sources: Vec<Source>,
}

/// A layer as a master's per-layer maps key it: (database id, which orders them; number).
type LayerKey = (u32, i32);

/// A polygon's points, in order.
type Points = Vec<(i32, i32)>;

/// A master's pieces per layer (by layer number, kept in layer-id order).
type LayerPieces = Vec<(i32, Vec<Piece>)>;

struct Walk<'n, 'a> {
    nw: &'n Network<'a>,
    trees: HashMap<usize, PackedRTree<Rect>>,
    shorts: Vec<Short>,
}

impl Walk<'_, '_> {
    fn li(&self, number: i32) -> Option<usize> {
        self.nw.tech.by_number(number)
    }

    /// Recorded, unless the list is full: then the walk stops.
    fn add(&mut self, li: usize, (bbox, rect): (Rect, bool), other: &Other) -> bool {
        if self.shorts.len() >= MAX_SHORT_ENTRIES {
            return false;
        }
        self.shorts.push(Short { li, bbox, rect, to: other.to.clone(), sources: other.sources.clone() });
        true
    }

    /// The net's shapes a rectangle overlaps (touching is not a short): each overlap.
    fn short_rect(&self, layer: Option<i32>, r: &Rect) -> (usize, Vec<(Rect, bool)>) {
        let Some(li) = layer.and_then(|l| self.li(l)) else { return (0, vec![]) };
        let Some(tree) = self.trees.get(&li) else { return (li, vec![]) };
        (li, tree.query(r).into_iter().filter(|e| overlaps(&e.1, r)).map(|e| (intersect(&e.1, r), true)).collect())
    }

    /// The net's shapes a region overlaps: per shape, every piece of the overlap. With
    /// `require_overlap` an overlap that does not reach into the shape is not one.
    fn short_region(&self, li: usize, rs: &[Rect], require_overlap: bool) -> Vec<(Rect, bool)> {
        let Some(tree) = self.trees.get(&li) else { return vec![] };
        let mut out = Vec::new();
        for e in tree.query(&bbox(rs)) {
            let shape = e.1;
            let overlap = clip(rs, &shape);
            if overlap.is_empty() {
                continue;
            }
            if require_overlap && !overlaps(&shape, &bbox(&overlap)) {
                continue;
            }
            out.extend(pieces(&overlap).iter().map(|p| (bbox(p), p.len() == 1)));
        }
        out
    }

    /// [`Self::short_region`] for either kind of piece.
    fn short_piece(&self, li: usize, p: &Piece, require_overlap: bool) -> Result<Vec<(Rect, bool)>, String> {
        let pts = match p {
            Piece::Rects(rs) => return Ok(self.short_region(li, rs, require_overlap)),
            Piece::Convex(pts) => pts,
        };
        let Some(tree) = self.trees.get(&li) else { return Ok(vec![]) };
        let bb = Rect {
            xl: pts.iter().map(|p| p.0).min().unwrap_or(0),
            yl: pts.iter().map(|p| p.1).min().unwrap_or(0),
            xh: pts.iter().map(|p| p.0).max().unwrap_or(0),
            yh: pts.iter().map(|p| p.1).max().unwrap_or(0),
        };
        let mut out = Vec::new();
        for e in tree.query(&bb) {
            let Some(o) = clip_convex(pts, &e.1)? else { continue };
            if require_overlap && !overlaps(&e.1, &o.0) {
                continue;
            }
            out.push(o);
        }
        Ok(out)
    }

    /// A box: a via's shapes layer by layer, or the box on its own layer.
    fn short_box(&mut self, g: &BoxGeom, to: &Other) -> bool {
        let parts: Vec<(Option<i32>, Rect)> = if g.is_via { g.via.iter().filter(|v| v.0.is_some()).copied().collect() } else { vec![(g.layer, g.rect)] };
        for (layer, r) in parts {
            let (li, found) = self.short_rect(layer, &r);
            for o in found {
                if !self.add(li, o, to) {
                    return false;
                }
            }
        }
        true
    }
}

/// Per layer, by layer id: the layer's region from these boxes (a via's shapes on their layers).
fn layer_regions(nw: &Network, boxes: &[BoxGeom], grow: i32) -> BTreeMap<LayerKey, Vec<Rect>> {
    let mut by: BTreeMap<LayerKey, Vec<Rect>> = BTreeMap::new();
    let mut put = |layer: i32, r: Rect| {
        if let Some(li) = nw.tech.by_number(layer) {
            by.entry((nw.tech.layers[li].id, layer)).or_default().push(bloat(&r, grow));
        }
    };
    for g in boxes {
        if g.is_via {
            for (l, r) in &g.via {
                if let Some(l) = l {
                    put(*l, *r);
                }
            }
        } else if let Some(l) = g.layer {
            put(l, g.rect);
        }
    }
    by
}

/// Polygons per layer key (as [`layer_regions`]): the Manhattan ones join that layer's slices,
/// the others are returned on their own.
fn add_polygons(nw: &Network, polygons: &[PolygonGeom], grow: i32, by: &mut BTreeMap<LayerKey, Vec<Rect>>) -> BTreeMap<LayerKey, Vec<Points>> {
    let mut skewed: BTreeMap<LayerKey, Vec<Points>> = BTreeMap::new();
    for p in polygons {
        let Some(l) = p.layer else { continue };
        let Some(li) = nw.tech.by_number(l) else { continue };
        let key = (nw.tech.layers[li].id, l);
        match manhattan_rects(&p.pts) {
            Some(rs) => by.entry(key).or_default().extend(rs.iter().map(|r| bloat(r, grow))),
            None => skewed.entry(key).or_default().push(p.pts.clone()),
        }
    }
    skewed
}

/// A master terminal's metal per layer, as pieces. A layer whose metal is one convex polygon off
/// the Manhattan grid is that polygon; any other mix with such a polygon cannot be measured yet.
fn master_terms(nw: &Network, t: &TermGeom) -> Result<LayerPieces, String> {
    let mut by = layer_regions(nw, &t.boxes, 0);
    let mut skewed = add_polygons(nw, &t.polygons, 0, &mut by);
    for k in skewed.keys() {
        by.entry(*k).or_default();
    }
    let mut out = Vec::new();
    for (key, rs) in by {
        match skewed.remove(&key) {
            None => out.push((key.1, pieces(&region(&rs)).into_iter().map(Piece::Rects).collect())),
            Some(mut ps) if ps.len() == 1 && rs.is_empty() && convex(&ps[0]) => out.push((key.1, vec![Piece::Convex(ps.remove(0))])),
            Some(_) => return Err("a pin with a polygon that is not Manhattan among other metal".to_string()),
        }
    }
    Ok(out)
}

/// A master's obstructions per layer, clipped to its boundary and without its pins (grown by one
/// so nothing is left flush against a pin), as pieces.
fn master_obstructions(nw: &Network, m: &MasterGeom) -> Result<LayerPieces, String> {
    let obstructions: Vec<BoxGeom> = m.obstructions.iter().filter(|g| !g.is_via).cloned().collect();
    let mut obs = layer_regions(nw, &obstructions, 0);
    if !add_polygons(nw, &m.obstruction_polygons, 0, &mut obs).is_empty() {
        return Err("an obstruction polygon that is not Manhattan".to_string());
    }
    let all_pins: Vec<BoxGeom> = m.mterms.iter().flat_map(|t| t.boxes.iter()).cloned().collect();
    let all_polygons: Vec<PolygonGeom> = m.mterms.iter().flat_map(|t| t.polygons.iter()).cloned().collect();
    let mut pins = layer_regions(nw, &all_pins, 1);
    let skewed_pins = add_polygons(nw, &all_polygons, 1, &mut pins);
    let mut out = Vec::new();
    for (key, rs) in obs {
        if skewed_pins.contains_key(&key) {
            return Err("an obstruction on a layer with a pin polygon that is not Manhattan".to_string());
        }
        let mut r = clip(&region(&rs), &m.boundary);
        if let Some(p) = pins.get(&key) {
            r = minus(&r, &region(p));
        }
        if r.is_empty() {
            continue;
        }
        out.push((key.1, pieces(&r).into_iter().map(Piece::Rects).collect()));
    }
    Ok(out)
}

/// The check's walk. `Ok((shorts, stopped))`; `Err` for an object the walk cannot measure yet.
pub fn check_short(nw: &Network, si: &ShortInputs, check_placed: bool) -> Result<(Vec<Short>, bool), String> {
    let mut trees = HashMap::new();
    for (k, shapes) in &nw.shapes {
        trees.insert(k.li, ref_tree(shapes.iter().map(|s| (s.rect, s.rect)).collect()));
    }
    let mut w = Walk { nw, trees, shorts: Vec::new() };
    let counts = |placed: bool, fixed: bool| if check_placed { placed } else { fixed };
    for net in &si.nets {
        for (bterm, pins) in &net.bterms {
            let to = Other { to: format!("terminal {bterm} (net {})", net.name), sources: vec![Source::BTerm(bterm.clone()), Source::Net(net.name.clone())] };
            for pin in pins.iter().filter(|p| counts(p.placed, p.fixed)) {
                for g in &pin.boxes {
                    if !w.short_box(g, &to) {
                        return Ok((w.shorts, true));
                    }
                }
            }
        }
        let to = Other { to: format!("net {}", net.name), sources: vec![Source::Net(net.name.clone())] };
        for (octilinear, g) in &net.sboxes {
            if *octilinear {
                return Err(format!("an octilinear special wire on net {}", net.name));
            }
            if !w.short_box(g, &to) {
                return Ok((w.shorts, true));
            }
        }
        for g in &net.wire {
            if !w.short_box(g, &to) {
                return Ok((w.shorts, true));
            }
        }
    }
    let mut terms: HashMap<(usize, usize), LayerPieces> = HashMap::new();
    let mut obstructions: HashMap<usize, LayerPieces> = HashMap::new();
    for inst in si.insts.iter().filter(|i| counts(i.placed, i.fixed)) {
        let m = &si.masters[inst.master];
        for it in &inst.iterms {
            if it.on_net || (it.net.is_none() && it.sig == si.sig) {
                continue;
            }
            let t = match terms.entry((inst.master, it.mterm)) {
                std::collections::hash_map::Entry::Occupied(e) => e.get().clone(),
                std::collections::hash_map::Entry::Vacant(e) => e.insert(master_terms(nw, &m.mterms[it.mterm]).map_err(|e| format!("{e} on {}", it.name))?).clone(),
            };
            let mut sources = vec![Source::ITerm(it.name.clone())];
            sources.extend(it.net.iter().map(|n| Source::Net(n.clone())));
            let to = Other { to: format!("terminal {} (net {})", it.name, it.net.as_deref().unwrap_or("unconnected")), sources };
            for (layer, ps) in &t {
                let Some(li) = w.li(*layer) else { continue };
                for p in ps {
                    for o in w.short_piece(li, &p.placed(inst.orient, inst.offset), false)? {
                        if !w.add(li, o, &to) {
                            return Ok((w.shorts, true));
                        }
                    }
                }
            }
        }
        let obs = match obstructions.entry(inst.master) {
            std::collections::hash_map::Entry::Occupied(e) => e.get().clone(),
            std::collections::hash_map::Entry::Vacant(e) => e.insert(master_obstructions(nw, m).map_err(|e| format!("{e} on {}", inst.name))?).clone(),
        };
        let to = Other { to: format!("instance {}", inst.name), sources: vec![Source::Inst(inst.name.clone())] };
        for (layer, ps) in &obs {
            let Some(li) = w.li(*layer) else { continue };
            for p in ps {
                for o in w.short_piece(li, &p.placed(inst.orient, inst.offset), true)? {
                    if !w.add(li, o, &to) {
                        return Ok((w.shorts, true));
                    }
                }
            }
        }
    }
    for (i, obs) in si.obstructions.iter().enumerate().filter(|(_, o)| !(o.except_pg || o.slot || o.fill)) {
        let mut sources = vec![Source::Obstruction(i)];
        sources.extend(obs.inst.iter().map(|n| Source::Inst(n.clone())));
        let to = match &obs.inst {
            Some(i) => format!("obstruction of {i}"),
            None => "obstruction".to_string(),
        };
        let to = Other { to, sources };
        let (li, found) = w.short_rect(obs.layer, &obs.rect);
        for o in found {
            if !w.add(li, o, &to) {
                return Ok((w.shorts, true));
            }
        }
    }
    for f in &si.fills {
        let to = if f.mask == 0 { "fill".to_string() } else { format!("fill with mask {}", f.mask) };
        // The marker cannot name a fill.
        let to = Other { to, sources: vec![] };
        let (li, found) = w.short_rect(f.layer, &f.rect);
        for o in found {
            if !w.add(li, o, &to) {
                return Ok((w.shorts, true));
            }
        }
    }
    Ok((w.shorts, false))
}

/// The lines the check prints for its shorts: the stop notice first, then each short.
pub fn report_shorts(nw: &Network, net: &str, shorts: &[Short], stopped: bool) -> Vec<String> {
    let dbu = f64::from(nw.tech.dbus);
    let um = |v: i32| format!("{:4.3}", f64::from(v) / dbu);
    let mut out = Vec::new();
    if stopped {
        out.push(format!(
            "[WARNING PSM-0044] Stopped checking {net} after {} shorted shapes, the design holds more than this reports.",
            shorts.len()
        ));
    }
    for s in shorts {
        out.push(format!(
            "[WARNING PSM-0043] Shorted shape on net {net} to {} at ({}um, {}um) - ({}um, {}um), layer: {}.",
            s.to,
            um(s.bbox.xl),
            um(s.bbox.yl),
            um(s.bbox.xh),
            um(s.bbox.yh),
            nw.tech.layers[s.li].name
        ));
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::{Dir, Layer, NetInput, SBox, Tech};

    fn r(xl: i32, yl: i32, xh: i32, yh: i32) -> Rect {
        Rect { xl, yl, xh, yh }
    }

    /// m1 (horizontal), v1 (cut), m2 (vertical); 1000 units per micron.
    fn tech() -> &'static Tech {
        let layer = |id: u32, number: i32, name: &str, routing_level: i32, dir: Dir| Layer {
            id,
            number,
            name: name.into(),
            routing_level,
            pitch: 200,
            dir,
            backside: false,
            upper: None,
            cut_classes: vec![],
        };
        Box::leak(Box::new(Tech {
            dbus: 1000,
            front_base: Some(0),
            back_base: None,
            layers: vec![layer(1, 0, "m1", 1, Dir::Horizontal), layer(2, 1, "v1", 0, Dir::None), layer(3, 2, "m2", 2, Dir::Vertical)],
        }))
    }

    /// VDD: one m1 strap, 0..10000 x 0..1000.
    fn network() -> Network<'static> {
        let net = NetInput {
            name: "VDD".into(),
            supply: true,
            swires: vec![vec![SBox::Wire { layer: 0, rect: r(0, 0, 10000, 1000) }]],
            iterms: vec![],
            bterms: vec![],
        };
        Network::build(tech(), &net, false, None).unwrap().0
    }

    fn wire(layer: i32, rect: Rect) -> BoxGeom {
        BoxGeom { is_via: false, layer: Some(layer), rect, via: vec![] }
    }

    fn inputs() -> ShortInputs {
        ShortInputs { sig: 1, nets: vec![], insts: vec![], masters: vec![], obstructions: vec![], fills: vec![] }
    }

    fn other(name: &str, wire: Vec<BoxGeom>) -> OtherNet {
        OtherNet { name: name.into(), bterms: vec![], sboxes: vec![], wire }
    }

    // A wire that only touches the strap is not a short (the tree's query is closed, the overlap
    // test strict); one that overlaps it reports the overlap, on the strap's layer. A wire on
    // another layer is not compared.
    #[test]
    fn touching_is_not_a_short_overlapping_reports_the_overlap() {
        let nw = network();
        let mut si = inputs();
        si.nets = vec![other("A", vec![wire(0, r(2000, 1000, 3000, 2000)), wire(0, r(4000, 500, 5000, 1500)), wire(2, r(0, 0, 100, 100))])];
        let (shorts, stopped) = check_short(&nw, &si, true).unwrap();
        assert!(!stopped);
        assert_eq!(shorts, vec![Short { li: 0, bbox: r(4000, 500, 5000, 1000), rect: true, to: "net A".into(), sources: vec![Source::Net("A".into())] }]);
        let lines = report_shorts(&nw, "VDD", &shorts, stopped);
        assert_eq!(lines, vec!["[WARNING PSM-0043] Shorted shape on net VDD to net A at (4.000um, 0.500um) - (5.000um, 1.000um), layer: m1."]);
    }

    // The walk's object order: each other net's block pins, then its special wires, then its
    // routing; then instances; then block obstructions (not the except-PG, slot or fill kinds),
    // then fill. A pin counts when placed (checking placed) or when fixed (floorplanning).
    #[test]
    fn objects_are_walked_pins_swires_wires_insts_obstructions_fill() {
        let nw = network();
        let mut si = inputs();
        let pin = |placed, fixed| BPinGeom { placed, fixed, boxes: vec![wire(0, r(100, 100, 200, 200))] };
        si.nets = vec![OtherNet {
            name: "A".into(),
            bterms: vec![("a".into(), vec![pin(true, false), pin(false, true)])],
            sboxes: vec![(false, wire(0, r(300, 100, 400, 200)))],
            wire: vec![wire(0, r(500, 100, 600, 200))],
        }];
        let obs = |except_pg, inst: Option<&str>| BlockObstruction { except_pg, slot: false, fill: false, layer: Some(0), rect: r(700, 100, 800, 200), inst: inst.map(String::from) };
        si.obstructions = vec![obs(true, None), obs(false, Some("u1")), obs(false, None)];
        si.fills = vec![FillGeom { layer: Some(0), rect: r(900, 100, 950, 200), mask: 0 }, FillGeom { layer: Some(0), rect: r(950, 100, 990, 200), mask: 2 }];
        let to = |placed| check_short(&nw, &si, placed).unwrap().0.into_iter().map(|s| s.to).collect::<Vec<_>>();
        let tail = ["net A", "net A", "obstruction of u1", "obstruction", "fill", "fill with mask 2"];
        let mut want = vec!["terminal a (net A)"];
        want.extend(tail);
        assert_eq!(to(true), want);
        assert_eq!(to(false), want);
        // What each marker names besides the checked net: a block pin its terminal and that
        // terminal's net, an obstruction itself (by position, the skipped ones counted) and its
        // instance, a fill nothing.
        let sources: Vec<Vec<Source>> = check_short(&nw, &si, true).unwrap().0.into_iter().map(|s| s.sources).collect();
        let net = Source::Net("A".into());
        assert_eq!(
            sources,
            vec![
                vec![Source::BTerm("a".into()), net.clone()],
                vec![net.clone()],
                vec![net],
                vec![Source::Obstruction(1), Source::Inst("u1".into())],
                vec![Source::Obstruction(2)],
                vec![],
                vec![],
            ]
        );
    }

    // A full list stops the walk: the shorts kept are the first MAX_SHORT_ENTRIES, and the stop
    // notice prints before them.
    #[test]
    fn the_walk_stops_at_the_cap() {
        let nw = network();
        let mut si = inputs();
        si.nets = vec![other("A", (0..MAX_SHORT_ENTRIES as i32 + 1).map(|i| wire(0, r(i % 9000, 0, i % 9000 + 1, 1))).collect())];
        let (shorts, stopped) = check_short(&nw, &si, true).unwrap();
        assert!(stopped);
        assert_eq!(shorts.len(), MAX_SHORT_ENTRIES);
        let lines = report_shorts(&nw, "VDD", &shorts[..1], stopped);
        assert!(lines[0].starts_with("[WARNING PSM-0044] Stopped checking VDD after 1 shorted"));
        si.nets[0].wire.pop();
        assert!(!check_short(&nw, &si, true).unwrap().1);
    }

    // Placement orientations, then the offset: R90 is (x, y) -> (-y, x), MX mirrors y, MXR90
    // swaps the axes.
    #[test]
    fn instance_transforms() {
        let a = r(10, 20, 30, 60);
        assert_eq!(transform(0, (5, 5), &a), r(15, 25, 35, 65));
        assert_eq!(transform(1, (0, 0), &a), r(-60, 10, -20, 30));
        assert_eq!(transform(2, (0, 0), &a), r(-30, -60, -10, -20));
        assert_eq!(transform(3, (0, 0), &a), r(20, -30, 60, -10));
        assert_eq!(transform(4, (0, 0), &a), r(-30, 20, -10, 60));
        assert_eq!(transform(5, (0, 0), &a), r(-60, -30, -20, -10));
        assert_eq!(transform(6, (0, 0), &a), r(10, -60, 30, -20));
        assert_eq!(transform(7, (0, 0), &a), r(20, 10, 60, 30));
    }

    // Pieces come by their right x, then the low y of their edge on that line.
    #[test]
    fn pieces_come_by_right_edge_then_its_low_y() {
        let rs = [r(0, 0, 10, 10), r(0, 20, 30, 30), r(15, 0, 30, 5), r(40, 0, 50, 50), r(20, 40, 30, 50)];
        let order: Vec<Rect> = pieces(&rs).iter().map(|p| bbox(p)).collect();
        assert_eq!(order, vec![r(0, 0, 10, 10), r(15, 0, 30, 5), r(0, 20, 30, 30), r(20, 40, 30, 50), r(40, 0, 50, 50)]);
    }

    fn master(boundary: Rect, mterms: Vec<Vec<BoxGeom>>, obstructions: Vec<BoxGeom>) -> MasterGeom {
        let mterms = mterms.into_iter().map(|boxes| TermGeom { boxes, polygons: vec![] }).collect();
        MasterGeom { boundary, mterms, obstructions, obstruction_polygons: vec![] }
    }

    fn inst(iterms: Vec<ITermGeom>) -> InstGeom {
        InstGeom { name: "u1".into(), placed: true, fixed: false, master: 0, orient: 0, offset: (1000, 0), iterms }
    }

    fn iterm(name: &str, on_net: bool, net: Option<&str>, sig: i64, mterm: usize) -> ITermGeom {
        ITermGeom { name: name.into(), on_net, net: net.map(String::from), sig, mterm }
    }

    // Instance pins: a pin on the net, or an unconnected pin of the net's own type (joined by
    // abutment), is no short; an unconnected pin of another type is.
    #[test]
    fn instance_pins_skip_the_net_and_abutted_supplies() {
        let nw = network();
        let mut si = inputs();
        let pin = vec![wire(0, r(0, 0, 100, 100))];
        si.masters = vec![master(r(0, 0, 500, 500), vec![pin.clone(), pin.clone(), pin.clone(), pin], vec![])];
        si.insts = vec![inst(vec![iterm("u1/VDD", true, Some("VDD"), 1, 0), iterm("u1/VPB", false, None, 1, 1), iterm("u1/VNB", false, None, 2, 2), iterm("u1/A", false, Some("n1"), 0, 3)])];
        let to: Vec<String> = check_short(&nw, &si, true).unwrap().0.into_iter().map(|s| s.to).collect();
        assert_eq!(to, vec!["terminal u1/VNB (net unconnected)", "terminal u1/A (net n1)"]);
    }

    // Master obstructions are clipped to the cell and lose the pins grown by one unit: an
    // obstruction across a pin the cell's height and past the cell's edges leaves two pieces, each
    // ending one unit clear of the pin and at the cell edge.
    #[test]
    fn master_obstructions_are_clipped_and_lose_the_grown_pins() {
        let nw = network();
        let mut si = inputs();
        si.masters = vec![master(r(0, 0, 500, 2000), vec![vec![wire(0, r(100, 0, 200, 2000))]], vec![wire(0, r(-300, 0, 800, 2000))])];
        si.insts = vec![inst(vec![iterm("u1/A", false, Some("n1"), 0, 0)])];
        let s: Vec<(Rect, String)> = check_short(&nw, &si, true).unwrap().0.into_iter().map(|s| (s.bbox, s.to)).collect();
        assert_eq!(
            s,
            vec![
                (r(1100, 0, 1200, 1000), "terminal u1/A (net n1)".to_string()),
                (r(1000, 0, 1099, 1000), "instance u1".to_string()),
                (r(1201, 0, 1500, 1000), "instance u1".to_string()),
            ]
        );
    }

    /// A 1000 x 1000 square with 300 cut off each corner at 45 degrees.
    fn octagon() -> Vec<(i32, i32)> {
        vec![(0, 300), (300, 0), (700, 0), (1000, 300), (1000, 700), (700, 1000), (300, 1000), (0, 700)]
    }

    fn polygon_master(polygons: Vec<PolygonGeom>, boxes: Vec<BoxGeom>) -> MasterGeom {
        MasterGeom { boundary: r(0, 0, 1000, 1000), mterms: vec![TermGeom { boxes, polygons }], obstructions: vec![], obstruction_polygons: vec![] }
    }

    // A pin that is one convex polygon off the Manhattan grid is cut to each shape exactly: an
    // octagon half over the strap is reported by the bounding box of its part on the strap.
    #[test]
    fn an_octagonal_pin_is_cut_to_the_shape() {
        let nw = network();
        let mut si = inputs();
        si.masters = vec![polygon_master(vec![PolygonGeom { layer: Some(0), pts: octagon() }], vec![])];
        si.insts = vec![InstGeom { offset: (1000, 500), ..inst(vec![iterm("pad/PAD", false, Some("out"), 0, 0)]) }];
        let s: Vec<(Rect, bool)> = check_short(&nw, &si, true).unwrap().0.into_iter().map(|s| (s.bbox, s.rect)).collect();
        // Not a rectangle: the cut corners lie on the strap.
        assert_eq!(s, vec![(r(1000, 500, 2000, 1000), false)]);
        // The strap's corner over the octagon's cut corner: a 45 degree edge from (200, 100) to
        // (300, 0) bounds the part on the strap, so its box starts at x 200, not at the cell edge.
        si.insts[0].offset = (9500, 900);
        let s: Vec<(Rect, bool)> = check_short(&nw, &si, true).unwrap().0.into_iter().map(|s| (s.bbox, s.rect)).collect();
        assert_eq!(s, vec![(r(9700, 900, 10000, 1000), false)]);
        // Across the octagon's straight middle the overlap IS its box; touching it is nothing.
        assert_eq!(clip_convex(&octagon(), &r(-50, 400, 1050, 600)), Ok(Some((r(0, 400, 1000, 600), true))));
        assert_eq!(clip_convex(&octagon(), &r(1000, 400, 1100, 600)), Ok(None));
    }

    // Such a polygon is only refused where it is measured: on a pin of the net (skipped) it is
    // nothing; mixed with other metal on its layer, or as an obstruction, it refuses. So does an
    // octilinear special wire.
    #[test]
    fn an_off_grid_polygon_refuses_only_where_measured() {
        let nw = network();
        let oct = PolygonGeom { layer: Some(0), pts: octagon() };
        let mut si = inputs();
        si.masters = vec![polygon_master(vec![oct.clone()], vec![])];
        si.insts = vec![inst(vec![iterm("bump/VDD", true, Some("VDD"), 1, 0)])];
        assert!(check_short(&nw, &si, true).is_ok());
        si.masters = vec![polygon_master(vec![oct.clone()], vec![wire(0, r(0, 0, 10, 10))])];
        si.insts = vec![inst(vec![iterm("pad/PAD", false, Some("out"), 0, 0)])];
        assert!(check_short(&nw, &si, true).is_err());
        si.masters = vec![MasterGeom { obstruction_polygons: vec![oct], ..polygon_master(vec![], vec![]) }];
        assert!(check_short(&nw, &si, true).is_err());
        let mut si = inputs();
        si.nets = vec![OtherNet { name: "A".into(), bterms: vec![], sboxes: vec![(true, wire(0, r(0, 0, 1, 1)))], wire: vec![] }];
        assert!(check_short(&nw, &si, true).is_err());
    }

    // A Manhattan polygon is its slices: an L pin polygon shorts as the same L built of boxes.
    #[test]
    fn a_manhattan_polygon_is_its_slices() {
        let l = vec![(0, 0), (600, 0), (600, 200), (200, 200), (200, 1000), (0, 1000)];
        assert_eq!(manhattan_rects(&l).unwrap(), region(&[r(0, 0, 600, 200), r(0, 200, 200, 1000)]));
        assert_eq!(manhattan_rects(&octagon()), None);
        let nw = network();
        let mut si = inputs();
        si.masters = vec![polygon_master(vec![PolygonGeom { layer: Some(0), pts: l }], vec![])];
        si.insts = vec![InstGeom { offset: (1000, 900), ..inst(vec![iterm("u1/A", false, Some("n1"), 0, 0)]) }];
        let s: Vec<(Rect, bool)> = check_short(&nw, &si, true).unwrap().0.into_iter().map(|s| (s.bbox, s.rect)).collect();
        assert_eq!(s, vec![(r(1000, 900, 1600, 1000), true)]);
        // Lower, the L's corner is on the strap: one piece, not its box.
        si.insts[0].offset = (1000, 700);
        let s: Vec<(Rect, bool)> = check_short(&nw, &si, true).unwrap().0.into_iter().map(|s| (s.bbox, s.rect)).collect();
        assert_eq!(s, vec![(r(1000, 700, 1600, 1000), false)]);
    }
}
