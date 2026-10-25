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
use vyges_drt::rtree::{PackedRTree, StdLib};

use crate::input::{Dir, Geom, NetInput, Rect, SBox, Tech, ViaShape};

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

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Shape {
    pub rect: Rect,
    pub li: usize,
    pub id: usize,
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

/// An R-tree packed as the reference packs one: it is built against libstdc++, whose
/// `nth_element` permutes the packing's slices and whose heap orders a nearest query's pending
/// branches, so two nodes at the same distance are told apart exactly as there.
pub fn ref_tree<T>(values: Vec<(Rect, T)>) -> PackedRTree<T> {
    PackedRTree::new_in(values, StdLib::LibStdCxx)
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
    pub fn build(tech: &'a Tech, net: &NetInput, floorplanning: bool, dump: Option<DumpFn<'a>>) -> Result<(Network<'a>, BuildLog), String> {
        if !net.supply {
            return Err(format!("{} is not a supply net.", net.name));
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
        let log = n.construct(net);
        Ok((n, log))
    }

    fn construct(&mut self, net: &NetInput) -> BuildLog {
        self.generate_routing_layer_shapes_and_nodes(net);
        self.generate_cut_layer_nodes(net);
        if !self.has_nodes() {
            return BuildLog { empty: true };
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
        BuildLog::default()
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

    fn polygons_from_swire(&self, wire: &[SBox], into: &mut BTreeMap<LayerKey, Polygon90Set>) {
        for b in wire {
            let g = match b {
                SBox::Wire { layer, rect } => Geom { is_via: false, layer: Some(*layer), rect: *rect, via_shapes: vec![] },
                SBox::Via { rect, shapes, .. } => Geom { is_via: true, layer: None, rect: *rect, via_shapes: shapes.clone() },
            };
            for (k, rs) in self.polygons_from_box(&g) {
                let set = into.entry(k).or_default();
                rs.into_iter().for_each(|r| set.insert_rect(r));
            }
        }
    }

    /// Instance terminals: a base node per terminal at its pins' average point, on the side its
    /// pins are on, and a terminal node per pin shape linked to it. Floorplanning skips any
    /// instance not FIXED; if none was skipped, floorplanning is turned off.
    fn polygons_from_iterms(&mut self, net: &NetInput, into: &mut BTreeMap<LayerKey, Polygon90Set>) {
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
            for g in &it.geoms {
                let pin_shapes = self.polygons_from_box(g);
                if pin_shapes.is_empty() {
                    continue;
                }
                for (k, rs) in &pin_shapes {
                    let set = into.entry(*k).or_default();
                    rs.iter().for_each(|r| set.insert_rect(*r));
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
            if has_routing_term {
                self.iterm_nodes.push(base);
            }
        }
        if self.floorplanning && !floorplan_asserted {
            self.floorplanning = false;
        }
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
    fn polygons_from_bterms(&mut self, net: &NetInput, into: &mut BTreeMap<LayerKey, Polygon90Set>) {
        for bt in &net.bterms {
            for pin in &bt.pins {
                for b in &pin.boxes {
                    for (k, rs) in self.polygons_from_box(&b.geom) {
                        let set = into.entry(k).or_default();
                        rs.into_iter().for_each(|r| set.insert_rect(r));
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
            new_shapes.push(Shape { rect: *rect, li, id: 0 });
        }
    }

    fn generate_routing_layer_shapes_and_nodes(&mut self, net: &NetInput) {
        let mut shapes_by_layer: BTreeMap<LayerKey, Polygon90Set> = BTreeMap::new();
        for wire in &net.swires {
            self.polygons_from_swire(wire, &mut shapes_by_layer);
        }
        self.polygons_from_iterms(net, &mut shapes_by_layer);
        self.polygons_from_bterms(net, &mut shapes_by_layer);
        let mut all_poly_shapes = Vec::new();
        for (k, mut set) in shapes_by_layer {
            for p in set.polygons() {
                all_poly_shapes.push((k.li, p));
            }
        }
        let (mut poly_shapes, mut poly_nodes) = (Vec::new(), Vec::new());
        for (li, p) in all_poly_shapes {
            self.process_polygon_to_rectangles(li, p, &mut poly_shapes, &mut poly_nodes);
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
            out.push(self.new_node(start, s.li, NodeKind::Node, None));
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
            let tree = ref_tree(shapes.iter().map(|s| (s.rect, ())).collect());
            for &n in nodes {
                if tree.query(&point_rect(self.arena[n].pt)).len() > 1 {
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
        let sorted = self.nodes_on(tree, &s.rect);
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
        let sorted = self.nodes_on(layer_tree, &s.rect);
        let tree = ref_tree(sorted.iter().map(|&n| (point_rect(self.arena[n].pt), n)).collect());
        let mut used: HashSet<NodeId> = HashSet::new();
        let mut out = Vec::new();
        for &n in &sorted {
            let pt = self.arena[n].pt;
            used.insert(n);
            let Some(&(_, other)) = tree.nearest_one(pt, |v| !used.contains(v)) else { continue };
            let op = self.arena[other].pt;
            let (len_x, len_y) = ((op.0 - pt.0).abs(), (op.1 - pt.1).abs());
            let (length, width) = if len_x > len_y { (len_x, s.rect.dy()) } else { (len_y, s.rect.dx()) };
            out.push(self.new_conn(n, other, ConnKind::Layer { length, width }));
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
        assert_eq!(Network::build(t, &n, false, None).err().unwrap(), "VDD is not a supply net.");
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
        let s = Shape { rect: r(0, 0, 100, 6000), li: 2, id: 0 };
        let mut nw = nw;
        let made: Vec<(i32, i32)> = nw.create_filler_nodes(&s, 2000).iter().map(|&i| nw.arena[i].pt).collect();
        assert_eq!(made, vec![(50, 1000), (50, 3000), (50, 5000)]);
        assert!(nw.create_filler_nodes(&Shape { rect: r(0, 0, 100, 2000), li: 2, id: 0 }, 2000).iter().all(|&i| nw.arena[i].pt.1 < 2000));
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
