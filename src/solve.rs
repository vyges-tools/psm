// SPDX-License-Identifier: Apache-2.0
//! The static IR-drop solve on a built network: layer resistances, the conductance of every
//! connection, the voltage sources, the instance currents, the conductance matrix, and the solution.
//!
//! Each function is one step of the solve, in the order the solve runs them.

use std::cmp::Ordering;
use std::collections::{BTreeMap, BTreeSet, HashMap};

use vyges_drt::rtree::PackedRTree;

use crate::input::{Dir, NetInput, Rect};
use crate::network::{ref_tree, ConnId, ConnKind, LayerKey, Network, NodeId, NodeKind};

/// A tech layer's kind, as far as resistance goes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LayerType {
    Routing,
    Cut,
    Other,
}

/// What the solve reads from a tech layer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LayerElec {
    pub kind: LayerType,
    pub width: i32,
    /// The database resistance: ohms per square on a routing layer, per cut on a cut layer.
    pub resistance: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SigType {
    Power,
    Ground,
    Other,
}

/// The solve's database inputs besides the network.
#[derive(Debug, Clone, PartialEq)]
pub struct SolveInputs {
    pub die: Rect,
    pub core: Rect,
    /// One per tech layer, in the tech's layer order (the network's layer index).
    pub layers: Vec<LayerElec>,
    pub sig: SigType,
    /// Per instance terminal of the net, in the net's order: its instance's id.
    pub iterm_inst: Vec<i32>,
}

/// Decode the `psm_solve_inputs` stream (layout at the opendb shim).
pub fn decode_solve_inputs(v: &[f64]) -> Result<SolveInputs, String> {
    let mut at = 0;
    let mut next = || -> Result<f64, String> {
        let x = *v.get(at).ok_or("the solve-input stream ends early")?;
        at += 1;
        Ok(x)
    };
    let rect = |next: &mut dyn FnMut() -> Result<f64, String>| -> Result<Rect, String> {
        Ok(Rect { xl: next()? as i32, yl: next()? as i32, xh: next()? as i32, yh: next()? as i32 })
    };
    let die = rect(&mut next)?;
    let core = rect(&mut next)?;
    let n = next()? as usize;
    let mut layers = Vec::with_capacity(n);
    for _ in 0..n {
        let kind = match next()? as i32 {
            1 => LayerType::Routing,
            2 => LayerType::Cut,
            _ => LayerType::Other,
        };
        let width = next()? as i32;
        let resistance = next()?;
        layers.push(LayerElec { kind, width, resistance });
    }
    let sig = match next()? as i32 {
        1 => SigType::Power,
        2 => SigType::Ground,
        _ => SigType::Other,
    };
    let n = next()? as usize;
    let iterm_inst = (0..n).map(|_| next().map(|x| x as i32)).collect::<Result<_, _>>()?;
    Ok(SolveInputs { die, core, layers, sig, iterm_inst })
}

/// C++ `{:.Ne}`: a mantissa with `prec` decimals and an exponent of at least two digits with its
/// sign (`3.522585992540406e-01`).
pub fn fmt_e(v: f64, prec: usize) -> String {
    let s = format!("{v:.prec$e}");
    let Some((mant, exp)) = s.split_once('e') else { return s };
    let (sign, digits) = match exp.strip_prefix('-') {
        Some(d) => ('-', d),
        None => ('+', exp),
    };
    format!("{mant}e{sign}{digits:0>2}")
}

// ---- resistance ------------------------------------------------------------------------------

/// The timer's resistance and distance units, from the first library read: `pulling_resistance_unit`
/// (1 ohm when absent) and `distance_unit` (1 micron), each a `float`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RcUnits {
    pub resistance: f32,
    pub distance: f32,
}

impl Default for RcUnits {
    fn default() -> Self {
        RcUnits { resistance: 1.0, distance: 1e-6 }
    }
}

/// A liberty unit `<1|10|100><scale><suffix>`; an unknown multiplier or scale stays 1.
pub fn read_unit(value: Option<&str>, suffix: &str, default: f32) -> f32 {
    let Some(units) = value.filter(|v| !v.is_empty()) else { return default };
    let mult_end = units.find(|c: char| !c.is_ascii_digit());
    let (mult, scale_suffix) = match mult_end {
        Some(k) => (match &units[..k] { "1" => 1.0f32, "10" => 10.0, "100" => 100.0, _ => 1.0 }, &units[k..]),
        None => (1.0, units),
    };
    let mut scale_mult = 1.0f32;
    if scale_suffix.len() == suffix.len() + 1 && scale_suffix[1..].eq_ignore_ascii_case(suffix) {
        scale_mult = match scale_suffix.as_bytes()[0].to_ascii_lowercase() {
            b'k' => 1e3,
            b'm' => 1e-3,
            b'u' => 1e-6,
            b'n' => 1e-9,
            b'p' => 1e-12,
            b'f' => 1e-15,
            _ => 1.0,
        };
    }
    scale_mult * mult
}

/// The parasitics estimate's per-layer resistance on one corner, from the session's layer-RC
/// settings in order (`-layer L -resistance R`: ohms per metre; `-via V -resistance R`: ohms per
/// cut; without `-corner` every corner). Stored in single precision, as the estimate keeps them.
/// `Err` names a setting this cannot read.
pub fn layer_rc_resistance(rows: &[Vec<String>], corner: &str, units: RcUnits, layer_index: impl Fn(&str) -> Option<usize>) -> Result<HashMap<usize, f64>, String> {
    let mut out = HashMap::new();
    for row in rows {
        let mut k: HashMap<&str, &str> = HashMap::new();
        let mut i = 0;
        while i + 1 < row.len() {
            k.insert(row[i].as_str(), row[i + 1].as_str());
            i += 2;
        }
        if k.get("-corner").is_some_and(|c| *c != corner) {
            continue;
        }
        let Some(r) = k.get("-resistance") else { continue };
        let r: f64 = r.trim().parse().map_err(|_| format!("layer RC resistance `{r}`"))?;
        let (name, res) = if let Some(l) = k.get("-layer") {
            (l, (r * f64::from(units.resistance)) / (1.0 * f64::from(units.distance)))
        } else if let Some(v) = k.get("-via") {
            (v, r * f64::from(units.resistance))
        } else {
            return Err("a layer RC setting without -layer or -via".into());
        };
        let li = layer_index(name).ok_or_else(|| format!("layer RC on unknown layer {name}"))?;
        out.insert(li, f64::from(res as f32));
    }
    Ok(out)
}

/// Per tech layer, its resistance: a routing layer's per square (the estimate's ohms per metre
/// times the layer's width in metres), a cut layer's per cut (the estimate's); where the estimate
/// is zero, the database resistance.
pub fn resistance_map(inputs: &SolveInputs, dbus: i32, estimate: &HashMap<usize, f64>) -> Vec<f64> {
    inputs
        .layers
        .iter()
        .enumerate()
        .map(|(li, l)| {
            let est = estimate.get(&li).copied().unwrap_or(0.0);
            let res = match l.kind {
                LayerType::Routing => {
                    let width_meter = f64::from(l.width) / f64::from(dbus) * 1e-6;
                    est * width_meter
                }
                LayerType::Cut => est,
                LayerType::Other => 0.0,
            };
            if res == 0.0 {
                l.resistance
            } else {
                res
            }
        })
        .collect()
}

/// The layers the network uses: every layer with nodes, and the cut layer above the lower node of
/// every via.
pub fn required_layers(nw: &Network) -> BTreeSet<LayerKey> {
    let mut out: BTreeSet<LayerKey> = nw.nodes.keys().copied().collect();
    for &c in &nw.connections {
        let conn = &nw.conns[c];
        if let (ConnKind::Via { .. }, Some(n0)) = (conn.kind, conn.n0) {
            if let Some(li) = upper_layer(nw, nw.arena[n0].li) {
                out.insert(LayerKey { id: nw.tech.layers[li].id, li });
            }
        }
    }
    out
}

fn upper_layer(nw: &Network, li: usize) -> Option<usize> {
    nw.tech.layers[li].upper.and_then(|n| nw.tech.by_number(n))
}

/// Every required routing or cut layer with zero resistance is a warning, in layer-id order; any
/// one is then an error. `Err` holds the lines, the error last.
pub fn assert_resistance_map(nw: &Network, inputs: &SolveInputs, res: &[f64]) -> Result<(), Vec<String>> {
    let mut lines = Vec::new();
    for k in required_layers(nw) {
        let kind = inputs.layers[k.li].kind;
        if res[k.li] == 0.0 && matches!(kind, LayerType::Routing | LayerType::Cut) {
            lines.push(format!("[WARNING PSM-0020] {} has zero resistance.", nw.tech.layers[k.li].name));
        }
    }
    if lines.is_empty() {
        return Ok(());
    }
    lines.push("[ERROR PSM-0021] Resistance map constains invalid values.".into());
    Err(lines)
}

/// Terminal and bridge connections are a fixed milliohm.
const TERM_RESISTANCE: f64 = 0.001;

/// A connection's resistance: squares times the sheet resistance of its lower node's layer, the cut
/// layer's resistance shared among its cuts, or the fixed terminal value.
pub fn connection_resistance(nw: &Network, c: ConnId, res: &[f64]) -> f64 {
    let conn = &nw.conns[c];
    let li0 = conn.n0.map(|n| nw.arena[n].li);
    match conn.kind {
        ConnKind::Layer { length, width } => {
            let squares = f64::from(length) / f64::from(width);
            squares * li0.map_or(0.0, |li| res[li])
        }
        ConnKind::Via { cuts } => {
            let via = li0.and_then(|li| upper_layer(nw, li));
            via.map_or(0.0, |li| res[li]) / f64::from(cuts)
        }
        ConnKind::Term | ConnKind::Bridge => TERM_RESISTANCE,
    }
}

/// The conductance of every connection, keyed in connection order. Two connections that order as
/// equal are one key: the first one's, holding the last one's value.
pub fn conductance_map(nw: &Network, conns: &[ConnId], res: &[f64]) -> Vec<(ConnId, f64)> {
    let mut v: Vec<(ConnId, f64)> = conns.iter().map(|&c| (c, 1.0 / connection_resistance(nw, c, res))).collect();
    v.sort_by(|a, b| nw.conn_order(a.0, b.0));
    let mut out: Vec<(ConnId, f64)> = Vec::with_capacity(v.len());
    for (c, g) in v {
        match out.last_mut() {
            Some(last) if nw.conn_order(last.0, c) == Ordering::Equal => last.1 = g,
            _ => out.push((c, g)),
        }
    }
    out
}

/// The conductance dump: one line per key, its two nodes and the value.
pub fn conductance_lines(nw: &Network, net: &NetInput, map: &[(ConnId, f64)]) -> Vec<String> {
    map.iter()
        .map(|&(c, g)| {
            let conn = &nw.conns[c];
            let d = |n: Option<usize>| n.map_or_else(String::new, |n| nw.describe_full(n, net));
            format!("{} -> {}: {}", d(conn.n0), d(conn.n1), fmt_e(g, 15))
        })
        .collect()
}

// ---- sources ---------------------------------------------------------------------------------

/// Where generated sources go when the net has no block-terminal pins.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceType {
    Full,
    Straps,
    Bumps,
}

impl SourceType {
    pub fn parse(s: &str) -> Option<SourceType> {
        match s {
            "FULL" => Some(SourceType::Full),
            "STRAPS" => Some(SourceType::Straps),
            "BUMPS" => Some(SourceType::Bumps),
            _ => None,
        }
    }
}

/// The generated-source settings of the session.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SourceSettings {
    pub bump_dx: i32,
    pub bump_dy: i32,
    pub bump_size: i32,
    pub bump_interval: i32,
    pub strap_track_pitch: i32,
    /// Ohms between each source and the grid; 0 is none.
    pub resistance: f32,
}

impl Default for SourceSettings {
    fn default() -> Self {
        SourceSettings { bump_dx: 140, bump_dy: 140, bump_size: 70, bump_interval: 3, strap_track_pitch: 10, resistance: 0.0 }
    }
}

impl SourceSettings {
    /// One settings command: each value given and positive replaces the current one; a zero (the
    /// value of an option not given) or a negative leaves it.
    pub fn apply(&mut self, given: &SourceSettings) {
        if given.bump_dx > 0 {
            self.bump_dx = given.bump_dx;
        }
        if given.bump_dy > 0 {
            self.bump_dy = given.bump_dy;
        }
        if given.bump_interval > 0 {
            self.bump_interval = given.bump_interval;
        }
        if given.bump_size > 0 {
            self.bump_size = given.bump_size;
        }
        if given.strap_track_pitch > 0 {
            self.strap_track_pitch = given.strap_track_pitch;
        }
        if given.resistance > 0.0 {
            self.resistance = given.resistance;
        }
    }
}

/// A node of the solve's matrix: a network node, or a source node at the same point as the node it
/// stands for (`SrcNode::source`).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum MNode {
    Net(NodeId),
    Src(usize),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SrcNode {
    pub pt: (i32, i32),
    pub li: usize,
    pub source: MNode,
}

/// The source nodes the solve made, and the matrix-node order over them and the network.
#[derive(Debug, Default)]
pub struct Sources {
    pub nodes: Vec<SrcNode>,
}

impl Sources {
    fn add(&mut self, nw: &Network, source: MNode) -> usize {
        let (pt, li) = self.at(nw, source);
        self.nodes.push(SrcNode { pt, li, source });
        self.nodes.len() - 1
    }

    fn at(&self, nw: &Network, n: MNode) -> ((i32, i32), usize) {
        match n {
            MNode::Net(id) => (nw.arena[id].pt, nw.arena[id].li),
            MNode::Src(i) => (self.nodes[i].pt, self.nodes[i].li),
        }
    }

    /// Node order over network and source nodes: layer number, x, y, then kind (node, source,
    /// iterm, bpin) and the terminal's id.
    pub fn compare(&self, nw: &Network, a: MNode, b: MNode) -> Ordering {
        let key = |n: MNode| {
            let (pt, li) = self.at(nw, n);
            let rank = match n {
                MNode::Src(_) => (1, 0),
                MNode::Net(id) => match nw.arena[id].kind {
                    NodeKind::Node => (0, 0),
                    NodeKind::ITerm { id, .. } => (2, id),
                    NodeKind::BPin { id, .. } => (3, id),
                },
            };
            (nw.tech.layers[li].number, pt.0, pt.1, rank)
        };
        key(a).cmp(&key(b))
    }

    /// A matrix node as the dumps print it.
    pub fn describe(&self, nw: &Network, net: &NetInput, n: MNode) -> String {
        match n {
            MNode::Net(id) => nw.describe_full(id, net),
            MNode::Src(i) => {
                let s = &self.nodes[i];
                let dbus = f64::from(nw.tech.dbus);
                format!(
                    "Source Node: ({:.4}, {:.4}) on {}",
                    f64::from(s.pt.0) / dbus,
                    f64::from(s.pt.1) / dbus,
                    nw.tech.layers[s.li].name
                )
            }
        }
    }
}

/// What the source generation printed and found.
#[derive(Debug, Default)]
pub struct SourceLog {
    pub lines: Vec<String>,
}

fn rect_key(r: &Rect) -> (i32, i32, i32, i32) {
    (r.xl, r.yl, r.xh, r.yh)
}

fn rect_of(k: (i32, i32, i32, i32)) -> Rect {
    Rect { xl: k.0, yl: k.1, xh: k.2, yh: k.3 }
}

fn touches(a: &Rect, b: &Rect) -> bool {
    b.xh >= a.xl && b.xl <= a.xh && b.yh >= a.yl && b.yl <= a.yh
}

fn node_tree(nw: &Network, k: LayerKey) -> PackedRTree<NodeId> {
    let ids = nw.nodes.get(&k).map(Vec::as_slice).unwrap_or(&[]);
    ref_tree(ids.iter().map(|&n| (Rect { xl: nw.arena[n].pt.0, yl: nw.arena[n].pt.1, xh: nw.arena[n].pt.0, yh: nw.arena[n].pt.1 }, n)).collect())
}

/// The sources of the net: from the source file when one is given, else on its block-terminal
/// pins, else generated in the requested pattern. Also the source voltage: the file's, else the
/// net's. `Err` holds the lines of a failure, the error last.
#[allow(clippy::too_many_arguments)]
pub fn generate_source_nodes(
    nw: &Network,
    inputs: &SolveInputs,
    settings: &SourceSettings,
    kind: SourceType,
    source_file: Option<&str>,
    net_voltage: f64,
    srcs: &mut Sources,
    log: &mut SourceLog,
) -> Result<(Vec<usize>, f64), Vec<String>> {
    let mut src_voltage = net_voltage;
    let sources = if let Some(file) = source_file {
        let (v, s) = generate_source_nodes_from_source_file(nw, file, srcs, log)?;
        src_voltage = v;
        s
    } else {
        let mut s = generate_source_nodes_from_bterms(nw, srcs);
        if s.is_empty() {
            s = match kind {
                SourceType::Full => generate_source_nodes_generic_full(nw, srcs, log),
                SourceType::Straps => generate_source_nodes_generic_straps(nw, inputs, settings, srcs, log)?,
                SourceType::Bumps => generate_source_nodes_generic_bumps(nw, inputs, settings, srcs, log),
            };
        }
        s
    };
    if sources.is_empty() {
        let mut lines = std::mem::take(&mut log.lines);
        lines.push("[ERROR PSM-0070] Unable to map source nodes into power grid".into());
        return Err(lines);
    }
    Ok((sources, src_voltage))
}

/// One source per network node under a connectable block-terminal pin shape: per layer (in
/// layer-id order) the distinct pin shapes, each matched against the layer's nodes; node order.
pub fn generate_source_nodes_from_bterms(nw: &Network, srcs: &mut Sources) -> Vec<usize> {
    let mut by_layer: BTreeMap<LayerKey, BTreeSet<(i32, i32, i32, i32)>> = BTreeMap::new();
    for &b in &nw.bpin_nodes {
        let node = &nw.arena[b];
        if let (NodeKind::BPin { disconnect: false, .. }, Some(shape)) = (node.kind, node.shape) {
            by_layer.entry(LayerKey { id: nw.tech.layers[node.li].id, li: node.li }).or_default().insert(rect_key(&shape));
        }
    }
    let mut found: Vec<NodeId> = Vec::new();
    for (k, shapes) in by_layer {
        let tree = node_tree(nw, k);
        for s in shapes {
            found.extend(tree.query(&rect_of(s)).into_iter().map(|v| v.1));
        }
    }
    found.sort_by(|a, b| nw.compare(*a, *b));
    found.dedup_by(|a, b| nw.compare(*a, *b) == Ordering::Equal);
    found.into_iter().map(|n| srcs.add(nw, MNode::Net(n))).collect()
}

/// Every top-layer node is a source.
pub fn generate_source_nodes_generic_full(nw: &Network, srcs: &mut Sources, log: &mut SourceLog) -> Vec<usize> {
    let Some(top) = nw.top_layer() else { return Vec::new() };
    log.lines.push(format!("[INFO PSM-0071] Using all nodes on {} as sources.", nw.tech.layers[top.li].name));
    nw.nodes[&top].iter().map(|&n| srcs.add(nw, MNode::Net(n))).collect()
}

/// Straps on the top layer across the die, every `strap_track_pitch` tracks of the routing layer
/// just outside it (pitch and width from that layer), from half a pitch into the core.
pub fn generate_source_nodes_generic_straps(
    nw: &Network,
    inputs: &SolveInputs,
    settings: &SourceSettings,
    srcs: &mut Sources,
    log: &mut SourceLog,
) -> Result<Vec<usize>, Vec<String>> {
    let Some(top) = nw.top_layer() else { return Ok(Vec::new()) };
    let top_layer = &nw.tech.layers[top.li];
    let connect_level = top_layer.routing_level + if top_layer.backside { -1 } else { 1 };
    let connect = if connect_level > 0 { nw.tech.layers.iter().position(|l| l.routing_level == connect_level) } else { None };
    let connect = connect.unwrap_or(top.li);
    let pitch = settings.strap_track_pitch.wrapping_mul(nw.tech.layers[connect].pitch);
    if pitch <= 0 {
        let mut lines = std::mem::take(&mut log.lines);
        lines.push("[ERROR PSM-0076] Invalid pitch for strap generation.".into());
        return Err(lines);
    }
    let offset = pitch / 2;
    let width = inputs.layers[connect].width;
    let dbus = f64::from(nw.tech.dbus);
    log.lines.push(format!(
        "[INFO PSM-0072] Using strap pattern on {} with {} pitch {:.4}um and offset {:.4}um.",
        top_layer.name,
        nw.tech.layers[connect].name,
        f64::from(pitch) / dbus,
        f64::from(offset) / dbus
    ));
    let (core, die) = (inputs.core, inputs.die);
    let mut straps = BTreeSet::new();
    if top_layer.dir == Dir::Horizontal {
        let mut x = core.xl + offset;
        while x < core.xh {
            straps.insert((x, die.yl, x + width, die.yh));
            x += pitch;
        }
    } else {
        let mut y = core.yl + offset;
        while y < core.yh {
            straps.insert((die.xl, y, die.xh, y + width));
            y += pitch;
        }
    }
    if straps.is_empty() {
        let shift = ((core.xl + core.xh) / 2 - width / 2, (core.yl + core.yh) / 2 - width / 2);
        if top_layer.dir == Dir::Horizontal {
            straps.insert((shift.0, die.yl, shift.0 + width, die.yh));
        } else {
            straps.insert((die.xl, shift.1, die.xh, shift.1 + width));
        }
    }
    Ok(generate_source_nodes_from_shapes(nw, &straps, srcs))
}

/// Square bumps over the die on a dx by dy grid, every `bump_interval`-th one, staggered by
/// column; the die's centre when none fits.
pub fn generate_source_nodes_generic_bumps(
    nw: &Network,
    inputs: &SolveInputs,
    settings: &SourceSettings,
    srcs: &mut Sources,
    log: &mut SourceLog,
) -> Vec<usize> {
    let dbus = f64::from(nw.tech.dbus);
    let dx = (f64::from(settings.bump_dx) * dbus) as i32;
    let dy = (f64::from(settings.bump_dy) * dbus) as i32;
    let size = (f64::from(settings.bump_size) * dbus) as i32;
    let top = nw.top_layer().map_or_else(String::new, |k| nw.tech.layers[k.li].name.clone());
    log.lines.push(format!(
        "[INFO PSM-0073] Using bump pattern on {top} with x-pitch {:.4}um, y-pitch {:.4}um, and size {:.4}um with a reduction factor of {}x.",
        f64::from(dx) / dbus,
        f64::from(dy) / dbus,
        f64::from(size) / dbus,
        settings.bump_interval
    ));
    let die = inputs.die;
    let mut bumps = BTreeSet::new();
    let mut row = 0;
    let mut x = die.xl + dx / 2;
    while x < die.xh {
        let col_offset = row % settings.bump_interval;
        let mut col = 0;
        let mut y = die.yl + dy / 2;
        while y < die.yh {
            if (col_offset + col) % settings.bump_interval == 0 {
                bumps.insert((x - size / 2, y - size / 2, x + size / 2, y + size / 2));
            }
            col += 1;
            y += dy;
        }
        row += 1;
        x += dx;
    }
    if bumps.is_empty() {
        let c = ((die.xl + die.xh) / 2, (die.yl + die.yh) / 2);
        bumps.insert((c.0 - size / 2, c.1 - size / 2, c.0 + size / 2, c.1 + size / 2));
    }
    generate_source_nodes_from_shapes(nw, &bumps, srcs)
}

/// The top-layer nodes under each source shape: a shape is first clipped to every top-layer shape
/// it touches (kept whole when it touches none); each clipped shape takes every node inside it,
/// or the node nearest its centre when none is. One source per point, in point order.
pub fn generate_source_nodes_from_shapes(nw: &Network, shapes: &BTreeSet<(i32, i32, i32, i32)>, srcs: &mut Sources) -> Vec<usize> {
    let Some(top) = nw.top_layer() else { return Vec::new() };
    let top_shapes = nw.shapes.get(&top).map(Vec::as_slice).unwrap_or(&[]);
    let mut clipped = BTreeSet::new();
    for &k in shapes {
        let shape = rect_of(k);
        let mut used = false;
        for t in top_shapes {
            if touches(&shape, &t.rect) {
                used = true;
                clipped.insert((shape.xl.max(t.rect.xl), shape.yl.max(t.rect.yl), shape.xh.min(t.rect.xh), shape.yh.min(t.rect.yh)));
            }
        }
        if !used {
            clipped.insert(k);
        }
    }
    let tree = node_tree(nw, top);
    let mut by_point: BTreeMap<(i32, i32), NodeId> = BTreeMap::new();
    for k in clipped {
        let shape = rect_of(k);
        let hits = tree.query(&shape);
        if hits.is_empty() {
            let c = ((shape.xl + shape.xh) / 2, (shape.yl + shape.yh) / 2);
            if let Some(&(_, n)) = tree.nearest_one(c, |_| true) {
                by_point.entry(nw.arena[n].pt).or_insert(n);
            }
        } else {
            for &(_, n) in hits {
                by_point.entry(nw.arena[n].pt).or_insert(n);
            }
        }
    }
    by_point.into_values().map(|n| srcs.add(nw, MNode::Net(n))).collect()
}

/// `std::stod` / `std::stof` on a decimal number: leading white space, then the longest prefix of
/// the form sign, digits, point, digits, exponent. `None` when no digit is read.
pub fn parse_leading_f64(s: &str) -> Option<f64> {
    let t = s.trim_start_matches([' ', '\t', '\n', '\r', '\x0b', '\x0c']).as_bytes();
    let mut i = 0;
    if i < t.len() && (t[i] == b'+' || t[i] == b'-') {
        i += 1;
    }
    let digits = |i: &mut usize| {
        let start = *i;
        while *i < t.len() && t[*i].is_ascii_digit() {
            *i += 1;
        }
        *i - start
    };
    let mut n = digits(&mut i);
    if i < t.len() && t[i] == b'.' {
        i += 1;
        n += digits(&mut i);
    }
    if n == 0 {
        return None;
    }
    if i < t.len() && (t[i] == b'e' || t[i] == b'E') {
        let mut j = i + 1;
        if j < t.len() && (t[j] == b'+' || t[j] == b'-') {
            j += 1;
        }
        if digits(&mut j) > 0 {
            i = j;
        }
    }
    std::str::from_utf8(&t[..i]).ok()?.parse().ok()
}

/// The source file: one `x,y,size,voltage` line per source (microns, volts); each is a square of
/// that size at that point. Every line must give the same voltage. `Err` holds the failure's lines.
pub fn generate_source_nodes_from_source_file(
    nw: &Network,
    file: &str,
    srcs: &mut Sources,
    log: &mut SourceLog,
) -> Result<(f64, Vec<usize>), Vec<String>> {
    let fail = |log: &mut SourceLog, line: String| {
        let mut lines = std::mem::take(&mut log.lines);
        lines.push(line);
        lines
    };
    let text = match std::fs::read(file) {
        Ok(t) => String::from_utf8_lossy(&t).into_owned(),
        Err(_) => return Err(fail(log, format!("[ERROR PSM-0089] Unable to open {file}."))),
    };
    log.lines.push(format!("[INFO PSM-0015] Reading location of sources from: {file}."));
    let dbus = nw.tech.dbus;
    let mut records: Vec<(i32, i32, i32, f64)> = Vec::new();
    let mut lines: Vec<&str> = text.split('\n').collect();
    if text.ends_with('\n') {
        lines.pop();
    }
    for line in lines {
        let mut values = [0.0f64; 4];
        let mut rest = Some(line);
        for (i, v) in values.iter_mut().enumerate() {
            let Some(r) = rest else {
                return Err(fail(log, format!("[ERROR PSM-0075] Expected four values on line: \"{line}\"")));
            };
            let (value, next) = match r.split_once(',') {
                Some((a, b)) => (a, Some(b)),
                None => (r, None),
            };
            rest = next;
            let Some(x) = parse_leading_f64(value) else {
                return Err(fail(log, format!("the source file value `{value}` is not a number")));
            };
            *v = if i == 3 { f64::from(x as f32) } else { x };
        }
        let d = f64::from(dbus);
        records.push(((d * values[0]) as i32, (d * values[1]) as i32, (d * values[2]) as i32, values[3]));
    }
    let Some(first) = records.first() else {
        return Err(fail(log, "the source file holds no source".into()));
    };
    let src_voltage = first.3;
    if records.iter().any(|r| r.3 != src_voltage) {
        return Err(fail(log, "[ERROR PSM-0080] Source voltage cannot be different".into()));
    }
    let shapes: BTreeSet<(i32, i32, i32, i32)> =
        records.iter().map(|&(x, y, size, _)| (x - size / 2, y - size / 2, x + size / 2, y + size / 2)).collect();
    Ok((src_voltage, generate_source_nodes_from_shapes(nw, &shapes, srcs)))
}

// ---- matrix nodes ----------------------------------------------------------------------------

/// The matrix's node order: the network nodes the conductance map touches (node order, equal
/// nodes once), then the real source nodes, then the sources (each in node order).
pub fn assign_node_ids(nw: &Network, cond: &[(ConnId, f64)], srcs: &Sources, real: &[usize], sources: &[usize]) -> Vec<MNode> {
    let mut all: Vec<MNode> = Vec::new();
    for &(c, _) in cond {
        let conn = &nw.conns[c];
        all.extend(conn.n0.into_iter().chain(conn.n1).map(MNode::Net));
    }
    let set = |mut v: Vec<MNode>| {
        v.sort_by(|a, b| srcs.compare(nw, *a, *b));
        v.dedup_by(|a, b| srcs.compare(nw, *a, *b) == Ordering::Equal);
        v
    };
    let mut out = set(all);
    out.extend(set(real.iter().map(|&i| MNode::Src(i)).collect()));
    out.extend(set(sources.iter().map(|&i| MNode::Src(i)).collect()));
    out
}

/// With a source resistance, each source found becomes a "real" source joined to its grid node
/// through that resistance, and the voltage source sits on a new node at the same point; without
/// one the sources found are the sources. (real, sources).
pub fn split_sources(nw: &Network, settings: &SourceSettings, found: Vec<usize>, srcs: &mut Sources) -> (Vec<usize>, Vec<usize>) {
    if settings.resistance > 0.0 {
        let sources = found.iter().map(|&i| srcs.add(nw, MNode::Src(i))).collect();
        (found, sources)
    } else {
        (Vec::new(), found)
    }
}

/// The node dump: one line per matrix index.
pub fn node_lines(nw: &Network, net: &NetInput, srcs: &Sources, order: &[MNode]) -> Vec<String> {
    order.iter().enumerate().map(|(i, &n)| format!("{i}: {}", srcs.describe(nw, net, n))).collect()
}

// ---- voltages --------------------------------------------------------------------------------

/// A supply voltage set by the user for a net (and corner).
#[derive(Debug, Clone, PartialEq)]
pub struct UserVoltage {
    pub net_id: u32,
    pub power: bool,
    pub corner: String,
    pub voltage: f64,
}

/// The voltage the power net is at. The power net is the analysed net when it is a power net,
/// else the first power net (by database id) the user set a voltage on. Its voltage: a previous
/// solution of the same net on this corner, else the user's for this corner, else the default
/// library's default operating conditions. `None` when none applies.
pub fn power_net_voltage(
    sig: SigType,
    net_id: u32,
    user: &[UserVoltage],
    corner: &str,
    previous_solution: Option<f64>,
    pvt: Option<f32>,
) -> Option<f64> {
    let mut by_id: Vec<&UserVoltage> = user.iter().collect();
    by_id.sort_by_key(|u| u.net_id);
    let power_net = if sig == SigType::Power { Some(net_id) } else { by_id.iter().find(|u| u.power).map(|u| u.net_id) };
    if power_net == Some(net_id) {
        if let Some(v) = previous_solution {
            return Some(v);
        }
    }
    if let Some(p) = power_net {
        // The last setting for (net, corner) is the one kept.
        if let Some(u) = user.iter().rev().find(|u| u.net_id == p && u.corner == corner) {
            return Some(u.voltage);
        }
    }
    pvt.map(f64::from)
}

/// The analysed net's voltage: zero for a ground net, else the power net's.
pub fn net_voltage(sig: SigType, power: impl FnOnce() -> Option<f64>) -> Option<f64> {
    if sig == SigType::Ground {
        return Some(0.0);
    }
    power()
}

/// The default library's default operating conditions' voltage: the library's
/// `default_operating_conditions` names an `operating_conditions` group, whose `voltage` (0 when
/// absent) applies. `None` when the library names none, or a group it does not have.
pub fn pvt_voltage(library: &vyges_sta::liberty_parse::Group) -> Option<f32> {
    let name = library.attr_text("default_operating_conditions").filter(|n| !n.is_empty())?;
    let group = library.groups_of("operating_conditions").filter(|g| g.name().as_deref() == Some(name.as_str())).last()?;
    Some(group.attr_float("voltage").unwrap_or(0.0))
}

// ---- currents --------------------------------------------------------------------------------

/// Each instance's power drawn evenly through its terminal nodes on the net: the timer's power,
/// then the user's power, both added. Instances in id order; the total is the last power counted
/// per instance (the user's replacing the timer's), summed in single precision.
pub fn build_node_current_map(
    nw: &Network,
    inputs: &SolveInputs,
    net: &NetInput,
    power_voltage: f64,
    timer_power: &HashMap<String, f32>,
    user_power: &HashMap<String, f32>,
) -> (HashMap<NodeId, f64>, f32) {
    let mut inst_nodes: BTreeMap<i32, (String, Vec<NodeId>)> = BTreeMap::new();
    for &n in &nw.iterm_nodes {
        if let NodeKind::ITerm { iterm, .. } = nw.arena[n].kind {
            let name = net.iterms[iterm].name.rsplit_once('/').map_or(net.iterms[iterm].name.as_str(), |(inst, _)| inst);
            inst_nodes.entry(inputs.iterm_inst[iterm]).or_insert_with(|| (name.to_string(), Vec::new())).1.push(n);
        }
    }
    for (_, nodes) in inst_nodes.values_mut() {
        nodes.sort_by(|a, b| nw.compare(*a, *b));
        nodes.dedup_by(|a, b| nw.compare(*a, *b) == Ordering::Equal);
    }
    let mut currents: HashMap<NodeId, f64> = HashMap::new();
    let mut instance_powers: BTreeMap<i32, f32> = BTreeMap::new();
    for source in [timer_power, user_power] {
        for (id, (name, nodes)) in &inst_nodes {
            let Some(&power) = source.get(name) else { continue };
            instance_powers.insert(*id, power);
            let current = f64::from(power) / power_voltage;
            for n in nodes {
                *currents.entry(*n).or_insert(0.0) += current / nodes.len() as f64;
            }
        }
    }
    let total = instance_powers.values().fold(0.0f32, |t, p| t + p);
    (currents, total)
}

// ---- the matrix ------------------------------------------------------------------------------

/// The conductance matrix and current vector. `g` is keyed (column, row), the order the matrix
/// dump walks it.
#[derive(Debug, Default)]
pub struct Matrix {
    pub g: BTreeMap<(usize, usize), f64>,
    pub j: Vec<f64>,
}

/// The source resistances: from each real source's grid node to the real source.
pub fn source_conductances(nw: &Network, srcs: &Sources, settings: &SourceSettings, real: &[usize]) -> Vec<(MNode, MNode, f64)> {
    real.iter()
        .map(|&i| {
            let (mut a, mut b) = (srcs.nodes[i].source, MNode::Src(i));
            if srcs.compare(nw, a, b) == Ordering::Greater {
                std::mem::swap(&mut a, &mut b);
            }
            (a, b, 1.0 / f64::from(settings.resistance))
        })
        .collect()
}

/// G and J: per node, its current (negated unless the net is ground), minus each connection's
/// conductance to the other node and their sum on the diagonal, the connections in connection
/// order; then each voltage source as a unit conductance between it and the node it drives, with
/// the source voltage as its current.
#[allow(clippy::too_many_arguments)]
pub fn build_cond_matrix_and_voltages(
    nw: &Network,
    srcs: &Sources,
    cond: &[(ConnId, f64)],
    src_conds: &[(MNode, MNode, f64)],
    order: &[MNode],
    currents: &HashMap<NodeId, f64>,
    src_voltage: f64,
    sources: &[usize],
) -> Matrix {
    let index: HashMap<MNode, usize> = order.iter().enumerate().map(|(i, &n)| (n, i)).collect();
    let mut node_conns: HashMap<MNode, Vec<(MNode, MNode, f64)>> = HashMap::new();
    let edges = cond
        .iter()
        .filter_map(|&(c, g)| {
            let conn = &nw.conns[c];
            Some((MNode::Net(conn.n0?), MNode::Net(conn.n1?), g))
        })
        .chain(src_conds.iter().copied());
    for e in edges {
        node_conns.entry(e.0).or_default().push(e);
        node_conns.entry(e.1).or_default().push(e);
    }
    let mut m = Matrix { g: BTreeMap::new(), j: vec![0.0; order.len()] };
    let mut nodes: Vec<&MNode> = node_conns.keys().collect();
    nodes.sort_by_key(|n| index[n]);
    for &node in nodes {
        let mut conns = node_conns[&node].clone();
        conns.sort_by(|x, y| srcs.compare(nw, x.0, y.0).then_with(|| srcs.compare(nw, x.1, y.1)));
        conns.dedup_by(|x, y| srcs.compare(nw, x.0, y.0) == Ordering::Equal && srcs.compare(nw, x.1, y.1) == Ordering::Equal);
        let row = index[&node];
        m.j[row] = match node {
            MNode::Net(id) => currents.get(&id).copied().unwrap_or(0.0),
            MNode::Src(_) => 0.0,
        };
        let mut node_cond = 0.0;
        for (a, b, g) in conns {
            let other = if a == node { b } else { a };
            node_cond += g;
            *m.g.entry((index[&other], row)).or_insert(0.0) += -g;
        }
        *m.g.entry((row, row)).or_insert(0.0) += node_cond;
    }
    if src_voltage != 0.0 {
        for j in &mut m.j {
            *j = -*j;
        }
    }
    for &s in sources {
        let idx = index[&MNode::Src(s)];
        m.j[idx] = src_voltage / 1.0;
        let real = index[&srcs.nodes[s].source];
        m.g.insert((real, idx), 1.0);
        m.g.insert((idx, real), 1.0);
    }
    m
}

/// G as the solver takes it: compressed by column, rows ascending.
pub fn csc_of(m: &Matrix) -> vyges_sparselu::Csc {
    let n = m.j.len();
    let mut a = vyges_sparselu::Csc { nrows: n, ncols: n, col_ptr: vec![0; n + 1], row_idx: Vec::with_capacity(m.g.len()), values: Vec::with_capacity(m.g.len()) };
    for (&(c, r), &v) in &m.g {
        a.col_ptr[c + 1] += 1;
        a.row_idx.push(r);
        a.values.push(v);
    }
    for c in 0..n {
        a.col_ptr[c + 1] += a.col_ptr[c];
    }
    a
}

pub fn matrix_lines(m: &Matrix) -> Vec<String> {
    m.g.iter().map(|(&(c, r), &v)| format!("G[{c}, {r}] = {}", fmt_e(v, 15))).collect()
}

pub fn vector_lines(name: &str, v: &[f64]) -> Vec<String> {
    v.iter().enumerate().map(|(i, &x)| format!("{name}[{i}] = {}", fmt_e(x, 15))).collect()
}

/// G exactly, for an instrument: `column row bits` per entry, the IEEE bits in hex.
pub fn matrix_bits_lines(m: &Matrix) -> Vec<String> {
    m.g.iter().map(|(&(c, r), &v)| format!("{c} {r} {:016x}", v.to_bits())).collect()
}

/// A vector exactly, for an instrument: the IEEE bits in hex, one per line.
pub fn vector_bits_lines(v: &[f64]) -> Vec<String> {
    v.iter().map(|x| format!("{:016x}", x.to_bits())).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    // The C++ exponent always carries a sign and at least two digits.
    // `stod` reads the longest numeric prefix after white space: " 1.5e3x" is 1500, "2." is 2,
    // "1e" is 1 (an exponent needs a digit), and nothing numeric is a failure.
    #[test]
    fn numbers_are_read_as_stod_reads_them() {
        assert_eq!(parse_leading_f64(" 1.5e3x"), Some(1500.0));
        assert_eq!(parse_leading_f64("2."), Some(2.0));
        assert_eq!(parse_leading_f64("1e"), Some(1.0));
        assert_eq!(parse_leading_f64("-.5"), Some(-0.5));
        assert_eq!(parse_leading_f64("abc"), None);
        assert_eq!(parse_leading_f64(""), None);
    }

    // Settings accumulate: a value given positive replaces, an absent one (0) leaves the last.
    #[test]
    fn source_settings_accumulate_positive_values() {
        let mut s = SourceSettings::default();
        s.apply(&SourceSettings { bump_dx: 45, bump_dy: 0, bump_size: 25, bump_interval: 2, strap_track_pitch: 0, resistance: 0.0 });
        s.apply(&SourceSettings { bump_dx: 0, bump_dy: 0, bump_size: 0, bump_interval: 0, strap_track_pitch: 5, resistance: 10.0 });
        assert_eq!(s, SourceSettings { bump_dx: 45, bump_dy: 140, bump_size: 25, bump_interval: 2, strap_track_pitch: 5, resistance: 10.0 });
    }

    #[test]
    fn exponents_print_as_cpp_prints_them() {
        assert_eq!(fmt_e(0.3522585992540406, 15), "3.522585992540406e-01");
        assert_eq!(fmt_e(1000.0, 15), "1.000000000000000e+03");
        assert_eq!(fmt_e(-0.0, 15), "-0.000000000000000e+00");
        assert_eq!(fmt_e(1.38e-4, 2), "1.38e-04");
        assert_eq!(fmt_e(1e-120, 2), "1.00e-120");
    }
}
