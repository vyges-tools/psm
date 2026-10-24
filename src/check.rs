// SPDX-License-Identifier: Apache-2.0
//! The connectivity check on a built network: block terminals present, nothing open.
//!
//! Each function is one step of the check, in the order the check runs them; the messages are the
//! lines the check prints, in the order it prints them.

use std::collections::{HashMap, HashSet, VecDeque};

use vyges_drt::rtree::PackedRTree;

use crate::input::{NetInput, Rect};
use crate::network::{Network, NodeId, NodeKind};
use crate::short::{Short, Source};

/// Some pin of some block terminal of the net is placed.
pub fn check_bterms(net: &NetInput) -> bool {
    net.bterms.iter().any(|bt| bt.pins.iter().any(|p| p.placed))
}

pub fn missing_bterm_message(net: &NetInput) -> String {
    format!("[ERROR PSM-0025] {} does not contain any terminals", net.name)
}

/// Walk the network breadth-first from the first node of the top layer; the nodes reached.
/// `Err` when that start node has no connection at all (the reference cannot look it up).
pub fn check_open(nw: &Network) -> Result<HashSet<NodeId>, String> {
    let mut map: HashMap<NodeId, Vec<NodeId>> = HashMap::new();
    for &c in &nw.connections {
        let conn = &nw.conns[c];
        if let (Some(a), Some(b)) = (conn.n0, conn.n1) {
            map.entry(a).or_default().push(b);
            map.entry(b).or_default().push(a);
        }
    }
    let top = nw.top_layer().ok_or("the network has no layer nodes")?;
    let start = nw.nodes[&top][0];
    let mut visited = HashSet::new();
    let mut queue = VecDeque::from([start]);
    while let Some(node) = queue.pop_front() {
        if !visited.insert(node) {
            continue;
        }
        let next = map.get(&node).ok_or_else(|| format!("node {} has no connection", nw.describe(node)))?;
        for &n in next {
            if !visited.contains(&n) {
                queue.push_back(n);
            }
        }
    }
    Ok(visited)
}

/// Every layer node reached.
pub fn is_connected(nw: &Network, visited: &HashSet<NodeId>) -> bool {
    nw.nodes.values().flatten().all(|n| visited.contains(n))
}

fn um(v: i32, dbu: f64) -> String {
    format!("{:4.3}", f64::from(v) / dbu)
}

/// What an open network reports, and the markers it records.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct OpenReport {
    pub lines: Vec<String>,
    /// Some layer node is unreached: the "unconnected shape" marker group is made (even empty).
    pub shape_group: bool,
    /// Each reported shape: layer index, rect.
    pub shapes: Vec<(usize, Rect)>,
    /// The unreached instance terminals, by name (their instances get the markers).
    pub iterms: Vec<String>,
}

/// What an open network reports: each shape holding an unreached node (once, in node order and
/// the layer's shape-tree order within a node), then each unreached instance terminal.
pub fn report_unconnected_nodes(nw: &Network, net: &NetInput, visited: &HashSet<NodeId>) -> OpenReport {
    let dbu = f64::from(nw.tech.dbus);
    let mut nodes: Vec<NodeId> = nw.nodes.values().flatten().copied().filter(|n| !visited.contains(n)).collect();
    nodes.sort_by(|a, b| nw.compare(*a, *b));
    nodes.dedup_by(|a, b| nw.compare(*a, *b).is_eq());
    let mut iterms: Vec<NodeId> = nw.iterm_nodes.iter().copied().filter(|n| !visited.contains(n)).collect();
    iterms.sort_by(|a, b| nw.compare(*a, *b));
    iterms.dedup_by(|a, b| nw.compare(*a, *b).is_eq());
    let mut rep = OpenReport { shape_group: !nodes.is_empty(), ..OpenReport::default() };
    if nodes.is_empty() && iterms.is_empty() {
        return rep;
    }
    let mut trees: HashMap<usize, PackedRTree<(Rect, usize)>> = HashMap::new();
    let mut reported: HashSet<(usize, usize)> = HashSet::new();
    for &n in &nodes {
        let node = &nw.arena[n];
        let tree = trees.entry(node.li).or_insert_with(|| {
            let shapes = nw.shapes.iter().find(|(k, _)| k.li == node.li).map(|(_, v)| v.as_slice()).unwrap_or(&[]);
            PackedRTree::new(shapes.iter().map(|s| (s.rect, (s.rect, s.id))).collect())
        });
        let p = Rect { xl: node.pt.0, yl: node.pt.1, xh: node.pt.0, yh: node.pt.1 };
        for e in tree.query(&p) {
            let (rect, id) = e.1;
            if !reported.insert((node.li, id)) {
                continue;
            }
            rep.shapes.push((node.li, rect));
            rep.lines.push(format!(
                "[WARNING PSM-0038] Unconnected shape on net {} at ({}um, {}um) - ({}um, {}um), layer: {}.",
                net.name,
                um(rect.xl, dbu),
                um(rect.yl, dbu),
                um(rect.xh, dbu),
                um(rect.yh, dbu),
                nw.tech.layers[node.li].name
            ));
        }
    }
    for &n in &iterms {
        let node = &nw.arena[n];
        let NodeKind::ITerm { iterm, .. } = node.kind else { continue };
        rep.iterms.push(net.iterms[iterm].name.clone());
        rep.lines.push(format!(
            "[WARNING PSM-0039] Unconnected instance {} at location ({}um, {}um).",
            net.iterms[iterm].name,
            um(node.pt.0, dbu),
            um(node.pt.1, dbu)
        ));
    }
    rep
}

/// The error file's marker stream (layout at the opendb shim, `psm_write_error_file`) and the
/// names it indexes. `Err` for a short whose overlap is not one rectangle: its marker is a polygon
/// whose point order this build does not reproduce.
pub fn error_file_stream(nw: &Network, open: &OpenReport, shorts: &[Short]) -> Result<(Vec<i64>, Vec<String>), String> {
    let mut v = Vec::new();
    let mut names: Vec<String> = Vec::new();
    let mut name = |n: &str| -> i64 {
        names.push(n.to_string());
        names.len() as i64 - 1
    };
    let rect = |v: &mut Vec<i64>, r: &Rect| v.extend([r.xl, r.yl, r.xh, r.yh].map(i64::from));
    v.push(i64::from(open.shape_group));
    v.push(open.shapes.len() as i64);
    for (li, r) in &open.shapes {
        v.push(i64::from(nw.tech.layers[*li].number));
        rect(&mut v, r);
    }
    v.push(i64::from(!open.iterms.is_empty()));
    v.push(open.iterms.len() as i64);
    for it in &open.iterms {
        v.push(name(it));
    }
    v.push(shorts.len() as i64);
    for sh in shorts {
        if !sh.rect {
            return Err(format!("a short to {} whose overlap is not one rectangle", sh.to));
        }
        v.push(i64::from(nw.tech.layers[sh.li].number));
        rect(&mut v, &sh.bbox);
        v.push(sh.sources.len() as i64);
        for src in &sh.sources {
            let (kind, value) = match src {
                Source::Net(n) => (0, name(n)),
                Source::BTerm(n) => (1, name(n)),
                Source::ITerm(n) => (2, name(n)),
                Source::Inst(n) => (3, name(n)),
                Source::Obstruction(i) => (4, *i as i64),
            };
            v.extend([kind, value]);
        }
    }
    Ok((v, names))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::input::{Dir, Layer, SBox, Tech};

    fn network() -> Network<'static> {
        let layer = |id: u32, number: i32, name: &str| Layer { id, number, name: name.into(), routing_level: 1, pitch: 200, dir: Dir::Horizontal, backside: false, upper: None, cut_classes: vec![] };
        let tech: &'static Tech = Box::leak(Box::new(Tech { dbus: 1000, front_base: Some(3), back_base: None, layers: vec![layer(1, 3, "m1")] }));
        let net = NetInput { name: "VDD".into(), supply: true, swires: vec![vec![SBox::Wire { layer: 3, rect: Rect { xl: 0, yl: 0, xh: 100, yh: 10 } }]], iterms: vec![], bterms: vec![] };
        Network::build(tech, &net, false, None).unwrap().0
    }

    // The error file's stream: the shape group (made, count, layer NUMBER + rect each), the
    // instance group (made, terminal names), then each short with its sources; a short whose
    // overlap is not its box cannot be written.
    #[test]
    fn the_error_file_stream_in_layout_order() {
        let nw = network();
        let r = Rect { xl: 1, yl: 2, xh: 3, yh: 4 };
        let open = OpenReport { lines: vec![], shape_group: true, shapes: vec![(0, r)], iterms: vec!["u1/VDD".into()] };
        let short = Short { li: 0, bbox: r, rect: true, to: "x".into(), sources: vec![Source::ITerm("u2/A".into()), Source::Net("n1".into()), Source::Obstruction(7)] };
        let (v, names) = error_file_stream(&nw, &open, std::slice::from_ref(&short)).unwrap();
        assert_eq!(v, vec![1, 1, 3, 1, 2, 3, 4, 1, 1, 0, 1, 3, 1, 2, 3, 4, 3, 2, 1, 0, 2, 4, 7]);
        assert_eq!(names, vec!["u1/VDD", "u2/A", "n1"]);
        // Nothing found: both groups unmade, no shorts.
        assert_eq!(error_file_stream(&nw, &OpenReport::default(), &[]).unwrap().0, vec![0, 0, 0, 0, 0]);
        let polygon = Short { rect: false, ..short };
        assert!(error_file_stream(&nw, &open, &[polygon]).is_err());
    }
}
