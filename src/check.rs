// SPDX-License-Identifier: Apache-2.0
//! The connectivity check on a built network: block terminals present, nothing open.
//!
//! Each function is one step of the check, in the order the check runs them; the messages are the
//! lines the check prints, in the order it prints them.

use std::collections::{HashMap, HashSet, VecDeque};

use vyges_drt::rtree::PackedRTree;

use crate::input::{NetInput, Rect};
use crate::network::{Network, NodeId, NodeKind};

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

/// What an open network reports: each shape holding an unreached node (once, in node order and
/// the layer's shape-tree order within a node), then each unreached instance terminal.
pub fn report_unconnected_nodes(nw: &Network, net: &NetInput, visited: &HashSet<NodeId>) -> Vec<String> {
    let dbu = f64::from(nw.tech.dbus);
    let mut nodes: Vec<NodeId> = nw.nodes.values().flatten().copied().filter(|n| !visited.contains(n)).collect();
    nodes.sort_by(|a, b| nw.compare(*a, *b));
    nodes.dedup_by(|a, b| nw.compare(*a, *b).is_eq());
    let mut iterms: Vec<NodeId> = nw.iterm_nodes.iter().copied().filter(|n| !visited.contains(n)).collect();
    iterms.sort_by(|a, b| nw.compare(*a, *b));
    iterms.dedup_by(|a, b| nw.compare(*a, *b).is_eq());
    let mut out = Vec::new();
    if nodes.is_empty() && iterms.is_empty() {
        return out;
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
            out.push(format!(
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
        out.push(format!(
            "[WARNING PSM-0039] Unconnected instance {} at location ({}um, {}um).",
            net.iterms[iterm].name,
            um(node.pt.0, dbu),
            um(node.pt.1, dbu)
        ));
    }
    out
}
