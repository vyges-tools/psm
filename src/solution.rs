// SPDX-License-Identifier: Apache-2.0
//! The solutions a session keeps between `analyze` calls, in a file (`--save-solution`), since each
//! call is its own process.
//!
//! What the reference keeps per net, per corner, until the design is edited:
//!
//! - the supply voltage of the last solve. It is kept even through a failed solve, and a LATER
//!   solve of the same power net takes it as the power net's voltage before the user's setting;
//! - the node voltages and whether the solve drew current. A solution is reusable
//!   (`-allow_reuse`) only when it has both voltages and currents;
//! - the total power, which the reused report prints.
//!
//! Any edit to the design drops every solution. Here the design's fingerprint (its file's bytes)
//! stands in for "not edited": a store written for another fingerprint is empty.

use crate::network::NodeId;

/// One net's solution on one corner.
#[derive(Debug, Clone, PartialEq)]
pub struct Solution {
    pub net: String,
    pub corner: String,
    pub src_voltage: f64,
    pub total_power: f32,
    /// Reusable: the solve produced voltages and drew current.
    pub has_solution: bool,
    pub voltages: Vec<(NodeId, f64)>,
}

#[derive(Debug, Clone, PartialEq, Default)]
pub struct Store {
    pub design: u64,
    pub solutions: Vec<Solution>,
}

const MAGIC: &str = "vyges-psm-solutions 1";

/// The design's fingerprint: FNV-1a over its bytes.
pub fn fingerprint(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325u64, |h, &b| (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3))
}

impl Store {
    /// The store for `design` read from `text`; empty when it was written for another design.
    pub fn parse(text: &str, design: u64) -> Result<Store, String> {
        let mut lines = text.lines();
        if lines.next() != Some(MAGIC) {
            return Err("not a solution file".into());
        }
        let bad = |what: &str| format!("malformed solution file: {what}");
        let fp = lines.next().and_then(|l| l.strip_prefix("design ")).ok_or_else(|| bad("design"))?;
        let fp = u64::from_str_radix(fp, 16).map_err(|_| bad("design"))?;
        if fp != design {
            return Ok(Store { design, solutions: Vec::new() });
        }
        let mut store = Store { design, solutions: Vec::new() };
        while let Some(head) = lines.next() {
            let w: Vec<&str> = head.split(' ').collect();
            let [tag, net, corner, src, power, has, n] = w[..] else { return Err(bad("solution header")) };
            if tag != "solution" {
                return Err(bad("solution header"));
            }
            let hex64 = |s: &str| u64::from_str_radix(s, 16).map(f64::from_bits).map_err(|_| bad("value"));
            let n: usize = n.parse().map_err(|_| bad("count"))?;
            let mut voltages = Vec::with_capacity(n);
            for _ in 0..n {
                let l = lines.next().ok_or_else(|| bad("voltages"))?;
                let (id, v) = l.split_once(' ').ok_or_else(|| bad("voltage"))?;
                voltages.push((id.parse().map_err(|_| bad("node"))?, hex64(v)?));
            }
            store.solutions.push(Solution {
                net: net.into(),
                corner: corner.into(),
                src_voltage: hex64(src)?,
                total_power: f32::from_bits(u32::from_str_radix(power, 16).map_err(|_| bad("power"))?),
                has_solution: has == "1",
                voltages,
            });
        }
        Ok(store)
    }

    /// The file's text: every value as IEEE bits, so a reused solution is the solved one exactly.
    pub fn text(&self) -> String {
        let mut out = format!("{MAGIC}\ndesign {:016x}\n", self.design);
        for s in &self.solutions {
            out += &format!(
                "solution {} {} {:016x} {:08x} {} {}\n",
                s.net,
                s.corner,
                s.src_voltage.to_bits(),
                s.total_power.to_bits(),
                u8::from(s.has_solution),
                s.voltages.len()
            );
            for (id, v) in &s.voltages {
                out += &format!("{id} {:016x}\n", v.to_bits());
            }
        }
        out
    }

    pub fn get(&self, net: &str, corner: &str) -> Option<&Solution> {
        self.solutions.iter().find(|s| s.net == net && s.corner == corner)
    }

    /// Keep `s` as its net's solution on its corner, replacing the one before.
    pub fn put(&mut self, s: Solution) {
        self.solutions.retain(|x| !(x.net == s.net && x.corner == s.corner));
        self.solutions.push(s);
    }

    /// A solve has started: the net's voltages and currents on the corner are reset (no longer
    /// reusable), but its supply voltage and power are kept until the solve stores new ones.
    pub fn reset(&mut self, net: &str, corner: &str) {
        if let Some(s) = self.solutions.iter_mut().find(|s| s.net == net && s.corner == corner) {
            s.has_solution = false;
            s.voltages.clear();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sol(net: &str, v: f64) -> Solution {
        Solution { net: net.into(), corner: "default".into(), src_voltage: 1.1, total_power: 1.38e-4, has_solution: true, voltages: vec![(3, v), (7, -0.0)] }
    }

    // The file keeps every value to the bit (a negative zero included) and reads back as written.
    #[test]
    fn the_store_reads_back_exactly() {
        let mut s = Store { design: 42, solutions: vec![] };
        s.put(sol("VDD", 1.0999999999999999));
        s.put(sol("VSS", 1e-300));
        let back = Store::parse(&s.text(), 42).unwrap();
        assert_eq!(back, s);
        assert_eq!(back.get("VDD", "default").unwrap().voltages[1].1.to_bits(), (-0.0f64).to_bits());
    }

    // Any edit to the design drops every solution: a store of another design reads empty.
    #[test]
    fn another_design_reads_empty() {
        let mut s = Store { design: fingerprint(b"one"), solutions: vec![] };
        s.put(sol("VDD", 1.0));
        assert!(Store::parse(&s.text(), fingerprint(b"two")).unwrap().solutions.is_empty());
        assert!(Store::parse("junk", 1).is_err());
    }

    // A solve resets the net's voltages (no longer reusable) but keeps its supply voltage: a later
    // solve of the power net takes the previous solution's voltage.
    #[test]
    fn a_reset_keeps_the_supply_voltage() {
        let mut s = Store::default();
        s.put(sol("VDD", 1.0));
        s.reset("VDD", "default");
        let e = s.get("VDD", "default").unwrap();
        assert!(!e.has_solution && e.voltages.is_empty());
        assert_eq!(e.src_voltage, 1.1);
        s.put(sol("VDD", 2.0));
        assert_eq!(s.solutions.len(), 1);
    }
}
