// SPDX-License-Identifier: Apache-2.0
//! The SDC supply voltage: `set_voltage` as the analysis reads it (`IRSolver::getSDCVoltage`).
//!
//! `set_voltage [-min MIN] [-object_list NETS] MAX` (OpenSTA `Sdc.tcl`) always sets the global
//! max-case voltage and, for each net named, that net's max-case voltage. `-min` sets the min-case
//! ones, which this analysis never reads (it asks for `MinMax::max()`). A later call replaces an
//! earlier one. The voltage is stored as a `float` (`Sdc::setVoltage(..., float)`).
//!
//! The timer never reads it: OpenSTA's delay calculation and power read the operating conditions
//! and the library's supply voltages, not `Sdc::voltage`. So the commands are taken out of the text
//! the timer is given ([`split_set_voltage`]).

/// The `set_voltage` state of the constraints: the global max-case voltage and each net's.
#[derive(Debug, Clone, Default, PartialEq)]
pub struct SdcVoltages {
    pub global: Option<f32>,
    /// `(net name, voltage)`, the last setting per net, in the order first set.
    pub nets: Vec<(String, f32)>,
}

impl SdcVoltages {
    /// `getSDCVoltage`: the net's own max voltage, else the global one.
    pub fn of_net(&self, net: &str) -> Option<f32> {
        self.nets.iter().find(|(n, _)| n == net).map(|(_, v)| *v).or(self.global)
    }

    fn set(&mut self, net: &str, v: f32) {
        match self.nets.iter_mut().find(|(n, _)| n == net) {
            Some(e) => e.1 = v,
            None => self.nets.push((net.to_string(), v)),
        }
    }
}

/// Tcl words of one command line: braces and brackets group (nested), quotes too.
fn words(line: &str) -> Result<Vec<String>, String> {
    let mut out = Vec::new();
    let mut cur = String::new();
    let mut depth: i32 = 0;
    let mut quoted = false;
    for ch in line.chars() {
        match ch {
            '{' | '[' if !quoted => {
                depth += 1;
                cur.push(ch);
            }
            '}' | ']' if !quoted => {
                depth -= 1;
                cur.push(ch);
            }
            '"' if depth == 0 => quoted = !quoted,
            c if c.is_whitespace() && depth == 0 && !quoted => {
                if !cur.is_empty() {
                    out.push(std::mem::take(&mut cur));
                }
            }
            c => cur.push(c),
        }
    }
    if depth != 0 || quoted {
        return Err(format!("unbalanced braces or quotes in `{line}`"));
    }
    if !cur.is_empty() {
        out.push(cur);
    }
    Ok(out)
}

/// The net names an `-object_list` value names: `VDD`, `{VDD VSS}`, `[get_nets VDD]`,
/// `[get_nets {VDD VSS}]`. A pattern is refused: what it matches is the design's to say.
fn net_names(arg: &str) -> Result<Vec<String>, String> {
    let mut a = arg.trim();
    if let Some(inner) = a.strip_prefix('[').and_then(|s| s.strip_suffix(']')) {
        let w = words(inner)?;
        match w.split_first() {
            Some((cmd, rest)) if cmd == "get_nets" && !rest.iter().any(|x| x.starts_with('-')) => {
                return rest.iter().map(|r| net_names(r)).collect::<Result<Vec<_>, _>>().map(|v| v.concat());
            }
            _ => return Err(format!("-object_list {arg}: only net names or get_nets of names are read")),
        }
    }
    if let Some(inner) = a.strip_prefix('{').and_then(|s| s.strip_suffix('}')) {
        a = inner;
    }
    let names: Vec<String> = a.split_whitespace().map(String::from).collect();
    if names.iter().any(|n| n.contains(['*', '?'])) {
        return Err(format!("-object_list {arg}: a net pattern is not resolved here"));
    }
    Ok(names)
}

/// Apply one `set_voltage` command's words (the command name first) to `v`.
fn apply(w: &[String], v: &mut SdcVoltages) -> Result<(), String> {
    let mut max = None;
    let mut nets = Vec::new();
    let mut i = 1;
    while i < w.len() {
        match w[i].as_str() {
            "-min" => i += 1, // the min-case voltage: never read by the analysis
            "-object_list" => {
                let arg = w.get(i + 1).ok_or("set_voltage -object_list needs a value")?;
                nets = net_names(arg)?;
                i += 1;
            }
            x if max.is_none() => {
                max = Some(x.parse::<f32>().map_err(|_| format!("set_voltage: `{x}` is not a voltage"))?)
            }
            x => return Err(format!("set_voltage: unexpected `{x}`")),
        }
        i += 1;
    }
    let max = max.ok_or("set_voltage needs a max-case voltage")?;
    // `set_voltage_global "max"`, then `set_voltage_net` for each net: the order Sdc.tcl calls them
    v.global = Some(max);
    for n in &nets {
        v.set(n, max);
    }
    Ok(())
}

/// The `set_voltage` state of `sdcs` (in order), and the texts with those commands taken out; a text
/// left with nothing but blank lines and comments is dropped, so the timer sees only constraints.
pub fn split_set_voltage(sdcs: &[String]) -> Result<(SdcVoltages, Vec<String>), String> {
    let mut v = SdcVoltages::default();
    let mut rest = Vec::new();
    for text in sdcs {
        let mut kept = String::new();
        let mut any = false;
        let mut prev_continues = false;
        for line in text.lines() {
            let continues = line.trim_end().ends_with('\\');
            let is_cmd = !prev_continues && line.split_whitespace().next() == Some("set_voltage");
            if is_cmd {
                if continues {
                    return Err("set_voltage continued over lines is not read".into());
                }
                let body = line.split_once(';').map_or(line, |(a, _)| a);
                apply(&words(body)?, &mut v)?;
            } else {
                kept.push_str(line);
                kept.push('\n');
                let t = line.trim();
                any |= prev_continues || !(t.is_empty() || t.starts_with('#'));
            }
            prev_continues = continues;
        }
        if any {
            rest.push(kept);
        }
    }
    Ok((v, rest))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Rule (OpenSTA `set_voltage`): the max-case voltage is always set globally and also on each
    /// net of `-object_list`; `-min` is the min case, which psm never reads; a later call replaces.
    #[test]
    fn a_set_voltage_sets_the_global_and_each_named_net() {
        let sdc = "create_clock -period 10 [get_ports clk]\nset_voltage 1.3 -object_list VDD\n".to_string();
        let (v, rest) = split_set_voltage(&[sdc]).unwrap();
        assert_eq!(v.of_net("VDD"), Some(1.3f32));
        assert_eq!(v.of_net("VDD2"), Some(1.3f32), "the global one for any other net");
        assert_eq!(rest, vec!["create_clock -period 10 [get_ports clk]\n".to_string()]);

        let (v, _) = split_set_voltage(&["set_voltage -min 0.9 1.2 -object_list [get_nets {VDD VDDX}]\nset_voltage 1.0\n".into()]).unwrap();
        assert_eq!(v.of_net("VDDX"), Some(1.2), "the net keeps its own");
        assert_eq!(v.of_net("VSS"), Some(1.0), "the later global replaced the earlier");
    }

    /// The value is a `float` in the reference (`Sdc::setVoltage`), so 1.3 reaches the solve as
    /// 1.2999999523..., not 1.3 (a user voltage is a `double`).
    #[test]
    fn the_sdc_voltage_is_single_precision() {
        let (v, _) = split_set_voltage(&["set_voltage 1.3\n".into()]).unwrap();
        assert_eq!(f64::from(v.of_net("VDD").unwrap()), f64::from(1.3f32));
        assert_ne!(f64::from(v.of_net("VDD").unwrap()), 1.3f64);
    }

    #[test]
    fn a_file_of_only_set_voltage_leaves_the_timer_nothing() {
        let (v, rest) = split_set_voltage(&["# session\nset_voltage 1.3 -object_list VDD\n\n".into()]).unwrap();
        assert!(rest.is_empty(), "{rest:?}");
        assert_eq!(v.global, Some(1.3));
        assert!(split_set_voltage(&["set_voltage 1 -object_list [get_nets V*]\n".into()]).is_err(), "a pattern");
        assert!(split_set_voltage(&["set_voltage 1 \\\n -object_list VDD\n".into()]).is_err(), "a continuation");
    }
}
