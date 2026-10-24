// SPDX-License-Identifier: Apache-2.0
//! `vyges-psm` — power-grid analysis on the design database: connectivity check, static IR drop, per-node voltages, SPICE export.
//!
//! ## Exit codes
//!
//! ⛔ **"found something" and "could not run" never share a code.** A CI gate that cannot tell a
//! broken tool from a bad design will eventually read a missing engine as a clean run.
//!
//! | code | meaning |
//! | --- | --- |
//! | 0 | ok |
//! | 1 | a finding (the check ran and failed) |
//! | 2 | usage, or input that could not be read |
//! | 3 | the command is NOT IMPLEMENTED in this build |

use std::process::ExitCode;

const USAGE: &str = "\
vyges loom psm — power-grid analysis on the design database: connectivity check, static IR drop, per-node voltages, SPICE export

USAGE:
  vyges loom psm check        <design.odb> --net NET [--floorplanning] [--dont-require-terminals]
                                [--dump-dir DIR] [--stats]
                                                  every power/ground shape and instance pin reaches a supply
  vyges loom psm run          <design.odb> <job>  static IR drop: per-node voltages, worst and average drop
  vyges loom psm write-spice  <design.odb> <job>  write the power grid as a SPICE netlist
  vyges loom psm --describe
  vyges loom psm --help
  vyges loom psm --version

OPTIONS:
  --net NET             the supply net to analyse
  --floorplanning       check: only FIXED instances count (the grid before placement)
  --dont-require-terminals  check: a net without a placed block terminal is not an error
  --dump-dir DIR        write the network after each construction stage (psm_<stage>.txt) to DIR
  --stats               print the network's shape, node and connection counts
  -o FILE               write the report (or the output database) to FILE
  --json                machine-readable output instead of the text report
  -q, --quiet           suppress non-essential output
  -v, --verbose         extra detail on stderr
  --describe            print a machine-readable JSON description of the command
  --bug-report          file a bug (central: vyges/community)
  --feature-request     request a feature (central)

EXIT STATUS:
  0  ok
  1  a finding — the check ran and failed
  2  usage or unreadable input
  3  refused — this build does not implement the command yet
";

/// The commands this engine will answer. `check` is built; the others refuse (exit 3).
pub const COMMANDS: &[&str] = &["check", "run", "write-spice"];

const DESCRIBE: &str = r#"{
  "schema": "vyges-tool-descriptor/1.1",
  "name": "psm",
  "summary": "Power-grid analysis on the design database: connectivity check, static IR drop, per-node voltages, SPICE export.",
  "maturity": "discovered",
  "provenance_limitations": [
    "check: the error file (-error_file) is not written yet.",
    "check refuses (exit 3) on an octilinear special wire of another net, a pin layer mixing a polygon off the Manhattan grid with other metal, and an off-grid obstruction polygon.",
    "run and write-spice refuse with exit 3."
  ],
  "invocation": { "args_template": ["check"], "emits_json": true },
  "inputs": { "type": "object", "properties": {} },
  "artifacts": [],
  "assertion": { "id": "psm-ok", "field": "status", "pass_when": { "equals": "ok" } },
  "consumes": []
}"#;

fn refuse(what: &str) -> ExitCode {
    vyges_events::emit(
        &vyges_events::Event::new("vyges-psm", vyges_events::Severity::Error, format!("{what} is not implemented in this build"))
            .with_code("PSM-REFUSED"),
    );
    ExitCode::from(3)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    match args.first().map(String::as_str) {
        None | Some("-h") | Some("--help") => {
            print!("{USAGE}");
            ExitCode::SUCCESS
        }
        Some("-V") | Some("--version") => {
            println!("vyges-psm {} ({})", env!("CARGO_PKG_VERSION"), env!("VYGES_GIT_SHA"));
            println!("Copyright 2026 Vyges. Apache-2.0.");
            ExitCode::SUCCESS
        }
        Some("--describe") => {
            println!("{DESCRIBE}");
            ExitCode::SUCCESS
        }
        Some("check") => check(&args[1..]),
        Some(cmd) if COMMANDS.contains(&cmd) => refuse(cmd),
        Some(other) => {
            eprintln!("vyges-psm: unknown command `{other}`\n\n{USAGE}");
            ExitCode::from(2)
        }
    }
}

fn usage_error(msg: &str) -> ExitCode {
    eprintln!("vyges-psm: {msg}\n\n{USAGE}");
    ExitCode::from(2)
}

/// `check`: build the net's network, then check it in the check's order — block terminals, opens,
/// shorts — and report connected (exit 0) or failed (exit 1). A short the walk cannot measure (an
/// octilinear wire, a skewed master polygon) refuses (exit 3) rather than pass it over.
fn check(args: &[String]) -> ExitCode {
    let (mut design, mut net, mut dump_dir) = (None, None, None);
    let (mut floorplanning, mut stats, mut require_bterm) = (false, false, true);
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--net" => match it.next() {
                Some(v) => net = Some(v.clone()),
                None => return usage_error("--net needs a net name"),
            },
            "--dump-dir" => match it.next() {
                Some(v) => dump_dir = Some(std::path::PathBuf::from(v)),
                None => return usage_error("--dump-dir needs a directory"),
            },
            "--floorplanning" => floorplanning = true,
            "--stats" => stats = true,
            "--dont-require-terminals" => require_bterm = false,
            v if v.starts_with('-') => return usage_error(&format!("unknown option `{v}`")),
            v if design.is_none() => design = Some(v.to_string()),
            v => return usage_error(&format!("unexpected argument `{v}`")),
        }
    }
    let (Some(design), Some(net)) = (design, net) else { return usage_error("check needs <design.odb> and --net") };
    let db = match vyges_opendb::Db::open(&design) {
        Ok(db) => db,
        Err(e) => {
            eprintln!("vyges-psm: cannot read {design}: {e}");
            return ExitCode::from(2);
        }
    };
    let input = match vyges_psm::read::tech(&db).and_then(|t| vyges_psm::read::net(&db, &net).map(|n| (t, n))) {
        Ok(x) => x,
        Err(e) => {
            eprintln!("vyges-psm: cannot read net {net}: {e}");
            return ExitCode::from(2);
        }
    };
    let (tech, net_input) = input;
    let dump: Option<vyges_psm::network::DumpFn> = dump_dir.map(|dir| {
        Box::new(move |stage: &str, n: &vyges_psm::network::Network| {
            let mut text = n.dump_lines().join("\n");
            text.push('\n');
            if let Err(e) = std::fs::write(dir.join(format!("psm_{stage}.txt")), text) {
                eprintln!("vyges-psm: cannot write the {stage} dump: {e}");
            }
        }) as vyges_psm::network::DumpFn
    });
    let (network, log) = match vyges_psm::network::Network::build(&tech, &net_input, floorplanning, dump) {
        Ok(x) => x,
        Err(e) => {
            println!("[ERROR PSM-0087] {e}");
            return ExitCode::from(1);
        }
    };
    if log.empty {
        println!("[WARNING PSM-0086] Net {net} is empty.");
        return connectivity_failed(&net);
    }
    if stats {
        for l in network.stats_lines() {
            println!("{l}");
        }
    }
    if require_bterm && !vyges_psm::check::check_bterms(&net_input) {
        println!("{}", vyges_psm::check::missing_bterm_message(&net_input));
        return ExitCode::from(1);
    }
    let visited = match vyges_psm::check::check_open(&network) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("vyges-psm: the open check cannot start: {e}");
            return refuse("an open check from an unconnected start node");
        }
    };
    let mut connected = true;
    if !vyges_psm::check::is_connected(&network, &visited) {
        for l in vyges_psm::check::report_unconnected_nodes(&network, &net_input, &visited) {
            println!("{l}");
        }
        connected = false;
    }
    let shorts = vyges_psm::read::short_inputs(&db, &net).and_then(|si| vyges_psm::short::check_short(&network, &si, !floorplanning));
    let (shorts, stopped) = match shorts {
        Ok(x) => x,
        Err(e) => {
            eprintln!("vyges-psm: the short check cannot measure {e}");
            return refuse("this short check");
        }
    };
    for l in vyges_psm::short::report_shorts(&network, &net, &shorts, stopped) {
        println!("{l}");
    }
    if !shorts.is_empty() {
        connected = false;
    }
    if !connected {
        return connectivity_failed(&net);
    }
    println!("[INFO PSM-0040] All shapes on net {net} are connected.");
    ExitCode::SUCCESS
}

fn connectivity_failed(net: &str) -> ExitCode {
    println!("[ERROR PSM-0069] Check connectivity failed on {net}.");
    ExitCode::from(1)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_descriptor_is_valid_json_with_a_legal_maturity() {
        let d: serde_json::Value = serde_json::from_str(DESCRIBE).expect("descriptor is valid JSON");
        assert_eq!(d["schema"], "vyges-tool-descriptor/1.1");
        assert_eq!(d["name"], "psm");
        // Anything else is read as `discovered` and every verdict becomes `unknown`.
        let m = d["maturity"].as_str().unwrap();
        assert!(["discovered", "structured", "workflow-validated"].contains(&m), "{m}");
    }

    #[test]
    fn the_usage_spells_the_engine_under_its_group() {
        // The dispatcher REFUSES a wrong group prefix, so the help must spell the right one.
        for c in COMMANDS {
            assert!(USAGE.contains(&format!("vyges loom psm {c}")), "{c}");
        }
    }
}
