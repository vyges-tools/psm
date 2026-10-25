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
                                [--error-file FILE] [--dump-dir DIR] [--stats]
                                                  every power/ground shape and instance pin reaches a supply
  vyges loom psm analyze      <design.odb> --net NET [--voltage-file FILE] [--enable-em [--em-file FILE]]
                                [--save-solution FILE [--allow-reuse]] [--dump-dir DIR]
                                                  static IR drop: per-node voltages, worst and average drop
  vyges loom psm write-spice  <design.odb> <job>  write the power grid as a SPICE netlist
  vyges loom psm --describe
  vyges loom psm --help
  vyges loom psm --version

OPTIONS:
  --net NET             the supply net to analyse
  --floorplanning       check: only FIXED instances count (the grid before placement)
  --dont-require-terminals  check: a net without a placed block terminal is not an error
  --error-file FILE     check: write the net's unconnected and shorted shapes as a marker report
  --voltage-file FILE   analyze: write each instance terminal's voltage (CSV)
  --enable-em           analyze: report the grid's connection currents (maximum, average, count)
  --em-file FILE        analyze: with --enable-em, write each connection's current (CSV)
  --save-solution FILE  analyze: the session's solutions, kept between calls (read, then updated)
  --allow-reuse         analyze: report the saved solution of this net and corner instead of solving
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
pub const COMMANDS: &[&str] = &["check", "analyze", "write-spice"];

const DESCRIBE: &str = r#"{
  "schema": "vyges-tool-descriptor/1.1",
  "name": "psm",
  "summary": "Power-grid analysis on the design database: connectivity check, static IR drop, per-node voltages, SPICE export.",
  "maturity": "discovered",
  "provenance_limitations": [
    "check: an error file holding a short whose overlap is not one rectangle refuses (exit 3).",
    "check refuses (exit 3) on an octilinear special wire of another net, a pin layer mixing a polygon off the Manhattan grid with other metal, and an off-grid obstruction polygon.",
    "analyze is built end to end; a solution is kept between calls only through --save-solution; write-spice refuses with exit 3."
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
        Some("analyze") => analyze(&args[1..]),
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
    let (mut design, mut net, mut dump_dir, mut error_file) = (None, None, None, None);
    let (mut floorplanning, mut stats, mut require_bterm) = (false, false, true);
    let mut it = args.iter();
    while let Some(a) = it.next() {
        match a.as_str() {
            "--net" => match it.next() {
                Some(v) => net = Some(v.clone()),
                None => return usage_error("--net needs a net name"),
            },
            "--error-file" => match it.next() {
                Some(v) => error_file = Some(v.clone()),
                None => return usage_error("--error-file needs a file"),
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
    let opts = Connectivity { floorplanning, require_bterm, stats, dump_dir, error_file };
    let checked = check_connectivity(&db, &tech, &net_input, &opts).map(|_| ());
    match checked {
        Ok(()) => ExitCode::SUCCESS,
        Err(code) => code,
    }
}

/// How a connectivity check runs: `check` sets each from its flags; `analyze` checks with
/// `floorplanning` and `require_bterm` off.
struct Connectivity {
    floorplanning: bool,
    require_bterm: bool,
    stats: bool,
    dump_dir: Option<std::path::PathBuf>,
    error_file: Option<String>,
}

/// The connectivity check: build the net's network, then check it in the check's order — block
/// terminals, opens, shorts — printing what the check prints. The network when connected, else
/// the exit code (1 failed, 3 refused, 2 unwritable error file).
fn check_connectivity<'t>(
    db: &vyges_opendb::Db,
    tech: &'t vyges_psm::input::Tech,
    net_input: &vyges_psm::input::NetInput,
    opts: &Connectivity,
) -> Result<vyges_psm::network::Network<'t>, ExitCode> {
    let (floorplanning, require_bterm, stats) = (opts.floorplanning, opts.require_bterm, opts.stats);
    let (net, error_file) = (net_input.name.as_str(), opts.error_file.clone());
    let dump: Option<vyges_psm::network::DumpFn> = opts.dump_dir.clone().map(|dir| {
        Box::new(move |stage: &str, n: &vyges_psm::network::Network| {
            let mut text = n.dump_lines().join("\n");
            text.push('\n');
            if let Err(e) = std::fs::write(dir.join(format!("psm_{stage}.txt")), text) {
                eprintln!("vyges-psm: cannot write the {stage} dump: {e}");
            }
        }) as vyges_psm::network::DumpFn
    });
    let (network, log) = match vyges_psm::network::Network::build(tech, net_input, floorplanning, dump) {
        Ok(x) => x,
        Err(e) => {
            println!("[ERROR PSM-0087] {e}");
            return Err(ExitCode::from(1));
        }
    };
    if log.empty {
        println!("[WARNING PSM-0086] Net {net} is empty.");
        if let Some(code) = write_error_file(db, net, error_file.as_deref(), &network, &Default::default(), &[]) {
            return Err(code);
        }
        return Err(connectivity_failed(net));
    }
    if stats {
        for l in network.stats_lines() {
            println!("{l}");
        }
    }
    if require_bterm && !vyges_psm::check::check_bterms(net_input) {
        println!("{}", vyges_psm::check::missing_bterm_message(net_input));
        return Err(ExitCode::from(1));
    }
    let visited = match vyges_psm::check::check_open(&network) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("vyges-psm: the open check cannot start: {e}");
            return Err(refuse("an open check from an unconnected start node"));
        }
    };
    let mut connected = true;
    let mut open = vyges_psm::check::OpenReport::default();
    if !vyges_psm::check::is_connected(&network, &visited) {
        open = vyges_psm::check::report_unconnected_nodes(&network, net_input, &visited);
        for l in &open.lines {
            println!("{l}");
        }
        connected = false;
    }
    let shorts = vyges_psm::read::short_inputs(db, net).and_then(|si| vyges_psm::short::check_short(&network, &si, !floorplanning));
    let (shorts, stopped) = match shorts {
        Ok(x) => x,
        Err(e) => {
            eprintln!("vyges-psm: the short check cannot measure {e}");
            return Err(refuse("this short check"));
        }
    };
    for l in vyges_psm::short::report_shorts(&network, net, &shorts, stopped) {
        println!("{l}");
    }
    if !shorts.is_empty() {
        connected = false;
    }
    if let Some(code) = write_error_file(db, net, error_file.as_deref(), &network, &open, &shorts) {
        return Err(code);
    }
    if !connected {
        return Err(connectivity_failed(net));
    }
    println!("[INFO PSM-0040] All shapes on net {net} are connected.");
    Ok(network)
}

/// What `analyze` is given besides the design and the net: the session's settings as the harness
/// or the user states them.
#[derive(Default)]
struct AnalyzeOpts {
    vsrc: Option<String>,
    source_type: Option<String>,
    source_settings: Vec<String>,
    net_voltages: Vec<String>,
    inst_power: Option<String>,
    user_powers: Vec<String>,
    liberties: Vec<String>,
    sdcs: Vec<String>,
    layer_rcs: Vec<String>,
    corner: Option<String>,
}

/// `analyze`: the connectivity check (terminals not required, not floorplanning), then the solve in
/// its order: the solve (or, with `--allow-reuse`, the saved solution of this net and corner), its
/// report, EM, the instance voltage file.
fn analyze(args: &[String]) -> ExitCode {
    let (mut design, mut net, mut dump_dir, mut error_file) = (None, None, None, None);
    let (mut voltage_file, mut enable_em, mut em_file) = (None, false, None::<String>);
    let (mut save_solution, mut allow_reuse) = (None::<String>, false);
    let mut o = AnalyzeOpts::default();
    let mut it = args.iter();
    while let Some(a) = it.next() {
        let mut value = |what: &str| it.next().cloned().ok_or_else(|| usage_error(&format!("{a} needs {what}")));
        let r: Result<(), ExitCode> = (|| {
            match a.as_str() {
                "--net" => net = Some(value("a net name")?),
                "--error-file" => error_file = Some(value("a file")?),
                "--dump-dir" => dump_dir = Some(std::path::PathBuf::from(value("a directory")?)),
                "--vsrc" => o.vsrc = Some(value("a file")?),
                "--source-type" => o.source_type = Some(value("FULL, STRAPS or BUMPS")?),
                "--source-settings" => o.source_settings.push(value("settings")?),
                "--net-voltage" => o.net_voltages.push(value("NET=VOLTS@CORNER")?),
                "--inst-power" => o.inst_power = Some(value("a file")?),
                "--user-power" => o.user_powers.push(value("INST=WATTS@CORNER")?),
                "--liberty" => o.liberties.push(value("a file")?),
                "--sdc" => o.sdcs.push(value("a file")?),
                "--layer-rc" => o.layer_rcs.push(value("set_layer_rc arguments")?),
                "--corner" => o.corner = Some(value("a corner name")?),
                "--voltage-file" => voltage_file = Some(value("a file")?),
                "--enable-em" => enable_em = true,
                "--em-file" => em_file = Some(value("a file")?),
                "--save-solution" => save_solution = Some(value("a file")?),
                "--allow-reuse" => allow_reuse = true,
                v if v.starts_with('-') => return Err(usage_error(&format!("unknown option `{v}`"))),
                v if design.is_none() => design = Some(v.to_string()),
                v => return Err(usage_error(&format!("unexpected argument `{v}`"))),
            }
            Ok(())
        })();
        if let Err(code) = r {
            return code;
        }
    }
    let (Some(design), Some(net)) = (design, net) else { return usage_error("analyze needs <design.odb> and --net") };
    if em_file.is_some() && !enable_em {
        return fail(&["[ERROR PSM-0055] EM file cannot be specified without enabling em analysis.".to_string()]);
    }
    let db = match vyges_opendb::Db::open(&design) {
        Ok(db) => db,
        Err(e) => {
            eprintln!("vyges-psm: cannot read {design}: {e}");
            return ExitCode::from(2);
        }
    };
    let read = vyges_psm::read::tech(&db).and_then(|t| {
        let n = vyges_psm::read::net(&db, &net)?;
        let s = vyges_psm::read::solve_inputs(&db, &net)?;
        Ok((t, n, s))
    });
    let (tech, net_input, inputs) = match read {
        Ok(x) => x,
        Err(e) => {
            eprintln!("vyges-psm: cannot read net {net}: {e}");
            return ExitCode::from(2);
        }
    };
    let session = match analyze_session(&db, &o) {
        Ok(s) => s,
        Err(code) => return code,
    };
    let opts = Connectivity { floorplanning: false, require_bterm: false, stats: false, dump_dir: dump_dir.clone(), error_file };
    let network = match check_connectivity(&db, &tech, &net_input, &opts) {
        Ok(n) => n,
        Err(code) => return code,
    };
    use vyges_psm::solve;
    let net_id = db.net_id(&net).unwrap_or(0);
    let mut store = match &save_solution {
        Some(path) => match load_store(path, &design) {
            Ok(s) => Some(s),
            Err(code) => return code,
        },
        None => None,
    };
    let c = Call { net: &net, net_id, tech: &tech, net_input: &net_input, inputs: &inputs, session: &session, o: &o, network: &network, dump_dir: dump_dir.as_deref() };
    let previous = store.as_ref().and_then(|s| s.get(&net, &session.corner)).cloned();
    let solved = match previous.filter(|p| allow_reuse && p.has_solution) {
        Some(p) => {
            println!("[INFO PSM-0011] Reusing previous solution");
            Solved { voltages: p.voltages.into_iter().collect(), src_voltage: p.src_voltage, total_power: p.total_power, cond: None }
        }
        None => match solve_net(&c, &mut store, save_solution.as_deref()) {
            Ok(s) => s,
            Err(code) => return code,
        },
    };
    if let Err(code) = report(&c, &solved) {
        return code;
    }
    if enable_em {
        if let Err(code) = report_em(&c, &solved, em_file.as_deref()) {
            return code;
        }
    }
    if let Some(path) = voltage_file {
        let Some(lines) = solve::instance_voltage_lines(&network, &net_input, tech.dbus, &solved.voltages) else {
            return refuse("a terminal node outside the matrix");
        };
        let mut text = lines.join("\n");
        text.push('\n');
        if std::fs::write(&path, text).is_err() {
            return fail(&[format!("[ERROR PSM-0090] Unable to open {path} to write instance voltage file")]);
        }
    }
    ExitCode::SUCCESS
}

/// What one `analyze` call reads.
struct Call<'a> {
    net: &'a str,
    net_id: u32,
    tech: &'a vyges_psm::input::Tech,
    net_input: &'a vyges_psm::input::NetInput,
    inputs: &'a vyges_psm::solve::SolveInputs,
    session: &'a Session,
    o: &'a AnalyzeOpts,
    network: &'a vyges_psm::network::Network<'a>,
    dump_dir: Option<&'a std::path::Path>,
}

/// A net's solution on the corner: solved now, or reused (then without a conductance map).
struct Solved {
    voltages: std::collections::HashMap<vyges_psm::network::NodeId, f64>,
    src_voltage: f64,
    total_power: f32,
    cond: Option<Vec<(vyges_psm::network::ConnId, f64)>>,
}

/// The session's solution store; a missing file is an empty store.
fn load_store(path: &str, design: &str) -> Result<vyges_psm::solution::Store, ExitCode> {
    use vyges_psm::solution::{fingerprint, Store};
    let bytes = std::fs::read(design).map_err(|e| {
        eprintln!("vyges-psm: cannot read {design}: {e}");
        ExitCode::from(2)
    })?;
    let fp = fingerprint(&bytes);
    match std::fs::read_to_string(path) {
        Err(e) if e.kind() == std::io::ErrorKind::NotFound => Ok(Store { design: fp, solutions: Vec::new() }),
        Err(e) => {
            eprintln!("vyges-psm: cannot read {path}: {e}");
            Err(ExitCode::from(2))
        }
        Ok(text) => Store::parse(&text, fp).map_err(|e| {
            eprintln!("vyges-psm: {path}: {e}");
            ExitCode::from(2)
        }),
    }
}

fn save_store(store: &Option<vyges_psm::solution::Store>, path: Option<&str>) -> Result<(), ExitCode> {
    if let (Some(s), Some(p)) = (store, path) {
        if let Err(e) = std::fs::write(p, s.text()) {
            eprintln!("vyges-psm: cannot write {p}: {e}");
            return Err(ExitCode::from(2));
        }
    }
    Ok(())
}

/// Each connection's resistance on the corner: the layer RC estimate, then the map.
fn resistances(c: &Call) -> Result<Vec<f64>, ExitCode> {
    use vyges_psm::solve;
    let s = c.session;
    let estimate = solve::layer_rc_resistance(&s.layer_rcs, &s.corner, s.units, |n| c.tech.layers.iter().position(|l| l.name == n));
    let estimate = estimate.map_err(|e| {
        eprintln!("vyges-psm: {e}");
        refuse("this layer RC setting")
    })?;
    Ok(solve::resistance_map(c.inputs, c.tech.dbus, &estimate))
}

/// The solve (`IRSolver::solve`), in its order. The power net's voltage is the previous solution's
/// on this corner when there is one; the stored solution is reset once the resistances are checked
/// and replaced at the end.
fn solve_net(c: &Call, store: &mut Option<vyges_psm::solution::Store>, save: Option<&str>) -> Result<Solved, ExitCode> {
    use vyges_psm::solve;
    let (s, network, net) = (c.session, c.network, c.net);
    let previous = store.as_ref().and_then(|st| st.get(net, &s.corner)).map(|p| p.src_voltage);
    let power_voltage = || solve::power_net_voltage(c.inputs.sig, c.net_id, &s.voltages, &s.corner, previous, s.pvt);
    let res = resistances(c)?;
    solve::assert_resistance_map(network, c.inputs, &res).map_err(|lines| fail(&lines))?;
    if let Some(st) = store.as_mut() {
        st.reset(net, &s.corner);
    }
    save_store(store, save)?;
    let net_voltage = solve::net_voltage(c.inputs.sig, power_voltage)
        .ok_or_else(|| fail(&[format!("[ERROR PSM-0079] Cannot determine the supply voltage for {net}.")]))?;
    let mut srcs = solve::Sources::default();
    let mut log = solve::SourceLog::default();
    let found = solve::generate_source_nodes(network, c.inputs, &s.settings, s.source_type, c.o.vsrc.as_deref(), net_voltage, &mut srcs, &mut log);
    let (found, src_voltage) = found.map_err(|lines| fail(&lines))?;
    for l in &log.lines {
        println!("{l}");
    }
    let (real, sources) = solve::split_sources(network, &s.settings, found, &mut srcs);
    let cond = solve::conductance_map(network, &network.connections, &res);
    if let Some(dir) = c.dump_dir {
        write_dump(dir, "cond", &solve::conductance_lines(network, c.net_input, &cond));
    }
    let src_conds = solve::source_conductances(network, &srcs, &s.settings, &real);
    let power_voltage = power_voltage().ok_or_else(|| fail(&[format!("[ERROR PSM-0079] Cannot determine the supply voltage for {net}.")]))?;
    if power_voltage == 0.0 {
        return Err(fail(&["[ERROR PSM-0074] Unable to determine voltage for power nets.".to_string()]));
    }
    let (currents, total_power) = solve::build_node_current_map(network, c.inputs, c.net_input, power_voltage, &s.timer_power, &s.user_power);
    let order = solve::assign_node_ids(network, &cond, &srcs, &real, &sources);
    let m = solve::build_cond_matrix_and_voltages(network, &srcs, &cond, &src_conds, &order, &currents, src_voltage, &sources);
    if let Some(dir) = c.dump_dir {
        write_dump(dir, "nodes", &solve::node_lines(network, c.net_input, &srcs, &order));
        write_dump(dir, "G", &solve::matrix_lines(&m));
        write_dump(dir, "J", &solve::vector_lines("J", &m.j));
        write_dump(dir, "G_bits", &solve::matrix_bits_lines(&m));
        write_dump(dir, "J_bits", &solve::vector_bits_lines(&m.j));
    }
    let Ok(lu) = vyges_sparselu::SparseLu::compute(&solve::csc_of(&m), vyges_sparselu::REFERENCE_CACHES) else {
        return Err(refuse("a structurally singular conductance matrix"));
    };
    let (_, _, v) = lu.solve_stages(&m.j);
    if let Some(dir) = c.dump_dir {
        write_dump(dir, "lu", &lu.stage_lines(&m.j));
        write_dump(dir, "V", &solve::vector_lines("V", &v));
    }
    let voltages = solve::node_voltages(&order, &v);
    if let Some(st) = store.as_mut() {
        let mut kept: Vec<(vyges_psm::network::NodeId, f64)> = voltages.iter().map(|(&k, &x)| (k, x)).collect();
        kept.sort_by_key(|p| p.0);
        st.put(vyges_psm::solution::Solution {
            net: net.to_string(),
            corner: s.corner.clone(),
            src_voltage,
            total_power,
            has_solution: !voltages.is_empty() && !currents.is_empty(),
            voltages: kept,
        });
    }
    save_store(store, save)?;
    Ok(Solved { voltages, src_voltage, total_power, cond: Some(cond) })
}

/// The IR report (`report`): the power net's voltage is now the solution's when the net is it.
fn report(c: &Call, sol: &Solved) -> Result<(), ExitCode> {
    use vyges_psm::solve;
    let s = c.session;
    let power_voltage = solve::power_net_voltage(c.inputs.sig, c.net_id, &s.voltages, &s.corner, Some(sol.src_voltage), s.pvt)
        .ok_or_else(|| fail(&[format!("[ERROR PSM-0079] Cannot determine the supply voltage for {}.", c.net)]))?;
    let results = solve::get_solution(&c.network.nodes, &sol.voltages, sol.src_voltage, sol.total_power, power_voltage)
        .ok_or_else(|| refuse("a grid node outside the matrix"))?;
    let report = solve::report_lines(c.net, &s.corner, &results);
    for l in &report {
        println!("{l}");
    }
    if let Some(dir) = c.dump_dir {
        write_dump(dir, "report", &report);
    }
    Ok(())
}

/// The EM report and file (`reportEM`, `writeEMFile`): the conductances are the corner's, taken
/// again when the solution was reused.
fn report_em(c: &Call, sol: &Solved, em_file: Option<&str>) -> Result<(), ExitCode> {
    use vyges_psm::solve;
    let cond = match &sol.cond {
        Some(cond) => cond.clone(),
        None => solve::conductance_map(c.network, &c.network.connections, &resistances(c)?),
    };
    let currents = solve::current_map(c.network, &cond, &sol.voltages).ok_or_else(|| refuse("a connection node outside the matrix"))?;
    let em = solve::em_report_lines(c.net, &c.session.corner, &solve::get_em_solution(&currents));
    for l in &em {
        println!("{l}");
    }
    if let Some(dir) = c.dump_dir {
        write_dump(dir, "em", &em);
    }
    if let Some(path) = em_file {
        let mut text = solve::em_file_lines(c.network, c.tech.dbus, &currents).join("\n");
        text.push('\n');
        if std::fs::write(path, text).is_err() {
            return Err(fail(&[format!("[ERROR PSM-0091] Unable to open {path} to write EM file")]));
        }
    }
    Ok(())
}

/// Print a failure's lines; exit 1.
fn fail(lines: &[String]) -> ExitCode {
    for l in lines {
        println!("{l}");
    }
    ExitCode::from(1)
}

/// The session state `analyze` runs in, resolved for its corner.
struct Session {
    corner: String,
    settings: vyges_psm::solve::SourceSettings,
    source_type: vyges_psm::solve::SourceType,
    voltages: Vec<vyges_psm::solve::UserVoltage>,
    timer_power: std::collections::HashMap<String, f32>,
    user_power: std::collections::HashMap<String, f32>,
    pvt: Option<f32>,
    units: vyges_psm::solve::RcUnits,
    layer_rcs: Vec<Vec<String>>,
}

fn analyze_session(db: &vyges_opendb::Db, o: &AnalyzeOpts) -> Result<Session, ExitCode> {
    use vyges_psm::solve;
    let corner = o.corner.clone().unwrap_or_else(|| "default".into());
    let source_type = match o.source_type.as_deref() {
        None => solve::SourceType::Bumps,
        Some(t) => solve::SourceType::parse(t).ok_or_else(|| usage_error(&format!("unknown source type `{t}`")))?,
    };
    let mut settings = solve::SourceSettings::default();
    for s in &o.source_settings {
        settings.apply(&parse_source_settings(s).map_err(|e| usage_error(&e))?);
    }
    // `NAME=VALUE@CORNER`, the corner optional (the analysed one).
    let split = |s: &str| -> Option<(String, String, String)> {
        let (name, rest) = s.split_once('=')?;
        let (v, c) = rest.split_once('@').unwrap_or((rest, corner.as_str()));
        Some((name.to_string(), v.to_string(), c.to_string()))
    };
    let mut voltages = Vec::new();
    for s in &o.net_voltages {
        let (name, v, c) = split(s).ok_or_else(|| usage_error(&format!("bad --net-voltage `{s}`")))?;
        let voltage = v.parse::<f64>().map_err(|_| usage_error(&format!("bad voltage `{v}`")))?;
        let net_id = db.net_id(&name).map_err(|e| usage_error(&format!("net {name}: {e}")))?;
        voltages.push(solve::UserVoltage { net_id, power: db.net_sigtype(&name) == "POWER", corner: c, voltage });
    }
    let mut user_power = std::collections::HashMap::new();
    for s in &o.user_powers {
        let (name, v, c) = split(s).ok_or_else(|| usage_error(&format!("bad --user-power `{s}`")))?;
        let power = v.parse::<f32>().map_err(|_| usage_error(&format!("bad power `{v}`")))?;
        if c == corner {
            user_power.insert(name, power);
        }
    }
    let mut timer_power = std::collections::HashMap::new();
    if let Some(f) = &o.inst_power {
        let text = std::fs::read_to_string(f).map_err(|e| usage_error(&format!("cannot read {f}: {e}")))?;
        for line in text.lines() {
            let mut w = line.split_whitespace();
            if let (Some(name), Some(v)) = (w.next(), w.next()) {
                let p = v.parse::<f32>().map_err(|_| usage_error(&format!("bad power `{v}` in {f}")))?;
                timer_power.insert(name.to_string(), p);
            }
        }
    }
    for f in &o.sdcs {
        let text = std::fs::read_to_string(f).map_err(|e| usage_error(&format!("cannot read {f}: {e}")))?;
        if text.lines().any(|l| matches!(l.split_whitespace().next(), Some("set_voltage" | "set_operating_conditions"))) {
            eprintln!("vyges-psm: {f} sets a voltage or operating conditions; reading them is not built");
            return Err(refuse("an SDC supply voltage"));
        }
    }
    let (pvt, units) = match o.liberties.first() {
        None => (None, solve::RcUnits::default()),
        Some(f) => {
            let text = read_maybe_gz(f).map_err(|e| usage_error(&format!("cannot read {f}: {e}")))?;
            let g = vyges_sta::liberty_parse::parse(&text).map_err(|e| usage_error(&format!("cannot parse {f}: {e}")))?;
            let lib = g.groups_of("library").next().unwrap_or(&g);
            let text_attr = |n: &str| lib.attr_text(n);
            let units = solve::RcUnits {
                resistance: solve::read_unit(text_attr("pulling_resistance_unit").as_deref(), "ohm", 1.0),
                distance: solve::read_unit(text_attr("distance_unit").as_deref(), "m", 1e-6),
            };
            (solve::pvt_voltage(lib), units)
        }
    };
    let layer_rcs = o.layer_rcs.iter().map(|r| r.split_whitespace().map(String::from).collect()).collect();
    Ok(Session { corner, settings, source_type, voltages, timer_power, user_power, pvt, units, layer_rcs })
}

/// One settings command's words: `-bump_dx N -bump_dy N -bump_size N -bump_interval N
/// -strap_track_pitch N -external_resistance R`; an option not given is 0 (leave as is).
fn parse_source_settings(s: &str) -> Result<vyges_psm::solve::SourceSettings, String> {
    let mut out = vyges_psm::solve::SourceSettings { bump_dx: 0, bump_dy: 0, bump_size: 0, bump_interval: 0, strap_track_pitch: 0, resistance: 0.0 };
    let mut w = s.split_whitespace();
    while let Some(k) = w.next() {
        let v = w.next().ok_or_else(|| format!("{k} needs a value"))?;
        let int = || v.parse::<i32>().map_err(|_| format!("{k} `{v}` is not an integer"));
        match k {
            "-bump_dx" => out.bump_dx = int()?,
            "-bump_dy" => out.bump_dy = int()?,
            "-bump_size" => out.bump_size = int()?,
            "-bump_interval" => out.bump_interval = int()?,
            "-strap_track_pitch" => out.strap_track_pitch = int()?,
            "-external_resistance" => out.resistance = v.parse::<f32>().map_err(|_| format!("{k} `{v}` is not a number"))?,
            _ => return Err(format!("unknown source setting {k}")),
        }
    }
    Ok(out)
}

/// A text file, inflated first when it is gzipped.
fn read_maybe_gz(path: &str) -> Result<String, String> {
    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    if bytes.len() < 18 || bytes[0] != 0x1f || bytes[1] != 0x8b {
        return Ok(String::from_utf8_lossy(&bytes).into_owned());
    }
    let flags = bytes[3];
    let mut at = 10;
    if flags & 4 != 0 {
        let n = usize::from(bytes[at]) | usize::from(bytes[at + 1]) << 8;
        at += 2 + n;
    }
    for bit in [8u8, 16] {
        if flags & bit != 0 {
            while at < bytes.len() && bytes[at] != 0 {
                at += 1;
            }
            at += 1;
        }
    }
    if flags & 2 != 0 {
        at += 2;
    }
    let raw = bytes.get(at..).ok_or("a truncated gzip header")?;
    let out = miniz_oxide::inflate::decompress_to_vec(raw).map_err(|e| format!("gzip: {e:?}"))?;
    Ok(String::from_utf8_lossy(&out).into_owned())
}

fn write_dump(dir: &std::path::Path, name: &str, lines: &[String]) {
    let mut text = lines.join("\n");
    text.push('\n');
    if let Err(e) = std::fs::write(dir.join(format!("psm_{name}.txt")), text) {
        eprintln!("vyges-psm: cannot write the {name} dump: {e}");
    }
}

/// The error file, when asked for: the check's markers recorded and the net's group written. `Some`
/// exit code when it could not be.
fn write_error_file(
    db: &vyges_opendb::Db,
    net: &str,
    path: Option<&str>,
    network: &vyges_psm::network::Network,
    open: &vyges_psm::check::OpenReport,
    shorts: &[vyges_psm::short::Short],
) -> Option<ExitCode> {
    let path = path?;
    let (v, names) = match vyges_psm::check::error_file_stream(network, open, shorts) {
        Ok(x) => x,
        Err(e) => {
            eprintln!("vyges-psm: the error file cannot hold {e}");
            return Some(refuse("this error file"));
        }
    };
    if let Err(e) = db.psm_write_error_file(net, &v, &names, path) {
        eprintln!("vyges-psm: cannot write the error file {path}: {e}");
        return Some(ExitCode::from(2));
    }
    None
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
