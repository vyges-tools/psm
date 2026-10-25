// SPDX-License-Identifier: Apache-2.0
//! The timer the instance power is read from, over the design database: the netlist as the
//! database holds it, the libraries, the constraints, the delays (the slews bound each pin's
//! transition density) and the arrivals (they name the clock at each clock pin).
//!
//! Built as the resizer builds its timer, for one scene and with no parasitics (no psm case
//! estimates or reads any: every net is lumped at its pin capacitance):
//! - the netlist from the database (instances in database order, each pin in its master's order);
//! - a clock is ideal unless the constraints propagate it, and an ideal clock's registers read
//!   the ideal clock slew, 0;
//! - the delays, then the arrivals (`powerPreamble`: arrivals first, then the clock network).

use std::collections::HashMap;

use vyges_opendb::Db;
use vyges_rsz::preamble::Libs;
use vyges_sta::liberty::{Direction, Library};
use vyges_sta::power::{self, Activities, ActivityInputs};
use vyges_sta::search::Search;

/// The corners (`define_corners`; none: one scene), the libraries in read order (each with the
/// corner it was read for, `None`: every corner, and its text) and the constraints (each file's
/// text) of the case.
pub struct TimerInputs<'t> {
    pub corners: &'t [String],
    pub liberties: &'t [(Option<String>, String)],
    pub sdcs: &'t [String],
}

/// `ensureActivities` over the design: the activity of every pin the propagation reached.
pub fn activities(db: &Db, inp: &TimerInputs<'_>) -> Result<Activities, String> {
    Ok(activities_and_powers(db, inp, None)?.0)
}

/// `ensureActivities`, then, for the scene given (`analyze_power_grid -corner`), `findInstPowers`:
/// each instance with a liberty cell, by name, in instance order.
pub fn activities_and_powers(db: &Db, inp: &TimerInputs<'_>, powers_scene: Option<usize>) -> Result<(Activities, Vec<(String, power::InstPower)>), String> {
    let mut liberty = vyges_est::liberty::LibertyClocks::default();
    let mut libs = Libs { scenes: inp.corners.to_vec(), scene_libs: vec![Vec::new(); inp.corners.len()], ..Libs::default() };
    for (corner, text) in inp.liberties {
        liberty.read(text)?;
        let group = vyges_sta::liberty_parse::parse(text)?;
        if group.simple.iter().any(|(n, _)| n.starts_with("k_")) || group.groups_of("scaling_factors").next().is_some() {
            return Err("liberty k_ scale factors are not modelled".into());
        }
        let lib = Library::read(&group)?;
        // `read_liberty -corner c`: that scene's library only; without it, every scene's.
        let scene = match corner {
            Some(c) => Some(inp.corners.iter().position(|s| s == c).ok_or_else(|| format!("read_liberty -corner {c}: corner not defined"))?),
            None => None,
        };
        if !libs.scene_libs.is_empty() {
            if libs.libs.is_empty() {
                libs.link_scene = scene;
            }
            for (k, sl) in libs.scene_libs.iter_mut().enumerate() {
                if scene.is_none_or(|c| c == k) {
                    sl.push(lib.clone());
                }
            }
        } else if scene.is_some() {
            return Err("read_liberty -corner without define_corners".into());
        }
        libs.libs.push(lib);
    }
    let lib0 = libs.default_library().ok_or("the instance power needs a liberty library")?;
    let time_scale = lib0.time_scale;
    // No constraints: no clock, so the default input activity is per time unit.
    let sdc_text = match inp.sdcs {
        [] => "",
        [one] => one.as_str(),
        _ => return Err(format!("{} constraint files: one is modelled", inp.sdcs.len())),
    };
    let mut s = vyges_loom::sdc::Sdc::parse(sdc_text).map_err(|e| format!("constraints: {e:?}"))?;
    let netlist = vyges_grt::timer::netlist(db);
    let constants = vyges_rsz::env::sim_constants(db, &liberty)?;
    let mpins = vyges_rsz::env::master_pins(db)?;
    let unlinked = unlinked_masters(db, &libs)?;
    // A constraint names a pin by its path name (`get_pins {a.b[0]/IN}`), the database by its
    // escaped name (`a\.b\[0\]/IN`): each clock source is resolved to the pin it names before the
    // timer, the arrival search or the propagation reads it.
    let mut by_path: HashMap<String, String> = netlist.ports.iter().map(|(p, _)| (power::path_name(p), p.clone())).collect();
    for inst in &netlist.insts {
        let (name, cell) = &**inst;
        let pins = mpins.get(cell).cloned().or_else(|| unlinked.get(cell).map(|u| u.iter().map(|(p, _)| p.clone()).collect())).unwrap_or_default();
        for p in pins {
            let pin = format!("{name}/{p}");
            by_path.insert(power::path_name(&pin), pin);
        }
    }
    for c in &mut s.clocks {
        for src in &mut c.sources {
            if let Some(name) = by_path.get(src.as_str()) {
                *src = name.clone();
            }
        }
    }
    let clock_sources: Vec<String> = s.clocks.iter().filter(|c| !c.is_virtual()).flat_map(|c| c.sources.iter().cloned()).collect();
    let mut env = vyges_rsz::env::sdc_env(&s, &libs, &netlist)?;
    env.constants = constants;
    let propagated = !s.propagated.is_empty();
    // One graph per scene, each timed with its own libraries: the slew bound reads every
    // analysis point. The propagation walks the first (every scene has the same graph).
    let mut graphs = Vec::new();
    for k in 0..libs.scene_count() {
        let mut g = vyges_sta::graph::Graph::build_with_unlinked(libs.scene_slice(k), &netlist, Some(&mpins), &unlinked)?;
        g.sdc = env.clone();
        if !propagated {
            g.ideal_clock = vyges_rsz::timing::clock_pins(&g, &clock_sources).into_iter().collect();
        }
        g.find_delays(&HashMap::new(), None)?;
        graphs.push(g);
    }
    let g = &graphs[0];
    let ssdc = vyges_rsz::env::search_sdc_on(&s, &netlist, time_scale, propagated, false)?;
    let mut search = Search::in_graph_order(g, &ssdc);
    search.constraints_modelled()?;
    search.find_arrivals()?;

    let n = g.vertices.len();
    let level = g.levels()?;
    let clock_source: Vec<bool> = g.vertices.iter().map(|v| clock_sources.contains(&v.name)).collect();
    let is_clock: Vec<bool> = (0..n).map(|v| search.is_clock(v)).collect();
    let clock: Vec<Option<power::Activity>> = (0..n)
        .map(|v| if is_clock[v] { search.find_clk(v).map(|c| power::clock_activity(c.period, c.waveform[0], c.waveform[1])) } else { None })
        .collect();
    let min_rf_slew: Vec<f32> = (0..n).map(|v| graphs.iter().map(|gk| min_rf_slew(&gk.slew[v])).fold(f32::INFINITY, f32::min)).collect();
    let min_period = if ssdc.clocks.is_empty() { 0.0 } else { ssdc.clocks.iter().map(|c| c.period).fold(f32::INFINITY, f32::min) };
    let inputs = ActivityInputs { level: &level, clock_source: &clock_source, is_clock: &is_clock, clock: &clock, min_rf_slew: &min_rf_slew, min_period, time_unit: time_scale };
    let act = power::ensure_activities(g, &inputs)?;
    let Some(k) = powers_scene else { return Ok((act, Vec::new())) };
    if sdc_text.lines().any(|l| l.split_whitespace().next() == Some("set_clock_transition")) {
        return Err("set_clock_transition (an ideal clock's slew) is not modelled".into());
    }
    let gk = graphs.get(k).ok_or_else(|| format!("scene {k} is not defined"))?;
    // A pin's vertices: its first, then a bidirect pin's driver (same name).
    let pin_drvr = |v: usize| -> Option<usize> {
        if gk.vertices[v].is_driver {
            Some(v)
        } else {
            gk.vertices.get(v + 1).filter(|w| w.is_driver && w.name == gk.vertices[v].name).map(|_| v + 1)
        }
    };
    let no_parasitics = HashMap::new();
    // `loadCap(pin, scene, max)` of each output pin: its pin loads and port loads, no wire.
    let load_cap: Vec<f32> = (0..n).map(|v| pin_drvr(v).map_or(0.0, |d| gk.load_cap(d, &no_parasitics))).collect();
    // `ClkNetwork::idealClkSlew`: without `set_clock_transition`, 0 on an ideal clock's pins.
    let ideal_clock_slew: Vec<Option<[f32; 2]>> = (0..n).map(|v| (gk.ideal_clock.contains(&v) || pin_drvr(v).is_some_and(|d| gk.ideal_clock.contains(&d))).then_some([0.0; 2])).collect();
    let pw = power::PowerInputs { constants: &gk.sdc.constants, ideal_clock_slew: &ideal_clock_slew, load_cap: &load_cap, link_libs: &libs.libs };
    let found = power::inst_powers(gk, &inputs, &act, &pw)?;
    let named = found.into_iter().enumerate().filter_map(|(i, p)| p.map(|p| (power::path_name(&netlist.insts[i].0), p))).collect();
    Ok((act, named))
}

/// The masters no library holds, each with its signal terminals in order and their directions
/// from the database (`dbNetwork::direction` of a pin with no liberty port: INOUT and FEEDTHRU are
/// bidirect). The timer makes their vertices too.
fn unlinked_masters(db: &Db, libs: &Libs) -> Result<HashMap<String, Vec<(String, Direction)>>, String> {
    let mut out = HashMap::new();
    for i in 0..db.num_masters().map_err(|e| e.to_string())? {
        let name = db.nth_master_name(i).map_err(|e| e.to_string())?;
        if libs.libs.iter().any(|l| l.cells.contains_key(&name)) {
            continue;
        }
        let supply: Vec<String> = db.master_mterms(&name).map_err(|e| e.to_string())?.into_iter().filter(|(_, t)| t == "POWER" || t == "GROUND").map(|(n, _)| n).collect();
        let pins = db
            .master_mterm_io(&name)
            .map_err(|e| e.to_string())?
            .into_iter()
            .filter(|(n, _)| !supply.contains(n))
            .map(|(n, io)| {
                let dir = match io.as_str() {
                    "INPUT" => Direction::Input,
                    "OUTPUT" => Direction::Output,
                    _ => Direction::Bidirect,
                };
                (n, dir)
            })
            .collect();
        out.insert(name, pins);
    }
    Ok(out)
}

/// `getMinRfSlew`: over the analysis points (one scene: min, then max), the least of the rise and
/// fall slews' mean: `rise + fall` in float, halved in double, narrowed.
pub fn min_rf_slew(slew: &[[f32; 2]; 2]) -> f32 {
    let mut least = f32::INFINITY;
    for mm in [vyges_sta::liberty::MIN, vyges_sta::liberty::MAX] {
        let avg = (f64::from(slew[vyges_sta::liberty::RISE][mm] + slew[vyges_sta::liberty::FALL][mm]) / 2.0) as f32;
        if avg < least {
            least = avg;
        }
    }
    least
}
