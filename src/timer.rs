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
use vyges_sta::liberty::Library;
use vyges_sta::power::{self, Activities, ActivityInputs};
use vyges_sta::search::Search;

/// The libraries (in read order, as text) and constraints (each file's text) of the case.
pub struct TimerInputs<'t> {
    pub liberties: &'t [String],
    pub sdcs: &'t [String],
}

/// `ensureActivities` over the design: the activity of every pin the propagation reached.
pub fn activities(db: &Db, inp: &TimerInputs<'_>) -> Result<Activities, String> {
    let mut liberty = vyges_est::liberty::LibertyClocks::default();
    let mut libs = Libs::default();
    for text in inp.liberties {
        liberty.read(text)?;
        let group = vyges_sta::liberty_parse::parse(text)?;
        if group.simple.iter().any(|(n, _)| n.starts_with("k_")) || group.groups_of("scaling_factors").next().is_some() {
            return Err("liberty k_ scale factors are not modelled".into());
        }
        libs.libs.push(Library::read(&group)?);
    }
    let lib0 = libs.default_library().ok_or("the instance power needs a liberty library")?;
    let time_scale = lib0.time_scale;
    let [sdc_text] = inp.sdcs else {
        return Err(format!("{} constraint files: one is modelled", inp.sdcs.len()));
    };
    let s = vyges_loom::sdc::Sdc::parse(sdc_text).map_err(|e| format!("constraints: {e:?}"))?;
    let netlist = vyges_grt::timer::netlist(db);
    let constants = vyges_rsz::env::sim_constants(db, &liberty)?;
    let clock_sources: Vec<String> = s.clocks.iter().filter(|c| !c.is_virtual()).flat_map(|c| c.sources.iter().cloned()).collect();
    let mut env = vyges_rsz::env::sdc_env(&s, &libs, &netlist)?;
    env.constants = constants;
    let mpins = vyges_rsz::env::master_pins(db)?;
    let mut g = vyges_rsz::repair_design::timer_graph(&libs, 0, &netlist, &env, &mpins).map_err(|e| e.message().to_string())?;
    let propagated = !s.propagated.is_empty();
    if !propagated {
        g.ideal_clock = vyges_rsz::timing::clock_pins(&g, &clock_sources).into_iter().collect();
    }
    g.find_delays(&HashMap::new(), None)?;
    let ssdc = vyges_rsz::env::search_sdc_on(&s, &netlist, time_scale, propagated, false)?;
    let mut search = Search::in_graph_order(&g, &ssdc);
    search.constraints_modelled()?;
    search.find_arrivals()?;

    let n = g.vertices.len();
    let level = g.levels()?;
    let clock_source: Vec<bool> = g.vertices.iter().map(|v| clock_sources.contains(&v.name)).collect();
    let is_clock: Vec<bool> = (0..n).map(|v| search.is_clock(v)).collect();
    let clock: Vec<Option<power::Activity>> = (0..n)
        .map(|v| if is_clock[v] { search.find_clk(v).map(|c| power::clock_activity(c.period, c.waveform[0], c.waveform[1])) } else { None })
        .collect();
    let min_rf_slew: Vec<f32> = (0..n).map(|v| min_rf_slew(&g.slew[v])).collect();
    let min_period = if ssdc.clocks.is_empty() { 0.0 } else { ssdc.clocks.iter().map(|c| c.period).fold(f32::INFINITY, f32::min) };
    let inputs = ActivityInputs { level: &level, clock_source: &clock_source, is_clock: &is_clock, clock: &clock, min_rf_slew: &min_rf_slew, min_period, time_unit: time_scale };
    power::ensure_activities(&g, &inputs)
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
