// SPDX-License-Identifier: Apache-2.0
//! `vyges_psm` — power-grid analysis on the design database: connectivity check, static IR drop, per-node voltages, SPICE export.
//!
//! Stages are added here one at a time, each named after the step it performs and kept in the order
//! the command runs them.

// The crate's own name inside it: the command-line entry ([`cli_main`]) is built into the
// library too, and its `vyges_psm::…` paths resolve here.
extern crate self as vyges_psm;

pub mod check;
pub mod input;
pub mod network;
pub mod short;
pub mod solution;
pub mod solve;
#[cfg(feature = "odb")]
pub mod read;
#[cfg(feature = "odb")]
pub mod timer;

/// The `vyges-psm` command-line entry (the binary's `main`), built into the library too so a
/// suite binary can link every engine and run the one it is invoked as.
#[cfg(feature = "suite")]
#[doc(hidden)]
#[path = "main.rs"]
pub mod cli_main;
