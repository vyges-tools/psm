// SPDX-License-Identifier: Apache-2.0
//! `vyges_psm` — power-grid analysis on the design database: connectivity check, static IR drop, per-node voltages, SPICE export.
//!
//! Stages are added here one at a time, each named after the step it performs and kept in the order
//! the command runs them.

pub mod check;
pub mod input;
pub mod network;
pub mod short;
pub mod solution;
pub mod solve;
#[cfg(feature = "odb")]
pub mod read;
