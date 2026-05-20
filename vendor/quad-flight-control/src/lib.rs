#![cfg_attr(not(test), no_std)]

//! Hardware- and OS-independent quad flight control.
//!
//! A stateful [`Controller`] whose [`step`](Controller::step) is called
//! at a fixed cadence with sensor + RC input and returns motor
//! commands. No I/O, no async, no HAL — the `quad-fc-firmware-rs`
//! firmware and the `godot-swarm-sim` SITL link this same crate and
//! differ only in what they feed `step()` and what they do with its
//! output.
//!
//! # Conventions
//!
//! - **Frame:** FRD body — x forward, y right, z down. Axis order is
//!   `[roll(x), pitch(y), yaw(z)]` everywhere.
//! - **All internal math is radians (SI).**
//! - **Degrees only at the user edges.** The pilot-facing rate tuning
//!   numbers ([`rates::ActualAxis`] `max_rate` / `center_sensitivity`)
//!   are deg/s because that is what a pilot enters, and
//!   [`ActualAxis::evaluate`](rates::ActualAxis::evaluate) converts to
//!   rad/s on output. Presentation layers (OSD / dashboard) convert
//!   rad → deg for display. The crate boundary itself is pure SI.
//! - **Motor output** is `[f32; 4]` in `0.0..=1.0` in the crate's
//!   logical mixer order (`M0..M3`, see [`mixer`]). Board-specific
//!   channel remap and the f32 → ESC mapping are the caller's job.
//!
//! M1 scope: acro (rate) mode + arming + RC-link-loss failsafe. The
//! estimator, angle/horizon modes, and the mode state machine are M2.

pub mod arming;
pub mod mixer;
pub mod mode;
mod mathf;
pub mod pid;
pub mod rates;
pub mod rc_input;

mod controller;

pub use controller::{
    AxisGains, AxisRates, ControlInput, ControlOutput, Controller, ControllerConfig,
};
pub use rc_input::RcInput;

/// Radians → degrees (and rad/s → deg/s).
pub const RAD_TO_DEG: f32 = 57.295_78;

/// Degrees → radians (and deg/s → rad/s).
pub const DEG_TO_RAD: f32 = 0.017_453_292;
