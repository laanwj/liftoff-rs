//! godot-swarm-sim — multi-drone FPV simulator GDExtension.
//!
//! Pure-Rust simulation modules (testable without Godot) plus a thin
//! gdext layer that wires them into a `RigidBody3D`-based drone.
//!
//! Module map:
//! - `preset` — the all-in-one preset struct.
//! - The flight controller itself (rates / mode / PID / mixer / arming /
//!   RC decode) lives in the shared `quad-flight-control` crate, used
//!   directly by `pipeline` and friends.
//! - `sensor_sim` — turns rigid-body truth into simulated sensor
//!   readings (M1: gyro). The FC core sees only this, never truth.
//! - `physics::*` — submodels (motor, battery, thrust, drag, ground).
//! - `pipeline` — per-tick orchestrator tying it all together.
//! - `drone` — gdext class extending `RigidBody3D`.
//! - `crsf_io_trait` — abstract CRSF I/O interface trait.
//! - `zenoh_interface` — Zenoh-based CrsfIo node.
//! - `godot_input_interface` — keyboard-driven CrsfIo node (GodotInputInterface).
//! - `input_stub` — keyboard polling logic (used by `io_interface`).
//! - `preset_resource` — gdext `Resource` subclasses for `.tres` presets.

pub mod crsf_io;
pub mod crsf_io_trait;
pub mod damage;
pub mod input_router;
pub mod physics;
pub mod pipeline;
pub mod preset;
pub mod sensor_sim;

mod drone;
mod input_stub;
mod godot_input_interface;
mod preset_resource;
mod wake_node;
mod zenoh_interface;
mod zenoh_io;

use godot::prelude::*;

struct GodotSwarmSimExtension;

#[gdextension]
unsafe impl ExtensionLibrary for GodotSwarmSimExtension {}
