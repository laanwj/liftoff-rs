//! Physics submodels. Each is a pure-Rust module that takes scalar
//! inputs and returns scalar/vector outputs — no Godot types, so we
//! can unit-test them without a runtime.

pub mod battery;
pub mod drag;
pub mod ground;
pub mod motor;
pub mod thrust;
pub mod wake;
