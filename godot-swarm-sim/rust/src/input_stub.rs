//! Stick input via Godot's built-in `Input` singleton.
//!
//! This is the *stub* path — direct keyboard polling so we can fly the
//! drone in the editor without any Zenoh / CRSF infrastructure.
//!
//! Mappings (chosen to mirror real-radio AETRA layout while playable
//! on a keyboard):
//!
//! | Action               | Key      | Channel |
//! |----------------------|----------|---------|
//! | Roll right           | D        | 1 (A)   |
//! | Roll left            | A        | 1 (A)   |
//! | Pitch up (nose up)   | S        | 2 (E)   |
//! | Pitch down           | W        | 2 (E)   |
//! | Throttle up          | Shift    | 3 (T)   |
//! | Throttle down        | Ctrl     | 3 (T)   |
//! | Yaw right            | E        | 4 (R)   |
//! | Yaw left             | Q        | 4 (R)   |
//! | Arm                  | Space    | 5 (AUX1)|
//! | Reset                | Enter    |   --    |
//!
//! Throttle is held: tapping Shift raises throttle by one step,
//! Ctrl lowers, otherwise it stays where it was. This matches how
//! real radios maintain throttle via a self-centering-disabled stick.

use godot::classes::Input;
use godot::global::Key;
use godot::prelude::*;

use crate::rc_input::RcInput;

/// Optional autofly profile for headless smoke tests / CI runs.
///
/// Configured via the `GSS_AUTOFLY` env var:
/// - `hover`: arm at t=0.5 s, ramp throttle to 0.5, hold straight.
/// - `climb`: arm + throttle 0.7 (drone rises through tick window).
/// - unset / unrecognised: keyboard input as normal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AutoflyMode {
    Hover,
    Climb,
}

impl AutoflyMode {
    pub fn from_env() -> Option<Self> {
        match std::env::var("GSS_AUTOFLY").ok()?.as_str() {
            "hover" => Some(AutoflyMode::Hover),
            "climb" => Some(AutoflyMode::Climb),
            _ => None,
        }
    }
}

/// Holds the latched throttle value (no auto-centring) and the
/// current arming state, plus an edge detector for the reset key.
#[derive(Debug, Clone)]
pub struct StickStub {
    /// Throttle held between calls. Always in `[0, 1]`.
    throttle: f32,
    /// Per-tick throttle change rate, units of throttle per second.
    pub throttle_rate: f32,
    /// Latched arm state, toggled on a Space-press edge.
    arm_latched: bool,
    /// Previous arm-key state for edge detection.
    arm_key_prev: bool,
    /// Previous reset-key state for edge detection.
    reset_key_prev: bool,
    /// Set true on the tick a reset edge fires; cleared by `consume_reset`.
    reset_pending: bool,
    /// Optional autofly profile (overrides keyboard when set).
    autofly: Option<AutoflyMode>,
    /// Wall-clock seconds since stub started (for autofly state machine).
    elapsed_s: f32,
}

impl Default for StickStub {
    fn default() -> Self {
        Self {
            throttle: 0.0,
            throttle_rate: 0.6,
            arm_latched: false,
            arm_key_prev: false,
            reset_key_prev: false,
            reset_pending: false,
            autofly: AutoflyMode::from_env(),
            elapsed_s: 0.0,
        }
    }
}

impl StickStub {
    /// Poll the Godot Input singleton and update internal state.
    /// Returns a fully-decoded `RcInput`.
    pub fn poll(&mut self, dt: f32) -> RcInput {
        self.elapsed_s += dt;

        if let Some(mode) = self.autofly {
            return self.autofly_frame(mode);
        }

        let input = Input::singleton();

        let roll = axis(&input, Key::D, Key::A);
        let pitch = axis(&input, Key::S, Key::W);
        let yaw = axis(&input, Key::E, Key::Q);

        let up = input.is_key_pressed(Key::SHIFT);
        let down = input.is_key_pressed(Key::CTRL);
        let dthr = match (up, down) {
            (true, false) => self.throttle_rate,
            (false, true) => -self.throttle_rate,
            _ => 0.0,
        };
        self.throttle = (self.throttle + dthr * dt).clamp(0.0, 1.0);

        let arm_now = input.is_key_pressed(Key::SPACE);
        if arm_now && !self.arm_key_prev {
            self.arm_latched = !self.arm_latched;
        }
        self.arm_key_prev = arm_now;

        let reset_now = input.is_key_pressed(Key::ENTER);
        if reset_now && !self.reset_key_prev {
            self.reset_pending = true;
        }
        self.reset_key_prev = reset_now;

        RcInput::from_sticks(
            roll,
            pitch,
            self.throttle,
            yaw,
            self.arm_latched,
            false,
        )
    }

    pub fn consume_reset(&mut self) -> bool {
        let r = self.reset_pending;
        self.reset_pending = false;
        r
    }

    fn autofly_frame(&mut self, mode: AutoflyMode) -> RcInput {
        // Three-phase profile:
        //   t < 0.3 s: disarmed, throttle 0.
        //   0.3..0.6 s: arm switch high, throttle 0 (arming safe).
        //   t ≥ 0.6 s: ramp throttle to target.
        let target_thr: f32 = match mode {
            AutoflyMode::Hover => 0.5,
            AutoflyMode::Climb => 0.7,
        };

        let arm = self.elapsed_s >= 0.3;

        let stick = if self.elapsed_s < 0.6 {
            0.0
        } else {
            let t = ((self.elapsed_s - 0.6) / 0.4).clamp(0.0, 1.0);
            target_thr * t
        };

        self.throttle = stick;
        RcInput::from_sticks(0.0, 0.0, stick, 0.0, arm, false)
    }
}

fn axis(input: &Gd<Input>, positive: Key, negative: Key) -> f32 {
    let p = input.is_key_pressed(positive);
    let n = input.is_key_pressed(negative);
    match (p, n) {
        (true, false) => 1.0,
        (false, true) => -1.0,
        _ => 0.0,
    }
}


