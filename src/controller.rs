//! Top-level [`Controller`]: the per-step orchestration of arming →
//! rates → rate-PID → mixer, plus RC-link-loss failsafe.

use crate::arming::ArmingGate;
use crate::mixer::{self, MotorCommands};
use crate::mode::{AcroMode, BodyTruth, FlightMode, SticksNorm};
use crate::pid::{PidController, PidGains};
use crate::rates::{ActualAxis, ThrottleCurve};
use crate::rc_input::RcInput;

/// Per-axis "Actual" rate spec (roll/pitch/yaw), deg/s.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisRates {
    pub roll: ActualAxis,
    pub pitch: ActualAxis,
    pub yaw: ActualAxis,
}

/// Per-axis rate-PID gains plus the clamps shared by all three axes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisGains {
    pub roll: PidGains,
    pub pitch: PidGains,
    pub yaw: PidGains,
    /// Per-tick I-term leak, `(0, 1]`. 1.0 = pure integration.
    pub i_decay: f32,
    /// Hard clamp on each integrator.
    pub i_max: f32,
    /// Hard clamp on each axis' PID output.
    pub out_max: f32,
}

/// Static tuning. Held separately from [`Controller`] state so a sim
/// can sweep gains and reset state independently.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControllerConfig {
    pub rates: AxisRates,
    pub gains: AxisGains,
    pub throttle: ThrottleCurve,
    /// RC link is considered lost when the last good frame is older
    /// than this many microseconds.
    pub failsafe_link_timeout_us: u32,
}

impl Default for ControllerConfig {
    /// Placeholder 5"-class acro tuning. The gains are starting points
    /// to be tuned in SITL, not flight-validated values.
    fn default() -> Self {
        Self {
            rates: AxisRates {
                roll: ActualAxis::new(100.0, 800.0, 0.0),
                pitch: ActualAxis::new(100.0, 800.0, 0.0),
                yaw: ActualAxis::new(50.0, 400.0, 0.0),
            },
            gains: AxisGains {
                roll: PidGains::new(0.0025, 0.02, 0.00015),
                pitch: PidGains::new(0.0025, 0.02, 0.00015),
                yaw: PidGains::new(0.003, 0.02, 0.0),
                i_decay: 0.999,
                i_max: 50.0,
                out_max: 1.0,
            },
            throttle: ThrottleCurve::linear(),
            // 0.5 s — matches the sim's RC freshness window.
            failsafe_link_timeout_us: 500_000,
        }
    }
}

/// Sensor + RC input for one control step. SI units, FRD body frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlInput {
    /// Body angular rate, **rad/s**, `[roll(x), pitch(y), yaw(z)]`.
    pub gyro: [f32; 3],
    /// Decoded RC input — normalised sticks + arm switch.
    pub rc: RcInput,
    /// Microseconds since the last valid RC frame. Drives failsafe.
    pub rc_link_age_us: u32,
}

/// Result of one control step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlOutput {
    /// Per-motor command `0.0..=1.0`, crate logical mixer order
    /// (`M0..M3`). The caller maps to its ESC channel order.
    pub motors: MotorCommands,
    pub armed: bool,
    /// Rate setpoint used this step, rad/s `[roll, pitch, yaw]`
    /// (telemetry).
    pub rate_setpoint: [f32; 3],
    /// Inner-loop PID output, normalised `[-1, 1]` (telemetry).
    pub pid_output: [f32; 3],
    /// True when the RC link is stale and failsafe is active.
    pub link_lost: bool,
}

/// Stateful flight controller. Construct with [`Controller::new`], then
/// call [`step`](Controller::step) at a steady cadence (~1 kHz).
pub struct Controller {
    cfg: ControllerConfig,
    mode: AcroMode,
    pid_roll: PidController,
    pid_pitch: PidController,
    pid_yaw: PidController,
    throttle: ThrottleCurve,
    arming: ArmingGate,
}

impl Controller {
    pub fn new(cfg: ControllerConfig) -> Self {
        let mk = |g: PidGains| {
            PidController::new(g)
                .with_decay(cfg.gains.i_decay)
                .with_clamps(cfg.gains.i_max, cfg.gains.out_max)
        };
        Self {
            mode: AcroMode::new(cfg.rates.roll, cfg.rates.pitch, cfg.rates.yaw),
            pid_roll: mk(cfg.gains.roll),
            pid_pitch: mk(cfg.gains.pitch),
            pid_yaw: mk(cfg.gains.yaw),
            throttle: cfg.throttle,
            arming: ArmingGate::new(),
            cfg,
        }
    }

    pub fn armed(&self) -> bool {
        self.arming.armed()
    }

    /// Clear the inner-loop integrators/derivative state.
    pub fn reset(&mut self) {
        self.pid_roll.reset();
        self.pid_pitch.reset();
        self.pid_yaw.reset();
    }

    /// Force-disarm: motors off and the arm switch must cycle low
    /// before re-arming is allowed. Used by the sim for crash/respawn,
    /// and by the firmware for hard-stop paths (over-current, etc.).
    pub fn force_disarm(&mut self) {
        self.arming.force_disarm();
        self.reset();
    }

    /// Run one control step. `dt` is seconds since the previous step.
    pub fn step(&mut self, input: &ControlInput, dt: f32) -> ControlOutput {
        // Failsafe: a stale RC link forces disarm (motors off, and the
        // arm switch must cycle through the gate to re-arm).
        let link_lost = input.rc_link_age_us > self.cfg.failsafe_link_timeout_us;
        let armed = if link_lost {
            self.arming.force_disarm();
            false
        } else {
            self.arming.update(input.rc.arm, input.rc.throttle)
        };

        // Sticks → rate setpoint (rad/s). Acro ignores body truth.
        let sticks = SticksNorm {
            roll: input.rc.roll,
            pitch: input.rc.pitch,
            throttle: input.rc.throttle,
            yaw: input.rc.yaw,
        };
        let rate_setpoint = self.mode.rate_setpoint(&sticks, &BodyTruth::default(), dt);

        // Setpoint and gyro are both rad/s — the rate loop is pure SI,
        // no conversion.
        let pid_output = if armed {
            [
                self.pid_roll.step(rate_setpoint[0], input.gyro[0], dt),
                self.pid_pitch.step(rate_setpoint[1], input.gyro[1], dt),
                self.pid_yaw.step(rate_setpoint[2], input.gyro[2], dt),
            ]
        } else {
            self.reset();
            [0.0, 0.0, 0.0]
        };

        let throttle = self.throttle.evaluate(input.rc.throttle);
        let motors = if armed {
            mixer::mix(pid_output[0], pid_output[1], pid_output[2], throttle)
        } else {
            [0.0; 4]
        };

        ControlOutput {
            motors,
            armed,
            rate_setpoint,
            pid_output,
            link_lost,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn armed_input(throttle: f32) -> ControlInput {
        ControlInput {
            gyro: [0.0; 3],
            rc: RcInput::from_sticks(0.0, 0.0, throttle, 0.0, true, false),
            rc_link_age_us: 0,
        }
    }

    #[test]
    fn disarmed_motors_zero() {
        let mut c = Controller::new(ControllerConfig::default());
        let mut input = armed_input(0.0);
        input.rc.arm = false;
        let out = c.step(&input, 1.0 / 1000.0);
        assert!(!out.armed);
        assert_eq!(out.motors, [0.0; 4]);
    }

    #[test]
    fn armed_centre_sticks_all_motors_equal_throttle() {
        let mut c = Controller::new(ControllerConfig::default());
        // Arm with throttle low, then a hover-ish throttle, sticks centred.
        c.step(&armed_input(0.0), 1.0 / 1000.0);
        let out = c.step(&armed_input(0.5), 1.0 / 1000.0);
        assert!(out.armed);
        for m in out.motors {
            assert!((m - 0.5).abs() < 1e-4, "motor {m} != 0.5");
        }
    }

    #[test]
    fn link_loss_forces_disarm_and_zero_motors() {
        let mut c = Controller::new(ControllerConfig::default());
        c.step(&armed_input(0.0), 1.0 / 1000.0);
        assert!(c.armed());
        let mut input = armed_input(0.4);
        input.rc_link_age_us = 1_000_000; // 1 s > 0.5 s timeout
        let out = c.step(&input, 1.0 / 1000.0);
        assert!(out.link_lost);
        assert!(!out.armed);
        assert_eq!(out.motors, [0.0; 4]);
    }

    #[test]
    fn gyro_rad_s_boundary_opposes_setpoint() {
        // Roll-right stick with a body already rolling right (positive
        // gyro) should yield less roll command than with zero rate —
        // proves the rad/s→deg/s conversion feeds the PID with the
        // right sign/scale.
        let mut c = Controller::new(ControllerConfig::default());
        c.step(&armed_input(0.0), 1.0 / 1000.0);
        let mut a = armed_input(0.5);
        a.rc.roll = 0.5;
        let out_still = c.step(&a, 1.0 / 1000.0);

        let mut c2 = Controller::new(ControllerConfig::default());
        c2.step(&armed_input(0.0), 1.0 / 1000.0);
        let mut b = armed_input(0.5);
        b.rc.roll = 0.5;
        b.gyro[0] = 5.0; // rad/s, rolling right
        let out_rolling = c2.step(&b, 1.0 / 1000.0);

        assert!(out_rolling.pid_output[0] < out_still.pid_output[0]);
    }
}
