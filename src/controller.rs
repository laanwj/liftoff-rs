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
    /// X-quad mix matrix. Use one of the [`mixer`] presets
    /// ([`mixer::MIXER_MATRIX_PROPS_IN`] /
    /// [`mixer::MIXER_MATRIX_PROPS_OUT`]) or a custom matrix.
    pub mixer_matrix: [[f32; 4]; 4],
    /// Per-motor output ceiling, applied as a uniform scalar to every
    /// motor command after the mixer (so per-axis authority is
    /// preserved within the capped range). `1.0` = no cap; `0.5` =
    /// half the wire ceiling. Equivalent to Betaflight's
    /// `motor_output_limit`.
    pub motor_output_limit: f32,
    /// Motor idle floor — when armed, no motor command goes below this
    /// fraction of full throttle. Keeps motors spinning fast enough for
    /// clean sensorless BEMF detection on the ESC; without it the very
    /// low end of the armed throttle range (DShot ~48) cogs visibly
    /// instead of spinning smoothly. Applied after `motor_output_limit`
    /// and only when armed (disarmed motors stay at 0). Mirrors
    /// Betaflight's `motor_idle`.
    ///
    /// Clamped at runtime to `[0.0, motor_output_limit]` so a high idle
    /// can never push motors past the upper cap.
    pub motor_idle: f32,
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
                roll: ActualAxis::new(150.0, 533.0, 0.50),
                pitch: ActualAxis::new(150.0, 533.0, 0.50),
                yaw: ActualAxis::new(150.0, 533.0, 0.50),
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
            mixer_matrix: mixer::MIXER_MATRIX_PROPS_IN,
            // Conservative bench/initial-flight cap. Bump to 1.0 once
            // the airframe is trusted.
            motor_output_limit: 0.5,
            // ~5 % throttle ≈ DShot 148 — above the sensorless BEMF
            // detection floor on typical BLHeli/AM32 ESCs, matches
            // Betaflight's default.
            motor_idle: 0.05,
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
        let mut motors = if armed {
            mixer::mix(
                &self.cfg.mixer_matrix,
                pid_output[0],
                pid_output[1],
                pid_output[2],
                throttle,
            )
        } else {
            [0.0; 4]
        };
        // Apply the uniform output ceiling (Betaflight-style
        // `motor_output_limit`). Capping post-mixer preserves
        // per-axis authority within the capped range.
        let limit = self.cfg.motor_output_limit.clamp(0.0, 1.0);
        for m in &mut motors {
            *m *= limit;
        }

        // Motor idle floor: when armed, never let any motor drop below
        // the idle level. Keeps the ESCs' sensorless commutation happy
        // at the low end of the armed throttle range. Skipped when
        // disarmed — the disarmed branch above already set motors to
        // `[0; 4]` and we want the ESCs to see DShot 0 (off), not idle.
        if armed {
            let idle = self.cfg.motor_idle.clamp(0.0, limit);
            for m in &mut motors {
                *m = crate::mathf::max(*m, idle);
            }
        }

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

    fn disarmed_input(throttle: f32) -> ControlInput {
        ControlInput {
            gyro: [0.0; 3],
            rc: RcInput::from_sticks(0.0, 0.0, throttle, 0.0, false, false),
            rc_link_age_us: 0,
        }
    }

    /// Take a fresh `Controller` past the gate's "arm switch must be
    /// seen low before the first rising edge" requirement: one tick
    /// with arm=false (rc.arm=false), then one tick with arm=true and
    /// throttle low — the gate transitions to armed.
    ///
    /// Tests of controller logic (mixer signs, PID sign, etc.) use a
    /// no-cap, no-idle config so the expected motor magnitudes match
    /// the mathematical ideal; cap and idle are exercised in their own
    /// tests.
    fn armed_controller() -> Controller {
        let cfg = ControllerConfig {
            motor_output_limit: 1.0,
            motor_idle: 0.0,
            ..ControllerConfig::default()
        };
        let mut c = Controller::new(cfg);
        c.step(&disarmed_input(0.0), 1.0 / 1000.0);
        c.step(&armed_input(0.0), 1.0 / 1000.0);
        c
    }

    #[test]
    fn disarmed_motors_zero() {
        let mut c = Controller::new(ControllerConfig::default());
        let out = c.step(&disarmed_input(0.0), 1.0 / 1000.0);
        assert!(!out.armed);
        assert_eq!(out.motors, [0.0; 4]);
    }

    #[test]
    fn boot_with_arm_latched_high_does_not_auto_arm() {
        // A power-cycle with the arm switch already up plus throttle
        // low must NOT arm — arming requires an explicit low → high
        // transition observed by the gate.
        let mut c = Controller::new(ControllerConfig::default());
        let out = c.step(&armed_input(0.0), 1.0 / 1000.0);
        assert!(!out.armed);
        assert_eq!(out.motors, [0.0; 4]);
    }

    #[test]
    fn raise_arm_with_throttle_high_then_drop_throttle_does_not_arm() {
        // Pilot raises arm switch while throttle is up; gate refuses.
        // Subsequently lowering throttle (switch still up) must NOT
        // silently arm — only a fresh switch low → high cycle does.
        let mut c = Controller::new(ControllerConfig::default());
        c.step(&disarmed_input(0.0), 1.0 / 1000.0);
        let out1 = c.step(&armed_input(0.5), 1.0 / 1000.0);
        assert!(!out1.armed);
        let out2 = c.step(&armed_input(0.0), 1.0 / 1000.0);
        assert!(!out2.armed);
        c.step(&disarmed_input(0.0), 1.0 / 1000.0);
        let out3 = c.step(&armed_input(0.0), 1.0 / 1000.0);
        assert!(out3.armed);
    }

    #[test]
    fn armed_centre_sticks_all_motors_equal_throttle() {
        let mut c = armed_controller();
        let out = c.step(&armed_input(0.5), 1.0 / 1000.0);
        assert!(out.armed);
        for m in out.motors {
            assert!((m - 0.5).abs() < 1e-4, "motor {m} != 0.5");
        }
    }

    #[test]
    fn motor_output_limit_scales_all_motors() {
        // With limit = 0.5 and a full-throttle command at centre sticks,
        // every motor should be at 0.5 (mixer outputs 1.0; cap halves it).
        let cfg = ControllerConfig {
            motor_output_limit: 0.5,
            ..ControllerConfig::default()
        };
        let mut c = Controller::new(cfg);
        c.step(&disarmed_input(0.0), 1.0 / 1000.0);
        c.step(&armed_input(0.0), 1.0 / 1000.0);
        let out = c.step(&armed_input(1.0), 1.0 / 1000.0);
        assert!(out.armed);
        for m in out.motors {
            assert!((m - 0.5).abs() < 1e-4, "motor {m} != 0.5 under cap=0.5");
        }
    }

    #[test]
    fn default_motor_output_limit_is_0_5() {
        // Sanity-check the crate default; firmware ships this value
        // for initial bench/flight testing.
        assert_eq!(ControllerConfig::default().motor_output_limit, 0.5);
    }

    #[test]
    fn motor_idle_applies_when_armed() {
        // At throttle 0 with sticks centred, motors should sit at the
        // idle floor (not 0) so the ESCs keep sensorless commutation
        // happy.
        let cfg = ControllerConfig {
            motor_idle: 0.05,
            motor_output_limit: 1.0, // isolate from the cap
            ..ControllerConfig::default()
        };
        let mut c = Controller::new(cfg);
        c.step(&disarmed_input(0.0), 1.0 / 1000.0);
        c.step(&armed_input(0.0), 1.0 / 1000.0);
        let out = c.step(&armed_input(0.0), 1.0 / 1000.0);
        assert!(out.armed);
        for m in out.motors {
            assert!((m - 0.05).abs() < 1e-4, "motor {m} != idle 0.05");
        }
    }

    #[test]
    fn motor_idle_does_not_apply_when_disarmed() {
        // Even with a very high idle value, disarmed motors must be
        // 0 (ESC sees DShot 0 = motor off).
        let cfg = ControllerConfig {
            motor_idle: 0.5,
            ..ControllerConfig::default()
        };
        let mut c = Controller::new(cfg);
        let out = c.step(&disarmed_input(0.0), 1.0 / 1000.0);
        assert!(!out.armed);
        assert_eq!(out.motors, [0.0; 4]);
    }

    #[test]
    fn motor_idle_clamped_to_motor_output_limit() {
        // If idle is set above the cap (misconfiguration), it must not
        // push motors past the cap — the runtime clamp keeps things
        // safe.
        let cfg = ControllerConfig {
            motor_idle: 0.8,         // higher than cap
            motor_output_limit: 0.3, // tight cap
            ..ControllerConfig::default()
        };
        let mut c = Controller::new(cfg);
        c.step(&disarmed_input(0.0), 1.0 / 1000.0);
        c.step(&armed_input(0.0), 1.0 / 1000.0);
        let out = c.step(&armed_input(0.0), 1.0 / 1000.0);
        for m in out.motors {
            assert!(m <= 0.3 + 1e-5, "motor {m} exceeded cap 0.3");
            assert!((m - 0.3).abs() < 1e-4, "motor {m} != cap 0.3");
        }
    }

    #[test]
    fn default_motor_idle_is_0_05() {
        // Sanity-check the crate default; the firmware ships this value
        // and Betaflight's default `motor_idle` is in the same range.
        assert_eq!(ControllerConfig::default().motor_idle, 0.05);
    }

    #[test]
    fn motor_output_never_exceeds_cap() {
        // Safety property: with a cap of 0.5, NO motor command leaves
        // `Controller::step` above 0.5, regardless of stick
        // combination, gyro disturbance, or arming state. Sweep a
        // representative grid of inputs.
        let cfg = ControllerConfig {
            motor_output_limit: 0.5,
            ..ControllerConfig::default()
        };
        let mut c = Controller::new(cfg);
        c.step(&disarmed_input(0.0), 1.0 / 1000.0);
        c.step(&armed_input(0.0), 1.0 / 1000.0);
        for roll in [-1.0, -0.5, 0.0, 0.5, 1.0] {
            for pitch in [-1.0, -0.5, 0.0, 0.5, 1.0] {
                for yaw in [-1.0, -0.5, 0.0, 0.5, 1.0] {
                    for thr in [0.0, 0.25, 0.5, 0.75, 1.0] {
                        for gyro_axis in 0..3 {
                            let mut input = armed_input(thr);
                            input.rc.roll = roll;
                            input.rc.pitch = pitch;
                            input.rc.yaw = yaw;
                            // Inject a large gyro on one axis so the PID
                            // produces non-trivial mixer input.
                            input.gyro[gyro_axis] = 10.0;
                            let out = c.step(&input, 1.0 / 1000.0);
                            for &m in &out.motors {
                                assert!(
                                    m <= 0.5 + 1e-5,
                                    "motor {m} > cap 0.5 \
                                     (roll={roll} pitch={pitch} yaw={yaw} \
                                      thr={thr} gyro_axis={gyro_axis})"
                                );
                                assert!(
                                    m >= 0.0,
                                    "motor {m} < 0 (ESC would interpret as disarmed-frame)"
                                );
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn link_loss_forces_disarm_and_zero_motors() {
        let mut c = armed_controller();
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
        let mut c = armed_controller();
        let mut a = armed_input(0.5);
        a.rc.roll = 0.5;
        let out_still = c.step(&a, 1.0 / 1000.0);

        let mut c2 = armed_controller();
        let mut b = armed_input(0.5);
        b.rc.roll = 0.5;
        b.gyro[0] = 5.0; // rad/s, rolling right
        let out_rolling = c2.step(&b, 1.0 / 1000.0);

        assert!(out_rolling.pid_output[0] < out_still.pid_output[0]);
    }
}
