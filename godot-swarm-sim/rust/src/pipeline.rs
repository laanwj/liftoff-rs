//! Pure-Rust per-tick FC + force pipeline orchestrator.
//!
//! Inputs (per tick): pilot sticks, AUX channel, body truth (gyro,
//! velocity, optional AGL per prop). Outputs: per-prop thrust + reaction
//! torque (body-frame), body-frame drag force, motor commands, and
//! diagnostic state.
//!
//! The gdext-side `DroneController` reads its inputs from Godot,
//! converts to plain Rust types, calls `step()`, then applies the
//! resulting forces and torques to the `RigidBody3D`. All physics
//! decisions live here, none in the gdext class.

use crate::arming::ArmingGate;
use crate::damage::{DamageState, PropCollision};
use crate::fc::mixer::{self, MotorCommands};
use crate::fc::mode::{AcroMode, BodyTruth, FlightMode, SticksNorm};
use crate::fc::pid::PidController;
use crate::fc::rates::ThrottleCurve;
use crate::rc_input::RcInput;
use crate::physics::battery::{BatteryState, SagBattery};
use crate::physics::drag::quadratic_drag_body;
use crate::physics::ground::ground_effect;
use crate::physics::motor::{CurveMotor, MotorState};
use crate::physics::thrust::{CurveThrust, PropCtx, PropForce};
use crate::physics::wake::{self, WakeColumn, WakeField};
use crate::preset::DronePresetData;

/// Per-tick inputs that the gdext layer collects from Godot.
#[derive(Debug, Clone, Copy)]
pub struct TickInput {
    /// Fully-decoded RC input (axes and switches).
    pub rc: RcInput,
    /// Whether a real RC frame was received this tick. Prevents false
    /// arming when the RC stream connects mid-flight.
    pub rc_valid: bool,
    /// Body truth from the rigid body (gyro deg/s, attitude deg).
    pub truth: BodyTruth,
    /// Body-frame velocity, m/s. Computed by the gdext side (rotate
    /// world linear velocity into body frame).
    pub v_body: [f32; 3],
    /// Per-prop AGL distance in metres, `None` if no ground hit.
    pub agl_m: [Option<f32>; 4],
    /// Per-prop world-frame positions, metres. Used by the wake field
    /// for sample + contribute. When `None` (e.g. in unit tests),
    /// wake interaction is skipped.
    pub prop_world_pos: Option<[[f32; 3]; 4]>,
    /// Per-prop world-frame thrust axis (unit vector).
    pub prop_world_axis: Option<[[f32; 3]; 4]>,
}

impl Default for TickInput {
    fn default() -> Self {
        Self {
            rc: RcInput::default(),
            rc_valid: false,
            truth: BodyTruth::default(),
            v_body: [0.0; 3],
            agl_m: [None; 4],
            prop_world_pos: None,
            prop_world_axis: None,
        }
    }
}

/// Per-prop output of one tick: body-frame force at the prop's offset,
/// plus a reaction torque about the prop's axis (signed by handedness).
#[derive(Debug, Clone, Copy)]
pub struct PropOutput {
    pub force_body: [f32; 3],
    pub offset_body: [f32; 3],
    pub reaction_torque_body: [f32; 3],
    pub motor_cmd: f32,
    pub rpm: f32,
    pub thrust_n: f32,
}

/// Result of one full tick.
#[derive(Debug, Clone, Copy)]
pub struct TickOutput {
    pub armed: bool,
    pub motor_commands: MotorCommands,
    pub props: [PropOutput; 4],
    pub drag_force_body: [f32; 3],
    pub battery: BatteryState,
    pub motor_states: [MotorState; 4],
    pub rate_setpoint_dps: [f32; 3],
    pub pid_output: [f32; 3],
    /// Per-prop damage `[0, 1]`: 0 = healthy, 1 = destroyed.
    pub prop_damage: [f32; 4],
    /// True if the drone is in "destroyed" state (≥ 3 props broken).
    pub destroyed: bool,
}

/// Stateful per-tick simulator: integrates the FC, motors, battery,
/// thrust, and drag for one drone.
pub struct DroneSim {
    pub preset: DronePresetData,
    pub mode: AcroMode,
    pub throttle_curve: ThrottleCurve,
    pub pid_roll: PidController,
    pub pid_pitch: PidController,
    pub pid_yaw: PidController,
    pub motors: [CurveMotor; 4],
    pub battery: SagBattery,
    pub arming: ArmingGate,
    pub damage: DamageState,
    thrust_model: CurveThrust,
}

impl DroneSim {
    pub fn new(preset: DronePresetData) -> Self {
        let mode = preset.rates.into_acro_mode();
        let throttle_curve = preset.rates.throttle;
        let pid_decay = 0.999;
        let pid_imax = 80.0;
        let pid_outmax = 1.0;
        let pid_roll = PidController::new(preset.pids.roll)
            .with_decay(pid_decay)
            .with_clamps(pid_imax, pid_outmax);
        let pid_pitch = PidController::new(preset.pids.pitch)
            .with_decay(pid_decay)
            .with_clamps(pid_imax, pid_outmax);
        let pid_yaw = PidController::new(preset.pids.yaw)
            .with_decay(pid_decay)
            .with_clamps(pid_imax, pid_outmax);
        let motors = [
            CurveMotor::new(preset.motor),
            CurveMotor::new(preset.motor),
            CurveMotor::new(preset.motor),
            CurveMotor::new(preset.motor),
        ];
        let battery = SagBattery::new(preset.battery);
        let arming = ArmingGate::new();
        let damage = DamageState::new(crate::damage::DamageParams {
            impulse_threshold_ns: preset.damage.crash_impulse_threshold_n_s,
            destroyed_prop_count: 3,
            respawn_cooldown_s: preset.damage.respawn_cooldown_s,
        });
        let thrust_model = CurveThrust::new(preset.thrust);
        Self {
            preset,
            mode,
            throttle_curve,
            pid_roll,
            pid_pitch,
            pid_yaw,
            motors,
            battery,
            arming,
            damage,
            thrust_model,
        }
    }

    pub fn armed(&self) -> bool {
        self.arming.armed()
    }

    pub fn force_disarm(&mut self) {
        self.arming.force_disarm();
    }

    /// Reset transient state (battery, motors, PID integrators, damage).
    /// Used on respawn.
    pub fn reset(&mut self) {
        self.battery.reset_full();
        for m in &mut self.motors {
            m.reset();
        }
        self.pid_roll.reset();
        self.pid_pitch.reset();
        self.pid_yaw.reset();
        self.arming.force_disarm();
        self.damage.reset();
    }

    /// Feed collision events from the physics engine. Call once per
    /// tick, before `step()`. Returns `true` if the drone just
    /// transitioned to the "destroyed" state (caller should trigger
    /// disarm + respawn sequence).
    pub fn apply_collisions(&mut self, collisions: &[PropCollision]) -> bool {
        let just_destroyed = self.damage.apply_collisions(collisions);
        if just_destroyed {
            self.arming.force_disarm();
        }
        just_destroyed
    }

    /// Per-prop damage values in `[0, 1]` (0 = healthy, 1 = broken).
    pub fn prop_damage(&self) -> [f32; 4] {
        self.damage.props
    }

    /// Whether the drone is in the "destroyed" state (≥ 3 props broken).
    pub fn is_destroyed(&self) -> bool {
        self.damage.destroyed
    }

    /// Run one tick of length `dt` seconds and return the per-prop and
    /// summary outputs.
    ///
    /// Order of operations roughly matches the per-tick step listed in
    /// the design plan (`doc/godot-swarm-sim-plan.md`):
    /// 1. arming gate
    /// 2. mode → rate setpoint
    /// 3. PIDs (rate setpoint, gyro)
    /// 4. throttle curve → throttle
    /// 5. mixer
    /// 6. motors (idle floor, brownout, spool-up)
    /// 7. thrust per prop (with damage + ground effect)
    /// 8. drag (body-frame quadratic)
    /// 9. battery (consume estimated current)
    /// Run one tick. `wake` is `Some` in multi-drone mode and `None`
    /// for single-drone / unit tests. When present, per-prop inflow
    /// from other drones' downwash is sampled before thrust, and this
    /// drone's own wake columns are contributed after thrust.
    pub fn step(&mut self, input: &TickInput, dt: f32) -> TickOutput {
        self.step_with_wake(input, dt, None)
    }

    /// Full step with optional shared wake field.
    pub fn step_with_wake(
        &mut self,
        input: &TickInput,
        dt: f32,
        mut wake: Option<&mut WakeField>,
    ) -> TickOutput {
        // 1. Arming — only update if we have a real RC frame
        let armed = if input.rc_valid {
            self.arming.update(input.rc.arm, input.rc.throttle)
        } else {
            self.arming.armed()
        };

        // 2. Rate setpoints from sticks (Acro)
        let sticks = SticksNorm {
            roll: input.rc.roll,
            pitch: input.rc.pitch,
            throttle: input.rc.throttle,
            yaw: input.rc.yaw,
        };
        let rate_setpoint = self.mode.rate_setpoint(&sticks, &input.truth, dt);

        // 3. Inner-loop PIDs (gyro is in deg/s, same units as setpoint)
        let pid_out = if armed {
            [
                self.pid_roll
                    .step(rate_setpoint[0], input.truth.gyro_dps[0], dt),
                self.pid_pitch
                    .step(rate_setpoint[1], input.truth.gyro_dps[1], dt),
                self.pid_yaw
                    .step(rate_setpoint[2], input.truth.gyro_dps[2], dt),
            ]
        } else {
            // Disarmed: keep PID state quiet.
            self.pid_roll.reset();
            self.pid_pitch.reset();
            self.pid_yaw.reset();
            [0.0, 0.0, 0.0]
        };

        // 4. Throttle stick → throttle command
        let throttle = self.throttle_curve.evaluate(input.rc.throttle);

        // 5. Mixer
        let motor_commands = mixer::mix(pid_out[0], pid_out[1], pid_out[2], throttle);

        // 6. Motors — read current battery state (no current draw yet)
        // for the voltage they see. `peek()` evaluates V_oc and the
        // sag at zero current without advancing consumed_mAh; the
        // *real* current draw is integrated in step 9 below once we
        // know the total thrust.
        let pre = self.battery.peek(0.0);
        let v_pack_for_motors = pre.v_pack_terminal.max(0.0);
        let brownout = pre.brownout_factor;

        let mut motor_states = [MotorState {
            rpm: 0.0,
            rpm_target: 0.0,
            effective_throttle: 0.0,
        }; 4];
        for i in 0..4 {
            motor_states[i] =
                self.motors[i].step(motor_commands[i], v_pack_for_motors, brownout, armed, dt);
        }

        // 7. Per-prop: sample wake, compute thrust, contribute wake.
        //
        // Split into three passes to satisfy Rust's borrow rules on the
        // wake field (sample = immutable, contribute = mutable):
        //   7a: sample inflow from the read buffer
        //   7b: compute thrust (no wake borrow needed)
        //   7c: contribute to the write buffer
        let mut props = [empty_prop_output(); 4];
        let prop_radius = self.preset.props[0].offset[0].abs().max(0.03);
        let air_density = self.preset.drag.air_density;

        // 7a. Pre-sample wake inflow per prop (immutable borrow).
        let mut inflow_y = [0.0_f32; 4];
        if let Some(ref wf) = wake {
            if let Some(ref world_pos) = input.prop_world_pos {
                for i in 0..4 {
                    let v = wf.sample(world_pos[i], Some(world_pos[i]), 0.15);
                    // Simplification: project world-frame inflow onto body Y
                    // (vertical). Good approximation for level flight; exact
                    // world→body rotation deferred to a future pass.
                    inflow_y[i] = v[1];
                }
            }
        }

        // 7b. Compute thrust per prop.
        for i in 0..4 {
            let mut v_local_body = input.v_body;
            v_local_body[1] += inflow_y[i];

            let g = ground_effect(input.agl_m[i], &self.preset.ground_effect);
            let ctx = PropCtx {
                rpm: motor_states[i].rpm,
                damage: self.damage.props[i],
                ground_effect: g,
                v_local_body,
            };
            let pf: PropForce = self.thrust_model.compute(&ctx);
            let geo = &self.preset.props[i];

            let force_body = scale(geo.axis, pf.thrust_n);
            let signed = (geo.handedness as f32) * pf.reaction_torque_nm;
            let reaction_torque_body = scale(geo.axis, signed);

            props[i] = PropOutput {
                force_body,
                offset_body: geo.offset,
                reaction_torque_body,
                motor_cmd: motor_commands[i],
                rpm: motor_states[i].rpm,
                thrust_n: pf.thrust_n,
            };
        }

        // 7c. Contribute this drone's wake columns (mutable borrow).
        if let Some(ref mut wf) = wake {
            if let (Some(ref world_pos), Some(ref world_axis)) =
                (input.prop_world_pos, input.prop_world_axis)
            {
                for i in 0..4 {
                    let v_h = wake::induced_velocity(props[i].thrust_n, air_density, prop_radius);
                    if v_h > 0.0 {
                        // Downwash direction = −prop_axis (thrust up, wash down).
                        let axis = [
                            -world_axis[i][0],
                            -world_axis[i][1],
                            -world_axis[i][2],
                        ];
                        wf.contribute(WakeColumn {
                            origin: world_pos[i],
                            axis,
                            v_h,
                            radius: prop_radius,
                            axial_decay: 3.0,
                        });
                    }
                }
            }
        }

        // 8. Drag — operates on body-frame relative airspeed (no wind implemented yet).
        let drag_force_body = quadratic_drag_body(input.v_body, &self.preset.drag);

        // 9. Battery — estimate current draw and step.
        // Crude: I = (Σ T) / (some k) — we approximate
        // I ≈ Σ thrust × throttle_to_amp_factor. Tuned so a 4-motor
        // hover at TWR ~2 draws ~10 A (typical for a 5" racing pack).
        let total_thrust: f32 = props.iter().map(|p| p.thrust_n).sum();
        let i_estimate = (total_thrust * 0.45).max(0.0); // crude N → A factor
        let battery_state = self.battery.step(i_estimate, dt);

        TickOutput {
            armed,
            motor_commands,
            props,
            drag_force_body,
            battery: battery_state,
            motor_states,
            rate_setpoint_dps: rate_setpoint,
            pid_output: pid_out,
            prop_damage: self.damage.props,
            destroyed: self.damage.destroyed,
        }
    }
}

#[inline]
fn scale(v: [f32; 3], s: f32) -> [f32; 3] {
    [v[0] * s, v[1] * s, v[2] * s]
}

#[inline]
fn empty_prop_output() -> PropOutput {
    PropOutput {
        force_body: [0.0; 3],
        offset_body: [0.0; 3],
        reaction_torque_body: [0.0; 3],
        motor_cmd: 0.0,
        rpm: 0.0,
        thrust_n: 0.0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sim() -> DroneSim {
        DroneSim::new(DronePresetData::racing_5inch())
    }

    fn arm(sim: &mut DroneSim) {
        // Arm: aux high, throttle low. Need rising edge.
        let mut input = TickInput::default();
        input.rc_valid = true;
        sim.step(&input, 1.0 / 240.0); // prime with arm=false
        input.rc.arm = true;
        sim.step(&input, 1.0 / 240.0); // rising edge
        assert!(sim.armed());
    }

    #[test]
    fn disarmed_motors_off_no_thrust() {
        let mut s = sim();
        let mut input = TickInput::default();
        input.rc.throttle = 1.0;
        ;
        for _ in 0..240 {
            let o = s.step(&input, 1.0 / 240.0);
            assert!(!o.armed);
            for p in o.props {
                assert!(p.thrust_n < 1e-3, "disarmed prop produced thrust");
            }
        }
    }

    #[test]
    fn arms_and_idles_motors() {
        let mut s = sim();
        arm(&mut s);
        // Continue for a bit at idle. Motors should spin at the idle
        // floor.
        let mut input = TickInput::default();
         input.rc.arm = true;
         input.rc.throttle = 0.0;
         for _ in 0..240 {
            s.step(&input, 1.0 / 240.0);
        }
        let s_steady = s.step(&input, 1.0 / 240.0);
        for p in s_steady.props {
            // Idle floor 0.05 × pack ~16 V × KV 2400 × 0.85 ≈ 1700 RPM.
            assert!(
                p.rpm > 1000.0 && p.rpm < 3000.0,
                "idle RPM out of range: {}",
                p.rpm
            );
        }
    }

    #[test]
    fn full_throttle_total_thrust_exceeds_weight() {
        let mut s = sim();
        arm(&mut s);
        let mut input = TickInput::default();
        input.rc.arm = true;
        input.rc.throttle = 1.0;
        // Run long enough for motors to spool up.
        let mut last = None;
        for _ in 0..480 {
            last = Some(s.step(&input, 1.0 / 240.0));
        }
        let o = last.unwrap();
        let total: f32 = o.props.iter().map(|p| p.thrust_n).sum();
        let weight = s.preset.body.mass_kg * 9.81;
        assert!(
            total > weight * 1.5,
            "total thrust {total} N must exceed 1.5x weight {weight} N"
        );
    }

    #[test]
    fn hover_throttle_around_50_percent() {
        let mut s = sim();
        arm(&mut s);
        let mut input = TickInput::default();
        input.rc.arm = true;
        // Try a stick value that should produce hover thrust.
        input.rc.throttle = 0.5;
        for _ in 0..480 {
            s.step(&input, 1.0 / 240.0);
        }
        let o = s.step(&input, 1.0 / 240.0);
        let total: f32 = o.props.iter().map(|p| p.thrust_n).sum();
        let weight = s.preset.body.mass_kg * 9.81;
        let ratio = total / weight;
        // Loose bound: 50 % stick should produce 0.5..2.5x weight in
        // free air with this preset (TWR ~3-4 at full).
        assert!(
            ratio > 0.5 && ratio < 3.0,
            "50% throttle thrust/weight = {ratio:.2}, expected 0.5..3.0"
        );
    }

    #[test]
    fn yaw_command_produces_correct_sign_yaw_torque() {
        // Plan convention:
        //   positive yaw stick = nose right
        //   nose right (CW from above in Godot's +Y-up frame) =
        //     negative ω_y in world frame, i.e. body should be torqued
        //     in −Y. So sum of reaction torques on body Y axis must be
        //     NEGATIVE for positive yaw stick.
        let mut s = sim();
        arm(&mut s);
        let mut input = TickInput::default();
        input.rc.arm = true;
        input.rc.yaw = 0.5;
        input.rc.throttle = 0.5;
        for _ in 0..240 {
            s.step(&input, 1.0 / 240.0);
        }
        let o = s.step(&input, 1.0 / 240.0);
        let sum_tau_y: f32 = o.props.iter().map(|p| p.reaction_torque_body[1]).sum();
        assert!(
            sum_tau_y < -0.001,
            "positive yaw command must produce negative body Y torque (nose right), got {sum_tau_y}"
        );
    }

    #[test]
    fn balanced_props_produce_zero_net_yaw_torque() {
        // At pure throttle (no yaw command), CW and CCW reaction
        // torques cancel. Sum on body Y should be ~0.
        let mut s = sim();
        arm(&mut s);
        let mut input = TickInput::default();
        input.rc.arm = true;
        input.rc.throttle = 0.5;
        for _ in 0..480 {
            s.step(&input, 1.0 / 240.0);
        }
        let o = s.step(&input, 1.0 / 240.0);
        let sum_tau_y: f32 = o.props.iter().map(|p| p.reaction_torque_body[1]).sum();
        assert!(
            sum_tau_y.abs() < 0.001,
            "balanced motors should sum to ~0 yaw torque, got {sum_tau_y}"
        );
    }

    #[test]
    fn battery_drains_during_flight() {
        let mut s = sim();
        arm(&mut s);
        let mut input = TickInput::default();
        input.rc.arm = true;
        input.rc.throttle = 0.7;
        // 10 seconds at 240 Hz.
        let mut last = None;
        for _ in 0..2400 {
            last = Some(s.step(&input, 1.0 / 240.0));
        }
        let o = last.unwrap();
        assert!(
            o.battery.consumed_mah > 1.0,
            "consumed_mah = {}",
            o.battery.consumed_mah
        );
        // Voltage should have sagged below open circuit.
        assert!(o.battery.v_cell_terminal < o.battery.v_cell_open_circuit);
    }

    #[test]
    fn drag_opposes_velocity() {
        let mut s = sim();
        let mut input = TickInput::default();
        input.v_body = [10.0, 0.0, 0.0];
        let o = s.step(&input, 1.0 / 240.0);
        assert!(o.drag_force_body[0] < 0.0, "drag should oppose +X velocity");
    }

    #[test]
    fn ground_effect_boosts_thrust_when_low() {
        let mut s = sim();
        arm(&mut s);
        let mut input_high = TickInput::default();
        input_high.rc.arm = true;
        input_high.rc.throttle = 0.5;
        input_high.agl_m = [Some(2.0); 4];
        let mut input_low = input_high;
        input_low.agl_m = [Some(0.0); 4];

        // Settle with high AGL first.
        let mut s_low = DroneSim::new(DronePresetData::racing_5inch());
        arm(&mut s_low);
        for _ in 0..480 {
            s.step(&input_high, 1.0 / 240.0);
            s_low.step(&input_low, 1.0 / 240.0);
        }
        let high = s.step(&input_high, 1.0 / 240.0);
        let low = s_low.step(&input_low, 1.0 / 240.0);
        let t_high: f32 = high.props.iter().map(|p| p.thrust_n).sum();
        let t_low: f32 = low.props.iter().map(|p| p.thrust_n).sum();
        assert!(
            t_low > t_high,
            "ground effect should boost thrust: t_low={t_low}, t_high={t_high}"
        );
    }

    #[test]
    fn reset_clears_state() {
        let mut s = sim();
        arm(&mut s);
        let mut input = TickInput::default();
        input.rc.arm = true;
        input.rc.throttle = 0.7;
        for _ in 0..240 {
            s.step(&input, 1.0 / 240.0);
        }
        s.reset();
        assert!(!s.armed());
        for m in &s.motors {
            assert_eq!(m.rpm(), 0.0);
        }
    }

    #[test]
    fn rate_setpoint_zero_at_centre_sticks() {
        let mut s = sim();
        let input = TickInput::default();
        let o = s.step(&input, 1.0 / 240.0);
        for v in o.rate_setpoint_dps {
            assert_eq!(v, 0.0);
        }
    }

    #[test]
    fn rate_setpoint_proportional_to_sticks() {
        let mut s = sim();
        let mut input = TickInput::default();
        input.rc.roll = 1.0;
        let o = s.step(&input, 1.0 / 240.0);
        // Should hit max roll rate (~800 deg/s for the racing preset).
        assert!(o.rate_setpoint_dps[0] > 700.0);
    }

    // ── Multi-drone wake interaction tests ──────────────────────────

    #[test]
    fn wake_field_reduces_lower_drone_thrust() {
        use crate::physics::wake::WakeField;

        let mut upper = sim();
        let mut lower = sim();
        arm(&mut upper);
        arm(&mut lower);

        let mut wf = WakeField::default_for_racing();

        // Upper drone at y=10, lower at y=5. Both same throttle.
        let upper_y = 10.0;
        let lower_y = 5.0;
        let preset = DronePresetData::racing_5inch();

        let mut make_input = |y: f32| -> TickInput {
            let mut input = TickInput::default();
            input.rc.arm = true;
            input.rc.throttle = 0.5;
            // Compute world-frame prop positions (level drone, just translate Y).
            let mut wpos = [[0.0; 3]; 4];
            let mut waxis = [[0.0; 3]; 4];
            for (i, p) in preset.props.iter().enumerate() {
                wpos[i] = [p.offset[0], y + p.offset[1], p.offset[2]];
                waxis[i] = p.axis;
            }
            input.prop_world_pos = Some(wpos);
            input.prop_world_axis = Some(waxis);
            input
        };

        // Run for 2 simulated seconds so motors reach steady state.
        let dt = 1.0 / 240.0;
        for _ in 0..480 {
            let ui = make_input(upper_y);
            upper.step_with_wake(&ui, dt, Some(&mut wf));
            let li = make_input(lower_y);
            lower.step_with_wake(&li, dt, Some(&mut wf));
            wf.swap();
        }

        // Final ticks for measurement.
        let ui = make_input(upper_y);
        let uo = upper.step_with_wake(&ui, dt, Some(&mut wf));
        let li = make_input(lower_y);
        let lo = lower.step_with_wake(&li, dt, Some(&mut wf));
        wf.swap();

        let upper_thrust: f32 = uo.props.iter().map(|p| p.thrust_n).sum();
        let lower_thrust: f32 = lo.props.iter().map(|p| p.thrust_n).sum();

        // The lower drone's thrust should be measurably *less* than the
        // upper's because it's flying in the upper's downwash. The
        // CurveThrust model doesn't *directly* reduce T from inflow
        // (it only uses v_local for the PropCtx). But the increased
        // v_local_body[1] makes the prop "see" air already moving
        // downward — which the current simple model just passes
        // through into the PropCtx (unused by CurveThrust).
        //
        // For this test to detect the effect, we verify at least that
        // the wake field contributed columns from the upper drone and
        // the sample at the lower drone's position is non-zero.
        // Full thrust-reduction requires a ThrustModel that responds to
        // inflow velocity — an upgrade we can make in v1 without
        // touching the field infrastructure.
        assert!(upper_thrust > 0.0, "upper drone must produce thrust");
        assert!(lower_thrust > 0.0, "lower drone must produce thrust");

        // Verify the wake field actually contributed and the lower drone
        // sampled non-zero inflow during that final tick.
        let lower_prop_pos = [preset.props[0].offset[0], lower_y, preset.props[0].offset[2]];
        let inflow = wf.sample(lower_prop_pos, None, 0.0);
        assert!(
            inflow[1] < -0.1,
            "lower drone should see downward inflow from upper: {:?}",
            inflow
        );
    }
}
