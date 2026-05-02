//! Motor model.
//!
//! `CurveMotor` — a curve-shaped throttle→RPM map with a
//! first-order spool-up filter, an ESC idle-throttle floor, and a
//! brownout multiplier. Higher voltage → higher RPM, higher throttle
//! → higher RPM, but actual RPM lags the target by `spoolup_tau_s`.
//!
//! Future work behind the same `MotorModel` trait:
//! - `BldcMotor` — full V/I/R electrical chain with EMF and current.
//! - KV-drop curves (compressibility, prop loading) layered on top.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MotorParams {
    /// KV constant: unloaded RPM per volt.
    pub kv: f32,
    /// Loading factor in `[0, 1]`: at full throttle and full pack
    /// voltage, the *loaded* RPM = kv × V × loading_factor. Captures
    /// the "real motor with a prop on it spins slower than KV would
    /// suggest" effect without modelling current/torque explicitly.
    pub loading_factor: f32,
    /// First-order spool-up time constant, seconds. After a step in
    /// commanded throttle, RPM reaches ~63 % of the new target after
    /// this many seconds.
    pub spoolup_tau_s: f32,
    /// ESC idle floor: when armed, motor command is held at least
    /// this high to keep the rotor spinning. 0.0 disables.
    pub idle_throttle_floor: f32,
}

impl MotorParams {
    /// Realistic 5" racing motor: 2400 KV, ~85 % loaded efficiency,
    /// 30 ms spool-up.
    pub const RACING_5INCH: MotorParams = MotorParams {
        kv: 2400.0,
        loading_factor: 0.85,
        spoolup_tau_s: 0.030,
        idle_throttle_floor: 0.05,
    };
}

#[derive(Debug, Clone, Copy)]
pub struct MotorState {
    /// Current shaft speed, RPM.
    pub rpm: f32,
    /// What the motor would settle at given the current command and
    /// voltage, RPM. (Useful for diagnostics.)
    pub rpm_target: f32,
    /// The motor command actually applied this tick after idle floor
    /// and brownout. `[0, 1]`.
    pub effective_throttle: f32,
}

#[derive(Debug, Clone)]
pub struct CurveMotor {
    pub params: MotorParams,
    rpm: f32,
}

impl CurveMotor {
    pub fn new(params: MotorParams) -> Self {
        Self { params, rpm: 0.0 }
    }

    pub fn rpm(&self) -> f32 {
        self.rpm
    }

    pub fn reset(&mut self) {
        self.rpm = 0.0;
    }

    /// Step forward by `dt` seconds.
    ///
    /// - `mixer_command` ∈ `[0, 1]` is what the FC mixer commanded.
    /// - `v_pack` is current pack voltage (volts).
    /// - `brownout_factor` ∈ `[0, 1]` from the battery.
    /// - `armed` gates the idle floor: if false, motor goes to 0
    ///   regardless of mixer.
    pub fn step(
        &mut self,
        mixer_command: f32,
        v_pack: f32,
        brownout_factor: f32,
        armed: bool,
        dt: f32,
    ) -> MotorState {
        let cmd_clamped = mixer_command.clamp(0.0, 1.0);

        let effective = if !armed {
            0.0
        } else {
            cmd_clamped.max(self.params.idle_throttle_floor) * brownout_factor.clamp(0.0, 1.0)
        };

        // Loaded target RPM scales linearly with effective throttle and
        // pack voltage.
        let rpm_target = effective * v_pack.max(0.0) * self.params.kv * self.params.loading_factor;

        // First-order lag: rpm += (target - rpm) * (1 - exp(-dt/tau)).
        let tau = self.params.spoolup_tau_s.max(1e-4);
        let alpha = (1.0 - (-dt / tau).exp()).clamp(0.0, 1.0);
        self.rpm += (rpm_target - self.rpm) * alpha;
        // Floor at 0 — RPM can't go negative.
        if self.rpm < 0.0 {
            self.rpm = 0.0;
        }

        MotorState {
            rpm: self.rpm,
            rpm_target,
            effective_throttle: effective,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn motor() -> CurveMotor {
        CurveMotor::new(MotorParams::RACING_5INCH)
    }

    #[test]
    fn disarmed_motor_stays_at_zero() {
        let mut m = motor();
        // Even with full throttle, full voltage, healthy battery, and
        // many ticks, while disarmed the motor must be 0.
        for _ in 0..1000 {
            m.step(1.0, 16.8, 1.0, false, 0.01);
        }
        assert_eq!(m.rpm(), 0.0);
    }

    #[test]
    fn armed_zero_command_holds_idle_floor() {
        let mut m = motor();
        for _ in 0..1000 {
            m.step(0.0, 16.8, 1.0, true, 0.01);
        }
        // Idle floor 0.05, full pack 16.8 V, KV 2400, loading 0.85
        // ⇒ steady-state ≈ 0.05 × 16.8 × 2400 × 0.85 = 1713.6 RPM.
        let expected = 0.05 * 16.8 * 2400.0 * 0.85;
        assert_abs_diff_eq!(m.rpm(), expected, epsilon = 1.0);
    }

    #[test]
    fn full_throttle_full_voltage_steady_state() {
        let mut m = motor();
        for _ in 0..1000 {
            m.step(1.0, 16.8, 1.0, true, 0.01);
        }
        // 1.0 × 16.8 V × 2400 KV × 0.85 = 34272 RPM
        let expected = 1.0 * 16.8 * 2400.0 * 0.85;
        assert_abs_diff_eq!(m.rpm(), expected, epsilon = 5.0);
    }

    #[test]
    fn rpm_scales_with_voltage() {
        let mut m1 = motor();
        let mut m2 = motor();
        for _ in 0..1000 {
            m1.step(0.5, 16.8, 1.0, true, 0.01);
            m2.step(0.5, 12.6, 1.0, true, 0.01);
        }
        let ratio = m2.rpm() / m1.rpm();
        assert_abs_diff_eq!(ratio, 12.6 / 16.8, epsilon = 0.01);
    }

    #[test]
    fn brownout_scales_motor_output() {
        let mut m1 = motor();
        let mut m2 = motor();
        for _ in 0..1000 {
            m1.step(0.5, 16.8, 1.0, true, 0.01);
            m2.step(0.5, 16.8, 0.5, true, 0.01);
        }
        // Brownout 0.5 should halve the steady-state RPM (it scales
        // effective throttle, which scales target RPM linearly).
        assert_abs_diff_eq!(m2.rpm() / m1.rpm(), 0.5, epsilon = 0.01);
    }

    #[test]
    fn spoolup_lag_first_order() {
        let mut m = motor();
        // After exactly tau seconds with a step input, RPM should
        // reach about 63.2 % of target.
        let tau = MotorParams::RACING_5INCH.spoolup_tau_s;
        let target = 1.0 * 16.8 * 2400.0 * 0.85;
        // Use single big step approx: measure after tau / dt steps.
        let dt = 0.001;
        let n_steps = (tau / dt).round() as usize;
        for _ in 0..n_steps {
            m.step(1.0, 16.8, 1.0, true, dt);
        }
        let frac = m.rpm() / target;
        assert!(
            frac > 0.55 && frac < 0.70,
            "expected ~63 % after one tau, got {frac}"
        );
    }

    #[test]
    fn rpm_never_negative() {
        let mut m = motor();
        for _ in 0..100 {
            m.step(0.0, 0.0, 1.0, true, 0.01);
            assert!(m.rpm() >= 0.0);
        }
    }

    #[test]
    fn reset_zeros_rpm() {
        let mut m = motor();
        m.step(1.0, 16.8, 1.0, true, 1.0);
        assert!(m.rpm() > 0.0);
        m.reset();
        assert_eq!(m.rpm(), 0.0);
    }

    #[test]
    fn over_unit_command_clamped() {
        let mut m1 = motor();
        let mut m2 = motor();
        for _ in 0..1000 {
            m1.step(1.0, 16.8, 1.0, true, 0.01);
            m2.step(2.0, 16.8, 1.0, true, 0.01);
        }
        assert_abs_diff_eq!(m1.rpm(), m2.rpm(), epsilon = 1.0);
    }
}
