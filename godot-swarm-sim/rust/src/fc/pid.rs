//! Single-axis rate PID with anti-windup decay.
//!
//! The controller takes a target rate (deg/s) and a measured rate
//! (deg/s) and produces a normalised mixer command in `[-1, +1]`. The
//! integrator is leak-decayed each tick rather than hard-clamped to
//! avoid the discontinuities that pure clamping introduces.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidGains {
    pub p: f32,
    pub i: f32,
    pub d: f32,
}

impl PidGains {
    pub const fn new(p: f32, i: f32, d: f32) -> Self {
        Self { p, i, d }
    }
}

#[derive(Debug, Clone)]
pub struct PidController {
    pub gains: PidGains,
    /// Per-tick decay applied to the I-term, in `(0, 1]`. 1.0 = no
    /// decay (pure integration); 0.98 = 2 % decay per tick. The plan
    /// doc calls this anti-windup decay.
    pub i_decay: f32,
    /// Hard clamp on the integrator for safety.
    pub i_max: f32,
    /// Hard clamp on the final output.
    pub out_max: f32,

    integral: f32,
    last_error: f32,
    primed: bool,
}

impl PidController {
    pub fn new(gains: PidGains) -> Self {
        Self {
            gains,
            i_decay: 0.999,
            i_max: 50.0,
            out_max: 1.0,
            integral: 0.0,
            last_error: 0.0,
            primed: false,
        }
    }

    pub fn with_decay(mut self, decay: f32) -> Self {
        self.i_decay = decay;
        self
    }

    pub fn with_clamps(mut self, i_max: f32, out_max: f32) -> Self {
        self.i_max = i_max;
        self.out_max = out_max;
        self
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.last_error = 0.0;
        self.primed = false;
    }

    pub fn integral(&self) -> f32 {
        self.integral
    }

    pub fn last_error(&self) -> f32 {
        self.last_error
    }

    /// Run one PID step. `setpoint` and `measured` should be in the
    /// same units (deg/s for rate PIDs); `dt` is seconds. Output is
    /// clamped to `±out_max`.
    pub fn step(&mut self, setpoint: f32, measured: f32, dt: f32) -> f32 {
        let error = setpoint - measured;

        // Integrator with decay then accumulate, then clamp.
        self.integral = (self.integral * self.i_decay) + error * dt;
        self.integral = self.integral.clamp(-self.i_max, self.i_max);

        // Derivative on error (not on measurement) — simpler for tests
        // and matches the textbook PID. Skip on the very first tick
        // because last_error is zero by default and would produce a
        // spurious large derivative if the setpoint starts non-zero.
        let derivative = if self.primed {
            (error - self.last_error) / dt.max(1e-9)
        } else {
            0.0
        };

        self.last_error = error;
        self.primed = true;

        let out = self.gains.p * error + self.gains.i * self.integral + self.gains.d * derivative;
        out.clamp(-self.out_max, self.out_max)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn p_only(p: f32) -> PidController {
        PidController::new(PidGains::new(p, 0.0, 0.0))
    }

    #[test]
    fn zero_error_zero_output() {
        let mut pid = p_only(1.0);
        assert_abs_diff_eq!(pid.step(0.0, 0.0, 0.01), 0.0, epsilon = 1e-9);
    }

    #[test]
    fn p_only_proportional_to_error() {
        let mut pid = p_only(0.005).with_clamps(50.0, 100.0);
        // setpoint - measured = 100 deg/s, P = 0.005 → 0.5
        let out = pid.step(100.0, 0.0, 0.01);
        assert_abs_diff_eq!(out, 0.5, epsilon = 1e-6);
    }

    #[test]
    fn output_clamps_to_out_max() {
        let mut pid = p_only(1.0).with_clamps(50.0, 0.7);
        let out = pid.step(10.0, 0.0, 0.01);
        assert_abs_diff_eq!(out, 0.7, epsilon = 1e-6);
        let out_neg = pid.step(-10.0, 0.0, 0.01);
        assert_abs_diff_eq!(out_neg, -0.7, epsilon = 1e-6);
    }

    #[test]
    fn integral_builds_under_persistent_error() {
        let mut pid = PidController::new(PidGains::new(0.0, 1.0, 0.0))
            .with_decay(1.0)
            .with_clamps(1000.0, 1000.0);
        for _ in 0..100 {
            pid.step(10.0, 0.0, 0.01);
        }
        // 100 ticks × 0.01s × 10 deg/s = 10.0 accumulated error-seconds.
        assert_abs_diff_eq!(pid.integral(), 10.0, epsilon = 1e-3);
    }

    #[test]
    fn integral_decays_under_anti_windup() {
        let mut pid = PidController::new(PidGains::new(0.0, 1.0, 0.0))
            .with_decay(0.9)
            .with_clamps(1000.0, 1000.0);
        // Charge the integrator with one big-error tick.
        pid.step(10.0, 0.0, 0.1);
        let after_charge = pid.integral();
        // Now run 50 ticks with zero error; integrator should decay.
        for _ in 0..50 {
            pid.step(0.0, 0.0, 0.01);
        }
        let after_decay = pid.integral();
        assert!(
            after_decay.abs() < after_charge.abs() * 0.01,
            "integral should decay (after_charge={}, after_decay={})",
            after_charge,
            after_decay
        );
    }

    #[test]
    fn integral_hard_clamped() {
        let mut pid = PidController::new(PidGains::new(0.0, 1.0, 0.0))
            .with_decay(1.0)
            .with_clamps(5.0, 1000.0);
        for _ in 0..1000 {
            pid.step(1000.0, 0.0, 0.01);
        }
        assert!(pid.integral().abs() <= 5.0 + 1e-6);
    }

    #[test]
    fn derivative_skipped_on_first_step() {
        // Big setpoint right away — D term must NOT produce a kick.
        let mut pid = PidController::new(PidGains::new(0.0, 0.0, 1.0))
            .with_clamps(1000.0, 1000.0);
        let out = pid.step(100.0, 0.0, 0.01);
        assert_abs_diff_eq!(out, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn derivative_active_after_first_step() {
        let mut pid = PidController::new(PidGains::new(0.0, 0.0, 1.0))
            .with_clamps(1000.0, 100_000.0);
        // Establish baseline error.
        pid.step(100.0, 0.0, 0.01);
        // Error jumps from 100 → 200 in 0.01s. Derivative ≈ 10000.
        let out = pid.step(200.0, 0.0, 0.01);
        assert_abs_diff_eq!(out, 10000.0, epsilon = 1e-3);
    }

    #[test]
    fn reset_clears_state() {
        let mut pid = PidController::new(PidGains::new(0.5, 0.5, 0.5))
            .with_clamps(1000.0, 1000.0);
        for _ in 0..50 {
            pid.step(10.0, 0.0, 0.01);
        }
        pid.reset();
        assert_eq!(pid.integral(), 0.0);
        assert_eq!(pid.last_error(), 0.0);
        // First post-reset step has no derivative kick.
        let out = pid.step(50.0, 0.0, 0.01);
        // Only the P term contributes: 0.5 * 50 = 25, then clamped... but i_max didn't constrain.
        let expected_i_after = 50.0 * 0.01;
        let expected: f32 = 0.5 * 50.0 + 0.5 * expected_i_after;
        assert_abs_diff_eq!(out, expected.min(1000.0_f32), epsilon = 1e-3);
    }

    #[test]
    fn closed_loop_drives_error_toward_zero() {
        // Toy first-order plant: dmeasured = (cmd*K − measured) · α.
        // We use modest gains, a wide output clamp (no saturation), and
        // pure integration (no I-decay) so a PI controller can null the
        // steady-state error.
        let plant_k: f32 = 50.0; // cmd=1 ⇒ steady at 50
        let plant_alpha: f32 = 0.05;
        let dt = 1.0_f32 / 240.0;

        let mut pid = PidController::new(PidGains::new(0.005, 0.05, 0.0))
            .with_decay(1.0)
            .with_clamps(1000.0, 100.0);
        let setpoint = 30.0_f32;
        let mut measured = 0.0_f32;
        let mut errors = Vec::new();
        for _ in 0..6000 {
            let cmd = pid.step(setpoint, measured, dt);
            let target = cmd * plant_k;
            measured += (target - measured) * plant_alpha;
            errors.push((setpoint - measured).abs());
        }
        let final_err = (setpoint - measured).abs();
        assert!(
            final_err < 1.0,
            "expected steady-state error < 1.0, got {final_err}; final measured={measured}"
        );
        // Final error should be much smaller than initial.
        assert!(errors.last().unwrap() < &(errors[0] * 0.05));
    }
}
