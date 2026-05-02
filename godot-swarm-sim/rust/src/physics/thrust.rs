//! Per-prop thrust.
//!
//! Use `CurveThrust` — `T ≈ k_T · ω²` with optional damage and
//! ground-effect multipliers, plus a counter-torque scalar. The shape
//! is the textbook rotor-thrust scaling: thrust grows quadratically
//! with shaft speed.
//!
//! The trait is wider than CurveThrust uses (it accepts a `PropCtx`
//! with airspeed and inflow info) so that level-3 blade-element impls
//! can drop in later without touching the call sites.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrustParams {
    /// Thrust constant: T (N) = k_t * (rpm / 1000)². Tuned so that at
    /// the racing motor's full-throttle RPM (~34000) the per-prop
    /// thrust is ~6 N (TWR ~4 for a 0.6 kg drone).
    pub k_t: f32,
    /// Counter-torque coefficient: τ_reaction (N·m) = k_q · T (N).
    /// Real props have τ ≈ 0.01–0.02 × T at typical RPM.
    pub k_q: f32,
}

impl ThrustParams {
    /// Realistic 5" prop on a ~2400 KV motor: at full throttle
    /// ~34000 RPM ⇒ T ≈ k_t × 34² = k_t × 1156. For T = 6 N
    /// ⇒ k_t ≈ 0.0052.
    pub const RACING_5INCH: ThrustParams = ThrustParams {
        k_t: 0.0052,
        k_q: 0.014,
    };
}

/// Per-prop context passed to a thrust model. Most fields are unused
/// by `CurveThrust` but the level-3 blade-element model needs them.
#[derive(Debug, Clone, Copy)]
pub struct PropCtx {
    pub rpm: f32,
    /// Damage in `[0, 1]`. 0 = healthy, 1 = destroyed.
    pub damage: f32,
    /// Ground-effect multiplier in `[1.0, ground_effect_max]`. 1.0 = no
    /// effect (free air). The model just passes this through.
    pub ground_effect: f32,
    /// Air-relative velocity at the prop in body frame (m/s). Reserved
    /// for future blade-element impls; ignored by CurveThrust.
    pub v_local_body: [f32; 3],
}

#[derive(Debug, Clone, Copy)]
pub struct PropForce {
    /// Thrust magnitude along the prop's axis, Newtons.
    pub thrust_n: f32,
    /// Reaction torque magnitude, Newton-metres. Sign separately
    /// applied based on prop handedness at the call site.
    pub reaction_torque_nm: f32,
}

/// Damage scaling. Healthy prop = 1.0 thrust; broken prop = 0.0.
/// Slight non-linearity so a half-broken prop still does most of its
/// job — matches the "feel" of real chipped props.
pub fn damage_factor(damage: f32) -> f32 {
    let d = damage.clamp(0.0, 1.0);
    // 0 → 1.0, 0.5 → ~0.75, 1.0 → 0.0
    (1.0 - d) * (1.0 - 0.5 * d)
}

#[derive(Debug, Clone, Copy)]
pub struct CurveThrust {
    pub params: ThrustParams,
}

impl CurveThrust {
    pub fn new(params: ThrustParams) -> Self {
        Self { params }
    }

    pub fn compute(&self, ctx: &PropCtx) -> PropForce {
        let rpm_k = ctx.rpm / 1000.0;
        let raw = self.params.k_t * rpm_k * rpm_k;
        let scaled = raw * damage_factor(ctx.damage) * ctx.ground_effect.max(0.0);
        PropForce {
            thrust_n: scaled.max(0.0),
            reaction_torque_nm: self.params.k_q * scaled.max(0.0),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn ctx(rpm: f32) -> PropCtx {
        PropCtx {
            rpm,
            damage: 0.0,
            ground_effect: 1.0,
            v_local_body: [0.0, 0.0, 0.0],
        }
    }

    #[test]
    fn zero_rpm_zero_thrust() {
        let m = CurveThrust::new(ThrustParams::RACING_5INCH);
        let f = m.compute(&ctx(0.0));
        assert_eq!(f.thrust_n, 0.0);
        assert_eq!(f.reaction_torque_nm, 0.0);
    }

    #[test]
    fn thrust_quadratic_in_rpm() {
        let m = CurveThrust::new(ThrustParams::RACING_5INCH);
        let t1 = m.compute(&ctx(10000.0)).thrust_n;
        let t2 = m.compute(&ctx(20000.0)).thrust_n;
        // Doubling RPM should ~quadruple thrust.
        assert_abs_diff_eq!(t2 / t1, 4.0, epsilon = 0.01);
    }

    #[test]
    fn racing_preset_full_throttle_thrust_realistic() {
        let m = CurveThrust::new(ThrustParams::RACING_5INCH);
        let f = m.compute(&ctx(34000.0));
        // Should be in the 5-7 N range for a 5" racing prop.
        assert!(
            f.thrust_n > 5.0 && f.thrust_n < 7.0,
            "expected ~6 N, got {}",
            f.thrust_n
        );
    }

    #[test]
    fn damage_reduces_thrust_monotonically() {
        let m = CurveThrust::new(ThrustParams::RACING_5INCH);
        let mut prev = f32::INFINITY;
        for d in [0.0, 0.1, 0.3, 0.5, 0.8, 1.0] {
            let mut c = ctx(20000.0);
            c.damage = d;
            let t = m.compute(&c).thrust_n;
            assert!(t <= prev + 1e-4, "non-monotonic at damage={d}");
            prev = t;
        }
    }

    #[test]
    fn fully_broken_prop_zero_thrust() {
        let m = CurveThrust::new(ThrustParams::RACING_5INCH);
        let mut c = ctx(34000.0);
        c.damage = 1.0;
        assert_eq!(m.compute(&c).thrust_n, 0.0);
    }

    #[test]
    fn ground_effect_boosts_thrust() {
        let m = CurveThrust::new(ThrustParams::RACING_5INCH);
        let free = m.compute(&ctx(20000.0)).thrust_n;
        let mut c = ctx(20000.0);
        c.ground_effect = 1.5;
        let near_ground = m.compute(&c).thrust_n;
        assert_abs_diff_eq!(near_ground / free, 1.5, epsilon = 1e-4);
    }

    #[test]
    fn reaction_torque_proportional_to_thrust() {
        let m = CurveThrust::new(ThrustParams::RACING_5INCH);
        let f = m.compute(&ctx(20000.0));
        assert_abs_diff_eq!(
            f.reaction_torque_nm,
            f.thrust_n * ThrustParams::RACING_5INCH.k_q,
            epsilon = 1e-6
        );
    }

    #[test]
    fn damage_factor_endpoints() {
        assert_eq!(damage_factor(0.0), 1.0);
        assert_eq!(damage_factor(1.0), 0.0);
        // Out-of-range clamps.
        assert_eq!(damage_factor(-0.5), 1.0);
        assert_eq!(damage_factor(1.5), 0.0);
    }
}
