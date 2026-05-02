//! Ground-effect multiplier from a per-prop AGL (above-ground-level)
//! distance. This module is pure math; the actual raycast lives in
//! the gdext-side `DroneController`, which queries the world's
//! `PhysicsDirectSpaceState3D` once per prop per tick and feeds the
//! result here.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GroundEffectParams {
    /// Multiplier at zero AGL. 1.0 = no effect; 1.5 = 50 % thrust boost
    /// when sitting on the ground.
    pub max_multiplier: f32,
    /// Distance at which the effect fades to 1.0, metres. A common
    /// rule of thumb: ~1 prop diameter.
    pub fade_distance_m: f32,
}

impl GroundEffectParams {
    pub const RACING_5INCH: GroundEffectParams = GroundEffectParams {
        max_multiplier: 1.4,
        fade_distance_m: 0.30, // ~5" prop diameter + a margin
    };
}

/// Evaluate the per-prop ground-effect multiplier.
///
/// Returns 1.0 when AGL ≥ `fade_distance_m` or when AGL is unavailable
/// (`None`); falls off linearly to `max_multiplier` at AGL = 0.
pub fn ground_effect(agl_m: Option<f32>, params: &GroundEffectParams) -> f32 {
    let Some(agl) = agl_m else {
        return 1.0;
    };
    if agl >= params.fade_distance_m {
        return 1.0;
    }
    let agl = agl.max(0.0);
    let t = 1.0 - agl / params.fade_distance_m.max(1e-6);
    1.0 + (params.max_multiplier - 1.0) * t
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn p() -> GroundEffectParams {
        GroundEffectParams::RACING_5INCH
    }

    #[test]
    fn no_raycast_means_no_effect() {
        assert_eq!(ground_effect(None, &p()), 1.0);
    }

    #[test]
    fn far_above_means_no_effect() {
        assert_eq!(ground_effect(Some(2.0), &p()), 1.0);
        assert_eq!(ground_effect(Some(p().fade_distance_m + 0.01), &p()), 1.0);
    }

    #[test]
    fn at_zero_agl_max_multiplier() {
        assert_abs_diff_eq!(
            ground_effect(Some(0.0), &p()),
            p().max_multiplier,
            epsilon = 1e-6
        );
    }

    #[test]
    fn linear_fade_in_band() {
        let p = p();
        let half = p.fade_distance_m * 0.5;
        let mid_mul = ground_effect(Some(half), &p);
        let expected = 1.0 + (p.max_multiplier - 1.0) * 0.5;
        assert_abs_diff_eq!(mid_mul, expected, epsilon = 1e-6);
    }

    #[test]
    fn negative_agl_clamps_to_max() {
        // Defensive: if a raycast somehow returns a negative depth
        // (which shouldn't happen) we still get the max multiplier.
        assert_abs_diff_eq!(
            ground_effect(Some(-0.5), &p()),
            p().max_multiplier,
            epsilon = 1e-6
        );
    }

    #[test]
    fn monotonic_decrease_with_height() {
        let p = p();
        let mut prev = f32::INFINITY;
        for h in [0.0, 0.05, 0.1, 0.2, 0.3, 0.5, 1.0] {
            let g = ground_effect(Some(h), &p);
            assert!(g <= prev + 1e-6, "non-monotonic at h={h}");
            prev = g;
        }
    }
}
