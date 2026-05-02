//! Per-axis "Actual" rates evaluation (Betaflight-style) plus the
//! pilot throttle curve. All inputs in normalised stick deflection,
//! outputs in deg/s for axes and `[0,1]` for throttle.

/// Three-number "Actual" rates spec for one axis: roll, pitch, or yaw.
///
/// - `center_sensitivity` — deg/s of commanded angular rate per unit
///   stick at low deflection (the slope at stick = 0).
/// - `max_rate` — deg/s of commanded angular rate at full stick (±1).
/// - `expo` — curve shape blending between the two; 0 = linear, larger
///   values bend the curve so it stays flat near centre and sharpens
///   towards the edges.
///
/// Negative `expo` is clamped to zero — the Betaflight UI also enforces
/// non-negative expo for Actual rates.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ActualAxis {
    pub center_sensitivity: f32,
    pub max_rate: f32,
    pub expo: f32,
}

impl ActualAxis {
    pub const fn new(center_sensitivity: f32, max_rate: f32, expo: f32) -> Self {
        Self {
            center_sensitivity,
            max_rate,
            expo,
        }
    }

    /// Evaluate this axis at a given stick deflection in `[-1, +1]`.
    /// Returns the commanded angular rate in deg/s.
    pub fn evaluate(&self, stick: f32) -> f32 {
        let s = stick.clamp(-1.0, 1.0);
        let abs = s.abs();
        let expo = self.expo.max(0.0);

        // Linear contribution dominates near centre, scales with
        // center_sensitivity. The (1 + 8·expo) exponent matches the
        // Betaflight Actual-rates curve shape: at expo=0 the cur term
        // is just `max_rate · |s|` (linear), while higher expo shifts
        // it towards `|s|^9` style sharpness.
        let lin = abs * self.center_sensitivity;
        let cur = self.max_rate * abs.powf(1.0 + 8.0 * expo);

        // Blend: at low |s| the linear term dominates; near full stick
        // the curve term dominates.
        let blended = lin * (1.0 - abs) + cur * abs;
        s.signum() * blended
    }
}

/// Pilot throttle stick curve: shifts the 50%-output point and bends
/// the response around it.
///
/// - `mid` — stick value at which output equals 0.5 (default 0.5 ⇒
///   linear). Range `(0.0, 1.0)`. Out-of-range values are clamped
///   into a safe band so the formula stays well-defined.
/// - `expo` — curvature around `mid`. Positive softens the response
///   near `mid` (finer hover control); negative steepens it.
///
/// The shape is a smooth piecewise blend of `input` with a cubic bend
/// `(input − mid)³` around `mid`.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrottleCurve {
    pub mid: f32,
    pub expo: f32,
}

impl ThrottleCurve {
    pub const fn linear() -> Self {
        Self {
            mid: 0.5,
            expo: 0.0,
        }
    }

    pub fn evaluate(&self, stick: f32) -> f32 {
        let x = stick.clamp(0.0, 1.0);
        // Clamp mid away from 0 and 1 so the halves don't degenerate.
        let mid = self.mid.clamp(0.05, 0.95);
        // Clamp expo so the exponent stays in a sane range.
        let expo = self.expo.clamp(-0.95, 0.95);

        // Map each half to a unit interval on a signed coordinate `t`:
        //   x ∈ [0,   mid] → t = (x − mid) / mid       ∈ [-1, 0]
        //   x ∈ [mid, 1  ] → t = (x − mid) / (1 − mid) ∈ [ 0, 1]
        // So x = mid ⇔ t = 0, x = 0 ⇔ t = -1, x = 1 ⇔ t = +1.
        let t = if x <= mid {
            (x - mid) / mid
        } else {
            (x - mid) / (1.0 - mid)
        };

        // Bend exponent: p = exp(k · expo). At expo=0, p=1 (linear).
        //   expo > 0 → p > 1 → bend(t) hugs zero near t=0 (softens).
        //   expo < 0 → p < 1 → bend(t) lifts off zero quickly (sharpens).
        // k=2 chosen so expo=±0.5 gives a noticeable but not extreme
        // bend and the formula matches Betaflight's qualitative feel.
        let p = (2.0 * expo).exp();
        let bent = t.signum() * t.abs().powf(p);

        // Map signed bent value into [0, 1] around 0.5.
        (0.5 + 0.5 * bent).clamp(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn ax(c: f32, m: f32, e: f32) -> ActualAxis {
        ActualAxis::new(c, m, e)
    }

    #[test]
    fn actual_zero_stick_zero_rate() {
        assert_eq!(ax(100.0, 800.0, 0.5).evaluate(0.0), 0.0);
    }

    #[test]
    fn actual_full_positive_stick_hits_max_rate() {
        // At stick = +1, abs = 1, lin = center, cur = max, blend = 1.0 → cur.
        let a = ax(100.0, 800.0, 0.0);
        assert_abs_diff_eq!(a.evaluate(1.0), 800.0, epsilon = 1e-3);
    }

    #[test]
    fn actual_full_negative_stick_hits_minus_max() {
        let a = ax(100.0, 800.0, 0.5);
        assert_abs_diff_eq!(a.evaluate(-1.0), -800.0, epsilon = 1e-3);
    }

    #[test]
    fn actual_clamps_out_of_range_input() {
        let a = ax(100.0, 800.0, 0.0);
        assert_eq!(a.evaluate(1.5), a.evaluate(1.0));
        assert_eq!(a.evaluate(-2.0), a.evaluate(-1.0));
    }

    #[test]
    fn actual_zero_expo_is_pure_linear_blend() {
        let a = ax(100.0, 800.0, 0.0);
        // At stick = 0.5: lin = 50, cur = 400, blend = 0.5*lin + 0.5*cur = 225.
        assert_abs_diff_eq!(a.evaluate(0.5), 0.5 * 50.0 + 0.5 * 400.0, epsilon = 1e-3);
    }

    #[test]
    fn actual_expo_softens_near_centre() {
        // Higher expo → smaller rate at the same low/mid stick.
        let low = ax(100.0, 800.0, 0.0).evaluate(0.3);
        let high = ax(100.0, 800.0, 1.0).evaluate(0.3);
        assert!(high < low, "expo should reduce mid-stick response");
    }

    #[test]
    fn actual_negative_expo_clamped_to_zero() {
        let a_neg = ax(100.0, 800.0, -0.5);
        let a_zero = ax(100.0, 800.0, 0.0);
        assert_abs_diff_eq!(a_neg.evaluate(0.5), a_zero.evaluate(0.5), epsilon = 1e-6);
    }

    #[test]
    fn actual_is_odd_symmetric() {
        let a = ax(100.0, 800.0, 0.5);
        for s in [0.1, 0.3, 0.7, 0.95] {
            assert_abs_diff_eq!(a.evaluate(s), -a.evaluate(-s), epsilon = 1e-6);
        }
    }

    #[test]
    fn throttle_linear_passes_through() {
        let c = ThrottleCurve::linear();
        for x in [0.0, 0.25, 0.5, 0.75, 1.0] {
            assert_abs_diff_eq!(c.evaluate(x), x, epsilon = 1e-6);
        }
    }

    #[test]
    fn throttle_mid_at_05_yields_05_at_mid_stick() {
        let c = ThrottleCurve { mid: 0.5, expo: 0.6 };
        assert_abs_diff_eq!(c.evaluate(0.5), 0.5, epsilon = 1e-6);
    }

    #[test]
    fn throttle_mid_at_03_shifts_50pct_point() {
        let c = ThrottleCurve { mid: 0.3, expo: 0.0 };
        // At stick = 0.3, output should equal 0.5 because mid is 0.3.
        assert_abs_diff_eq!(c.evaluate(0.3), 0.5, epsilon = 1e-6);
        // At stick = 0, output is 0; at stick = 1, output is 1.
        assert_abs_diff_eq!(c.evaluate(0.0), 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(c.evaluate(1.0), 1.0, epsilon = 1e-6);
    }

    #[test]
    fn throttle_clamps_out_of_range_input() {
        let c = ThrottleCurve::linear();
        assert_abs_diff_eq!(c.evaluate(-0.5), 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(c.evaluate(2.0), 1.0, epsilon = 1e-6);
    }

    #[test]
    fn throttle_expo_keeps_endpoints_intact() {
        // Whatever the expo, x=0 → 0, x=mid → 0.5, x=1 → 1.
        for expo in [-0.5, -0.1, 0.0, 0.3, 0.8] {
            let c = ThrottleCurve { mid: 0.5, expo };
            assert_abs_diff_eq!(c.evaluate(0.0), 0.0, epsilon = 1e-3);
            assert_abs_diff_eq!(c.evaluate(0.5), 0.5, epsilon = 1e-3);
            assert_abs_diff_eq!(c.evaluate(1.0), 1.0, epsilon = 1e-3);
        }
    }

    #[test]
    fn throttle_positive_expo_softens_around_mid() {
        // With positive expo, output stays closer to 0.5 for stick
        // values near mid (smaller |out − 0.5|) than a pure linear curve.
        let lin = ThrottleCurve::linear();
        let soft = ThrottleCurve {
            mid: 0.5,
            expo: 0.8,
        };
        let lin_dev = (lin.evaluate(0.6) - 0.5).abs();
        let soft_dev = (soft.evaluate(0.6) - 0.5).abs();
        assert!(
            soft_dev < lin_dev,
            "positive expo should soften around mid (lin_dev={lin_dev}, soft_dev={soft_dev})"
        );
    }

    #[test]
    fn throttle_negative_expo_sharpens_around_mid() {
        // Negative expo should put output FURTHER from 0.5 for the
        // same stick deflection — sharper response near mid.
        let lin = ThrottleCurve::linear();
        let sharp = ThrottleCurve {
            mid: 0.5,
            expo: -0.8,
        };
        let lin_dev = (lin.evaluate(0.6) - 0.5).abs();
        let sharp_dev = (sharp.evaluate(0.6) - 0.5).abs();
        assert!(
            sharp_dev > lin_dev,
            "negative expo should sharpen around mid (lin_dev={lin_dev}, sharp_dev={sharp_dev})"
        );
    }

    #[test]
    fn throttle_monotonic() {
        // Monotonically non-decreasing across full input range for any
        // sane preset.
        let c = ThrottleCurve { mid: 0.4, expo: 0.5 };
        let mut prev = c.evaluate(0.0);
        let mut x = 0.0;
        while x < 1.0 {
            x += 0.01;
            let v = c.evaluate(x);
            assert!(
                v + 1e-4 >= prev,
                "non-monotonic at x={}: prev={}, v={}",
                x,
                prev,
                v
            );
            prev = v;
        }
    }
}
