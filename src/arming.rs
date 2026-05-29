//! Arming gate: edge-triggered arm switch + throttle-low safety.
//!
//! Arming requires an explicit low → high transition of the arm switch
//! while the throttle is below [`ARM_THROTTLE_THRESHOLD`]. A switch
//! already high at construction time does not count as a rising edge
//! — the pilot must cycle the switch low → high so arming is always
//! an *explicit* action, even after a power-cycle with a latched
//! switch.
//!
//! [`ArmingGate::force_disarm`] uses the same mechanism: it forces the
//! gate to require a fresh low → high cycle before re-arming.

/// Throttle fraction at or below which arming is permitted.
pub const ARM_THROTTLE_THRESHOLD: f32 = 0.05;

/// Mutable arming state.
#[derive(Debug, Clone, Copy)]
pub struct ArmingGate {
    armed: bool,
    /// Last-seen arm-switch state. Initialised `true` at construction
    /// so a switch that's already high at boot doesn't register as a
    /// rising edge; `force_disarm` also forces this `true` so the
    /// next arming requires a low → high cycle.
    prev_arm: bool,
}

impl ArmingGate {
    pub fn new() -> Self {
        Self {
            armed: false,
            prev_arm: true,
        }
    }

    pub fn armed(&self) -> bool {
        self.armed
    }

    /// Force disarm — used on whole-drone failure, respawn, link loss.
    /// Requires the arm switch to be observed low again before the
    /// next rising edge can re-arm.
    pub fn force_disarm(&mut self) {
        self.armed = false;
        self.prev_arm = true;
    }

    /// Update from this tick's decoded RC input.
    ///
    /// - `arm_switch`: decoded arm channel (true = pilot wants armed).
    /// - `throttle`: decoded throttle value in `[0, 1]`.
    ///
    /// Arms only on a rising edge of `arm_switch` while throttle is at
    /// or below [`ARM_THROTTLE_THRESHOLD`]. Disarms whenever
    /// `arm_switch` is low. Once armed, throttle can go up freely.
    pub fn update(&mut self, arm_switch: bool, throttle: f32) -> bool {
        let rising_edge = arm_switch && !self.prev_arm;
        if !arm_switch {
            self.armed = false;
        } else if rising_edge && throttle <= ARM_THROTTLE_THRESHOLD {
            self.armed = true;
        }
        self.prev_arm = arm_switch;
        self.armed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Fresh gate ready to be armed: pre-cycle a low arm-switch to
    /// satisfy the "must see low before the next rising edge"
    /// invariant. The bare `ArmingGate::new()` assumes the switch was
    /// already up at boot, which is the safe default but inconvenient
    /// for tests of post-cycle behaviour.
    fn armable_gate() -> ArmingGate {
        let mut g = ArmingGate::new();
        g.update(false, 0.0);
        g
    }

    #[test]
    fn default_disarmed() {
        assert!(!ArmingGate::new().armed());
    }

    #[test]
    fn boot_with_switch_high_does_not_auto_arm() {
        // Reset / power-on with the arm switch already latched up:
        // even with throttle low, must NOT arm without an explicit
        // low → high cycle.
        let mut g = ArmingGate::new();
        assert!(!g.update(true, 0.0));
        assert!(!g.armed());
        // After cycling the switch, the next rising edge arms.
        g.update(false, 0.0);
        assert!(g.update(true, 0.0));
    }

    #[test]
    fn arms_on_rising_edge_with_throttle_low() {
        let mut g = armable_gate();
        assert!(g.update(true, 0.0));
        assert!(g.armed());
    }

    #[test]
    fn refuses_to_arm_with_throttle_up() {
        let mut g = armable_gate();
        assert!(!g.update(true, 0.5));
        assert!(!g.armed());
    }

    #[test]
    fn raise_arm_then_lower_throttle_does_not_auto_arm() {
        // The bug: pilot raises the arm switch with throttle still up;
        // gate must refuse, AND a subsequent throttle-low (switch still
        // up, no cycle) must NOT silently arm.
        let mut g = armable_gate();
        assert!(!g.update(true, 0.5));
        assert!(!g.update(true, 0.0));
        // Only after cycling the switch low → high (with throttle low)
        // does it arm.
        g.update(false, 0.0);
        assert!(g.update(true, 0.0));
    }

    #[test]
    fn switch_low_disarms() {
        let mut g = armable_gate();
        g.update(true, 0.0); // arm
        assert!(g.armed());
        g.update(false, 0.0); // switch low
        assert!(!g.armed());
    }

    #[test]
    fn throttle_up_after_arming_stays_armed() {
        let mut g = armable_gate();
        g.update(true, 0.0); // arm with throttle low
        assert!(g.armed());
        let still = g.update(true, 0.5); // throttle goes up
        assert!(still);
    }

    #[test]
    fn force_disarm() {
        let mut g = armable_gate();
        g.update(true, 0.0);
        assert!(g.armed());
        g.force_disarm();
        assert!(!g.armed());
        // Switch still high — must not re-arm until switch cycles low.
        assert!(!g.update(true, 0.0));
        // Switch goes low.
        g.update(false, 0.0);
        // Now switch high again — re-arms.
        assert!(g.update(true, 0.0));
    }

    #[test]
    fn repeated_switch_high_while_armed_is_idempotent() {
        let mut g = armable_gate();
        g.update(true, 0.0);
        for _ in 0..10 {
            assert!(g.update(true, 0.5));
        }
    }
}
