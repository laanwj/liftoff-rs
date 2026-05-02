//! Arming gate: decoded arm-switch + throttle-low safety.
//!
//! The gate consumes the decoded `arm` bool and the throttle value
//! from `RcInput`. It compares throttle against a constant threshold
//! to determine the safety condition.

/// Throttle fraction at or below which arming is permitted.
pub const ARM_THROTTLE_THRESHOLD: f32 = 0.1;

/// Mutable arming state. Tracks the latched armed/disarmed flag.
#[derive(Debug, Clone, Copy)]
pub struct ArmingGate {
    armed: bool,
    /// After a force-disarm (e.g. respawn), require the arm switch to
    /// go low before re-arming is permitted. Prevents immediate
    /// re-arm when the input source still asserts arm=true.
    require_rearm: bool,
}

impl ArmingGate {
    pub fn new() -> Self {
        Self {
            armed: false,
            require_rearm: false,
        }
    }

    pub fn armed(&self) -> bool {
        self.armed
    }

    /// Force disarm — used on whole-drone failure, respawn, etc.
    /// Requires the arm switch to cycle low before re-arming.
    pub fn force_disarm(&mut self) {
        self.armed = false;
        self.require_rearm = true;
    }

    /// Update from this tick's decoded RC input.
    ///
    /// - `arm_switch`: decoded arm channel (true = pilot wants armed).
    /// - `throttle`: decoded throttle value in `[0, 1]`.
    ///
    /// Arms when the switch goes high while throttle is below
    /// `ARM_THROTTLE_THRESHOLD`. Disarms when the switch goes low.
    /// Once armed, throttle can go up without disarming.
    pub fn update(&mut self, arm_switch: bool, throttle: f32) -> bool {
        if !arm_switch {
            self.armed = false;
            self.require_rearm = false;
        } else if !self.armed && !self.require_rearm && throttle <= ARM_THROTTLE_THRESHOLD {
            self.armed = true;
        }
        self.armed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn gate() -> ArmingGate {
        ArmingGate::new()
    }

    #[test]
    fn default_disarmed() {
        assert!(!gate().armed());
    }

    #[test]
    fn arms_when_switch_high_throttle_low() {
        let mut g = gate();
        assert!(g.update(true, 0.0));
        assert!(g.armed());
    }

    #[test]
    fn refuses_to_arm_with_throttle_up() {
        let mut g = gate();
        assert!(!g.update(true, 0.5));
        assert!(!g.armed());
    }

    #[test]
    fn switch_low_disarms() {
        let mut g = gate();
        g.update(true, 0.0); // arm
        assert!(g.armed());
        g.update(false, 0.0); // switch low
        assert!(!g.armed());
    }

    #[test]
    fn throttle_up_after_arming_stays_armed() {
        let mut g = gate();
        g.update(true, 0.0); // arm with throttle low
        assert!(g.armed());
        let still = g.update(true, 0.5); // throttle goes up
        assert!(still);
    }

    #[test]
    fn force_disarm() {
        let mut g = gate();
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
        let mut g = gate();
        g.update(true, 0.0);
        for _ in 0..10 {
            assert!(g.update(true, 0.5));
        }
    }
}
