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
    /// Previous tick's arm switch state for edge detection.
    prev_arm_switch: bool,
}

impl ArmingGate {
    pub fn new() -> Self {
        Self {
            armed: false,
            require_rearm: false,
            prev_arm_switch: true, // Prevent false rising edge on first call
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
    /// Arms only on the rising edge of the arm switch when throttle is below
    /// `ARM_THROTTLE_THRESHOLD`. Disarms when the switch goes low.
    /// Once armed, throttle can go up without disarming.
    pub fn update(&mut self, arm_switch: bool, throttle: f32) -> bool {
        if !arm_switch {
            self.armed = false;
            self.require_rearm = false;
        } else if !self.armed && !self.require_rearm && throttle <= ARM_THROTTLE_THRESHOLD {
            // Only arm on rising edge of the arm switch.
            if !self.prev_arm_switch {
                self.armed = true;
            }
        }
        self.prev_arm_switch = arm_switch;
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
    fn arms_only_after_toggle_with_throttle_low() {
        let mut g = gate();
        // First tick with arm high doesn't arm (no rising edge)
        assert!(!g.update(true, 0.0));
        assert!(!g.armed());
        // Toggle low then high to arm
        g.update(false, 0.0);
        assert!(g.update(true, 0.0));
        assert!(g.armed());
    }

    #[test]
    fn arms_on_first_toggle_from_low() {
        let mut g = gate();
        // Start with arm low
        g.update(false, 0.0);
        // Toggle high to arm
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
        // First call with arm high doesn't arm (no rising edge)
        g.update(true, 0.0);
        // Toggle low then high to arm
        g.update(false, 0.0);
        g.update(true, 0.0);
        assert!(g.armed());
        // Switch low disarms
        g.update(false, 0.0);
        assert!(!g.armed());
    }

    #[test]
    fn throttle_up_after_arming_stays_armed() {
        let mut g = gate();
        // First call with arm high doesn't arm (no rising edge)
        g.update(true, 0.0);
        // Toggle low then high to arm
        g.update(false, 0.0);
        g.update(true, 0.0);
        assert!(g.armed());
        let still = g.update(true, 0.5); // throttle goes up
        assert!(still);
    }

    #[test]
    fn force_disarm() {
        let mut g = gate();
        // Toggle to arm
        g.update(true, 0.0);
        g.update(false, 0.0);
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
        // Toggle to arm
        g.update(true, 0.0);
        g.update(false, 0.0);
        g.update(true, 0.0);
        for _ in 0..10 {
            assert!(g.update(true, 0.5));
        }
    }

    #[test]
    fn throttle_low_then_arm_switch_rising_arms() {
        let mut g = gate();
        // Start with arm low
        g.update(false, 0.0);
        assert!(!g.armed());
        // Toggle high to arm
        g.update(true, 0.0);
        assert!(g.armed());
    }

    #[test]
    fn arm_switch_high_then_throttle_low_does_not_arm() {
        let mut g = gate();
        g.update(true, 0.5); // arm switch high, throttle high
        assert!(!g.armed());
        g.update(true, 0.0); // throttle goes low, but arm switch not rising
        assert!(!g.armed());
    }

    #[test]
    fn starting_with_arm_high_throttle_low_does_not_arm_on_first_tick() {
        let mut g = gate();
        // First tick: arm switch already high (no rising edge), throttle is low
        assert!(!g.update(true, 0.0));
        assert!(!g.armed());
        // Now toggle switch low then high to arm
        g.update(false, 0.0);
        assert!(g.update(true, 0.0));
        assert!(g.armed());
    }
}
