//! X-quad mixer: converts roll/pitch/yaw/throttle commands into per-motor
//! commands in `[0, 1]`.
//!
//! Geometry (viewed from above, nose forward toward −Z in body frame):
//!
//! ```text
//!     M0 (front-left, CW)        M1 (front-right, CCW)
//!                  \             /
//!                   \   body    /
//!                   /          \
//!     M3 (rear-left, CCW)       M2 (rear-right, CW)
//! ```
//!
//! Mixer matrix (columns: roll, pitch, yaw, throttle):
//!
//! ```text
//!     [ +1  +1  −1  +1 ]   [ roll ]
//!     [ −1  +1  +1  +1 ] · [ pitch ]
//!     [ −1  −1  −1  +1 ]   [ yaw ]
//!     [ +1  −1  +1  +1 ]   [ throttle ]
//! ```
//!
//! Conventions:
//! - positive roll  → drone rolls right (left-side motors thrust more)
//! - positive pitch → nose pitches up   (front motors thrust more)
//! - positive yaw   → nose yaws right   (CCW motors thrust more)

/// X-quad mixer matrix rows. Each row is `[roll, pitch, yaw, throttle]`.
/// Public so tests and tooling can refer to the same definition.
pub const MIXER_MATRIX: [[f32; 4]; 4] = [
    [1.0, 1.0, -1.0, 1.0], // M0 front-left,  CW
    [-1.0, 1.0, 1.0, 1.0], // M1 front-right, CCW
    [-1.0, -1.0, -1.0, 1.0], // M2 rear-right,  CW
    [1.0, -1.0, 1.0, 1.0], // M3 rear-left,   CCW
];

/// Per-motor command, in `[0, 1]`.
pub type MotorCommands = [f32; 4];

/// Multiply the mixer matrix by `[roll, pitch, yaw, throttle]` and
/// normalise so the largest motor command is at most 1.0.
///
/// `roll`, `pitch`, `yaw` are in `[-1, +1]` (PID outputs); `throttle`
/// is in `[0, 1]`. The result is clamped per-motor to `[0, 1]`.
///
/// Anti-saturation strategy: if any motor would exceed 1.0 after mixing,
/// scale ALL motor commands down by the same factor so authority on
/// roll/pitch/yaw is preserved. This is what Betaflight calls
/// "airmode-style" mix scaling.
pub fn mix(roll: f32, pitch: f32, yaw: f32, throttle: f32) -> MotorCommands {
    let v = [roll, pitch, yaw, throttle];
    let mut motors = [0.0_f32; 4];
    for (i, row) in MIXER_MATRIX.iter().enumerate() {
        motors[i] = row[0] * v[0] + row[1] * v[1] + row[2] * v[2] + row[3] * v[3];
    }

    // Anti-saturation: if any motor exceeds 1.0, scale all commands down
    // by the excess so roll/pitch/yaw authority survives.
    let max_cmd = motors.iter().cloned().fold(0.0_f32, f32::max);
    if max_cmd > 1.0 {
        let s = 1.0 / max_cmd;
        for m in &mut motors {
            *m *= s;
        }
    }

    // Final per-motor clamp to `[0, 1]`. Negative commands (a saturated
    // axis pushing one motor below zero) become motor-off rather than
    // bidirectional — racing quad ESCs can't reverse without 3D mode.
    for m in &mut motors {
        *m = m.clamp(0.0, 1.0);
    }

    motors
}

/// Per-motor handedness. +1 = CW, −1 = CCW. Standard X-quad layout
/// per the diagram at the top of this module; preset can override.
pub const DEFAULT_HANDEDNESS: [i8; 4] = [1, -1, 1, -1];

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn close_arr(a: MotorCommands, b: [f32; 4]) {
        for i in 0..4 {
            assert_abs_diff_eq!(a[i], b[i], epsilon = 1e-5);
        }
    }

    #[test]
    fn pure_throttle_all_motors_equal() {
        let m = mix(0.0, 0.0, 0.0, 0.5);
        close_arr(m, [0.5, 0.5, 0.5, 0.5]);
    }

    #[test]
    fn zero_inputs_zero_outputs() {
        let m = mix(0.0, 0.0, 0.0, 0.0);
        close_arr(m, [0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn positive_roll_left_motors_up_right_motors_down() {
        // From hover at throttle=0.5, add roll right.
        let m = mix(0.2, 0.0, 0.0, 0.5);
        // m0 (FL, +1 roll) and m3 (RL, +1 roll) should rise.
        // m1 (FR, -1 roll) and m2 (RR, -1 roll) should fall.
        assert!(m[0] > m[1]);
        assert!(m[3] > m[2]);
        assert!(m[0] > 0.5);
        assert!(m[1] < 0.5);
    }

    #[test]
    fn positive_pitch_front_motors_up_rear_motors_down() {
        let m = mix(0.0, 0.2, 0.0, 0.5);
        // m0 (FL, +1 pitch) and m1 (FR, +1 pitch) should rise.
        // m2 (RR, -1 pitch) and m3 (RL, -1 pitch) should fall.
        assert!(m[0] > m[2]);
        assert!(m[1] > m[3]);
    }

    #[test]
    fn positive_yaw_ccw_motors_up_cw_motors_down() {
        let m = mix(0.0, 0.0, 0.2, 0.5);
        // CCW: m1, m3 (yaw +1). CW: m0, m2 (yaw -1).
        assert!(m[1] > m[0]);
        assert!(m[3] > m[2]);
    }

    #[test]
    fn anti_saturation_preserves_axis_authority() {
        // Commanded mix that would exceed 1.0 on m0: throttle 1.0 + roll +0.3 + pitch +0.3 → 1.6.
        let m = mix(0.3, 0.3, 0.0, 1.0);
        // After scaling, max should be exactly 1.0.
        let max_cmd = m.iter().cloned().fold(0.0_f32, f32::max);
        assert_abs_diff_eq!(max_cmd, 1.0, epsilon = 1e-5);
        // Roll authority survives: m0 still > m1.
        assert!(m[0] > m[1]);
        // Pitch authority survives: m0 still > m2.
        assert!(m[0] > m[2]);
    }

    #[test]
    fn negative_commands_clamp_to_zero_not_reverse() {
        // Strong negative throttle bias would push some motors below 0.
        let m = mix(0.0, 0.0, 0.0, 0.0);
        for v in m {
            assert!(v >= 0.0);
        }
    }

    #[test]
    fn output_in_unit_range() {
        for r in [-1.0, 0.0, 1.0] {
            for p in [-1.0, 0.0, 1.0] {
                for y in [-1.0, 0.0, 1.0] {
                    for t in [0.0, 0.5, 1.0] {
                        let m = mix(r, p, y, t);
                        for v in m {
                            assert!(
                                (0.0..=1.0).contains(&v),
                                "out-of-range motor cmd {v} at r={r} p={p} y={y} t={t}"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn pure_yaw_no_axis_lift_change_average() {
        // Pure yaw should not change *average* throttle.
        let m = mix(0.0, 0.0, 0.4, 0.5);
        let avg = (m[0] + m[1] + m[2] + m[3]) / 4.0;
        assert_abs_diff_eq!(avg, 0.5, epsilon = 1e-5);
    }
}
