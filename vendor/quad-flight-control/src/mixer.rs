//! X-quad mixer: converts roll/pitch/yaw/throttle commands into per-motor
//! commands in `[0, 1]`.
//!
//! Geometry (viewed from above, nose forward toward −Z in body frame):
//!
//! ```text
//!     M0 (LF)                    M1 (RF)
//!                  \             /
//!                   \   body    /
//!                   /          \
//!     M3 (LB)                    M2 (RB)
//! ```
//!
//! Two named matrix presets cover the two conventional X-quad
//! handedness patterns; the [`mix`] function takes a matrix by
//! reference so airframes pick the right one (or supply a custom).
//!
//! - [`MIXER_MATRIX_PROPS_IN`] — diagonal pairs (M0+M2) spin CW,
//!   (M1+M3) spin CCW. Betaflight's default; common for standard
//!   X-quads.
//! - [`MIXER_MATRIX_PROPS_OUT`] — diagonal pairs (M0+M2) spin CCW,
//!   (M1+M3) spin CW. Yaw column inverted vs props-in.
//!
//! The roll, pitch and throttle columns are identical between the
//! two — only the yaw column differs, because only yaw reaction
//! torque depends on prop handedness.
//!
//! Conventions (matching Betaflight):
//! - positive roll  → drone rolls right (left-side motors thrust more)
//! - positive pitch → nose pitches down (rear motors thrust more)
//! - positive yaw   → nose yaws right

/// Mixer matrix rows for **props-in** X-quad. Each row is
/// `[roll, pitch, yaw, throttle]`.
pub const MIXER_MATRIX_PROPS_IN: [[f32; 4]; 4] = [
    [1.0, -1.0, -1.0, 1.0], // M0 LF, CW
    [-1.0, -1.0, 1.0, 1.0], // M1 RF, CCW
    [-1.0, 1.0, -1.0, 1.0], // M2 RB, CW
    [1.0, 1.0, 1.0, 1.0],   // M3 LB, CCW
];

/// Mixer matrix rows for **props-out** X-quad — yaw column inverted.
pub const MIXER_MATRIX_PROPS_OUT: [[f32; 4]; 4] = [
    [1.0, -1.0, 1.0, 1.0],   // M0 LF, CCW
    [-1.0, -1.0, -1.0, 1.0], // M1 RF, CW
    [-1.0, 1.0, 1.0, 1.0],   // M2 RB, CCW
    [1.0, 1.0, -1.0, 1.0],   // M3 LB, CW
];

/// Per-motor command, in `[0, 1]`.
pub type MotorCommands = [f32; 4];

/// Multiply `matrix` by `[roll, pitch, yaw, throttle]` and normalise so
/// the largest motor command is at most 1.0.
///
/// `roll`, `pitch`, `yaw` are in `[-1, +1]` (PID outputs); `throttle`
/// is in `[0, 1]`. The result is clamped per-motor to `[0, 1]`.
///
/// Anti-saturation strategy: if any motor would exceed 1.0 after mixing,
/// scale ALL motor commands down by the same factor so authority on
/// roll/pitch/yaw is preserved. This is what Betaflight calls
/// "airmode-style" mix scaling.
pub fn mix(
    matrix: &[[f32; 4]; 4],
    roll: f32,
    pitch: f32,
    yaw: f32,
    throttle: f32,
) -> MotorCommands {
    let v = [roll, pitch, yaw, throttle];
    let mut motors = [0.0_f32; 4];
    for (i, row) in matrix.iter().enumerate() {
        motors[i] = row[0] * v[0] + row[1] * v[1] + row[2] * v[2] + row[3] * v[3];
    }

    // Anti-saturation: if any motor exceeds 1.0, scale all commands down
    // by the excess so roll/pitch/yaw authority survives.
    let max_cmd = motors
        .iter()
        .cloned()
        .fold(0.0_f32, |a, b| crate::mathf::max(a, b));
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

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    /// Props-in convenience for the existing test body.
    fn mix_in(roll: f32, pitch: f32, yaw: f32, throttle: f32) -> MotorCommands {
        mix(&MIXER_MATRIX_PROPS_IN, roll, pitch, yaw, throttle)
    }

    fn close_arr(a: MotorCommands, b: [f32; 4]) {
        for i in 0..4 {
            assert_abs_diff_eq!(a[i], b[i], epsilon = 1e-5);
        }
    }

    #[test]
    fn pure_throttle_all_motors_equal() {
        let m = mix_in(0.0, 0.0, 0.0, 0.5);
        close_arr(m, [0.5, 0.5, 0.5, 0.5]);
    }

    #[test]
    fn zero_inputs_zero_outputs() {
        let m = mix_in(0.0, 0.0, 0.0, 0.0);
        close_arr(m, [0.0, 0.0, 0.0, 0.0]);
    }

    #[test]
    fn positive_roll_left_motors_up_right_motors_down() {
        // From hover at throttle=0.5, add roll right.
        let m = mix_in(0.2, 0.0, 0.0, 0.5);
        // M0 (LF, +1 roll) and M3 (LB, +1 roll) should rise.
        // M1 (RF, -1 roll) and M2 (RB, -1 roll) should fall.
        assert!(m[0] > m[1]);
        assert!(m[3] > m[2]);
        assert!(m[0] > 0.5);
        assert!(m[1] < 0.5);
    }

    #[test]
    fn positive_pitch_rear_motors_up_front_motors_down() {
        let m = mix_in(0.0, 0.2, 0.0, 0.5);
        // M0 (LF, -1 pitch) and M1 (RF, -1 pitch) should fall.
        // M2 (RB, +1 pitch) and M3 (LB, +1 pitch) should rise.
        assert!(m[0] < m[2]);
        assert!(m[1] < m[3]);
    }

    #[test]
    fn positive_yaw_ccw_motors_up_cw_motors_down() {
        let m = mix_in(0.0, 0.0, 0.2, 0.5);
        // CCW: m1, m3 (yaw +1). CW: m0, m2 (yaw -1).
        assert!(m[1] > m[0]);
        assert!(m[3] > m[2]);
    }

    #[test]
    fn props_out_yaw_inverts_vs_props_in() {
        // Pure yaw: PROPS_OUT must rise/fall the opposite motor set.
        let in_m = mix(&MIXER_MATRIX_PROPS_IN, 0.0, 0.0, 0.2, 0.5);
        let out_m = mix(&MIXER_MATRIX_PROPS_OUT, 0.0, 0.0, 0.2, 0.5);
        assert!(in_m[1] > in_m[0] && out_m[0] > out_m[1]);
        assert!(in_m[3] > in_m[2] && out_m[2] > out_m[3]);
    }

    #[test]
    fn props_in_and_out_agree_on_roll_pitch_throttle() {
        // Yaw column is the only difference between the two presets.
        for (r, p, t) in [(0.3, 0.0, 0.5), (0.0, 0.3, 0.5), (0.0, 0.0, 0.7)] {
            let in_m = mix(&MIXER_MATRIX_PROPS_IN, r, p, 0.0, t);
            let out_m = mix(&MIXER_MATRIX_PROPS_OUT, r, p, 0.0, t);
            for i in 0..4 {
                assert_abs_diff_eq!(in_m[i], out_m[i], epsilon = 1e-6);
            }
        }
    }

    #[test]
    fn anti_saturation_preserves_axis_authority() {
        // Commanded mix that would exceed 1.0 on m3: throttle 1.0 + roll +0.3 + pitch +0.3 → 1.6.
        let m = mix_in(0.3, 0.3, 0.0, 1.0);
        // After scaling, max should be exactly 1.0.
        let max_cmd = m.iter().cloned().fold(0.0_f32, f32::max);
        assert_abs_diff_eq!(max_cmd, 1.0, epsilon = 1e-5);
        // Roll authority survives: m3 still > m1.
        assert!(m[3] > m[1]);
        // Pitch authority survives: m3 still > m0.
        assert!(m[3] > m[0]);
    }

    #[test]
    fn negative_commands_clamp_to_zero_not_reverse() {
        // Strong negative throttle bias would push some motors below 0.
        let m = mix_in(0.0, 0.0, 0.0, 0.0);
        for v in m {
            assert!(v >= 0.0);
        }
    }

    #[test]
    fn output_in_unit_range() {
        for matrix in [&MIXER_MATRIX_PROPS_IN, &MIXER_MATRIX_PROPS_OUT] {
            for r in [-1.0, 0.0, 1.0] {
                for p in [-1.0, 0.0, 1.0] {
                    for y in [-1.0, 0.0, 1.0] {
                        for t in [0.0, 0.5, 1.0] {
                            let m = mix(matrix, r, p, y, t);
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
    }

    #[test]
    fn pure_yaw_no_axis_lift_change_average() {
        // Pure yaw should not change *average* throttle, for either preset.
        for matrix in [&MIXER_MATRIX_PROPS_IN, &MIXER_MATRIX_PROPS_OUT] {
            let m = mix(matrix, 0.0, 0.0, 0.4, 0.5);
            let avg = (m[0] + m[1] + m[2] + m[3]) / 4.0;
            assert_abs_diff_eq!(avg, 0.5, epsilon = 1e-5);
        }
    }
}
