//! Per-axis quadratic body drag.
//!
//! `F_drag = −½ · ρ · v_rel² · Cd · A` evaluated per body axis with
//! per-direction `Cd` and `A`. The drone presents different cross
//! sections to "front", "side", and "top" airflow; each axis has its
//! own coefficient.
//!
//! `v_rel` is the body-frame relative airspeed (subtract environment
//! wind from world-frame body velocity, then rotate into body frame).
//! The output force is in body frame; the call site rotates it back
//! into world for the rigid body.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DragParams {
    /// Drag coefficients, one per body axis. Indexing follows Godot's
    /// body frame: `[X, Y, Z]` = `[side, top, front]`.
    pub cd_per_axis: [f32; 3],
    /// Reference areas, m². Same `[X, Y, Z]` = `[side, top, front]`
    /// indexing — the area presented to airflow along each body axis.
    pub area_per_axis: [f32; 3],
    /// Air density, kg/m³.
    pub air_density: f32,
}

impl DragParams {
    /// Reasonable defaults for a 5" racing quad.
    /// - Front (along Z) presents the smallest area: just the arms +
    ///   motor rims when viewed nose-on.
    /// - Side (along X) is medium: the full body length.
    /// - Top (along Y) is the largest: props + body when viewed from
    ///   above.
    pub const RACING_5INCH: DragParams = DragParams {
        cd_per_axis: [1.0, 1.3, 1.0],         // [X side, Y top, Z front]
        area_per_axis: [0.025, 0.05, 0.012],  // [X side, Y top, Z front]
        air_density: 1.225,
    };
}

/// Compute body-frame drag force from body-frame relative airspeed.
///
/// Each axis contributes `−½ρ · v_axis² · Cd · A · sign(v_axis)`. The
/// implementation evaluates each axis independently and sums; this is
/// not strictly correct for oblique flow (a real drone has cross-axis
/// coupling) but matches what shipped sims call "per-axis quadratic
/// drag".
pub fn quadratic_drag_body(v_rel_body: [f32; 3], params: &DragParams) -> [f32; 3] {
    let half_rho = 0.5 * params.air_density;
    let mut f = [0.0_f32; 3];
    for i in 0..3 {
        let v = v_rel_body[i];
        let mag = -half_rho * v * v.abs() * params.cd_per_axis[i] * params.area_per_axis[i];
        f[i] = mag;
    }
    f
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn p() -> DragParams {
        DragParams::RACING_5INCH
    }

    #[test]
    fn zero_velocity_zero_drag() {
        let f = quadratic_drag_body([0.0, 0.0, 0.0], &p());
        assert_eq!(f, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn drag_opposes_velocity() {
        let f = quadratic_drag_body([5.0, 0.0, 0.0], &p());
        assert!(f[0] < 0.0);
        let f2 = quadratic_drag_body([-5.0, 0.0, 0.0], &p());
        assert!(f2[0] > 0.0);
        // Magnitude same in both directions.
        assert_abs_diff_eq!(f[0].abs(), f2[0].abs(), epsilon = 1e-6);
    }

    #[test]
    fn drag_quadratic_in_speed() {
        let f1 = quadratic_drag_body([5.0, 0.0, 0.0], &p()).map(f32::abs);
        let f2 = quadratic_drag_body([10.0, 0.0, 0.0], &p()).map(f32::abs);
        // Doubling speed → quadrupling drag.
        assert_abs_diff_eq!(f2[0] / f1[0], 4.0, epsilon = 0.001);
    }

    #[test]
    fn per_axis_independent() {
        // Only the relevant axis gets a force.
        let f = quadratic_drag_body([5.0, 0.0, 0.0], &p());
        assert!(f[0] != 0.0);
        assert_eq!(f[1], 0.0);
        assert_eq!(f[2], 0.0);
    }

    #[test]
    fn larger_top_area_means_more_vertical_drag_than_frontal() {
        // Top (Y) presents the largest area; Z-axis (frontal) the
        // smallest. Vertical drag should comfortably exceed frontal
        // drag at the same speed.
        let p = DragParams::RACING_5INCH;
        let frontal = quadratic_drag_body([0.0, 0.0, 5.0], &p)[2].abs();
        let vertical = quadratic_drag_body([0.0, 5.0, 0.0], &p)[1].abs();
        assert!(
            vertical > frontal * 3.0,
            "vertical drag {vertical} should be ≫ frontal drag {frontal}"
        );
    }

    #[test]
    fn drag_scales_with_air_density() {
        let mut p1 = DragParams::RACING_5INCH;
        p1.air_density = 1.0;
        let mut p2 = DragParams::RACING_5INCH;
        p2.air_density = 2.0;
        let f1 = quadratic_drag_body([10.0, 0.0, 0.0], &p1)[0];
        let f2 = quadratic_drag_body([10.0, 0.0, 0.0], &p2)[0];
        assert_abs_diff_eq!(f2 / f1, 2.0, epsilon = 1e-3);
    }

    #[test]
    fn realistic_terminal_velocity_bound() {
        // Sanity: at 30 m/s lateral, drag should be a few Newtons —
        // less than a hover thrust budget but enough to slow the drone.
        let f = quadratic_drag_body([30.0, 0.0, 0.0], &p());
        let mag = f[0].abs();
        assert!(mag > 1.0 && mag < 50.0, "lateral drag at 30 m/s = {mag} N");
    }
}
