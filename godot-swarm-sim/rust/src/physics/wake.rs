//! Inter-drone aerodynamic wake field.
//!
//! Each prop contributes a `WakeColumn` representing its downwash. Other
//! props sample the field to get an inflow velocity that reduces their
//! effective thrust (simulating the real effect of flying in another
//! drone's dirty air).
//!
//! The field is **double-buffered**: tick N samples from snapshot N-1
//! and writes contributions to buffer N. After all drones step, the
//! buffers swap. This eliminates iteration-order dependence.
//!
//! A **coarse 3D spatial hash** (uniform grid) provides O(1) lookup for
//! nearby wake columns, so cost is proportional to the local density of
//! drones rather than the total swarm size.

use std::collections::HashMap;

/// A single prop's contribution to the wake field.
#[derive(Debug, Clone, Copy)]
pub struct WakeColumn {
    /// World-frame position of the prop disc centre.
    pub origin: [f32; 3],
    /// Unit vector pointing in the downwash direction (away from the
    /// prop's thrust face — for a standard quad this is body −Y in
    /// world frame when the drone is level).
    pub axis: [f32; 3],
    /// Induced velocity at the disc, m/s. Computed from momentum theory:
    /// `v_h = sqrt(|T| / (2 · rho · A))` where T is thrust (N), rho is
    /// air density (kg/m³), and A is the prop disc area (m²).
    pub v_h: f32,
    /// Prop disc radius, metres. Determines the radial falloff width.
    pub radius: f32,
    /// Axial decay length scale, metres. Downwash decays as
    /// `exp(-axial_dist / axial_decay)` along the axis.
    pub axial_decay: f32,
}

/// Compute `v_h` from thrust and prop geometry using momentum theory.
/// Returns 0 if thrust is zero or negative.
pub fn induced_velocity(thrust_n: f32, air_density: f32, prop_radius_m: f32) -> f32 {
    if thrust_n <= 0.0 || air_density <= 0.0 || prop_radius_m <= 0.0 {
        return 0.0;
    }
    let disc_area = std::f32::consts::PI * prop_radius_m * prop_radius_m;
    (thrust_n / (2.0 * air_density * disc_area)).sqrt()
}

/// Cell key for the spatial hash.
type CellKey = [i32; 3];

fn cell_key(pos: [f32; 3], cell_size: f32) -> CellKey {
    [
        (pos[0] / cell_size).floor() as i32,
        (pos[1] / cell_size).floor() as i32,
        (pos[2] / cell_size).floor() as i32,
    ]
}

/// One side of the double buffer: a flat list of columns plus a spatial
/// hash for fast neighbourhood queries.
#[derive(Debug, Clone)]
struct WakeBuffer {
    columns: Vec<WakeColumn>,
    grid: HashMap<CellKey, Vec<u32>>,
}

impl WakeBuffer {
    fn new() -> Self {
        Self {
            columns: Vec::new(),
            grid: HashMap::new(),
        }
    }

    fn clear(&mut self) {
        self.columns.clear();
        self.grid.clear();
    }

    fn push(&mut self, col: WakeColumn, cell_size: f32) {
        let idx = self.columns.len() as u32;
        let key = cell_key(col.origin, cell_size);
        self.grid.entry(key).or_default().push(idx);
        self.columns.push(col);
    }
}

/// The double-buffered wake field.
#[derive(Debug, Clone)]
pub struct WakeField {
    /// Read side (last tick's contributions).
    read: WakeBuffer,
    /// Write side (this tick's contributions, accumulated).
    write: WakeBuffer,
    /// Uniform grid cell size. Tuned so that a typical drone's wake
    /// column fits within one cell.
    pub cell_size: f32,
}

impl WakeField {
    pub fn new(cell_size: f32) -> Self {
        Self {
            read: WakeBuffer::new(),
            write: WakeBuffer::new(),
            cell_size: cell_size.max(0.1),
        }
    }

    /// Default for 5" quads: cell size ≈ 2× max axial decay.
    pub fn default_for_racing() -> Self {
        Self::new(10.0)
    }

    /// Number of columns in the current read snapshot.
    pub fn column_count(&self) -> usize {
        self.read.columns.len()
    }

    /// Add a prop's contribution to the write buffer (current tick).
    /// Call once per prop per tick.
    pub fn contribute(&mut self, col: WakeColumn) {
        if col.v_h <= 0.0 {
            return; // no thrust → no wake contribution
        }
        self.write.push(col, self.cell_size);
    }

    /// Swap read ↔ write. Called once per physics frame by a
    /// coordinator (the `WakeFieldNode` in Godot) after all drones
    /// have stepped.
    pub fn swap(&mut self) {
        std::mem::swap(&mut self.read, &mut self.write);
        self.write.clear();
    }

    /// Sample the wake velocity at `pos` from the read snapshot.
    ///
    /// Returns the world-frame velocity contribution at `pos` — this
    /// is the additional inflow a prop sitting at `pos` experiences
    /// due to other drones' downwash. Always points in the downwash
    /// direction (i.e. along each column's axis, away from the prop
    /// face — typically downward for a level quad).
    ///
    /// `own_origin` is an optional position to exclude self-sampling
    /// (a prop shouldn't sample its own wake column). Columns whose
    /// origin is within `self_exclude_radius` of `own_origin` are
    /// skipped.
    pub fn sample(
        &self,
        pos: [f32; 3],
        own_origin: Option<[f32; 3]>,
        self_exclude_radius: f32,
    ) -> [f32; 3] {
        let key = cell_key(pos, self.cell_size);
        let mut acc = [0.0_f32; 3];

        // Query the 27 cells around `pos` (3×3×3 stencil).
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let k = [key[0] + dx, key[1] + dy, key[2] + dz];
                    if let Some(indices) = self.read.grid.get(&k) {
                        for &idx in indices {
                            let col = &self.read.columns[idx as usize];
                            // Self-exclusion.
                            if let Some(own) = own_origin {
                                if dist_sq(col.origin, own) < self_exclude_radius * self_exclude_radius
                                {
                                    continue;
                                }
                            }
                            let v = sample_column(col, pos);
                            acc[0] += v[0];
                            acc[1] += v[1];
                            acc[2] += v[2];
                        }
                    }
                }
            }
        }

        acc
    }
}

/// Evaluate one column's contribution at a world-space point.
fn sample_column(col: &WakeColumn, pos: [f32; 3]) -> [f32; 3] {
    let r = sub(pos, col.origin);
    let r_axial = dot(r, col.axis);
    // Only downstream of the disc (positive axial distance).
    if r_axial <= 0.0 {
        return [0.0; 3];
    }
    let r_radial_vec = [
        r[0] - col.axis[0] * r_axial,
        r[1] - col.axis[1] * r_axial,
        r[2] - col.axis[2] * r_axial,
    ];
    let r_radial = length(r_radial_vec);

    // Gaussian kernel: exponential decay axially and radially.
    let f_axial = (-r_axial / col.axial_decay.max(0.01)).exp();
    let f_radial = (-(r_radial / col.radius.max(0.01)).powi(2)).exp();

    let mag = col.v_h * f_axial * f_radial;
    // Velocity is in the downwash direction (along axis).
    [col.axis[0] * mag, col.axis[1] * mag, col.axis[2] * mag]
}

#[inline]
fn sub(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

#[inline]
fn length(v: [f32; 3]) -> f32 {
    dot(v, v).sqrt()
}

#[inline]
fn dist_sq(a: [f32; 3], b: [f32; 3]) -> f32 {
    let d = sub(a, b);
    dot(d, d)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn col_down(origin: [f32; 3], v_h: f32) -> WakeColumn {
        WakeColumn {
            origin,
            axis: [0.0, -1.0, 0.0], // downwash = −Y (standard level quad)
            v_h,
            radius: 0.065,
            axial_decay: 2.0,
        }
    }

    #[test]
    fn empty_field_zero_sample() {
        let f = WakeField::default_for_racing();
        let v = f.sample([0.0, 0.0, 0.0], None, 0.0);
        assert_eq!(v, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn contribute_and_swap_then_sample() {
        let mut f = WakeField::default_for_racing();
        f.contribute(col_down([0.0, 5.0, 0.0], 3.0));
        // Before swap, read buffer is empty.
        let v_pre = f.sample([0.0, 3.0, 0.0], None, 0.0);
        assert_eq!(v_pre, [0.0, 0.0, 0.0]);
        // After swap, contribution is visible.
        f.swap();
        let v_post = f.sample([0.0, 3.0, 0.0], None, 0.0);
        // Column at y=5 with axis −Y; sample point at y=3 is 2 m
        // below (positive axial distance along −Y). Should see a
        // non-zero downward velocity.
        assert!(v_post[1] < -0.1, "expected downward wake, got {:?}", v_post);
    }

    #[test]
    fn sample_upstream_is_zero() {
        let mut f = WakeField::default_for_racing();
        // Column at y=5, axis −Y.
        f.contribute(col_down([0.0, 5.0, 0.0], 3.0));
        f.swap();
        // Sample point ABOVE the prop (y=7) — upstream, should be zero.
        let v = f.sample([0.0, 7.0, 0.0], None, 0.0);
        assert_eq!(v, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn sample_decays_with_axial_distance() {
        let mut f = WakeField::default_for_racing();
        f.contribute(col_down([0.0, 10.0, 0.0], 3.0));
        f.swap();
        // Close below: 1 m
        let v_close = f.sample([0.0, 9.0, 0.0], None, 0.0);
        // Far below: 5 m
        let v_far = f.sample([0.0, 5.0, 0.0], None, 0.0);
        let mag_close = length(v_close);
        let mag_far = length(v_far);
        assert!(
            mag_close > mag_far * 2.0,
            "close={mag_close} far={mag_far}"
        );
    }

    #[test]
    fn sample_decays_with_radial_distance() {
        let mut f = WakeField::default_for_racing();
        f.contribute(col_down([0.0, 10.0, 0.0], 3.0));
        f.swap();
        // Directly below (r=0): 2 m axial.
        let v_on_axis = f.sample([0.0, 8.0, 0.0], None, 0.0);
        // Same axial, but offset radially by 0.5 m.
        let v_off_axis = f.sample([0.5, 8.0, 0.0], None, 0.0);
        let mag_on = length(v_on_axis);
        let mag_off = length(v_off_axis);
        assert!(
            mag_on > mag_off,
            "on_axis={mag_on}, off_axis={mag_off}"
        );
    }

    #[test]
    fn self_exclusion_skips_own_column() {
        let mut f = WakeField::default_for_racing();
        let origin = [0.0, 5.0, 0.0];
        f.contribute(col_down(origin, 3.0));
        f.swap();
        // Sample directly below the column but with self-exclusion.
        let v = f.sample([0.0, 3.0, 0.0], Some(origin), 0.1);
        // The only column is self — should be excluded.
        assert_eq!(v, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn self_exclusion_does_not_skip_others() {
        let mut f = WakeField::default_for_racing();
        let own = [10.0, 5.0, 0.0];
        let other = [0.0, 5.0, 0.0];
        f.contribute(col_down(own, 3.0));
        f.contribute(col_down(other, 3.0));
        f.swap();
        // Sample below `other`, excluding `own`.
        let v = f.sample([0.0, 3.0, 0.0], Some(own), 0.1);
        assert!(v[1] < -0.1, "should see other's wake: {:?}", v);
    }

    #[test]
    fn swap_clears_write_buffer() {
        let mut f = WakeField::default_for_racing();
        f.contribute(col_down([0.0, 5.0, 0.0], 3.0));
        f.swap();
        assert_eq!(f.column_count(), 1);
        // Nothing contributed this tick; swap again.
        f.swap();
        assert_eq!(f.column_count(), 0);
    }

    #[test]
    fn spatial_hash_finds_nearby_columns() {
        let mut f = WakeField::new(5.0);
        // Two columns far apart — only the nearby one should contribute.
        f.contribute(col_down([0.0, 10.0, 0.0], 3.0));
        f.contribute(col_down([100.0, 10.0, 0.0], 3.0));
        f.swap();
        let v = f.sample([0.0, 8.0, 0.0], None, 0.0);
        // Only the column at x=0 is within the 27-cell stencil.
        assert!(length(v) > 0.0);
        // The far column's contribution should be zero (outside stencil).
        let v_far = f.sample([100.0, 8.0, 0.0], None, 0.0);
        assert!(length(v_far) > 0.0);
        // The midpoint should see nothing (neither column's cell stencil
        // covers it because cell_size=5, distance=50).
        let v_mid = f.sample([50.0, 8.0, 0.0], None, 0.0);
        assert_abs_diff_eq!(length(v_mid), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn induced_velocity_typical() {
        // 5" prop, T = 1.5 N (hover per motor), rho = 1.225.
        let v = induced_velocity(1.5, 1.225, 0.065);
        // v_h = sqrt(1.5 / (2 * 1.225 * pi * 0.065²)) ≈ 5.4 m/s.
        assert!(v > 4.0 && v < 7.0, "v_h = {v}");
    }

    #[test]
    fn induced_velocity_zero_thrust() {
        assert_eq!(induced_velocity(0.0, 1.225, 0.065), 0.0);
        assert_eq!(induced_velocity(-1.0, 1.225, 0.065), 0.0);
    }

    #[test]
    fn zero_v_h_not_contributed() {
        let mut f = WakeField::default_for_racing();
        let mut c = col_down([0.0, 5.0, 0.0], 0.0);
        c.v_h = 0.0;
        f.contribute(c);
        f.swap();
        assert_eq!(f.column_count(), 0);
    }

    #[test]
    fn two_drones_stacked_lower_sees_wake() {
        // Scenario from the design plan smoke test: drone A at y=10,
        // drone B directly below at y=5. B should sample a non-trivial
        // downward velocity at its props from A's wake.
        let mut f = WakeField::default_for_racing();
        // A's 4 props, all at y≈10.
        for dx in [-0.074_f32, 0.074] {
            for dz in [-0.074_f32, 0.074] {
                f.contribute(WakeColumn {
                    origin: [dx, 10.0, dz],
                    axis: [0.0, -1.0, 0.0],
                    v_h: 5.0,
                    radius: 0.065,
                    axial_decay: 3.0,
                });
            }
        }
        f.swap();
        // B's prop at (0.074, 5.0, 0.074) samples.
        let v = f.sample([0.074, 5.0, 0.074], None, 0.0);
        let mag = length(v);
        assert!(
            mag > 0.5,
            "lower drone should feel substantial wake, got {mag} m/s"
        );
        // Direction should be predominantly downward (−Y).
        assert!(v[1] < -0.3, "wake should be downward, got {:?}", v);
    }
}
