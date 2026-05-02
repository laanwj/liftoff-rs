//! Battery model.
//!
//! The formula is `SagBattery` — `V_terminal = V_oc(SOC) − I · R_internal`,
//! per-cell open-circuit voltage from a tabulated LiPo discharge curve.
//! Reports a `brownout_factor` in `[0, 1]` that the motor model
//! multiplies into its output, so a dying battery degrades thrust
//! smoothly rather than cliff-edging when it crosses cutoff.

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BatteryParams {
    /// Cell count (4S = 4).
    pub cells: u8,
    /// Internal resistance per cell, milliohms.
    pub internal_r_mohm_per_cell: f32,
    /// Pack capacity in mAh.
    pub capacity_mah: f32,
    /// Per-cell voltage at which brownout-factor reaches 0.
    pub cutoff_v_per_cell: f32,
    /// Width of the brownout fade band, volts.
    pub cutoff_band_v: f32,
}

impl BatteryParams {
    /// Realistic 4S 1500 mAh racing pack.
    pub const RACING_4S_1500: BatteryParams = BatteryParams {
        cells: 4,
        internal_r_mohm_per_cell: 8.0,
        capacity_mah: 1500.0,
        cutoff_v_per_cell: 3.3,
        cutoff_band_v: 0.2,
    };
}

#[derive(Debug, Clone, Copy)]
pub struct BatteryState {
    /// Per-cell terminal voltage, volts (after sag).
    pub v_cell_terminal: f32,
    /// Per-cell open-circuit voltage from the discharge curve.
    pub v_cell_open_circuit: f32,
    /// Pack-level terminal voltage = cells × v_cell_terminal.
    pub v_pack_terminal: f32,
    /// Charge consumed from the pack since start, mAh.
    pub consumed_mah: f32,
    /// State of charge in `[0, 1]`, 1 = full, 0 = empty.
    pub soc: f32,
    /// Multiplier in `[0, 1]` to apply to motor output. 1 when V_cell
    /// is well above cutoff; fades to 0 across `cutoff_band_v` as
    /// V_cell drops to `cutoff_v_per_cell`.
    pub brownout_factor: f32,
}

#[derive(Debug, Clone)]
pub struct SagBattery {
    pub params: BatteryParams,
    consumed_mah: f32,
}

impl SagBattery {
    pub fn new(params: BatteryParams) -> Self {
        Self {
            params,
            consumed_mah: 0.0,
        }
    }

    pub fn consumed_mah(&self) -> f32 {
        self.consumed_mah
    }

    /// Reset to fully charged.
    pub fn reset_full(&mut self) {
        self.consumed_mah = 0.0;
    }

    /// Read battery state at the *current* SOC and a hypothetical
    /// load current, without advancing consumed_mAh. Used by callers
    /// that need V_pack to compute their own current draw before
    /// committing it via `step`.
    pub fn peek(&self, current_a: f32) -> BatteryState {
        self.compute_state(self.consumed_mah, current_a)
    }

    /// Step forward by `dt` seconds at the given pack-level current
    /// draw. Returns the new BatteryState.
    pub fn step(&mut self, current_a: f32, dt: f32) -> BatteryState {
        // Update consumed mAh: I (A) × dt (s) × (1000 mA/A) × (1 h / 3600 s)
        // = I · dt · 1000 / 3600 = I · dt · (1/3.6) mAh
        let dmah = current_a.max(0.0) * dt * (1000.0 / 3600.0);
        self.consumed_mah = (self.consumed_mah + dmah).min(self.params.capacity_mah);
        self.compute_state(self.consumed_mah, current_a)
    }

    pub fn soc(&self) -> f32 {
        soc_for(self.consumed_mah, self.params.capacity_mah)
    }

    fn compute_state(&self, consumed_mah: f32, current_a: f32) -> BatteryState {
        let soc = soc_for(consumed_mah, self.params.capacity_mah);
        let v_oc = lipo_v_oc_per_cell(soc);

        // Sag: V_terminal = V_oc - I · R_internal_total
        // R per cell in ohms = mOhm / 1000.0; per pack = R_per_cell × cells (series).
        let r_per_cell_ohm = self.params.internal_r_mohm_per_cell / 1000.0;
        let r_pack_ohm = r_per_cell_ohm * self.params.cells as f32;
        let v_pack_oc = v_oc * self.params.cells as f32;
        let v_pack_terminal = (v_pack_oc - current_a.max(0.0) * r_pack_ohm).max(0.0);
        let v_cell_terminal = v_pack_terminal / self.params.cells as f32;

        let brownout_factor = brownout_factor(
            v_cell_terminal,
            self.params.cutoff_v_per_cell,
            self.params.cutoff_band_v,
        );

        BatteryState {
            v_cell_terminal,
            v_cell_open_circuit: v_oc,
            v_pack_terminal,
            consumed_mah,
            soc,
            brownout_factor,
        }
    }
}

fn soc_for(consumed_mah: f32, capacity_mah: f32) -> f32 {
    let cap = capacity_mah.max(1e-6);
    (1.0 - consumed_mah / cap).clamp(0.0, 1.0)
}

/// Brownout fade: returns 1.0 well above cutoff, fades linearly to 0.0
/// over `band` volts as V_cell drops past `cutoff`. Below cutoff returns 0.
pub fn brownout_factor(v_cell: f32, cutoff: f32, band: f32) -> f32 {
    if band <= 0.0 {
        return if v_cell >= cutoff { 1.0 } else { 0.0 };
    }
    ((v_cell - cutoff) / band).clamp(0.0, 1.0)
}

/// Tabulated LiPo discharge curve, per cell: SOC ∈ `[0, 1]` → V_oc.
/// Standard shape: ~4.2 V at full charge, knee around 60% SOC,
/// ~3.7 V nominal at mid charge, ~3.3 V near empty.
///
/// Linear-interpolated from a hand-tuned 11-point table that's
/// "good enough" for sim feel — not a manufacturer datasheet.
pub fn lipo_v_oc_per_cell(soc: f32) -> f32 {
    let table = LIPO_DISCHARGE;
    let s = soc.clamp(0.0, 1.0);
    // Find segment.
    for w in table.windows(2) {
        let (s0, v0) = w[0];
        let (s1, v1) = w[1];
        if s >= s0 && s <= s1 {
            let t = if (s1 - s0).abs() < 1e-6 {
                0.0
            } else {
                (s - s0) / (s1 - s0)
            };
            return v0 + (v1 - v0) * t;
        }
    }
    // s exactly on the upper endpoint.
    table.last().unwrap().1
}

const LIPO_DISCHARGE: &[(f32, f32)] = &[
    (0.00, 3.20),
    (0.05, 3.45),
    (0.10, 3.55),
    (0.20, 3.65),
    (0.30, 3.72),
    (0.40, 3.77),
    (0.50, 3.81),
    (0.60, 3.86),
    (0.70, 3.91),
    (0.80, 3.98),
    (0.90, 4.08),
    (1.00, 4.20),
];

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn pack() -> SagBattery {
        SagBattery::new(BatteryParams::RACING_4S_1500)
    }

    #[test]
    fn full_pack_open_circuit_voltage() {
        assert_abs_diff_eq!(lipo_v_oc_per_cell(1.0), 4.20, epsilon = 1e-3);
    }

    #[test]
    fn empty_pack_open_circuit_voltage() {
        assert_abs_diff_eq!(lipo_v_oc_per_cell(0.0), 3.20, epsilon = 1e-3);
    }

    #[test]
    fn discharge_monotonic() {
        // V_oc strictly non-decreasing with SOC.
        let mut prev = 0.0;
        for i in 0..=100 {
            let v = lipo_v_oc_per_cell(i as f32 / 100.0);
            assert!(v + 1e-4 >= prev, "non-monotonic at i={i}");
            prev = v;
        }
    }

    #[test]
    fn discharge_clamps_out_of_range() {
        assert_eq!(lipo_v_oc_per_cell(-0.5), lipo_v_oc_per_cell(0.0));
        assert_eq!(lipo_v_oc_per_cell(1.5), lipo_v_oc_per_cell(1.0));
    }

    #[test]
    fn no_load_no_sag() {
        let mut b = pack();
        let s = b.step(0.0, 1.0);
        assert_abs_diff_eq!(s.v_cell_terminal, s.v_cell_open_circuit, epsilon = 1e-6);
    }

    #[test]
    fn under_load_voltage_sags() {
        let mut b = pack();
        let no_load = b.step(0.0, 0.0).v_cell_terminal;
        let mut b2 = pack();
        let loaded = b2.step(40.0, 0.0).v_cell_terminal;
        assert!(loaded < no_load, "no_load={no_load}, loaded={loaded}");
        // Sag = I × R = 40 A × (8 mΩ × 4 cells) / 1000 = 1.28 V at pack.
        // Per cell: 0.32 V.
        let expected_sag_per_cell = 40.0 * (8e-3 * 4.0) / 4.0;
        assert_abs_diff_eq!(no_load - loaded, expected_sag_per_cell, epsilon = 1e-3);
    }

    #[test]
    fn discharge_consumes_capacity() {
        let mut b = pack();
        // Draw 30 A for 60 s = 30/3600 × 60 × 1000 = 500 mAh.
        for _ in 0..6000 {
            b.step(30.0, 0.01);
        }
        assert_abs_diff_eq!(b.consumed_mah(), 500.0, epsilon = 1.0);
    }

    #[test]
    fn soc_decreases_under_load() {
        let mut b = pack();
        let initial = b.soc();
        for _ in 0..1000 {
            b.step(30.0, 0.01);
        }
        let later = b.soc();
        assert!(later < initial);
    }

    #[test]
    fn capacity_clamp_no_overdraw() {
        let mut b = pack();
        // Draw way more than capacity.
        for _ in 0..100000 {
            b.step(30.0, 0.01);
        }
        assert!(b.consumed_mah() <= b.params.capacity_mah + 1e-3);
    }

    #[test]
    fn brownout_smooth_fade() {
        let cutoff = 3.3;
        let band = 0.2;
        // Far above: 1.0
        assert_abs_diff_eq!(brownout_factor(4.0, cutoff, band), 1.0, epsilon = 1e-6);
        // At cutoff: 0.0
        assert_abs_diff_eq!(brownout_factor(3.3, cutoff, band), 0.0, epsilon = 1e-6);
        // Below cutoff: 0.0
        assert_abs_diff_eq!(brownout_factor(3.0, cutoff, band), 0.0, epsilon = 1e-6);
        // Mid-band: 0.5
        assert_abs_diff_eq!(brownout_factor(3.4, cutoff, band), 0.5, epsilon = 1e-6);
    }

    #[test]
    fn brownout_zero_band_is_step_function() {
        assert_eq!(brownout_factor(3.31, 3.3, 0.0), 1.0);
        assert_eq!(brownout_factor(3.29, 3.3, 0.0), 0.0);
    }

    #[test]
    fn reset_restores_full_charge() {
        let mut b = pack();
        for _ in 0..1000 {
            b.step(30.0, 0.01);
        }
        assert!(b.soc() < 1.0);
        b.reset_full();
        assert_eq!(b.soc(), 1.0);
        assert_eq!(b.consumed_mah(), 0.0);
    }

    #[test]
    fn peek_does_not_advance_consumption() {
        let mut b = pack();
        b.step(20.0, 0.5);
        let mah_before = b.consumed_mah();
        let _peeked = b.peek(50.0);
        let _again = b.peek(0.0);
        assert_eq!(b.consumed_mah(), mah_before);
    }

    #[test]
    fn peek_voltage_matches_step_at_same_load() {
        let mut b = pack();
        let peeked = b.peek(40.0);
        let stepped = b.step(40.0, 0.0);
        // Stepping with dt=0 doesn't consume any mAh, so the two
        // states should be identical.
        assert_abs_diff_eq!(
            peeked.v_pack_terminal,
            stepped.v_pack_terminal,
            epsilon = 1e-6
        );
        assert_abs_diff_eq!(
            peeked.brownout_factor,
            stepped.brownout_factor,
            epsilon = 1e-6
        );
    }
}
