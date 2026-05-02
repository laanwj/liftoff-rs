//! Flight modes. Currently only `AcroMode`; the trait is here to make
//! future modes (Angle, Horizon, AltHold, PosHold) drop-in additions
//! without touching the FC pipeline below.
//!
//! `FlightMode` produces a target angular rate (deg/s) for each of the
//! roll/pitch/yaw axes from the pilot's normalised stick commands plus
//! whatever truth state the mode needs (attitude, position, velocity).

use crate::fc::rates::ActualAxis;

/// Normalised pilot stick inputs. All axes in `[-1, +1]` except
/// throttle which is `[0, 1]`.
#[derive(Debug, Clone, Copy, Default)]
pub struct SticksNorm {
    /// Roll stick. +1 = right.
    pub roll: f32,
    /// Pitch stick. +1 = nose up (stick pulled back).
    pub pitch: f32,
    /// Throttle stick. 0 = idle, 1 = full.
    pub throttle: f32,
    /// Yaw stick. +1 = nose right.
    pub yaw: f32,
}

/// Truth-state of the body, in body-frame angular rates and world-frame
/// attitude. Modes that don't need attitude (Acro) can ignore those
/// fields.
#[derive(Debug, Clone, Copy, Default)]
pub struct BodyTruth {
    /// Body angular rates (deg/s): roll, pitch, yaw.
    pub gyro_dps: [f32; 3],
    /// Body attitude as Euler angles (deg) — used by Angle/Horizon
    /// modes when they exist; ignored by Acro.
    pub attitude_deg: [f32; 3],
}

/// FlightMode trait. Returns `[roll, pitch, yaw]` rate setpoints in
/// deg/s for the inner rate-PID to track.
pub trait FlightMode {
    fn rate_setpoint(&mut self, sticks: &SticksNorm, truth: &BodyTruth, dt: f32) -> [f32; 3];
}

/// Acro / Rate mode: stick deflection commands a body angular rate via
/// the per-axis Actual rates curves.
#[derive(Debug, Clone, Copy)]
pub struct AcroMode {
    pub roll: ActualAxis,
    pub pitch: ActualAxis,
    pub yaw: ActualAxis,
}

impl AcroMode {
    pub const fn new(roll: ActualAxis, pitch: ActualAxis, yaw: ActualAxis) -> Self {
        Self { roll, pitch, yaw }
    }
}

impl FlightMode for AcroMode {
    fn rate_setpoint(&mut self, sticks: &SticksNorm, _truth: &BodyTruth, _dt: f32) -> [f32; 3] {
        [
            self.roll.evaluate(sticks.roll),
            self.pitch.evaluate(sticks.pitch),
            self.yaw.evaluate(sticks.yaw),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn mode() -> AcroMode {
        AcroMode::new(
            ActualAxis::new(100.0, 800.0, 0.0),
            ActualAxis::new(100.0, 800.0, 0.0),
            ActualAxis::new(50.0, 400.0, 0.0),
        )
    }

    #[test]
    fn zero_sticks_zero_setpoint() {
        let mut m = mode();
        let sp = m.rate_setpoint(&SticksNorm::default(), &BodyTruth::default(), 0.01);
        for v in sp {
            assert_eq!(v, 0.0);
        }
    }

    #[test]
    fn full_sticks_hit_max_rate() {
        let mut m = mode();
        let sticks = SticksNorm {
            roll: 1.0,
            pitch: 1.0,
            yaw: 1.0,
            throttle: 1.0,
        };
        let sp = m.rate_setpoint(&sticks, &BodyTruth::default(), 0.01);
        assert_abs_diff_eq!(sp[0], 800.0, epsilon = 1e-3);
        assert_abs_diff_eq!(sp[1], 800.0, epsilon = 1e-3);
        assert_abs_diff_eq!(sp[2], 400.0, epsilon = 1e-3);
    }

    #[test]
    fn negative_sticks_negative_setpoint() {
        let mut m = mode();
        let sticks = SticksNorm {
            roll: -0.5,
            pitch: -0.5,
            yaw: -0.5,
            throttle: 0.0,
        };
        let sp = m.rate_setpoint(&sticks, &BodyTruth::default(), 0.01);
        assert!(sp[0] < 0.0);
        assert!(sp[1] < 0.0);
        assert!(sp[2] < 0.0);
    }

    #[test]
    fn ignores_truth_state_in_acro() {
        let mut m = mode();
        let sticks = SticksNorm::default();
        let truth_a = BodyTruth::default();
        let truth_b = BodyTruth {
            gyro_dps: [100.0, 200.0, 300.0],
            attitude_deg: [10.0, 20.0, 30.0],
        };
        let sp_a = m.rate_setpoint(&sticks, &truth_a, 0.01);
        let sp_b = m.rate_setpoint(&sticks, &truth_b, 0.01);
        for i in 0..3 {
            assert_eq!(sp_a[i], sp_b[i]);
        }
    }
}
