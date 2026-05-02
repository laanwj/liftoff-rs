//! `DronePresetData` — a pure-Rust struct holding everything the
//! per-tick simulation needs. Built from a Godot `DronePreset` Resource
//! (gdext-side, in the wrapper) at scene load. Defaults provide a
//! reasonable 5" racing quad.
//!
//! Keeping this struct free of Godot types means the pipeline can be
//! exercised end-to-end in pure-Rust unit tests.


use crate::fc::mode::AcroMode;
use crate::fc::pid::PidGains;
use crate::fc::rates::{ActualAxis, ThrottleCurve};
use crate::physics::battery::BatteryParams;
use crate::physics::drag::DragParams;
use crate::physics::ground::GroundEffectParams;
use crate::physics::motor::MotorParams;
use crate::physics::thrust::ThrustParams;

/// Body / inertial properties.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct BodyParams {
    pub mass_kg: f32,
    /// Principal moments of inertia (kg·m²), expressed as the diagonal
    /// of the inertia tensor in body frame: `[I_xx, I_yy, I_zz]`.
    ///
    /// This assumes the body-frame axes coincide with the principal
    /// axes of inertia — true for any symmetric quad with a centred
    /// battery. Godot/Jolt only accepts diagonal inertia
    /// (`RigidBody3D.inertia` is a `Vector3`); the engine doesn't
    /// support off-diagonal (product-of-inertia) terms directly.
    ///
    /// For asymmetric craft (off-centre payload, hex frames, etc.)
    /// the correct approach would be to diagonalise the full 3×3
    /// tensor, use the eigenvalues here, and apply the eigenvector
    /// rotation to the collision shape so body frame = principal frame.
    /// That's a future enhancement — for now all presets assume
    /// symmetric builds where the diagonal is exact.
    pub inertia_diag: [f32; 3],
}

impl BodyParams {
    pub const RACING_5INCH: BodyParams = BodyParams {
        mass_kg: 0.6,
        inertia_diag: [0.0035, 0.005, 0.0035],
    };
}

/// Per-prop body-frame geometry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropGeometry {
    /// Body-frame mount position in metres.
    pub offset: [f32; 3],
    /// Body-frame thrust axis (unit). Conventionally +Y for an X-quad.
    pub axis: [f32; 3],
    /// +1 = CW, -1 = CCW, viewed from above (looking down −Y).
    pub handedness: i8,
}

/// Geometry of a 5" quad with ~210 mm wheelbase.
///
/// Layout (Godot left-handed, +X right, +Y up, −Z forward):
///
/// ```text
/// M0 (FL, CW)   M1 (FR, CCW)
///       \        /
///        center
///        /     \
/// M3 (RL, CCW)  M2 (RR, CW)
/// ```
pub const RACING_5INCH_PROPS: [PropGeometry; 4] = [
    PropGeometry {
        offset: [-0.074, 0.0, -0.074],
        axis: [0.0, 1.0, 0.0],
        handedness: 1,
    },
    PropGeometry {
        offset: [0.074, 0.0, -0.074],
        axis: [0.0, 1.0, 0.0],
        handedness: -1,
    },
    PropGeometry {
        offset: [0.074, 0.0, 0.074],
        axis: [0.0, 1.0, 0.0],
        handedness: 1,
    },
    PropGeometry {
        offset: [-0.074, 0.0, 0.074],
        axis: [0.0, 1.0, 0.0],
        handedness: -1,
    },
];

/// FPV camera placement / lens.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CameraParams {
    /// Upward tilt about body X, degrees. Range typically 0–55.
    pub tilt_deg: f32,
    /// Pinhole vertical FOV, degrees. ~120 for FPV.
    pub fov_deg: f32,
    /// Body-frame mount position in metres.
    pub offset: [f32; 3],
}

impl Default for CameraParams {
    fn default() -> Self {
        Self {
            tilt_deg: 30.0,
            fov_deg: 120.0,
            offset: [0.0, 0.02, 0.04],
        }
    }
}

/// Pilot-set rate curves: per-axis Actual rates and the throttle curve.
#[derive(Debug, Clone, Copy)]
pub struct RatesPreset {
    pub roll: ActualAxis,
    pub pitch: ActualAxis,
    pub yaw: ActualAxis,
    pub throttle: ThrottleCurve,
}

impl RatesPreset {
    pub const RACING_5INCH: RatesPreset = RatesPreset {
        roll: ActualAxis::new(150.0, 800.0, 0.45),
        pitch: ActualAxis::new(150.0, 800.0, 0.45),
        yaw: ActualAxis::new(120.0, 600.0, 0.30),
        throttle: ThrottleCurve {
            mid: 0.5,
            expo: 0.0,
        },
    };

    pub fn into_acro_mode(self) -> AcroMode {
        AcroMode::new(self.roll, self.pitch, self.yaw)
    }
}

/// PID gains for the inner rate loop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PidPreset {
    pub roll: PidGains,
    pub pitch: PidGains,
    pub yaw: PidGains,
}

impl PidPreset {
    /// Conservative starting gains. Sized so that an angular-rate
    /// error of ~600 deg/s saturates the PID output at ±1.0 (mixer
    /// command). Real Betaflight tunes are aggressive; we err on
    /// the gentle side until the rest of the stack is in place.
    pub const RACING_5INCH: PidPreset = PidPreset {
        roll: PidGains::new(0.0015, 0.008, 0.00005),
        pitch: PidGains::new(0.0015, 0.008, 0.00005),
        yaw: PidGains::new(0.0015, 0.012, 0.0),
    };
}

/// Crash-detection / respawn knobs.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DamageParams {
    pub crash_impulse_threshold_n_s: f32,
    pub respawn_cooldown_s: f32,
}

impl Default for DamageParams {
    fn default() -> Self {
        Self {
            crash_impulse_threshold_n_s: 8.0,
            respawn_cooldown_s: 2.0,
        }
    }
}

/// The full preset, all pure Rust.
#[derive(Debug, Clone)]
pub struct DronePresetData {
    pub body: BodyParams,
    pub props: [PropGeometry; 4],
    pub motor: MotorParams,
    pub thrust: ThrustParams,
    pub battery: BatteryParams,
    pub drag: DragParams,
    pub ground_effect: GroundEffectParams,
    pub rates: RatesPreset,
    pub pids: PidPreset,

    pub damage: DamageParams,
    pub camera: CameraParams,
}

impl DronePresetData {
    /// Default 5" racing quad. All other modules use this as the
    /// "reasonable starting point" reference.
    pub fn racing_5inch() -> Self {
        Self {
            body: BodyParams::RACING_5INCH,
            props: RACING_5INCH_PROPS,
            motor: MotorParams::RACING_5INCH,
            thrust: ThrustParams::RACING_5INCH,
            battery: BatteryParams::RACING_4S_1500,
            drag: DragParams::RACING_5INCH,
            ground_effect: GroundEffectParams::RACING_5INCH,
            rates: RatesPreset::RACING_5INCH,
            pids: PidPreset::RACING_5INCH,

            damage: DamageParams::default(),
            camera: CameraParams::default(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn racing_5inch_assembles() {
        let p = DronePresetData::racing_5inch();
        // Sanity checks on the assembled preset.
        assert_eq!(p.body.mass_kg, 0.6);
        assert_eq!(p.props.len(), 4);
        // Two CW, two CCW for an X-quad.
        let cw = p.props.iter().filter(|q| q.handedness > 0).count();
        let ccw = p.props.iter().filter(|q| q.handedness < 0).count();
        assert_eq!(cw, 2);
        assert_eq!(ccw, 2);
    }

    #[test]
    fn props_in_x_layout() {
        let p = DronePresetData::racing_5inch();
        // Front-left and rear-right should be diagonally opposite, both CW.
        assert!(p.props[0].offset[0] < 0.0 && p.props[0].offset[2] < 0.0);
        assert!(p.props[2].offset[0] > 0.0 && p.props[2].offset[2] > 0.0);
        assert_eq!(p.props[0].handedness, p.props[2].handedness);
        // FR and RL — CCW pair.
        assert!(p.props[1].offset[0] > 0.0 && p.props[1].offset[2] < 0.0);
        assert!(p.props[3].offset[0] < 0.0 && p.props[3].offset[2] > 0.0);
        assert_eq!(p.props[1].handedness, p.props[3].handedness);
    }

    #[test]
    fn twr_above_two() {
        // Sanity check: thrust-to-weight ratio at full throttle should
        // be well above 1, otherwise a racing drone can't climb hard.
        // 4 × T_per_motor at full throttle / (mass × g).
        let p = DronePresetData::racing_5inch();
        let v_pack = (p.battery.cells as f32) * 4.0; // ~16 V at near-full
        let rpm_full = v_pack * p.motor.kv * p.motor.loading_factor;
        let rpm_k = rpm_full / 1000.0;
        let t_per_motor = p.thrust.k_t * rpm_k * rpm_k;
        let twr = (4.0 * t_per_motor) / (p.body.mass_kg * 9.81);
        assert!(
            twr >= 2.0 && twr <= 8.0,
            "TWR should be 2..8, got {twr:.2}"
        );
    }

    #[test]
    fn hover_throttle_below_full() {
        // At hover, total thrust must equal weight, and the per-motor
        // RPM that achieves that must be well below full-throttle RPM.
        let p = DronePresetData::racing_5inch();
        let v_pack = (p.battery.cells as f32) * 3.85; // mid-charge
        let rpm_full = v_pack * p.motor.kv * p.motor.loading_factor;
        // Hover thrust per motor:
        let t_hover = p.body.mass_kg * 9.81 / 4.0;
        // Solve k_t · (rpm/1000)² = t_hover ⇒ rpm = 1000 * sqrt(t_hover / k_t).
        let rpm_hover = 1000.0 * (t_hover / p.thrust.k_t).sqrt();
        let frac = rpm_hover / rpm_full;
        assert!(
            frac < 0.7 && frac > 0.3,
            "hover RPM frac should be 0.3..0.7, got {frac:.3}"
        );
    }
}
