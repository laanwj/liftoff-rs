//! `DronePresetData` — a pure-Rust struct holding everything the
//! per-tick simulation needs. Built from a Godot `DronePreset` Resource
//! (gdext-side, in the wrapper) at scene load. Defaults provide a
//! reasonable 5" racing quad.
//!
//! Keeping this struct free of Godot types means the pipeline can be
//! exercised end-to-end in pure-Rust unit tests.


use quad_flight_control::mixer::{MIXER_MATRIX_PROPS_IN, MIXER_MATRIX_PROPS_OUT};
use quad_flight_control::mode::AcroMode;
use quad_flight_control::pid::PidGains;
use quad_flight_control::rates::{ActualAxis, ThrottleCurve};
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

    /// BetaFPV Aquila16 with the 1100 mAh battery. 72.5 g takeoff
    /// weight; inertia is a point-mass-at-corners estimate with a
    /// ~1.7× correction for the distributed body/battery mass.
    /// Refinable from a bifilar-pendulum measurement.
    pub const AQUILA16: BodyParams = BodyParams {
        mass_kg: 0.0725,
        inertia_diag: [1.0e-4, 2.0e-4, 1.0e-4],
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

/// Geometry of the BetaFPV Aquila16 — 86 mm wheelbase, inverted motor
/// mount (rotor cap down) so the prop disc sits ~10 mm below CG.
/// Handedness is the props-out / "inverted-quadcopter" pattern: LF+RB
/// CCW, RF+LB CW — opposite the 5"-racing default. Per-motor offsets
/// are `±(wheelbase / 2) / √2` ≈ ±30.4 mm in each of X and Z.
pub const AQUILA16_PROPS: [PropGeometry; 4] = [
    PropGeometry {
        offset: [-0.0304, -0.010, -0.0304],
        axis: [0.0, 1.0, 0.0],
        handedness: -1, // M0 FL = vendor ch3 (LF), CCW
    },
    PropGeometry {
        offset: [0.0304, -0.010, -0.0304],
        axis: [0.0, 1.0, 0.0],
        handedness: 1, // M1 FR = vendor ch1 (RF), CW
    },
    PropGeometry {
        offset: [0.0304, -0.010, 0.0304],
        axis: [0.0, 1.0, 0.0],
        handedness: -1, // M2 RR = vendor ch0 (RB), CCW
    },
    PropGeometry {
        offset: [-0.0304, -0.010, 0.0304],
        axis: [0.0, 1.0, 0.0],
        handedness: 1, // M3 RL = vendor ch2 (LB), CW
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

    /// Aquila16 default rates. Pilot-tuned later via configurator
    /// equivalent. `max_rate` values are deg/s by convention; the
    /// crate's `ActualAxis::evaluate` converts to rad/s.
    pub const AQUILA16: RatesPreset = RatesPreset {
        roll: ActualAxis::new(150.0, 533.0, 0.50),
        pitch: ActualAxis::new(150.0, 533.0, 0.50),
        yaw: ActualAxis::new(150.0, 533.0, 0.50),
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
    /// error of ~10.5 rad/s (~600 deg/s) saturates the PID output at
    /// ±1.0 (mixer command). The PID now operates on rad/s errors
    /// (the FC core is SI internally), so the numbers are the
    /// previous deg/s-tuned gains multiplied through by RAD_TO_DEG
    /// ≈ 57.296 — same authority, just expressed in the new units.
    pub const RACING_5INCH: PidPreset = PidPreset {
        roll: PidGains::new(0.085_943_67, 0.458_366_24, 0.002_864_789),
        pitch: PidGains::new(0.085_943_67, 0.458_366_24, 0.002_864_789),
        yaw: PidGains::new(0.085_943_67, 0.687_549_4, 0.0),
    };

    /// Placeholder PID gains for the Aquila16. Order of magnitude
    /// scaled from the 5"-racing baseline by the inertia ratio (~35×
    /// less inertia → ~35× smaller gains for similar control authority),
    /// then back-converted for rad/s errors. Tune in SITL.
    pub const AQUILA16: PidPreset = PidPreset {
        roll: PidGains::new(0.0025, 0.013, 0.000_08),
        pitch: PidGains::new(0.0025, 0.013, 0.000_08),
        yaw: PidGains::new(0.0025, 0.020, 0.0),
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

    /// X-quad mixer matrix; must match the airframe's prop handedness
    /// pattern. The same value is forwarded into the FC core's
    /// `ControllerConfig`, so sim and firmware share one source of truth.
    pub mixer_matrix: [[f32; 4]; 4],
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

            mixer_matrix: MIXER_MATRIX_PROPS_IN,
        }
    }

    /// BetaFPV Aquila16 — the firmware's actual target airframe.
    /// Props-out handedness, 72.5 g with battery, 86 mm wheelbase,
    /// 1102 18000 KV motors on 45 mm 3-blade props. Several constants
    /// (thrust, inertia, PIDs) are educated estimates; refine in SITL
    /// and against bench data.
    pub fn aquila16() -> Self {
        Self {
            body: BodyParams::AQUILA16,
            props: AQUILA16_PROPS,
            motor: MotorParams::AQUILA16,
            thrust: ThrustParams::AQUILA16,
            battery: BatteryParams::AQUILA16_1S_1100,
            drag: DragParams::AQUILA16,
            ground_effect: GroundEffectParams::AQUILA16,
            rates: RatesPreset::AQUILA16,
            pids: PidPreset::AQUILA16,

            damage: DamageParams::default(),
            camera: CameraParams {
                tilt_deg: 20.0,
                fov_deg: 120.0,
                offset: [0.0, 0.005, 0.020],
            },

            mixer_matrix: MIXER_MATRIX_PROPS_OUT,
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

    #[test]
    fn aquila16_assembles() {
        let p = DronePresetData::aquila16();
        assert_eq!(p.body.mass_kg, 0.0725);
        assert_eq!(p.props.len(), 4);
        let cw = p.props.iter().filter(|q| q.handedness > 0).count();
        let ccw = p.props.iter().filter(|q| q.handedness < 0).count();
        assert_eq!(cw, 2);
        assert_eq!(ccw, 2);
        // Props-out: FL+RR diagonal is CCW (inverted from props-in).
        assert_eq!(p.props[0].handedness, -1); // FL CCW
        assert_eq!(p.props[2].handedness, -1); // RR CCW
        assert_eq!(p.props[1].handedness, 1); // FR CW
        assert_eq!(p.props[3].handedness, 1); // RL CW
        assert_eq!(p.mixer_matrix, MIXER_MATRIX_PROPS_OUT);
    }

    #[test]
    fn aquila16_props_below_cg() {
        // Inverted motor mount → prop disc sits below CG.
        let p = DronePresetData::aquila16();
        for prop in p.props.iter() {
            assert!(prop.offset[1] < 0.0, "prop Y offset should be negative");
        }
    }

    #[test]
    fn aquila16_hover_within_envelope() {
        // Sanity check on the educated-guess thrust/motor params.
        let p = DronePresetData::aquila16();
        let v_pack = (p.battery.cells as f32) * 3.85;
        let rpm_full = v_pack * p.motor.kv * p.motor.loading_factor;
        let t_hover = p.body.mass_kg * 9.81 / 4.0;
        let rpm_hover = 1000.0 * (t_hover / p.thrust.k_t).sqrt();
        let frac = rpm_hover / rpm_full;
        assert!(
            frac < 0.85 && frac > 0.3,
            "hover RPM frac should be 0.3..0.85, got {frac:.3}"
        );
    }
}
