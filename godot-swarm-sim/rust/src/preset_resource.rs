//! Godot Resource subclasses for `.tres` drone presets.
//!
//! These are *thin* wrappers: at scene load the gdext-side
//! `DroneController` reads the inspector-edited fields and converts
//! into a pure-Rust `DronePresetData` (in `crate::preset`) which is
//! what the simulation actually consumes. Keeping this layer thin
//! means the math is testable without instantiating Godot resources.

use godot::prelude::*;


use crate::fc::pid::PidGains;
use crate::fc::rates::{ActualAxis, ThrottleCurve};
use crate::physics::battery::BatteryParams;
use crate::physics::drag::DragParams;
use crate::physics::ground::GroundEffectParams;
use crate::physics::motor::MotorParams;
use crate::physics::thrust::ThrustParams;
use crate::preset::{
    BodyParams, CameraParams, DamageParams, DronePresetData, PidPreset, PropGeometry, RatesPreset,
};

/// Pilot stick shaping: per-axis Actual rates plus the throttle curve.
/// Lives in its own Resource so several drones can share one rates
/// preset without duplication.
#[derive(GodotClass)]
#[class(base=Resource, init, tool)]
pub struct ActualRatesPreset {
    /// Per-axis "Actual" rates: (center_sensitivity, max_rate, expo).
    /// Stored as `Vector3` for inspector convenience.
    #[export]
    #[init(val = Vector3::new(150.0, 800.0, 0.45))]
    pub roll: Vector3,
    #[export]
    #[init(val = Vector3::new(150.0, 800.0, 0.45))]
    pub pitch: Vector3,
    #[export]
    #[init(val = Vector3::new(120.0, 600.0, 0.30))]
    pub yaw: Vector3,
    /// Throttle stick midpoint: stick value at which output = 0.5.
    #[export]
    #[init(val = 0.5)]
    pub throttle_mid: f32,
    /// Throttle stick curvature around `throttle_mid`. + softens, - sharpens.
    #[export]
    #[init(val = 0.0)]
    pub throttle_expo: f32,
}

impl ActualRatesPreset {
    pub fn to_rates_preset(&self) -> RatesPreset {
        RatesPreset {
            roll: ActualAxis::new(self.roll.x, self.roll.y, self.roll.z),
            pitch: ActualAxis::new(self.pitch.x, self.pitch.y, self.pitch.z),
            yaw: ActualAxis::new(self.yaw.x, self.yaw.y, self.yaw.z),
            throttle: ThrottleCurve {
                mid: self.throttle_mid,
                expo: self.throttle_expo,
            },
        }
    }
}

/// Full per-drone preset. One `.tres` file per drone variant.
#[derive(GodotClass)]
#[class(base=Resource, init, tool)]
pub struct DronePreset {
    // ────── Body ───────────────────────────────────────────────────
    #[export]
    #[init(val = 0.6)]
    pub mass_kg: f32,
    /// Diagonal of the inertia tensor in body frame, kg·m².
    #[export]
    #[init(val = Vector3::new(0.0035, 0.005, 0.0035))]
    pub inertia_diag: Vector3,
    /// Drag coefficients (front, side, top) — body-frame X / Y / Z.
    #[export]
    #[init(val = Vector3::new(1.0, 1.3, 1.0))]
    pub cd_per_axis: Vector3,
    /// Reference areas, m². Same axis order.
    #[export]
    #[init(val = Vector3::new(0.012, 0.05, 0.025))]
    pub area_per_axis: Vector3,
    /// Air density, kg/m³. 1.225 at sea level.
    #[export]
    #[init(val = 1.225)]
    pub air_density: f32,

    // ────── Props (X-quad layout, 4 props) ─────────────────────────
    /// Body-frame mount position for each of the 4 props (M0..M3).
    #[export]
    pub prop_offset_m0: Vector3,
    #[export]
    pub prop_offset_m1: Vector3,
    #[export]
    pub prop_offset_m2: Vector3,
    #[export]
    pub prop_offset_m3: Vector3,
    /// Spin axis of each prop (unit vector, body-frame). Default +Y.
    #[export]
    #[init(val = Vector3::UP)]
    pub prop_axis: Vector3,
    /// Handedness: +1 = CW, -1 = CCW. M0/M2 default CW; M1/M3 default CCW.
    #[export]
    #[init(val = 1)]
    pub prop_handedness_m0: i32,
    #[export]
    #[init(val = -1)]
    pub prop_handedness_m1: i32,
    #[export]
    #[init(val = 1)]
    pub prop_handedness_m2: i32,
    #[export]
    #[init(val = -1)]
    pub prop_handedness_m3: i32,

    // ────── Powertrain ─────────────────────────────────────────────
    #[export]
    #[init(val = 2400.0)]
    pub motor_kv: f32,
    #[export]
    #[init(val = 0.85)]
    pub motor_loading_factor: f32,
    #[export]
    #[init(val = 0.030)]
    pub motor_spoolup_tau_s: f32,
    #[export]
    #[init(val = 0.05)]
    pub motor_idle_throttle_floor: f32,
    /// T (N) = thrust_k_t * (rpm/1000)^2.
    #[export]
    #[init(val = 0.0052)]
    pub thrust_k_t: f32,
    /// Counter-torque coefficient: τ_react = thrust_k_q · T.
    #[export]
    #[init(val = 0.014)]
    pub thrust_k_q: f32,

    // ────── Battery ────────────────────────────────────────────────
    #[export]
    #[init(val = 4)]
    pub battery_cells: i32,
    #[export]
    #[init(val = 8.0)]
    pub battery_internal_r_mohm_per_cell: f32,
    #[export]
    #[init(val = 1500.0)]
    pub battery_capacity_mah: f32,
    #[export]
    #[init(val = 3.3)]
    pub battery_cutoff_v_per_cell: f32,
    #[export]
    #[init(val = 0.2)]
    pub battery_cutoff_band_v: f32,

    // ────── Ground effect ──────────────────────────────────────────
    #[export]
    #[init(val = 1.4)]
    pub ground_effect_max: f32,
    #[export]
    #[init(val = 0.30)]
    pub ground_effect_distance_m: f32,

    // ────── FC PIDs ────────────────────────────────────────────────
    /// Roll PID (P, I, D).
    #[export]
    #[init(val = Vector3::new(0.0040, 0.020, 0.00012))]
    pub pid_roll: Vector3,
    #[export]
    #[init(val = Vector3::new(0.0040, 0.020, 0.00012))]
    pub pid_pitch: Vector3,
    #[export]
    #[init(val = Vector3::new(0.0035, 0.030, 0.0))]
    pub pid_yaw: Vector3,

    /// Pilot stick shaping. Optional — if null, racing defaults are used.
    #[export]
    pub rates: Option<Gd<ActualRatesPreset>>,

    // ────── Arming / safety ────────────────────────────────────────
    #[export]
    #[init(val = 5)]
    pub arm_aux_channel: i32,
    #[export]
    #[init(val = 1500)]
    pub arm_threshold_us: i32,
    #[export]
    #[init(val = 1100)]
    pub arm_throttle_max_us: i32,

    // ────── FPV camera ─────────────────────────────────────────────
    #[export]
    #[init(val = 30.0)]
    pub fpv_camera_tilt_deg: f32,
    #[export]
    #[init(val = 120.0)]
    pub fpv_camera_fov_deg: f32,
    #[export]
    #[init(val = Vector3::new(0.0, 0.02, 0.04))]
    pub fpv_camera_offset: Vector3,

    // ────── Damage / respawn ───────────────────────────────────────
    #[export]
    #[init(val = 8.0)]
    pub crash_impulse_threshold_n_s: f32,
    #[export]
    #[init(val = 2.0)]
    pub respawn_cooldown_s: f32,
}

impl DronePreset {
    /// Convert this Godot Resource into the pure-Rust `DronePresetData`
    /// the simulation consumes. Falls back to racing-5" defaults for
    /// any sub-resource that's null or zero (so a freshly-instantiated
    /// `DronePreset` produces a working drone).
    pub fn to_data(&self) -> DronePresetData {
        let mut data = DronePresetData::racing_5inch();

        // Body
        data.body = BodyParams {
            mass_kg: self.mass_kg,
            inertia_diag: vec3_to_arr(self.inertia_diag),
        };
        data.drag = DragParams {
            cd_per_axis: vec3_to_arr(self.cd_per_axis),
            area_per_axis: vec3_to_arr(self.area_per_axis),
            air_density: self.air_density,
        };

        // Props — only override if any prop_offset is set; otherwise keep
        // the racing-5" defaults.
        let any_offset_set = !self.prop_offset_m0.length_squared().eq(&0.0)
            || !self.prop_offset_m1.length_squared().eq(&0.0)
            || !self.prop_offset_m2.length_squared().eq(&0.0)
            || !self.prop_offset_m3.length_squared().eq(&0.0);
        if any_offset_set {
            let axis = if self.prop_axis.length_squared() > 1e-6 {
                vec3_to_arr(self.prop_axis.normalized())
            } else {
                [0.0, 1.0, 0.0]
            };
            data.props = [
                PropGeometry {
                    offset: vec3_to_arr(self.prop_offset_m0),
                    axis,
                    handedness: self.prop_handedness_m0 as i8,
                },
                PropGeometry {
                    offset: vec3_to_arr(self.prop_offset_m1),
                    axis,
                    handedness: self.prop_handedness_m1 as i8,
                },
                PropGeometry {
                    offset: vec3_to_arr(self.prop_offset_m2),
                    axis,
                    handedness: self.prop_handedness_m2 as i8,
                },
                PropGeometry {
                    offset: vec3_to_arr(self.prop_offset_m3),
                    axis,
                    handedness: self.prop_handedness_m3 as i8,
                },
            ];
        }

        // Powertrain
        data.motor = MotorParams {
            kv: self.motor_kv,
            loading_factor: self.motor_loading_factor,
            spoolup_tau_s: self.motor_spoolup_tau_s,
            idle_throttle_floor: self.motor_idle_throttle_floor,
        };
        data.thrust = ThrustParams {
            k_t: self.thrust_k_t,
            k_q: self.thrust_k_q,
        };
        data.battery = BatteryParams {
            cells: self.battery_cells.clamp(1, 12) as u8,
            internal_r_mohm_per_cell: self.battery_internal_r_mohm_per_cell,
            capacity_mah: self.battery_capacity_mah,
            cutoff_v_per_cell: self.battery_cutoff_v_per_cell,
            cutoff_band_v: self.battery_cutoff_band_v,
        };
        data.ground_effect = GroundEffectParams {
            max_multiplier: self.ground_effect_max,
            fade_distance_m: self.ground_effect_distance_m,
        };

        // FC PIDs
        data.pids = PidPreset {
            roll: vec3_to_pid(self.pid_roll),
            pitch: vec3_to_pid(self.pid_pitch),
            yaw: vec3_to_pid(self.pid_yaw),
        };

        // Rates
        if let Some(rates) = &self.rates {
            data.rates = rates.bind().to_rates_preset();
        }

        // Camera
        data.camera = CameraParams {
            tilt_deg: self.fpv_camera_tilt_deg,
            fov_deg: self.fpv_camera_fov_deg,
            offset: vec3_to_arr(self.fpv_camera_offset),
        };

        // Damage / respawn
        data.damage = DamageParams {
            crash_impulse_threshold_n_s: self.crash_impulse_threshold_n_s,
            respawn_cooldown_s: self.respawn_cooldown_s,
        };

        data
    }
}

fn vec3_to_arr(v: Vector3) -> [f32; 3] {
    [v.x, v.y, v.z]
}

fn vec3_to_pid(v: Vector3) -> PidGains {
    PidGains::new(v.x, v.y, v.z)
}
