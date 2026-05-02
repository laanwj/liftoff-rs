//! `DroneController` — gdext class extending `RigidBody3D`. Owns a
//! `DroneSim`, polls input from `CrsfIo` interface children, gathers
//! truth from the body, runs the sim step, and applies the resulting
//! forces / torques back to the body.
//!
//! CRSF I/O is provided by child nodes implementing `CrsfIo` (e.g.
//! `ZenohIOInterface`, `IOInterface`). The controller discovers them
//! at ready-time and polls/publishes through the trait interface.

use std::time::Instant;

use godot::classes::{Camera3D, IRigidBody3D, PhysicsRayQueryParameters3D, RigidBody3D};
use godot::prelude::*;

use telemetry_lib::crsf::{self, CrsfPacket};

use crate::crsf_io::{self, WorldState};
use crate::crsf_io_trait::{CrsfIo, RcFrame, RC_STREAM_AUTOPILOT, RC_STREAM_DIRECT};
use crate::fc::mode::BodyTruth;
use crate::input_router::{self, RcSample, RouterConfig, RouterInputs};
use crate::godot_input_interface::GodotInputInterface;
use crate::pipeline::{DroneSim, TickInput};
use crate::preset::DronePresetData;
use crate::preset_resource::DronePreset;
use crate::rc_input::{self, ChannelMap, RcInput};
use crate::wake_node::WakeFieldNode;
use crate::zenoh_interface::ZenohIOInterface;

const RAD_TO_DEG: f32 = 57.295_78_f32;

/// Where this tick's RC frame came from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum RcSourceTag {
    Autopilot,
    Direct,
    None,
}

/// Wrapper holding a reference to a CrsfIo implementation obtained
/// from a child node. We store an enum because gdext doesn't allow
/// trait objects across the GodotClass boundary directly.
enum IoHandle {
    Zenoh(Gd<ZenohIOInterface>),
    Local(Gd<GodotInputInterface>),
}

impl IoHandle {
    fn with_io<R>(&self, f: impl FnOnce(&dyn CrsfIo) -> R) -> Option<R> {
        match self {
            IoHandle::Zenoh(gd) => {
                let bound = gd.bind();
                bound.crsf_io().map(f)
            }
            IoHandle::Local(gd) => {
                let bound = gd.bind();
                bound.crsf_io().map(f)
            }
        }
    }
}

#[derive(GodotClass)]
#[class(base=RigidBody3D)]
pub struct DroneController {
    base: Base<RigidBody3D>,

    /// Inspector-editable preset resource. If null at ready-time we
    /// fall back to the racing 5" defaults.
    #[export]
    preset: Option<Gd<DronePreset>>,

    /// Optional path to the FPV `Camera3D` child. If set, its tilt and
    /// FOV are configured from the preset at ready-time.
    #[export]
    fpv_camera_path: NodePath,

    /// Drone ID. Informational only (topic routing is configured on
    /// interface nodes directly).
    #[export]
    drone_id: i64,

    /// Path to the sibling WakeFieldNode. If set and the node exists,
    /// the drone contributes to and samples from the shared wake field
    /// each tick.
    #[export]
    wake_field_path: NodePath,

    /// Cached reference, resolved once in `ready()`.
    wake_field_ref: Option<Gd<WakeFieldNode>>,

    /// Spawn pose, captured at `ready()`, used by `respawn()`.
    spawn_transform: Transform3D,

    /// Pure-Rust simulation instance.
    sim: Option<DroneSim>,

    /// CRSF I/O interfaces discovered from child nodes.
    interfaces: Vec<IoHandle>,

    /// Telemetry publish rate (seconds between publishes).
    telemetry_period_s: f32,
    /// Accumulator for telemetry rate limiting.
    telemetry_timer: f32,
    /// Counter for link-stats emission (~1 Hz).
    telemetry_publish_count: u64,

    /// Optional autoquit timer for headless / CI runs (env GSS_AUTOQUIT_S).
    autoquit_after: Option<f64>,
    elapsed_s: f64,

    /// Print a status line every 1 second of sim time.
    last_logged_second: u64,

    /// Last-tick linear velocity magnitude (for crash impulse estimation).
    prev_speed: f32,

    /// Accumulated collision events this tick, cleared after being fed
    /// into the sim.
    pending_collisions: Vec<crate::damage::PropCollision>,

    /// Timer for respawn cooldown after destruction.
    respawn_timer: f32,
}

#[godot_api]
impl IRigidBody3D for DroneController {
    fn init(base: Base<RigidBody3D>) -> Self {
        Self {
            base,
            preset: None,
            fpv_camera_path: NodePath::default(),
            drone_id: 0,
            wake_field_path: NodePath::from("../WakeField"),
            wake_field_ref: None,
            spawn_transform: Transform3D::IDENTITY,
            sim: None,
            interfaces: Vec::new(),
            telemetry_period_s: 0.050, // 20 Hz
            telemetry_timer: 0.0,
            telemetry_publish_count: 0,
            autoquit_after: None,
            elapsed_s: 0.0,
            last_logged_second: 0,
            prev_speed: 0.0,
            pending_collisions: Vec::new(),
            respawn_timer: 0.0,
        }
    }

    fn ready(&mut self) {
        // 1. Load preset → pure-Rust data.
        let preset_data = match self.preset.as_ref() {
            Some(p) => p.bind().to_data(),
            None => DronePresetData::racing_5inch(),
        };

        // 2. Configure the rigid body from the preset.
        self.base_mut().set_mass(preset_data.body.mass_kg);
        let inertia = Vector3::new(
            preset_data.body.inertia_diag[0],
            preset_data.body.inertia_diag[1],
            preset_data.body.inertia_diag[2],
        );
        self.base_mut().set_inertia(inertia);
        self.base_mut().set_linear_damp(0.0);
        self.base_mut().set_angular_damp(0.05);

        // Enable contact monitoring for crash impulse detection.
        self.base_mut().set_contact_monitor(true);
        self.base_mut().set_max_contacts_reported(8);

        // 3. Configure FPV camera from preset.
        if !self.fpv_camera_path.is_empty()
            && let Some(mut cam) = self.try_get_node_as_camera(&self.fpv_camera_path.clone())
        {
            self.apply_camera_preset(&mut cam, &preset_data);
        }

        // 4. Capture spawn pose for respawn.
        self.spawn_transform = self.base().get_global_transform();

        // 5. Build the sim.
        self.sim = Some(DroneSim::new(preset_data));

        // 5b. Resolve the shared WakeFieldNode.
        if !self.wake_field_path.is_empty() {
            let path = self.wake_field_path.clone();
            self.wake_field_ref = self.base().try_get_node_as::<WakeFieldNode>(&path);
        }

        // 6. Discover CrsfIo interface children.
        self.discover_interfaces();

        // 7. Optional autoquit (used by headless smoke tests).
        if let Ok(s) = std::env::var("GSS_AUTOQUIT_S")
            && let Ok(secs) = s.parse::<f64>()
        {
            self.autoquit_after = Some(secs);
            godot_print!("DroneController autoquit after {:.3}s (GSS_AUTOQUIT_S)", secs);
        }

        let configured_hz = godot::classes::Engine::singleton().get_physics_ticks_per_second();
        godot_print!(
            "DroneController ready — physics rate {} Hz, mass {} kg, drone_id {}, {} interface(s)",
            configured_hz,
            self.base().get_mass(),
            self.drone_id,
            self.interfaces.len(),
        );
    }

    fn physics_process(&mut self, dt: f64) {
        self.elapsed_s += dt;
        let dt_f = dt as f32;

        // 1. Input: poll all interfaces, feed through the router.
        let channel_map = ChannelMap::aetra_default();
        let (rc, source) = self.acquire_rc(&channel_map);
        let _ = source; // logged below

        // 2. Gather truth from the rigid body.
        let world_xform = self.base().get_global_transform();
        let body_basis = world_xform.basis;
        let world_lin_vel = self.base().get_linear_velocity();
        let world_ang_vel = self.base().get_angular_velocity();
        let body_basis_inv = body_basis.inverse();
        let body_lin_vel = body_basis_inv * world_lin_vel;
        let body_ang_vel = body_basis_inv * world_ang_vel;

        // Map Godot body-frame angular velocity into drone-FC convention.
        let truth = BodyTruth {
            gyro_dps: [
                -body_ang_vel.z * RAD_TO_DEG,
                body_ang_vel.x * RAD_TO_DEG,
                -body_ang_vel.y * RAD_TO_DEG,
            ],
            attitude_deg: [0.0; 3],
        };

        // 3. Collision-damage detection.
        let current_speed = world_lin_vel.length();
        if self.base().get_colliding_bodies().len() > 0 {
            let delta_v = (self.prev_speed - current_speed).max(0.0);
            let mass = self.base().get_mass();
            let impulse_ns = delta_v * mass;
            if impulse_ns > 0.1 {
                let vel_dir_body = body_basis_inv * world_lin_vel;
                let nearest_prop = self.nearest_prop_to_direction(vel_dir_body);
                self.pending_collisions.push(crate::damage::PropCollision {
                    prop_index: nearest_prop,
                    impulse_ns,
                });
            }
        }
        self.prev_speed = current_speed;

        // 3b. Handle respawn timer if destroyed.
        if let Some(sim) = self.sim.as_ref() {
            if sim.is_destroyed() {
                self.respawn_timer += dt_f;
                if self.respawn_timer >= sim.damage.params.respawn_cooldown_s {
                    self.respawn_timer = 0.0;
                    self.respawn();
                    return;
                }
            }
        }

        // 3c. Per-prop AGL raycast.
        let agl = self.compute_agl_per_prop();

        // 4. Build TickInput.
        let mut input = TickInput::default();
        input.rc = rc;
        input.truth = truth;
        input.v_body = [body_lin_vel.x, body_lin_vel.y, body_lin_vel.z];
        input.agl_m = agl;

        // 4b. Compute world-frame prop positions/axes for wake field.
        if let Some(sim) = self.sim.as_ref() {
            let body_origin = world_xform.origin;
            let mut wpos = [[0.0_f32; 3]; 4];
            let mut waxis = [[0.0_f32; 3]; 4];
            for (i, p) in sim.preset.props.iter().enumerate() {
                let off = Vector3::new(p.offset[0], p.offset[1], p.offset[2]);
                let ax = Vector3::new(p.axis[0], p.axis[1], p.axis[2]);
                let wp = body_origin + body_basis * off;
                let wa = (body_basis * ax).normalized();
                wpos[i] = [wp.x, wp.y, wp.z];
                waxis[i] = [wa.x, wa.y, wa.z];
            }
            input.prop_world_pos = Some(wpos);
            input.prop_world_axis = Some(waxis);
        }

        // 5. Feed collision events + step sim.
        let Some(sim) = self.sim.as_mut() else {
            return;
        };
        if !self.pending_collisions.is_empty() {
            let just_destroyed = sim.apply_collisions(&self.pending_collisions);
            self.pending_collisions.clear();
            if just_destroyed {
                godot_print!("DroneController: drone destroyed!");
            }
        }
        let out = if let Some(ref mut wf_gd) = self.wake_field_ref {
            let mut wf = wf_gd.bind_mut();
            sim.step_with_wake(&input, dt_f, Some(wf.field_mut()))
        } else {
            sim.step(&input, dt_f)
        };

        // 6. Apply per-prop force at offset.
        for prop in &out.props {
            let f_body = Vector3::new(prop.force_body[0], prop.force_body[1], prop.force_body[2]);
            let off_body = Vector3::new(
                prop.offset_body[0],
                prop.offset_body[1],
                prop.offset_body[2],
            );
            let tau_body = Vector3::new(
                prop.reaction_torque_body[0],
                prop.reaction_torque_body[1],
                prop.reaction_torque_body[2],
            );
            let f_world = body_basis * f_body;
            let off_world = body_basis * off_body;
            let tau_world = body_basis * tau_body;

            self.base_mut()
                .apply_force_ex(f_world)
                .position(off_world)
                .done();
            self.base_mut().apply_torque(tau_world);
        }

        // 7. Apply drag at the centre of mass.
        let drag_body = Vector3::new(
            out.drag_force_body[0],
            out.drag_force_body[1],
            out.drag_force_body[2],
        );
        let drag_world = body_basis * drag_body;
        self.base_mut().apply_central_force(drag_world);

        // 8. Telemetry: rate-limited publish to all interfaces.
        self.telemetry_timer += dt_f;
        if self.telemetry_timer >= self.telemetry_period_s {
            self.telemetry_timer -= self.telemetry_period_s;
            self.telemetry_publish_count += 1;

            let pos = self.base().get_global_position();
            let world_quat = world_xform.basis.get_quaternion();
            let world = WorldState {
                position: [pos.x, pos.y, pos.z],
                attitude: [world_quat.x, world_quat.y, world_quat.z, world_quat.w],
                velocity: [world_lin_vel.x, world_lin_vel.y, world_lin_vel.z],
                timestamp_s: self.elapsed_s as f32,
            };
            let total_thrust: f32 = out.props.iter().map(|p| p.thrust_n).sum();
            let current_a = (total_thrust * 0.45).max(0.0);
            let cells = self
                .sim
                .as_ref()
                .map(|s| s.preset.battery.cells)
                .unwrap_or(4);
            let timestamp_ms = (self.elapsed_s * 1000.0) as u64;
            let prop_damage = self.sim.as_ref().map(|s| s.prop_damage()).unwrap_or([0.0; 4]);
            let destroyed = self.sim.as_ref().map(|s| s.is_destroyed()).unwrap_or(false);

            let frames = crsf_io::build_crsf_frames(&world, &out, current_a, cells, timestamp_ms);
            for f in &frames {
                self.broadcast_telemetry(f);
            }
            if let Some(dmg) = crsf_io::build_damage_frame(&prop_damage, destroyed, timestamp_ms) {
                self.broadcast_telemetry(&dmg);
            }
            // Link stats at ~1 Hz.
            let ticks_per_sec = (1.0 / self.telemetry_period_s) as u64;
            if ticks_per_sec > 0 && (self.telemetry_publish_count % ticks_per_sec) == 0 {
                let ls = crsf_io::build_link_stats_frame();
                self.broadcast_telemetry(&ls);
            }
        }

        // 9. Reset key drives respawn.
        if rc.reset {
            self.respawn();
        }

        // 10. Log + autoquit.
        let whole = self.elapsed_s as u64;
        if whole > self.last_logged_second {
            self.last_logged_second = whole;
            let pos = self.base().get_global_position();
            godot_print!(
                "[t={:.3}s] src={:?} armed={} thr={:.2} alt={:.2}m vbat={:.2}V cmds=[{:.2} {:.2} {:.2} {:.2}] rpm=[{:.0} {:.0} {:.0} {:.0}] pid=[{:.3} {:.3} {:.3}] gyro=[{:.0} {:.0} {:.0}]",
                self.elapsed_s,
                source,
                out.armed,
                input.rc.throttle,
                pos.y,
                out.battery.v_pack_terminal,
                out.motor_commands[0], out.motor_commands[1], out.motor_commands[2], out.motor_commands[3],
                out.motor_states[0].rpm, out.motor_states[1].rpm, out.motor_states[2].rpm, out.motor_states[3].rpm,
                out.pid_output[0], out.pid_output[1], out.pid_output[2],
                input.truth.gyro_dps[0], input.truth.gyro_dps[1], input.truth.gyro_dps[2]
            );
        }

        if let Some(deadline) = self.autoquit_after
            && self.elapsed_s >= deadline
        {
            godot_print!("DroneController autoquit at t={:.3}s", self.elapsed_s);
            self.base().get_tree().quit();
            self.autoquit_after = None;
        }
    }
}

#[godot_api]
impl DroneController {
    /// Snap the drone back to its spawn pose with zero velocity and
    /// fresh battery / damage state.
    #[func]
    pub fn respawn(&mut self) {
        let xform = self.spawn_transform;
        self.base_mut().set_linear_velocity(Vector3::ZERO);
        self.base_mut().set_angular_velocity(Vector3::ZERO);
        self.base_mut().set_global_transform(xform);
        if let Some(sim) = self.sim.as_mut() {
            sim.reset();
        }
        godot_print!("DroneController respawned");
    }

    /// Live-tune FPV camera tilt (degrees).
    #[func]
    pub fn set_camera_tilt_deg(&mut self, deg: f32) {
        let path = self.fpv_camera_path.clone();
        if let Some(mut cam) = self.try_get_node_as_camera(&path) {
            apply_camera_tilt(&mut cam, deg);
        }
    }

    /// Live-tune FPV camera FOV (degrees).
    #[func]
    pub fn set_camera_fov_deg(&mut self, deg: f32) {
        let path = self.fpv_camera_path.clone();
        if let Some(mut cam) = self.try_get_node_as_camera(&path) {
            cam.set_fov(deg);
        }
    }

    /// True iff the FC currently considers the drone armed.
    #[func]
    pub fn is_armed(&self) -> bool {
        self.sim.as_ref().map(|s| s.armed()).unwrap_or(false)
    }
}

impl DroneController {
    /// Discover child nodes that implement CrsfIo.
    fn discover_interfaces(&mut self) {
        let children = self.base().get_children();
        for i in 0..children.len() {
            let Some(child) = children.get(i) else {
                continue;
            };
            if let Ok(zenoh_node) = child.clone().try_cast::<ZenohIOInterface>() {
                self.interfaces.push(IoHandle::Zenoh(zenoh_node));
            } else if let Ok(io_node) = child.try_cast::<GodotInputInterface>() {
                self.interfaces.push(IoHandle::Local(io_node));
            }
        }
    }

    /// Poll all interfaces for RC frames, route through the mux, and
    /// return the decoded input. Also extracts the reset flag from the
    /// winning frame.
    fn acquire_rc(&self, channel_map: &ChannelMap) -> (RcInput, RcSourceTag) {
        let now = Instant::now();
        let mut best_direct: Option<RcSample> = None;
        let mut best_autopilot: Option<RcSample> = None;

        for handle in &self.interfaces {
            let frames = handle.with_io(|io| io.poll_rc());
            let Some(frames) = frames else { continue };
            for frame in frames {
                let channels = match parse_rc_channels(&frame) {
                    Some(ch) => ch,
                    None => continue,
                };
                let sample = RcSample::new(channels, frame.received_at);
                match frame.stream_id {
                    RC_STREAM_DIRECT => {
                        replace_if_newer(&mut best_direct, sample);
                    }
                    RC_STREAM_AUTOPILOT => {
                        replace_if_newer(&mut best_autopilot, sample);
                    }
                    _ => {}
                }
            }
        }

        let inputs = RouterInputs {
            direct: best_direct,
            autopilot: best_autopilot,
        };
        let cfg = RouterConfig::default();

        if let Some(out) = input_router::select(&inputs, &cfg, now) {
            let decoded = rc_input::decode_rc(&out.channels, channel_map);
            let tag = match out.source {
                input_router::Selected::Autopilot => RcSourceTag::Autopilot,
                input_router::Selected::Direct => RcSourceTag::Direct,
            };
            return (decoded, tag);
        }

        (RcInput::default(), RcSourceTag::None)
    }

    /// Send a telemetry frame to all interfaces.
    fn broadcast_telemetry(&self, data: &[u8]) {
        for handle in &self.interfaces {
            handle.with_io(|io| io.send_telemetry(data));
        }
    }

    /// Heuristic: map a velocity-direction vector to the prop closest
    /// to the "leading edge" of the impact.
    fn nearest_prop_to_direction(&self, vel_body: Vector3) -> u8 {
        let Some(sim) = self.sim.as_ref() else {
            return 0;
        };
        let mut best = 0u8;
        let mut best_dot = f32::NEG_INFINITY;
        for (i, p) in sim.preset.props.iter().enumerate() {
            let off = Vector3::new(p.offset[0], p.offset[1], p.offset[2]);
            let d = off.dot(vel_body);
            if d > best_dot {
                best_dot = d;
                best = i as u8;
            }
        }
        best
    }

    fn try_get_node_as_camera(&self, path: &NodePath) -> Option<Gd<Camera3D>> {
        if path.is_empty() {
            return None;
        }
        self.base().try_get_node_as::<Camera3D>(path)
    }

    fn apply_camera_preset(&self, cam: &mut Gd<Camera3D>, data: &DronePresetData) {
        cam.set_fov(data.camera.fov_deg);
        let off = Vector3::new(
            data.camera.offset[0],
            data.camera.offset[1],
            data.camera.offset[2],
        );
        let mut xform = cam.get_transform();
        xform.origin = off;
        cam.set_transform(xform);
        apply_camera_tilt(cam, data.camera.tilt_deg);
    }

    fn compute_agl_per_prop(&self) -> [Option<f32>; 4] {
        let Some(sim) = self.sim.as_ref() else {
            return [None; 4];
        };
        let world_xform = self.base().get_global_transform();
        let body_origin = world_xform.origin;
        let body_basis = world_xform.basis;
        let max_distance = sim.preset.ground_effect.fade_distance_m * 2.0;
        let down = Vector3::new(0.0, -1.0, 0.0);
        let self_rid = self.base().get_rid();

        let Some(world) = self.base().get_world_3d() else {
            return [None; 4];
        };
        let Some(mut space_state) = world.get_direct_space_state() else {
            return [None; 4];
        };

        let mut out = [None; 4];
        for (i, prop) in sim.preset.props.iter().enumerate() {
            let body_off = Vector3::new(prop.offset[0], prop.offset[1], prop.offset[2]);
            let from = body_origin + body_basis * body_off;
            let to = from + down * max_distance;
            let mut params = PhysicsRayQueryParameters3D::create(from, to)
                .expect("failed to create ray query");
            let mut excludes = Array::new();
            excludes.push(self_rid);
            params.set_exclude(&excludes);
            let result = space_state.intersect_ray(&params);
            if !result.is_empty()
                && let Some(pos) = result.get("position")
            {
                if let Ok(hit) = pos.try_to::<Vector3>() {
                    let dist = (from - hit).length();
                    out[i] = Some(dist);
                }
            }
        }
        out
    }
}

/// Parse an RcFrame's raw CRSF bytes into 16 channel values.
fn parse_rc_channels(frame: &RcFrame) -> Option<[u16; 16]> {
    match crsf::parse_packet_check(&frame.data) {
        Some(CrsfPacket::RcChannelsPacked(c)) => Some(c.channels),
        _ => None,
    }
}

/// Replace `slot` with `sample` if sample is newer (or slot is empty).
fn replace_if_newer(slot: &mut Option<RcSample>, sample: RcSample) {
    match slot {
        Some(existing) if existing.received_at >= sample.received_at => {}
        _ => *slot = Some(sample),
    }
}

fn apply_camera_tilt(cam: &mut Gd<Camera3D>, deg: f32) {
    let mut xform = cam.get_transform();
    let rad = deg.to_radians();
    xform.basis = Basis::from_euler(
        godot::builtin::EulerOrder::XYZ,
        Vector3::new(rad, 0.0, 0.0),
    );
    cam.set_transform(xform);
}
