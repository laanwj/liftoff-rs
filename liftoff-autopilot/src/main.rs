use clap::Parser;
use liftoff_lib::crsf::{self, CrsfPacket};
use liftoff_lib::geo;
use liftoff_lib::topics;
use log::{debug, info, warn};
use nalgebra::{UnitQuaternion, Vector3};
use serde::Deserialize;
use std::f64::consts::PI;
use std::sync::Arc;
use tokio::sync::Mutex;
use tokio::time::{Duration, Instant, interval};
use zenoh::Config;

/// Shortest signed angle from `from` to `to`, wrapped to [-π, π].
fn angle_diff(to: f64, from: f64) -> f64 {
    ((to - from) + PI).rem_euclid(2.0 * PI) - PI
}

#[derive(Debug, Clone, Deserialize)]
struct Waypoint {
    lat: f64,
    lon: f64,
    alt: f64,
    /// Target heading in degrees (0 = North, 90 = East). If omitted, holds current yaw.
    orientation: Option<f64>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(rename_all = "snake_case")]
enum EndBehavior {
    Hover,
    Loop,
    Return,
}

impl Default for EndBehavior {
    fn default() -> Self {
        EndBehavior::Hover
    }
}

fn default_hold_time() -> f64 {
    5.0
}

fn default_arrival_radius() -> f64 {
    2.0
}

/// Convert GPS waypoints to local frame positions and yaw targets.
fn convert_waypoints(
    waypoints: &[Waypoint],
    gps_origin: (f64, f64), // (lat, lon)
    alt_origin: f64,
) -> (Vec<Vector3<f64>>, Vec<Option<f64>>) {
    let mut positions = Vec::with_capacity(waypoints.len());
    let mut yaws = Vec::with_capacity(waypoints.len());
    for (i, wp) in waypoints.iter().enumerate() {
        let local = geo::coord_from_gps(
            (wp.lon, wp.lat, wp.alt),
            (gps_origin.1, gps_origin.0),
        );
        let relative_alt = wp.alt - alt_origin;
        let local_pos = Vector3::new(local[0], relative_alt, local[2]);
        let yaw = wp.orientation.map(|deg| deg.to_radians());
        info!(
            "Waypoint {}: lat={:.6} lon={:.6} alt={:.1} yaw={} -> local E={:.1} Alt={:.1} N={:.1}",
            i, wp.lat, wp.lon, wp.alt,
            match yaw { Some(y) => format!("{:.1}deg", y.to_degrees()), None => "-".into() },
            local_pos[0], local_pos[1], local_pos[2]
        );
        positions.push(local_pos);
        yaws.push(yaw);
    }
    (positions, yaws)
}

#[derive(Debug, Clone, Deserialize)]
struct WaypointConfig {
    waypoints: Vec<Waypoint>,
    #[serde(default = "default_hold_time")]
    hold_time: f64,
    #[serde(default = "default_arrival_radius")]
    arrival_radius: f64,
    #[serde(default)]
    end_behavior: EndBehavior,
}

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Target Altitude (meters).
    #[arg(long, default_value_t = 10.0)]
    target_alt: f64,

    /// Path to waypoints JSON file.
    #[arg(long)]
    waypoints: Option<std::path::PathBuf>,

    /// Zenoh connect endpoint (e.g. tcp/192.168.1.1:7447). Omit for peer discovery.
    #[arg(long)]
    zenoh_connect: Option<String>,

    /// Zenoh mode (peer or client).
    #[arg(long, default_value = "peer")]
    zenoh_mode: String,

    /// Zenoh topic prefix.
    #[arg(long, default_value = topics::DEFAULT_PREFIX)]
    zenoh_prefix: String,
}

// PID Controller
struct Pid {
    kp: f64,
    ki: f64,
    kd: f64,
    integral: f64,
    prev_error: f64,
    output_limit: f64,
    integral_limit: f64,
}

impl Pid {
    fn new(kp: f64, ki: f64, kd: f64, output_limit: f64, integral_limit: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: 0.0,
            prev_error: 0.0,
            output_limit,
            integral_limit,
        }
    }

    fn update(&mut self, error: f64, dt: f64) -> f64 {
        self.integral += error * dt;
        self.integral = self
            .integral
            .clamp(-self.integral_limit, self.integral_limit);

        let derivative = (error - self.prev_error) / dt;
        self.prev_error = error;

        let output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative);
        output.clamp(-self.output_limit, self.output_limit)
    }

    fn reset(&mut self) {
        self.integral = 0.0;
        self.prev_error = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_pid_p() {
        let mut pid = Pid::new(1.0, 0.0, 0.0, 10.0, 10.0);
        assert_eq!(pid.update(1.0, 1.0), 1.0);
        assert_eq!(pid.update(-1.0, 1.0), -1.0);
        assert_eq!(pid.update(100.0, 1.0), 10.0); // Limit
    }

    #[test]
    fn test_pid_i() {
        let mut pid = Pid::new(0.0, 1.0, 0.0, 10.0, 10.0);
        assert_eq!(pid.update(1.0, 1.0), 1.0); // Int = 1
        assert_eq!(pid.update(1.0, 1.0), 2.0); // Int = 2
    }

    #[test]
    fn test_pid_d() {
        let mut pid = Pid::new(0.0, 0.0, 1.0, 10.0, 10.0);
        assert_eq!(pid.update(1.0, 1.0), 1.0); // err=1, prev=0, d=1
        assert_eq!(pid.update(1.0, 1.0), 0.0); // err=1, prev=1, d=0
    }
}

// State
#[derive(Debug, Clone)]
struct DroneState {
    position: Option<Vector3<f64>>, // x (East), y (Alt), z (North)
    velocity: Option<Vector3<f64>>,
    attitude: Option<UnitQuaternion<f64>>,
    yaw: f64,
    gps_origin: Option<(f64, f64)>, // Lat, Lon
    alt_origin: f64,
    prev_position: Option<Vector3<f64>>,
    prev_pos_time: Option<Instant>,
}

impl Default for DroneState {
    fn default() -> Self {
        Self {
            position: None,
            velocity: None,
            attitude: None,
            yaw: 0.0,
            gps_origin: None,
            alt_origin: 0.0,
            prev_position: None,
            prev_pos_time: None,
        }
    }
}

struct Controller {
    // Altitude Loop
    alt_pid: Pid,

    // Position Loop
    pos_x_pid: Pid,
    pos_y_pid: Pid,

    // Attitude Loop
    roll_pid: Pid,
    pitch_pid: Pid,
    yaw_pid: Pid,

    target_pos: Vector3<f64>,
    target_yaw: f64,

    last_update: Instant,
    armed: bool,

    // Waypoint navigation
    waypoints_local: Vec<Vector3<f64>>,
    waypoint_yaws: Vec<Option<f64>>, // target yaw per waypoint (radians), None = keep current
    current_waypoint: usize,
    arrival_time: Option<Instant>,
    hold_time: Duration,
    arrival_radius: f64,
    end_behavior: EndBehavior,
    route_complete: bool,
}

impl Controller {
    fn new(target_alt: f64) -> Self {
        Self {
            alt_pid: Pid::new(0.15, 0.1, 0.0, 0.35, 0.3), // Output correction around base throttle (max ~0.80)

            // Position -> Angle (direct, no velocity cascade)
            // Velocity is derived from position differences and used only as damping.
            pos_x_pid: Pid::new(0.05, 0.0, 0.0, 0.3, 0.0), // Max ~17 deg lean
            pos_y_pid: Pid::new(0.05, 0.0, 0.0, 0.3, 0.0),

            // Angle -> Rate -> Stick
            // Simplified: P-control on Angle Error gives Stick Input directly (~Rate)
            roll_pid: Pid::new(2.0, 0.0, 0.0, 1.0, 0.0),
            pitch_pid: Pid::new(2.0, 0.0, 0.0, 1.0, 0.0),
            yaw_pid: Pid::new(1.0, 0.0, 0.0, 1.0, 0.0),

            // Target Height 10m. X/Z 0 relative to start.
            // Note: position[1] is Altitude in our State struct to match geo::coord_from_gps output index 1
            target_pos: Vector3::new(0.0, target_alt, 0.0),
            target_yaw: 0.0,
            last_update: Instant::now(),
            armed: false,

            waypoints_local: Vec::new(),
            waypoint_yaws: Vec::new(),
            current_waypoint: 0,
            arrival_time: None,
            hold_time: Duration::from_secs_f64(5.0),
            arrival_radius: 2.0,
            end_behavior: EndBehavior::Hover,
            route_complete: false,
        }
    }

    fn update(&mut self, state: &DroneState) -> [u16; 16] {
        let now = Instant::now();
        let dt = (now - self.last_update).as_secs_f64().max(0.001); // Prevent div/0
        self.last_update = now;

        if !self.armed {
            // Disarmed, send low throttle and disarm switch
            let mut ch = [992; 16];
            ch[2] = 172; // Throttle low
            ch[4] = 172; // Disarm (Switch low)
            debug!("Disarmed: sending low throttle");
            return ch;
        }

        let mut channels = [992; 16];
        channels[4] = 1800; // ARM

        if let (Some(pos), Some(vel), Some(att)) = (state.position, state.velocity, state.attitude)
        {
            // Altitude Control
            // pos[1] is Altitude (Up), vel[1] is vertical speed from Vario
            let current_alt = pos[1];
            let alt_error = self.target_pos[1] - current_alt;
            let vz = vel[1];
            let throttle_correction = self.alt_pid.update(alt_error, dt) - 0.15 * vz;

            let base_throttle = 0.45; // Approx hover throttle
            let throttle = (base_throttle + throttle_correction).clamp(0.0, 1.0);
            channels[2] = liftoff_lib::crsf::us_to_ticks((1000.0 + throttle * 1000.0) as u16);

            debug!(
                "ALT: current={:.2} target={:.2} err={:.2} vz={:.2} correction={:.3} throttle={:.3} ch2={}",
                current_alt, self.target_pos[1], alt_error, vz, throttle_correction, throttle, channels[2]
            );

            // Position Control (direct position -> angle, velocity as damping)
            // pos[0] is East (X), pos[2] is North (Z)
            let pos_err_x = self.target_pos[0] - pos[0];
            let pos_err_z = self.target_pos[2] - pos[2];

            // Direct position PID gives target angle in global frame
            let global_angle_x = self.pos_x_pid.update(pos_err_x, dt);
            let global_angle_z = self.pos_y_pid.update(pos_err_z, dt);

            // Velocity damping in global frame (resists motion regardless of position error)
            let vel_damp = 0.08;
            let damped_angle_x = global_angle_x - vel_damp * vel[0];
            let damped_angle_z = global_angle_z - vel_damp * vel[2];

            debug!(
                "POS: pos=[{:.2}, {:.2}] err=[{:.2}, {:.2}] angle=[{:.3}, {:.3}] vel=[{:.2}, {:.2}] damped=[{:.3}, {:.3}]",
                pos[0], pos[2], pos_err_x, pos_err_z, global_angle_x, global_angle_z, vel[0], vel[2], damped_angle_x, damped_angle_z
            );

            // Rotate global angles to body frame
            let heading = state.yaw;
            let cos_h = heading.cos();
            let sin_h = heading.sin();

            // Body Forward = North(Z)*cos(h) + East(X)*sin(h)
            // Body Right   = East(X)*cos(h) - North(Z)*sin(h)
            let body_fwd = damped_angle_z * cos_h + damped_angle_x * sin_h;
            let body_right = damped_angle_x * cos_h - damped_angle_z * sin_h;

            // To accelerate forward, pitch down (negative angle)
            let target_pitch = -body_fwd;
            // To accelerate right, roll right (positive angle)
            let target_roll = body_right;

            // Attitude Control
            let (roll, pitch, yaw) = att.euler_angles();
            // Note: Check nalgebra euler order/definition. Usually Roll-Pitch-Yaw (XYZ).

            let pitch_err = angle_diff(target_pitch, pitch);
            let roll_err = angle_diff(target_roll, roll);
            let yaw_err = angle_diff(self.target_yaw, yaw);

            debug!(
                "ATT: heading={:.2} body=[fwd={:.3}, right={:.3}] target=[roll={:.3}, pitch={:.3}] actual=[roll={:.3}, pitch={:.3}, yaw={:.3}] err=[roll={:.3}, pitch={:.3}, yaw={:.3}]",
                heading, body_fwd, body_right, target_roll, target_pitch, roll, pitch, yaw, roll_err, pitch_err, yaw_err
            );

            // Stick Outputs
            let pitch_cmd = self.pitch_pid.update(pitch_err, dt);
            let roll_cmd = self.roll_pid.update(roll_err, dt);
            let yaw_cmd = self.yaw_pid.update(yaw_err, dt);

            // Map -1.0..1.0 to 1000..2000
            channels[0] = liftoff_lib::crsf::us_to_ticks((1500.0 + roll_cmd * 500.0) as u16);
            channels[1] = liftoff_lib::crsf::us_to_ticks((1500.0 - pitch_cmd * 500.0) as u16);
            channels[3] = liftoff_lib::crsf::us_to_ticks((1500.0 + yaw_cmd * 500.0) as u16);

            debug!(
                "CMD: roll={:.3} pitch={:.3} yaw={:.3} => ch=[{}, {}, {}, {}]",
                roll_cmd, pitch_cmd, yaw_cmd, channels[0], channels[1], channels[2], channels[3]
            );
        } else {
            debug!(
                "Armed but state incomplete: pos={} vel={} att={}",
                state.position.is_some(),
                state.velocity.is_some(),
                state.attitude.is_some()
            );
        }

        channels
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::init();
    let args = Args::parse();

    info!("Starting liftoff-autopilot");

    // Load waypoint config if provided
    let waypoint_config: Option<WaypointConfig> = if let Some(ref path) = args.waypoints {
        let data = std::fs::read_to_string(path)
            .unwrap_or_else(|e| panic!("Failed to read waypoints file {:?}: {}", path, e));
        let config: WaypointConfig = serde_json::from_str(&data)
            .unwrap_or_else(|e| panic!("Failed to parse waypoints file {:?}: {}", path, e));
        info!(
            "Loaded {} waypoints (hold={:.1}s, radius={:.1}m, end={:?})",
            config.waypoints.len(),
            config.hold_time,
            config.arrival_radius,
            config.end_behavior
        );
        Some(config)
    } else {
        None
    };

    // Zenoh session
    let mut config = Config::default();
    config.insert_json5("mode", &format!(r#""{}""#, args.zenoh_mode))?;
    if let Some(ref endpoint) = args.zenoh_connect {
        config.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))?;
    }

    let session = zenoh::open(config).await?;

    let crsf_tel_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_TELEMETRY);
    let crsf_rc_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_RC_AUTOPILOT);

    info!("Subscribing to: {}", crsf_tel_topic);
    info!("Publishing on: {}", crsf_rc_topic);

    let tel_subscriber = session.declare_subscriber(&crsf_tel_topic).await?;
    let rc_publisher = session.declare_publisher(&crsf_rc_topic).await?;

    let state = Arc::new(Mutex::new(DroneState::default()));

    // Telemetry Receiver Task
    let state_rx = state.clone();
    tokio::spawn(async move {
        loop {
            match tel_subscriber.recv_async().await {
                Ok(sample) => {
                    let payload = sample.payload().to_bytes();
                    if !payload.is_empty() {
                        if let Some(packet) = liftoff_lib::crsf::parse_packet(&payload) {
                            let mut s = state_rx.lock().await;

                            match &packet {
                                CrsfPacket::Attitude(att) => {
                                    let (pitch, roll, yaw) = att.as_radians();
                                    debug!("RX Attitude: pitch={:.3} roll={:.3} yaw={:.3}", pitch, roll, yaw);
                                    s.attitude = Some(UnitQuaternion::from_euler_angles(
                                        roll, pitch, yaw,
                                    ));
                                    s.yaw = yaw;
                                }
                                CrsfPacket::Gps(gps) => {
                                    let lat = gps.lat_deg();
                                    let lon = gps.lon_deg();
                                    let alt = gps.alt_m();
                                    debug!("RX GPS: lat={:.6} lon={:.6} alt={:.2} speed={:.1}km/h heading={:.1}deg",
                                        lat, lon, alt, gps.speed_kmh(), gps.heading_deg());

                                    if s.gps_origin.is_none() {
                                        info!("GPS Origin Set: {}, {} alt={}", lat, lon, alt);
                                        s.gps_origin = Some((lat, lon));
                                        s.alt_origin = alt;
                                    }

                                    if let Some(origin) = s.gps_origin {
                                        let local = geo::coord_from_gps(
                                            (lon, lat, alt),
                                            (origin.1, origin.0),
                                        );
                                        let relative_alt = alt - s.alt_origin;
                                        let new_pos = Vector3::new(
                                            local[0], relative_alt, local[2],
                                        );

                                        // Derive horizontal velocity from position differences
                                        // (GPS heading is yaw, not course-over-ground, so can't use it)
                                        let now = Instant::now();
                                        if let (Some(prev_pos), Some(prev_time)) = (s.prev_position, s.prev_pos_time) {
                                            let dt = (now - prev_time).as_secs_f64();
                                            if dt > 0.01 {
                                                let vel_x = (new_pos[0] - prev_pos[0]) / dt;
                                                let vel_z = (new_pos[2] - prev_pos[2]) / dt;
                                                let vy = if let Some(v) = s.velocity { v[1] } else { 0.0 };
                                                debug!("Derived vel: E={:.2} Up={:.2} N={:.2} dt={:.3}", vel_x, vy, vel_z, dt);
                                                s.velocity = Some(Vector3::new(vel_x, vy, vel_z));
                                            }
                                        }
                                        s.prev_position = Some(new_pos);
                                        s.prev_pos_time = Some(now);

                                        debug!("Local pos: E={:.2} Alt={:.2} (abs={:.2}) N={:.2}", new_pos[0], relative_alt, alt, new_pos[2]);
                                        s.position = Some(new_pos);
                                    }
                                }
                                CrsfPacket::Vario(vario) => {
                                    let vs = vario.vertical_speed_ms();
                                    debug!("RX Vario: vs={:.2} m/s", vs);
                                    if let Some(vel) = &mut s.velocity {
                                        vel[1] = vs;
                                    } else {
                                        s.velocity =
                                            Some(Vector3::new(0.0, vs, 0.0));
                                    }
                                }
                                _ => {
                                    debug!("RX other packet: {:?}", packet);
                                }
                            }
                        }
                    }
                }
                Err(e) => {
                    warn!("Telemetry subscriber error: {}", e);
                    break;
                }
            }
        }
    });

    // Control Loop
    let mut controller = Controller::new(args.target_alt);

    // Apply waypoint config settings
    if let Some(ref config) = waypoint_config {
        controller.hold_time = Duration::from_secs_f64(config.hold_time);
        controller.arrival_radius = config.arrival_radius;
        controller.end_behavior = config.end_behavior.clone();
    }

    let start_time = Instant::now();

    let mut ticker = interval(Duration::from_millis(10)); // 100Hz

    // Wait for GPS origin while sending disarmed packets
    loop {
        ticker.tick().await;
        let drone_state = state.lock().await.clone();

        let channels = controller.update(&drone_state);
        let rc_packet = CrsfPacket::RcChannelsPacked(crsf::RcChannelsPacked { channels });
        let frame = crsf::build_packet(crsf::device_address::FLIGHT_CONTROLLER, &rc_packet)
            .expect("channel values out of range");
        if let Err(e) = rc_publisher.put(frame.as_slice()).await {
            warn!("Publish error: {}", e);
        }

        if drone_state.gps_origin.is_some() {
            break;
        }
    }

    // GPS origin available — convert waypoints to local frame
    {
        let drone_state = state.lock().await.clone();
        let origin = drone_state.gps_origin.unwrap();
        if let Some(ref config) = waypoint_config {
            (controller.waypoints_local, controller.waypoint_yaws) =
                convert_waypoints(&config.waypoints, origin, drone_state.alt_origin);
            if let Some(first) = controller.waypoints_local.first() {
                controller.target_pos = *first;
                if let Some(yaw) = controller.waypoint_yaws[0] {
                    controller.target_yaw = yaw;
                }
                info!(
                    "Navigation started: targeting waypoint 0 at [{:.1}, {:.1}, {:.1}]",
                    first[0], first[1], first[2]
                );
            }
        }
    }

    // Main control loop
    let mut armed = false;
    let shutdown = tokio::signal::ctrl_c();
    tokio::pin!(shutdown);

    loop {
        tokio::select! {
            _ = &mut shutdown => {
                info!("Shutdown signal received, disarming...");
                break;
            }
            _ = ticker.tick() => {}
        }

        let drone_state = state.lock().await.clone();

        // Arming logic
        if !armed && start_time.elapsed().as_secs() > 2 {
            controller.target_yaw = drone_state.yaw;
            info!("Arming! target_yaw={:.3}", controller.target_yaw);
            armed = true;
            controller.armed = true;
        }

        // Waypoint navigation logic
        if armed && !controller.waypoints_local.is_empty() && !controller.route_complete {
            if let Some(pos) = drone_state.position {
                let target = controller.target_pos;
                let dx = pos[0] - target[0];
                let dy = pos[1] - target[1];
                let dz = pos[2] - target[2];
                let distance = (dx * dx + dy * dy + dz * dz).sqrt();

                if distance < controller.arrival_radius {
                    // Within arrival radius
                    if controller.arrival_time.is_none() {
                        controller.arrival_time = Some(Instant::now());
                        info!(
                            "Arrived at waypoint {} (dist={:.2}m)",
                            controller.current_waypoint, distance
                        );
                    }

                    if let Some(arrival) = controller.arrival_time {
                        if arrival.elapsed() >= controller.hold_time {
                            // Hold complete, advance
                            let next = controller.current_waypoint + 1;
                            if next < controller.waypoints_local.len() {
                                controller.current_waypoint = next;
                                controller.target_pos = controller.waypoints_local[next];
                                if let Some(yaw) = controller.waypoint_yaws[next] {
                                    controller.target_yaw = yaw;
                                }
                                controller.arrival_time = None;
                                // Reset position PIDs to avoid integral windup
                                controller.pos_x_pid.reset();
                                controller.pos_y_pid.reset();
                                info!(
                                    "Advancing to waypoint {} at [{:.1}, {:.1}, {:.1}]",
                                    next,
                                    controller.target_pos[0],
                                    controller.target_pos[1],
                                    controller.target_pos[2]
                                );
                            } else {
                                // Reached end of route
                                match controller.end_behavior {
                                    EndBehavior::Hover => {
                                        controller.route_complete = true;
                                        info!("Route complete: hovering at last waypoint");
                                    }
                                    EndBehavior::Loop => {
                                        controller.current_waypoint = 0;
                                        controller.target_pos = controller.waypoints_local[0];
                                        if let Some(yaw) = controller.waypoint_yaws[0] {
                                            controller.target_yaw = yaw;
                                        }
                                        controller.arrival_time = None;
                                        controller.pos_x_pid.reset();
                                        controller.pos_y_pid.reset();
                                        info!(
                                            "Route complete: looping back to waypoint 0 at [{:.1}, {:.1}, {:.1}]",
                                            controller.target_pos[0],
                                            controller.target_pos[1],
                                            controller.target_pos[2]
                                        );
                                    }
                                    EndBehavior::Return => {
                                        controller.target_pos =
                                            Vector3::new(0.0, controller.target_pos[1], 0.0);
                                        controller.route_complete = true;
                                        controller.arrival_time = None;
                                        controller.pos_x_pid.reset();
                                        controller.pos_y_pid.reset();
                                        info!("Route complete: returning to origin");
                                    }
                                }
                            }
                        }
                    }
                } else {
                    // Outside arrival radius, reset arrival timer
                    controller.arrival_time = None;
                }
            }
        }

        // Sending Channels
        let channels = controller.update(&drone_state);
        let rc_packet = CrsfPacket::RcChannelsPacked(crsf::RcChannelsPacked { channels });
        let frame = crsf::build_packet(crsf::device_address::FLIGHT_CONTROLLER, &rc_packet)
            .expect("channel values out of range");

        if let Err(e) = rc_publisher.put(frame.as_slice()).await {
            warn!("Publish error: {}", e);
        }
    }

    // Send disarm packets for 0.5s to ensure the FC receives them
    controller.armed = false;
    for _ in 0..50 {
        let drone_state = state.lock().await.clone();
        let channels = controller.update(&drone_state);
        let rc_packet = CrsfPacket::RcChannelsPacked(crsf::RcChannelsPacked { channels });
        let frame = crsf::build_packet(crsf::device_address::FLIGHT_CONTROLLER, &rc_packet)
            .expect("channel values out of range");
        if let Err(e) = rc_publisher.put(frame.as_slice()).await {
            warn!("Publish error: {}", e);
        }
        ticker.tick().await;
    }
    info!("Disarmed, exiting.");

    session.close().await?;
    Ok(())
}
