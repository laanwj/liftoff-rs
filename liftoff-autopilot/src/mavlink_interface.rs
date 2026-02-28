use log::{debug, info, warn};
use mavlink::common::*;
use mavlink::{MavHeader, MavlinkVersion, Message};
use mavlink::peek_reader::PeekReader;
use std::collections::HashMap;
use std::io::Cursor;
use std::net::SocketAddr;
use std::sync::Arc;
use std::sync::atomic::{AtomicU8, Ordering};
use tokio::net::UdpSocket;
use tokio::sync::{Mutex, mpsc};
use tokio::time::{Duration, Instant, interval};

// --- Parameter store ---

type ParamStore = Arc<Mutex<HashMap<String, f32>>>;

fn default_params() -> HashMap<String, f32> {
    HashMap::from([
        ("MIS_TAKEOFF_ALT".into(), 2.5), // ArduPilot default
    ])
}

/// Encode a parameter name into the MAVLink [u8; 16] format.
fn encode_param_id(name: &str) -> [u8; 16] {
    let mut id = [0u8; 16];
    let bytes = name.as_bytes();
    let len = bytes.len().min(16);
    id[..len].copy_from_slice(&bytes[..len]);
    id
}

/// Decode a MAVLink [u8; 16] param_id to a String.
fn decode_param_id(id: &[u8; 16]) -> String {
    let end = id.iter().position(|&b| b == 0).unwrap_or(16);
    String::from_utf8_lossy(&id[..end]).into_owned()
}

// --- Public types ---

#[derive(Debug, Clone)]
pub enum AutopilotCommand {
    Arm,
    Disarm,
    Takeoff { alt: f32 },
    Land,
    Goto {
        lat: f64,
        lon: f64,
        alt_msl: f32,
        yaw: f32,
    },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FlightMode {
    OnGround,
    Takeoff,
    InAir,
    Landing,
}

pub struct MavlinkState {
    pub armed: bool,
    pub flight_mode: FlightMode,
    pub boot_time: Instant,
}

// --- Shared drone state reader ---

/// Snapshot of drone telemetry for MAVLink reporting.
#[derive(Debug, Clone)]
pub struct TelemetrySnapshot {
    pub gps_lat: f64,
    pub gps_lon: f64,
    pub gps_alt: f64,
    pub alt_origin: f64,
    pub vel_n: f64,
    pub vel_e: f64,
    pub vel_d: f64,
    pub yaw: f64,
    pub has_gps: bool,
}

// --- MavSender helper ---

struct MavSender {
    socket: Arc<UdpSocket>,
    peer: Arc<Mutex<Option<SocketAddr>>>,
    sequence: AtomicU8,
}

impl MavSender {
    fn new(socket: Arc<UdpSocket>, peer: Arc<Mutex<Option<SocketAddr>>>) -> Self {
        Self {
            socket,
            peer,
            sequence: AtomicU8::new(0),
        }
    }

    async fn send(&self, msg: MavMessage) {
        let peer = *self.peer.lock().await;
        let Some(addr) = peer else { return };

        let header = MavHeader {
            system_id: 1,
            component_id: 1,
            sequence: self.sequence.fetch_add(1, Ordering::Relaxed),
        };

        let mut buf = Vec::with_capacity(280);
        mavlink::write_v2_msg(&mut buf, header, &msg).ok();
        if let Err(e) = self.socket.send_to(&buf, addr).await {
            debug!("MAVLink send error: {}", e);
        }
    }
}

// --- Message parsing ---

/// Try parsing a MAVLink v2 message.
fn parse_mavlink(data: &[u8]) -> Option<MavMessage> {
    let mut reader = PeekReader::new(Cursor::new(data));
    if let Ok((_header, msg)) = mavlink::read_v2_msg::<MavMessage, _>(&mut reader) {
        return Some(msg);
    }
    // Retry as raw v2 and parse manually
    let mut reader = PeekReader::new(Cursor::new(data));
    if let Ok(raw) = mavlink::read_v2_raw_message::<MavMessage, _>(&mut reader) {
        if let Ok(msg) = MavMessage::parse(MavlinkVersion::V2, raw.message_id(), raw.payload()) {
            return Some(msg);
        }
    }
    None
}

// --- Receiver task ---

async fn receiver_task(
    socket: Arc<UdpSocket>,
    peer: Arc<Mutex<Option<SocketAddr>>>,
    sender: Arc<MavSender>,
    cmd_tx: mpsc::Sender<AutopilotCommand>,
    params: ParamStore,
) {
    let mut buf = [0u8; 512];

    loop {
        let (n, from) = match socket.recv_from(&mut buf).await {
            Ok(r) => r,
            Err(e) => {
                warn!("MAVLink recv error: {}", e);
                continue;
            }
        };

        // Learn peer address from first incoming packet
        {
            let mut p = peer.lock().await;
            if p.is_none() {
                info!("MAVLink peer discovered: {}", from);
            }
            *p = Some(from);
        }

        // Parse MAVLink message
        let Some(msg) = parse_mavlink(&buf[..n]) else {
            continue;
        };

        match msg {
            MavMessage::HEARTBEAT(_) => {
                // Ignore GCS heartbeats
                debug!("MAVLink: received GCS heartbeat from {}", from);
            }
            MavMessage::COMMAND_LONG(cmd) => {
                handle_command_long(&sender, &cmd_tx, &cmd, &params).await;
            }
            MavMessage::COMMAND_INT(cmd) => {
                handle_command_int(&sender, &cmd_tx, &cmd).await;
            }
            MavMessage::PARAM_SET(p) => {
                handle_param_set(&sender, &params, &p).await;
            }
            MavMessage::PARAM_REQUEST_READ(p) => {
                handle_param_request_read(&sender, &params, &p).await;
            }
            _ => {
                debug!("MAVLink: ignoring message {:?}", msg.message_id());
            }
        }
    }
}

async fn handle_command_long(
    sender: &MavSender,
    cmd_tx: &mpsc::Sender<AutopilotCommand>,
    cmd: &COMMAND_LONG_DATA,
    params: &ParamStore,
) {
    let result = match cmd.command {
        // MAV_CMD_COMPONENT_ARM_DISARM (400)
        MavCmd::MAV_CMD_COMPONENT_ARM_DISARM => {
            if cmd.param1 > 0.5 {
                info!("MAVLink: ARM command received");
                let _ = cmd_tx.send(AutopilotCommand::Arm).await;
            } else {
                info!("MAVLink: DISARM command received");
                let _ = cmd_tx.send(AutopilotCommand::Disarm).await;
            }
            MavResult::MAV_RESULT_ACCEPTED
        }
        // MAV_CMD_NAV_TAKEOFF (22)
        MavCmd::MAV_CMD_NAV_TAKEOFF => {
            // param7 carries altitude; if 0, use stored MIS_TAKEOFF_ALT
            let alt = if cmd.param7.abs() > f32::EPSILON {
                cmd.param7
            } else {
                let store = params.lock().await;
                *store.get("MIS_TAKEOFF_ALT").unwrap_or(&2.5)
            };
            info!("MAVLink: TAKEOFF command, alt={:.1}m", alt);
            let _ = cmd_tx.send(AutopilotCommand::Takeoff { alt }).await;
            MavResult::MAV_RESULT_ACCEPTED
        }
        // MAV_CMD_NAV_LAND (21)
        MavCmd::MAV_CMD_NAV_LAND => {
            info!("MAVLink: LAND command received");
            let _ = cmd_tx.send(AutopilotCommand::Land).await;
            MavResult::MAV_RESULT_ACCEPTED
        }
        // MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES (520) — MAVSDK discovery
        MavCmd::MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES => {
            debug!("MAVLink: AUTOPILOT_CAPABILITIES requested");
            send_autopilot_version(sender).await;
            MavResult::MAV_RESULT_ACCEPTED
        }
        // MAV_CMD_REQUEST_MESSAGE (512) — MAVSDK requests specific messages
        MavCmd::MAV_CMD_REQUEST_MESSAGE => {
            let msg_id = cmd.param1 as u32;
            debug!("MAVLink: REQUEST_MESSAGE id={}", msg_id);
            match msg_id {
                // 148 = AUTOPILOT_VERSION
                148 => {
                    send_autopilot_version(sender).await;
                    MavResult::MAV_RESULT_ACCEPTED
                }
                _ => {
                    // ACK as accepted for messages we already stream
                    // (GLOBAL_POSITION_INT=33, GPS_RAW_INT=24, HOME_POSITION=242, etc.)
                    MavResult::MAV_RESULT_ACCEPTED
                }
            }
        }
        // MAV_CMD_DO_SET_MODE — MAVSDK sets mode before takeoff/land
        MavCmd::MAV_CMD_DO_SET_MODE => {
            debug!("MAVLink: DO_SET_MODE (ignored, autopilot manages modes)");
            MavResult::MAV_RESULT_ACCEPTED
        }
        // MAV_CMD_SET_MESSAGE_INTERVAL — MAVSDK requests telemetry rate control
        MavCmd::MAV_CMD_SET_MESSAGE_INTERVAL => {
            debug!("MAVLink: SET_MESSAGE_INTERVAL (ignored, fixed rates)");
            MavResult::MAV_RESULT_ACCEPTED
        }
        _ => {
            warn!(
                "MAVLink: unsupported COMMAND_LONG cmd={:?}",
                cmd.command
            );
            MavResult::MAV_RESULT_UNSUPPORTED
        }
    };

    send_command_ack(sender, cmd.command, result).await;
}

async fn send_autopilot_version(sender: &MavSender) {
    sender
        .send(MavMessage::AUTOPILOT_VERSION(AUTOPILOT_VERSION_DATA {
            capabilities: MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_MAVLINK2
                | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_COMMAND_INT
                | MavProtocolCapability::MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT,
            uid: 0,
            flight_sw_version: 0x0100_0000, // v1.0.0
            middleware_sw_version: 0,
            os_sw_version: 0,
            board_version: 0,
            vendor_id: 0,
            product_id: 0,
            flight_custom_version: [0; 8],
            middleware_custom_version: [0; 8],
            os_custom_version: [0; 8],
        }))
        .await;
}

async fn handle_command_int(
    sender: &MavSender,
    cmd_tx: &mpsc::Sender<AutopilotCommand>,
    cmd: &COMMAND_INT_DATA,
) {
    let result = match cmd.command {
        // MAV_CMD_DO_REPOSITION (192)
        MavCmd::MAV_CMD_DO_REPOSITION => {
            let lat = cmd.x as f64 / 1e7;
            let lon = cmd.y as f64 / 1e7;
            let alt_msl = cmd.z;
            let yaw = cmd.param4;
            info!(
                "MAVLink: GOTO lat={:.6} lon={:.6} alt={:.1} yaw={:.1}",
                lat, lon, alt_msl, yaw
            );
            let _ = cmd_tx
                .send(AutopilotCommand::Goto {
                    lat,
                    lon,
                    alt_msl,
                    yaw,
                })
                .await;
            MavResult::MAV_RESULT_ACCEPTED
        }
        _ => {
            warn!(
                "MAVLink: unsupported COMMAND_INT cmd={:?}",
                cmd.command
            );
            MavResult::MAV_RESULT_UNSUPPORTED
        }
    };

    send_command_ack(sender, cmd.command, result).await;
}

async fn send_command_ack(sender: &MavSender, command: MavCmd, result: MavResult) {
    sender
        .send(MavMessage::COMMAND_ACK(COMMAND_ACK_DATA {
            command,
            result,
        }))
        .await;
}

// --- Parameter handling ---

async fn handle_param_set(sender: &MavSender, params: &ParamStore, p: &PARAM_SET_DATA) {
    let name = decode_param_id(&p.param_id);
    let value = p.param_value;

    {
        let mut store = params.lock().await;
        store.insert(name.clone(), value);
    }
    info!("MAVLink: PARAM_SET {}={}", name, value);

    // Respond with PARAM_VALUE to confirm
    let store = params.lock().await;
    let count = store.len() as u16;
    sender
        .send(MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
            param_id: p.param_id,
            param_value: value,
            param_type: p.param_type,
            param_count: count,
            param_index: 0,
        }))
        .await;
}

async fn handle_param_request_read(
    sender: &MavSender,
    params: &ParamStore,
    p: &PARAM_REQUEST_READ_DATA,
) {
    let name = decode_param_id(&p.param_id);
    let store = params.lock().await;

    if let Some(&value) = store.get(&name) {
        let count = store.len() as u16;
        debug!("MAVLink: PARAM_REQUEST_READ {}={}", name, value);
        sender
            .send(MavMessage::PARAM_VALUE(PARAM_VALUE_DATA {
                param_id: encode_param_id(&name),
                param_value: value,
                param_type: MavParamType::MAV_PARAM_TYPE_REAL32,
                param_count: count,
                param_index: 0,
            }))
            .await;
    } else {
        debug!("MAVLink: PARAM_REQUEST_READ unknown param '{}'", name);
    }
}

// --- Heartbeat task ---

async fn heartbeat_task(sender: Arc<MavSender>, mav_state: Arc<Mutex<MavlinkState>>) {
    let mut ticker = interval(Duration::from_secs(1));

    loop {
        ticker.tick().await;

        let state = mav_state.lock().await;
        let mut base_mode = MavModeFlag::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        if state.armed {
            base_mode |= MavModeFlag::MAV_MODE_FLAG_SAFETY_ARMED;
        }
        drop(state);

        sender
            .send(MavMessage::HEARTBEAT(HEARTBEAT_DATA {
                custom_mode: 0,
                mavtype: MavType::MAV_TYPE_QUADROTOR,
                autopilot: MavAutopilot::MAV_AUTOPILOT_ARDUPILOTMEGA,
                base_mode,
                system_status: MavState::MAV_STATE_ACTIVE,
                mavlink_version: 0x03,
            }))
            .await;
    }
}

// --- Telemetry task ---

async fn telemetry_task(
    sender: Arc<MavSender>,
    mav_state: Arc<Mutex<MavlinkState>>,
    telem_source: Arc<Mutex<dyn Fn() -> TelemetrySnapshot + Send>>,
) {
    let mut ticker_position = interval(Duration::from_millis(250)); // 4Hz
    let mut ticker_gps_raw = interval(Duration::from_secs(1)); // 1Hz
    let mut ticker_ext_state = interval(Duration::from_secs(1)); // 1Hz
    loop {
        tokio::select! {
            _ = ticker_position.tick() => {
                let snap = (telem_source.lock().await)();
                if !snap.has_gps { continue; }

                let state = mav_state.lock().await;
                let boot_ms = state.boot_time.elapsed().as_millis() as u32;
                drop(state);

                let alt_msl_mm = (snap.gps_alt * 1000.0) as i32;
                let relative_alt_mm = ((snap.gps_alt - snap.alt_origin) * 1000.0) as i32;

                // Velocity: internal [E,Up,N] → MAVLink [N,E,Down] in cm/s
                let vx = (snap.vel_n * 100.0) as i16; // North
                let vy = (snap.vel_e * 100.0) as i16; // East
                let vz = (-snap.vel_d * 100.0) as i16; // Down (negate Up)

                let hdg = ((snap.yaw.to_degrees().rem_euclid(360.0)) * 100.0) as u16;

                sender.send(MavMessage::GLOBAL_POSITION_INT(GLOBAL_POSITION_INT_DATA {
                    time_boot_ms: boot_ms,
                    lat: (snap.gps_lat * 1e7) as i32,
                    lon: (snap.gps_lon * 1e7) as i32,
                    alt: alt_msl_mm,
                    relative_alt: relative_alt_mm,
                    vx,
                    vy,
                    vz,
                    hdg,
                })).await;
            }
            _ = ticker_gps_raw.tick() => {
                let snap = (telem_source.lock().await)();
                if !snap.has_gps { continue; }

                sender.send(MavMessage::GPS_RAW_INT(GPS_RAW_INT_DATA {
                    time_usec: 0,
                    lat: (snap.gps_lat * 1e7) as i32,
                    lon: (snap.gps_lon * 1e7) as i32,
                    alt: (snap.gps_alt * 1000.0) as i32,
                    eph: 100, // HDOP 1.0
                    epv: 100, // VDOP 1.0
                    vel: 0,
                    cog: 0,
                    fix_type: GpsFixType::GPS_FIX_TYPE_3D_FIX,
                    satellites_visible: 12,
                })).await;

                sender.send(MavMessage::HOME_POSITION(HOME_POSITION_DATA {
                    latitude: (snap.gps_lat * 1e7) as i32,
                    longitude: (snap.gps_lon * 1e7) as i32,
                    altitude: (snap.alt_origin * 1000.0) as i32,
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    q: [1.0, 0.0, 0.0, 0.0],
                    approach_x: 0.0,
                    approach_y: 0.0,
                    approach_z: 0.0,
                })).await;
            }
            _ = ticker_ext_state.tick() => {
                let state = mav_state.lock().await;
                let landed = match state.flight_mode {
                    FlightMode::OnGround => MavLandedState::MAV_LANDED_STATE_ON_GROUND,
                    FlightMode::Takeoff => MavLandedState::MAV_LANDED_STATE_TAKEOFF,
                    FlightMode::InAir => MavLandedState::MAV_LANDED_STATE_IN_AIR,
                    FlightMode::Landing => MavLandedState::MAV_LANDED_STATE_LANDING,
                };
                drop(state);

                sender.send(MavMessage::EXTENDED_SYS_STATE(EXTENDED_SYS_STATE_DATA {
                    vtol_state: MavVtolState::MAV_VTOL_STATE_UNDEFINED,
                    landed_state: landed,
                })).await;
            }
        }
    }
}

// --- Public entry point ---

pub async fn start(
    bind_addr: &str,
    telem_source: Arc<Mutex<dyn Fn() -> TelemetrySnapshot + Send>>,
) -> Result<
    (mpsc::Receiver<AutopilotCommand>, Arc<Mutex<MavlinkState>>),
    Box<dyn std::error::Error + Send + Sync>,
> {
    let socket = Arc::new(UdpSocket::bind(bind_addr).await?);
    info!("MAVLink interface bound to {}", bind_addr);

    let peer: Arc<Mutex<Option<SocketAddr>>> = Arc::new(Mutex::new(None));
    let sender = Arc::new(MavSender::new(socket.clone(), peer.clone()));

    let mav_state = Arc::new(Mutex::new(MavlinkState {
        armed: false,
        flight_mode: FlightMode::OnGround,
        boot_time: Instant::now(),
    }));

    let params: ParamStore = Arc::new(Mutex::new(default_params()));

    let (cmd_tx, cmd_rx) = mpsc::channel(32);

    // Spawn receiver task
    tokio::spawn(receiver_task(
        socket.clone(),
        peer.clone(),
        sender.clone(),
        cmd_tx,
        params,
    ));

    // Spawn heartbeat task
    tokio::spawn(heartbeat_task(sender.clone(), mav_state.clone()));

    // Spawn telemetry task
    tokio::spawn(telemetry_task(
        sender.clone(),
        mav_state.clone(),
        telem_source,
    ));

    Ok((cmd_rx, mav_state))
}
