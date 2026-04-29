use std::time::Duration;

use clap::Parser;
use futures_util::StreamExt;
use liftoff_lib::crsf_tx;
use liftoff_lib::telemetry::TelemetryPacket;
use liftoff_lib::topics;
use log::{debug, error, info, warn};
use serde::Deserialize;
use tokio_tungstenite::{connect_async, tungstenite::protocol::Message};
use zenoh::Config;

/// WebSocket → Zenoh bridge for Velocidrone.
///
/// Velocidrone exposes per-frame IMU telemetry (position, attitude, velocity,
/// gyro rate) over its built-in WebSocket server when both the
/// `use-web-socket` and `web-socket-imu` toggles are enabled in the in-game
/// settings. This binary consumes that stream, repackages it into
/// `liftoff_lib::telemetry::TelemetryPacket`, and publishes CRSF telemetry
/// frames on the `{prefix}/crsf/telemetry` Zenoh topic — the same topic
/// `liftoff-input` publishes to, so the rest of the workspace
/// (autopilot, dashboard, mavlink-bridge) consumes Velocidrone the same way
/// it consumes Liftoff.
#[derive(Parser, Debug)]
#[command(version, about)]
struct Args {
    /// Velocidrone WebSocket URL. Port 60003 is the UnityWSServer default;
    /// the service path is `velocidrone` (Velocidrone overrides the WSServer
    /// default of `ws` via the scene asset's SerializeField). The host is
    /// the IP Velocidrone discovered via UDP-to-8.8.8.8 — typically the
    /// machine's LAN or public IP, NOT loopback. Use `ss -ltn | grep 60003`
    /// to find the address Velocidrone bound to.
    #[arg(long, default_value = "ws://127.0.0.1:60003/velocidrone")]
    ws_url: String,

    /// Zenoh connect endpoint (e.g. tcp/127.0.0.1:7447). Optional.
    #[arg(long)]
    zenoh_connect: Option<String>,

    /// Zenoh session mode: "peer" or "client".
    #[arg(long, default_value = "peer")]
    zenoh_mode: String,

    /// Topic prefix; defaults match the rest of the workspace.
    #[arg(long, default_value_t = topics::DEFAULT_PREFIX.to_string())]
    zenoh_prefix: String,

    /// Reconnect backoff floor when the WebSocket drops.
    #[arg(long, default_value = "500")]
    reconnect_min_ms: u64,

    /// Reconnect backoff ceiling when the WebSocket drops.
    #[arg(long, default_value = "5000")]
    reconnect_max_ms: u64,
}

/// Shape of the `imu` payload Velocidrone emits.
///
/// Source: `SocketManager.sendIMUData` in Assembly-CSharp.dll. Field naming
/// follows Velocidrone's wire — `roll/pitch/yaw` are the raw `gyroADC` axes
/// (gyro angular rates, deg/s), NOT euler attitude angles. The actual
/// attitude is the `Attitude{X,Y,Z,W}` quaternion separately.
#[derive(Debug, Deserialize)]
struct ImuPayload {
    roll: f32,
    pitch: f32,
    yaw: f32,
    #[serde(rename = "PositionX")] pos_x: f32,
    #[serde(rename = "PositionY")] pos_y: f32,
    #[serde(rename = "PositionZ")] pos_z: f32,
    #[serde(rename = "AttitudeX")] att_x: f32,
    #[serde(rename = "AttitudeY")] att_y: f32,
    #[serde(rename = "AttitudeZ")] att_z: f32,
    #[serde(rename = "AttitudeW")] att_w: f32,
    #[serde(rename = "SpeedX")] vel_x: f32,
    #[serde(rename = "SpeedY")] vel_y: f32,
    #[serde(rename = "SpeedZ")] vel_z: f32,
    /// `Time.time * 1000f` — float milliseconds since game start.
    timestamp: f32,
}

impl ImuPayload {
    fn into_telemetry(self) -> TelemetryPacket {
        TelemetryPacket {
            // Liftoff's wire timestamp is f32 seconds; Velocidrone gives us
            // ms-as-float. Convert so downstream consumers see consistent units.
            timestamp: Some(self.timestamp / 1000.0),
            position: Some([self.pos_x, self.pos_y, self.pos_z]),
            attitude: Some([self.att_x, self.att_y, self.att_z, self.att_w]),
            velocity: Some([self.vel_x, self.vel_y, self.vel_z]),
            gyro: Some([self.roll, self.pitch, self.yaw]),
            input: None,
            battery: None,
            motor_rpm: None,
        }
    }
}

#[tokio::main(flavor = "multi_thread", worker_threads = 1)]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::init();
    let args = Args::parse();

    info!("Starting velocidrone-input");
    info!("WebSocket: {}", args.ws_url);

    let mut config = Config::default();
    config.insert_json5("mode", &format!(r#""{}""#, args.zenoh_mode))?;
    if let Some(ref endpoint) = args.zenoh_connect {
        config.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))?;
    }
    let session = zenoh::open(config).await?;

    let crsf_tel_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_TELEMETRY);
    info!("Publishing CRSF telemetry on: {}", crsf_tel_topic);
    let crsf_tel_publisher = session.declare_publisher(crsf_tel_topic).await?;

    let mut backoff_ms = args.reconnect_min_ms;
    loop {
        match run_once(&args.ws_url, &crsf_tel_publisher).await {
            Ok(()) => {
                warn!("WebSocket closed cleanly; will reconnect.");
                backoff_ms = args.reconnect_min_ms;
            }
            Err(e) => {
                error!("WebSocket session error: {e}; reconnecting in {backoff_ms} ms");
            }
        }
        tokio::time::sleep(Duration::from_millis(backoff_ms)).await;
        backoff_ms = (backoff_ms.saturating_mul(2)).min(args.reconnect_max_ms);
    }
}

async fn run_once(
    url: &str,
    crsf_pub: &zenoh::pubsub::Publisher<'_>,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    info!("Connecting to {url}");
    let (ws, response) = connect_async(url).await?;
    info!("Connected ({}). Awaiting frames…", response.status());
    let (_write, mut read) = ws.split();

    let mut frames_seen: u64 = 0;
    while let Some(msg) = read.next().await {
        // Velocidrone wraps every JSON payload in a WebSocket binary frame
        // (opcode 0x2) rather than text (0x1) — see WSServer.WSClient.SendData
        // calling EncodePackage(data, fin: true, 2, …). The bytes are still
        // UTF-8 JSON, so decode and route through the same handler as text
        // frames.
        let payload: Option<String> = match msg? {
            Message::Text(text) => Some(text),
            Message::Binary(bytes) => match String::from_utf8(bytes) {
                Ok(s) => Some(s),
                Err(e) => {
                    debug!("dropping non-UTF8 binary frame: {e}");
                    None
                }
            },
            Message::Ping(_) | Message::Pong(_) | Message::Frame(_) => None,
            Message::Close(reason) => {
                info!("WebSocket close from server: {reason:?}");
                return Ok(());
            }
        };

        if let Some(text) = payload {
            if let Err(e) = handle_text(&text, crsf_pub, &mut frames_seen).await {
                debug!("dropping message ({e}): {}", truncate(&text, 200));
            }
        }
    }
    Ok(())
}

async fn handle_text(
    text: &str,
    crsf_pub: &zenoh::pubsub::Publisher<'_>,
    frames_seen: &mut u64,
) -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    let value: serde_json::Value = serde_json::from_str(text)?;
    let obj = value
        .as_object()
        .ok_or("top-level message is not a JSON object")?;

    if let Some(imu_value) = obj.get("imu") {
        let imu: ImuPayload = serde_json::from_value(imu_value.clone())?;
        let packet = imu.into_telemetry();

        // Velocidrone has no battery via WS; pass None and let CRSF generation
        // emit only the frames it can fill from position/attitude/velocity.
        let crsf_frames = crsf_tx::generate_crsf_telemetry(&packet, None);
        for frame in &crsf_frames {
            crsf_pub.put(frame.as_slice()).await?;
        }

        *frames_seen += 1;
        if *frames_seen <= 3 || *frames_seen % 600 == 0 {
            info!(
                "imu frame #{frames_seen}: pos=[{:.2},{:.2},{:.2}] gyro=[{:.1},{:.1},{:.1}] -> {} CRSF frames",
                packet.position.unwrap()[0],
                packet.position.unwrap()[1],
                packet.position.unwrap()[2],
                packet.gyro.unwrap()[0],
                packet.gyro.unwrap()[1],
                packet.gyro.unwrap()[2],
                crsf_frames.len(),
            );
        }
        return Ok(());
    }

    // Race / lobby / countdown messages — all the things the user already
    // knew about. Logged at debug for now; future home is a separate
    // {prefix}/events topic.
    debug!("non-imu message: {}", truncate(text, 200));
    Ok(())
}

fn truncate(s: &str, max: usize) -> String {
    if s.len() <= max {
        s.to_string()
    } else {
        format!("{}…[+{}B]", &s[..max], s.len() - max)
    }
}
