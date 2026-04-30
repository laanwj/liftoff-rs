//! Zenoh-driven CRSF → uinput joystick service.
//!
//! Subscribes to two CRSF RC channel topics — manual (`{prefix}/crsf/rc`)
//! and autopilot (`{prefix}/crsf/rc/autopilot`) — muxes between them based
//! on radio presence and the SA switch (channel 7), and applies the
//! winning frame to a virtual `/dev/uinput` controller via
//! [`crsf_joystick::Joystick`].
//!
//! Mux rules:
//! - **No manual frame within `MANUAL_TIMEOUT`**: autopilot wins.
//! - **Manual frame fresh, SA switch low (channel 7 < `AXIS_MID`)**: manual wins.
//! - **Manual frame fresh, SA switch high**: autopilot wins.
//!
//! This matches the SA-switch handoff convention used elsewhere in the
//! workspace; the simulator-side bridges (e.g. `liftoff-input`) don't
//! see RC channels at all — they only handle telemetry.
use std::time::Duration;

use clap::Parser;
use crsf_joystick::{AXIS_MAX, AXIS_MID, Joystick};
use log::{error, info, trace, warn};
use metrics::{Unit, counter, describe_counter};
use metrics_exporter_tcp::TcpBuilder;
use telemetry_lib::crsf::{self, CrsfPacket};
use telemetry_lib::topics;
use zenoh::Config;

const MANUAL_TIMEOUT: Duration = Duration::from_millis(500);

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Zenoh connect endpoint (e.g. tcp/192.168.1.1:7447). Omit for peer discovery.
    #[arg(long)]
    zenoh_connect: Option<String>,

    /// Zenoh mode (peer or client).
    #[arg(long, default_value = "peer")]
    zenoh_mode: String,

    /// Zenoh topic prefix.
    #[arg(long, default_value = topics::DEFAULT_PREFIX)]
    zenoh_prefix: String,

    /// Enable metrics reporting using metrics-rs-tcp-exporter.
    #[arg(long, default_value_t = false)]
    metrics_tcp: bool,

    /// Bind address for metrics-rs-tcp-exporter.
    #[arg(long, default_value = "127.0.0.1:5004")]
    metrics_tcp_bind: std::net::SocketAddr,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::init();
    let args = Args::parse();

    info!("Starting crsf-joystick");

    if args.metrics_tcp {
        let builder = TcpBuilder::new().listen_address(args.metrics_tcp_bind);
        builder
            .install()
            .expect("failed to install metrics TCP exporter");
    }

    describe_counter!("joystick.crsf.rx", Unit::Count, "CRSF frames received");
    describe_counter!(
        "joystick.crsf.rx_rc_channels",
        Unit::Count,
        "CRSF RC_CHANNELS frames received"
    );
    describe_counter!(
        "joystick.uinput.update",
        Unit::Count,
        "Updates to virtual input device"
    );

    let mut config = Config::default();
    config.insert_json5("mode", &format!(r#""{}""#, args.zenoh_mode))?;
    if let Some(ref endpoint) = args.zenoh_connect {
        config.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))?;
    }
    let session = zenoh::open(config).await?;

    let crsf_rc_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_RC);
    let crsf_rc_ap_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_RC_AUTOPILOT);
    info!("Subscribing to: {} (manual)", crsf_rc_topic);
    info!("Subscribing to: {} (autopilot)", crsf_rc_ap_topic);

    let rc_subscriber = session.declare_subscriber(&crsf_rc_topic).await?;
    let rc_ap_subscriber = session.declare_subscriber(&crsf_rc_ap_topic).await?;

    // /dev/uinput requires write permission — typically achieved via udev
    // rule or running as a member of the `input` group.
    let mut joystick = Joystick::new()?;

    // Mux state: track manual radio presence and the SA switch position.
    let mut last_manual_time: Option<tokio::time::Instant> = None;
    let mut last_manual_ch7: u16 = 0; // SA switch, low = manual
    let mut active_source = "none";

    loop {
        let (payload, source) = tokio::select! {
            result = rc_subscriber.recv_async() => match result {
                Ok(sample) => (sample.payload().to_bytes().to_vec(), "manual"),
                Err(e) => { error!("RC subscriber error: {}", e); break; }
            },
            result = rc_ap_subscriber.recv_async() => match result {
                Ok(sample) => (sample.payload().to_bytes().to_vec(), "autopilot"),
                Err(e) => { error!("RC autopilot subscriber error: {}", e); break; }
            },
        };

        trace!("rx crsf ({}) {:02x?}", source, &*payload);
        counter!("joystick.crsf.rx").increment(1);

        let Some(CrsfPacket::RcChannelsPacked(channels)) =
            crsf::parse_packet_check(&payload)
        else {
            continue;
        };
        counter!("joystick.crsf.rx_rc_channels").increment(1);
        if channels.channels.iter().any(|&c| c > AXIS_MAX) {
            warn!("Channel out of range: {:?}", channels.channels);
            continue;
        }

        if source == "manual" {
            last_manual_time = Some(tokio::time::Instant::now());
            last_manual_ch7 = channels.channels[7];
        }

        let manual_active = last_manual_time
            .map(|t| t.elapsed() < MANUAL_TIMEOUT)
            .unwrap_or(false);
        let selected = if manual_active && last_manual_ch7 < AXIS_MID {
            "manual"
        } else {
            "autopilot"
        };

        if active_source != selected {
            info!("RC source switched to {}", selected);
            active_source = selected;
        }

        if source == selected {
            if let Err(e) = joystick.update(channels.channels) {
                error!("Failed to update uinput: {}", e);
            }
        }
    }

    session.close().await?;
    Ok(())
}
