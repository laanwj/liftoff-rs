use clap::Parser;
use liftoff_lib::topics;
use log::{debug, info, warn};
use std::net::SocketAddr;
use std::sync::Arc;
use tokio::net::UdpSocket;
use tokio::sync::Mutex;
use zenoh::Config;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// MAVLink UDP bind address.
    #[arg(long, default_value = "0.0.0.0:14550")]
    mavlink_bind: String,

    /// System ID to forward from Zenoh to UDP (outbound filter).
    #[arg(long, default_value_t = 1)]
    system_id: u8,

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

// --- UDP → Zenoh bridge (inbound) ---

async fn udp_to_zenoh_task(
    socket: Arc<UdpSocket>,
    peer: Arc<Mutex<Option<SocketAddr>>>,
    zenoh_pub: zenoh::pubsub::Publisher<'static>,
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

        // Publish raw received bytes to Zenoh
        if let Err(e) = zenoh_pub.put(&buf[..n]).await {
            debug!("MAVLink zenoh publish error: {}", e);
        }
    }
}

// --- Zenoh → UDP bridge (outbound) ---

async fn zenoh_to_udp_task(
    subscriber: zenoh::pubsub::Subscriber<zenoh::handlers::FifoChannelHandler<zenoh::sample::Sample>>,
    socket: Arc<UdpSocket>,
    peer: Arc<Mutex<Option<SocketAddr>>>,
    system_id: u8,
) {
    loop {
        let sample = match subscriber.recv_async().await {
            Ok(s) => s,
            Err(e) => {
                warn!("MAVLink zenoh_to_udp subscriber error: {}", e);
                break;
            }
        };

        let payload = sample.payload().to_bytes();
        // Only forward frames from our system_id (outgoing messages)
        // MAVLink v2 frame: buf[0]==0xFD, buf[5]==system_id
        if payload.len() >= 6 && payload[0] == 0xFD && payload[5] == system_id {
            let addr = *peer.lock().await;
            if let Some(addr) = addr {
                if let Err(e) = socket.send_to(&payload, addr).await {
                    debug!("MAVLink UDP send error: {}", e);
                }
            }
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::init();
    let args = Args::parse();

    info!("Starting liftoff-mavlink-bridge");

    // Zenoh session
    let mut config = Config::default();
    config.insert_json5("mode", &format!(r#""{}""#, args.zenoh_mode))?;
    if let Some(ref endpoint) = args.zenoh_connect {
        config.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))?;
    }

    let session = zenoh::open(config).await?;

    let mavlink_topic = topics::topic(&args.zenoh_prefix, topics::MAVLINK);
    info!("MAVLink topic: {}", mavlink_topic);

    let socket = Arc::new(UdpSocket::bind(&args.mavlink_bind).await?);
    info!("MAVLink UDP bound to {}", args.mavlink_bind);

    // Publisher for UDP→Zenoh (inbound GCS messages)
    let udp_in_publisher = session.declare_publisher(mavlink_topic.clone()).await?;

    // Subscriber for Zenoh→UDP (outbound autopilot messages)
    let udp_out_subscriber = session.declare_subscriber(mavlink_topic).await?;

    let peer: Arc<Mutex<Option<SocketAddr>>> = Arc::new(Mutex::new(None));

    // Spawn UDP → Zenoh bridge (inbound)
    tokio::spawn(udp_to_zenoh_task(
        socket.clone(),
        peer.clone(),
        udp_in_publisher,
    ));

    // Spawn Zenoh → UDP bridge (outbound)
    tokio::spawn(zenoh_to_udp_task(
        udp_out_subscriber,
        socket.clone(),
        peer.clone(),
        args.system_id,
    ));

    info!(
        "Bridge running (system_id={}, bind={})",
        args.system_id, args.mavlink_bind
    );

    // Wait for Ctrl+C
    tokio::signal::ctrl_c().await?;
    info!("Shutdown signal received, exiting.");

    session.close().await?;
    Ok(())
}
