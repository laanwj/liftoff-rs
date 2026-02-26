use clap::Parser;
use liftoff_lib::topics;
use log::{error, info, trace};
use metrics::{Unit, counter, describe_counter};
use metrics_exporter_tcp::TcpBuilder;
use tokio::net::UdpSocket;
use tokio::signal;
use zenoh::Config;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Bind address for simulator telemetry UDP.
    #[arg(long, default_value = "127.0.0.1:9001")]
    sim_bind: std::net::SocketAddr,

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
    #[arg(long, default_value = "127.0.0.1:5001")]
    metrics_tcp_bind: std::net::SocketAddr,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::init();
    let args = Args::parse();

    info!("Starting liftoff-bridge");
    info!("Simulator telemetry on {}", args.sim_bind);

    if args.metrics_tcp {
        let builder = TcpBuilder::new().listen_address(args.metrics_tcp_bind);
        builder
            .install()
            .expect("failed to install metrics TCP exporter");
    }

    describe_counter!(
        "bridge.packet.rx",
        Unit::Count,
        "Incoming telemetry packets from sim"
    );
    describe_counter!(
        "bridge.packet.tx",
        Unit::Count,
        "Telemetry packets published to Zenoh"
    );

    // Zenoh session
    let mut config = Config::default();
    config.insert_json5("mode", &format!(r#""{}""#, args.zenoh_mode))?;
    if let Some(ref endpoint) = args.zenoh_connect {
        config.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))?;
    }

    let session = zenoh::open(config).await?;
    let tel_topic = topics::topic(&args.zenoh_prefix, topics::TELEMETRY);
    info!("Publishing on Zenoh topic: {}", tel_topic);
    let publisher = session.declare_publisher(&tel_topic).await?;

    // Sim UDP socket
    let sock = UdpSocket::bind(&args.sim_bind).await?;

    let mut buf = [0u8; 4096];
    loop {
        tokio::select! {
            res = sock.recv_from(&mut buf) => {
                match res {
                    Ok((len, _addr)) => {
                        trace!("rx sim {} bytes", len);
                        counter!("bridge.packet.rx").increment(1);
                        publisher.put(&buf[..len]).await?;
                        counter!("bridge.packet.tx").increment(1);
                    }
                    Err(e) => error!("UDP recv error: {}", e),
                }
            }
            _ = signal::ctrl_c() => {
                info!("Shutting down");
                break;
            }
        }
    }

    session.close().await?;
    Ok(())
}
