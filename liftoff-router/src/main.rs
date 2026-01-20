use clap::Parser;
use liftoff_lib::router_protocol::Opcode;
use log::{debug, error, info, trace, warn};
use metrics::{Unit, counter, describe_counter, describe_gauge, gauge};
use metrics_exporter_tcp::TcpBuilder;
use std::collections::HashSet;
use std::net::SocketAddr;
use std::sync::Arc;
use tokio::net::UdpSocket;
use tokio::signal;
use tokio::sync::Mutex;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Bind address for telemetry router.
    #[arg(long, default_value = "127.0.0.1:9003")]
    cmd_bind: std::net::SocketAddr,

    /// Bind address for incoming telemetry.
    #[arg(long, default_value = "127.0.0.1:9001")]
    tel_bind: std::net::SocketAddr,

    /// Enable metrics reporting using metrics-rs-tcp-exporter.
    #[arg(long, default_value_t = false)]
    metrics_tcp: bool,

    /// Bind address for metrics-rs-tcp-exporter.
    #[arg(long, default_value = "127.0.0.1:5001")]
    metrics_tcp_bind: std::net::SocketAddr,
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();

    info!("Starting telemetry router");
    info!("Command listener on {}", args.cmd_bind);
    info!("Telemetry listener on {}", args.tel_bind);

    if args.metrics_tcp {
        let builder = TcpBuilder::new().listen_address(args.metrics_tcp_bind);
        builder
            .install()
            .expect("failed to install metrics TCP exporter");
    }

    describe_counter!(
        "router.packet.rx",
        Unit::Count,
        "Incoming telemetry packets"
    );
    describe_counter!(
        "router.packet.tx",
        Unit::Count,
        "Forwarded telemetry packets"
    );
    describe_counter!("router.cmd.rx", Unit::Count, "Commands received");
    describe_gauge!(
        "router.clients.count",
        Unit::Count,
        "Number of registered clients"
    );

    let sock_cmd = Arc::new(UdpSocket::bind(&args.cmd_bind).await?);
    let sock_tel = Arc::new(UdpSocket::bind(&args.tel_bind).await?);

    let clients: Arc<Mutex<HashSet<SocketAddr>>> = Arc::new(Mutex::new(HashSet::new()));
    let clients_cmd = clients.clone();
    let sock_cmd_rx = sock_cmd.clone();

    // Command listener task
    tokio::spawn(async move {
        let mut buf = [0u8; 1024];
        loop {
            match sock_cmd_rx.recv_from(&mut buf).await {
                Ok((len, addr)) => {
                    debug!("rx cmd {:02x?}", &buf[0..len]);
                    counter!("router.cmd.rx").increment(1);
                    if len > 0 {
                        if let Some(op) = Opcode::from_u8(buf[0]) {
                            let mut c = clients_cmd.lock().await;
                            let mut log_clients = false;
                            match op {
                                Opcode::Register => {
                                    if c.insert(addr) {
                                        info!("Registered client: {}", addr);
                                        log_clients = true;
                                    }
                                }
                                Opcode::Unregister => {
                                    if c.remove(&addr) {
                                        info!("Unregistered client: {}", addr);
                                        log_clients = true;
                                    }
                                }
                            }
                            if log_clients {
                                info!("Clients: {:?}", c);
                            }
                            gauge!("router.clients.count").set(c.len() as f64);
                        }
                    }
                }
                Err(e) => error!("Command socket error: {}", e),
            }
        }
    });

    // Telemetry forwarder loop
    let mut buf = [0u8; 4096];
    loop {
        tokio::select! {
            res = sock_tel.recv_from(&mut buf) => {
                match res {
                    Ok((len, _addr)) => {
                        trace!("rx tel {:02x?}", &buf[0..len]);
                        counter!("router.packet.rx").increment(1);
                        let data = &buf[0..len];
                        let mut to_remove = Vec::new();
                        let mut c = clients.lock().await;
                        for client in c.iter() {
                            if let Err(e) = sock_cmd.send_to(data, client).await {
                                warn!("Failed to send to {}: {}", client, e);
                                to_remove.push(*client);
                            } else {
                                counter!("router.packet.tx").increment(1);
                            }
                        }

                        let mut log_clients = false;
                        for client in to_remove {
                            if c.remove(&client) {
                                info!("Removed client due to error: {}", client);
                                log_clients = true;
                            }
                            if log_clients {
                                info!("Clients: {:?}", c);
                            }
                        }
                        gauge!("router.clients.count").set(c.len() as f64);
                    }
                    Err(e) => error!("Telemetry socket error: {}", e),
                }
            }
            _ = signal::ctrl_c() => {
                info!("Shutting down");
                break;
            }
        }
    }

    Ok(())
}
