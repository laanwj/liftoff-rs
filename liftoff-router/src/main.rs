use clap::Parser;
use liftoff_lib::router_protocol::Opcode;
use log::{debug, error, info, trace, warn};
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
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();

    info!("Starting telemetry router");
    info!("Command listener on {}", args.cmd_bind);
    info!("Telemetry listener on {}", args.tel_bind);

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
                        let data = &buf[0..len];
                        let mut to_remove = Vec::new();
                        let mut c = clients.lock().await;
                        for client in c.iter() {
                            if let Err(e) = sock_cmd.send_to(data, client).await {
                                warn!("Failed to send to {}: {}", client, e);
                                to_remove.push(*client);
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
