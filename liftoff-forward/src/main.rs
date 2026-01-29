use clap::Parser;
use liftoff_lib::crsf::{self};
use log::{error, info, trace, warn};
use metrics::{Unit, counter, describe_counter, describe_histogram, histogram};
use metrics_exporter_tcp::TcpBuilder;
use std::sync::Arc;
use tokio::io::{AsyncReadExt, AsyncWriteExt};
use tokio::net::UdpSocket;
use tokio_serial::SerialPortBuilderExt;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Serial port to use.
    #[arg(short, long, default_value = "/dev/ttyUSB0")]
    port: String,

    /// Serial baudrate to use.
    #[arg(short, long, default_value_t = 420000)]
    baud: u32,

    /// Destination for UDP packets.
    #[arg(long, default_value = "127.0.0.1:9005")]
    dest: std::net::SocketAddr,

    /// Source (bind) for UDP packets.
    #[arg(long, default_value = "127.0.0.1:9006")]
    src: std::net::SocketAddr,

    /// Enable metrics reporting using metrics-rs-tcp-exporter.
    #[arg(long, default_value_t = false)]
    metrics_tcp: bool,

    /// Bind address for metrics-rs-tcp-exporter.
    #[arg(long, default_value = "127.0.0.1:5000")]
    metrics_tcp_bind: std::net::SocketAddr,
}

async fn udp_forward_loop(rx: Arc<UdpSocket>, tx: tokio::sync::mpsc::Sender<Vec<u8>>, name: &str) {
    let mut buf = [0u8; 64];
    loop {
        match rx.recv(&mut buf).await {
            Ok(len) => {
                let packet = buf[0..len].to_vec();
                if tx.send(packet).await.is_err() {
                    break;
                }
            }
            Err(e) => {
                error!("UDP {} recv error: {}", name, e);
                break;
            }
        }
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();

    if args.metrics_tcp {
        let builder = TcpBuilder::new().listen_address(args.metrics_tcp_bind);
        builder
            .install()
            .expect("failed to install metrics TCP exporter");
    }

    describe_counter!(
        "crsf.tx.count",
        Unit::Count,
        "Sent telemetry CRSF packet count"
    );
    describe_counter!("crsf.rx.count", Unit::Count, "Received CRSF packet count");
    describe_counter!(
        "crsf.rx.valid",
        Unit::Count,
        "Valid received CRSF packet count"
    );
    describe_counter!(
        "crsf.rx.crc_err",
        Unit::Count,
        "Number of received CRSF packets with CRC mismatch"
    );
    describe_histogram!("crsf.rx.packet_size", Unit::Bytes, "Receive packet size");
    describe_histogram!(
        "crsf.tx.packet_size",
        Unit::Bytes,
        "Sent telemetry packet size"
    );

    info!("Starting liftoff-forward");
    info!("Serial Port: {} @ {}", args.port, args.baud);
    info!("UDP Dest: {}", args.dest);
    info!("UDP Src: {}", args.src);

    let port = tokio_serial::new(&args.port, args.baud).open_native_async()?;

    // UDP Sockets
    let sock = UdpSocket::bind("0.0.0.0:0").await?;
    sock.connect(&args.dest).await?;

    let tel_sock = UdpSocket::bind(&args.src).await?;

    // Wrap sockets in Arc
    let sock = Arc::new(sock);
    let tel_sock = Arc::new(tel_sock);

    let (mut reader, mut writer) = tokio::io::split(port);
    let (tx, mut rx) = tokio::sync::mpsc::channel::<Vec<u8>>(32);

    // Task 1a: Telemetry Socket -> Channel
    let tel_sock_rx = tel_sock.clone();
    let tx_1 = tx.clone();
    tokio::spawn(async move {
        udp_forward_loop(tel_sock_rx, tx_1, "tel").await;
    });

    // Task 1b: Destination Socket (Return traffic) -> Channel
    let sock_rx = sock.clone();
    let tx_2 = tx.clone();
    tokio::spawn(async move {
        udp_forward_loop(sock_rx, tx_2, "dest").await;
    });

    // Task 1c: Channel -> Serial
    let mut writer_handle = tokio::spawn(async move {
        while let Some(packet) = rx.recv().await {
            let frame_size = packet.len() + 3; // Add address, length and CRC
            if frame_size > crsf::MAX_FRAME_SIZE {
                warn!("Packet too large: {}", packet.len());
                continue;
            }

            let mut frame = Vec::with_capacity(frame_size);
            frame.push(crsf::device_address::FLIGHT_CONTROLLER); // We are the flight controller in this context.
            frame.push((packet.len() + 1) as u8); // Add one byte for CRC
            frame.extend_from_slice(&packet);

            let crc = crsf::calc_crc8(&packet);
            frame.push(crc);

            trace!("tx: {:02x?}", frame);
            counter!("crsf.tx.count").increment(1);
            histogram!("crsf.tx.packet_size").record(frame.len() as f64);

            if let Err(e) = writer.write_all(&frame).await {
                error!("Serial write error: {}", e);
                break;
            }
        }
    });

    // Task 2: Serial -> UDP (Input)
    // Needs framing logic.
    let mut reader_handle = tokio::spawn(async move {
        let mut buf = Vec::new(); // Buffer for incoming data
        let mut tmp = [0u8; 1024];

        loop {
            match reader.read(&mut tmp).await {
                Ok(0) => {
                    // EOF
                    error!("Serial EOF");
                    break;
                }
                Ok(n) => {
                    buf.extend_from_slice(&tmp[0..n]);

                    // Process buffer
                    loop {
                        // Find sync byte (we are the flight controller, in this context).
                        if let Some(pos) = buf
                            .iter()
                            .position(|&b| b == crsf::device_address::FLIGHT_CONTROLLER)
                        {
                            // Trim garbage before sync
                            if pos > 0 {
                                buf.drain(0..pos);
                            }

                            // Check length
                            if buf.len() < 2 {
                                break; // Need more data
                            }
                            let len = buf[1] as usize; // Length of Payload + CRC
                            let total_len = len + 2; // Sync + Len + Payload + CRC

                            if total_len > crsf::MAX_FRAME_SIZE {
                                // "Each CRSF frame is not longer than 64 bytes (including the Sync and CRC bytes)"
                                // This packet would be too long. Drop sync byte and try again.
                                buf.remove(0);
                                continue;
                            }
                            if buf.len() < total_len {
                                break; // Need more data
                            }
                            counter!("crsf.rx.count").increment(1);
                            histogram!("crsf.rx.packet_size").record(total_len as f64);

                            // Full packet found
                            let frame = &buf[0..total_len];
                            // Verify CRC
                            let payload = &frame[2..total_len - 1];
                            let crc_byte = frame[total_len - 1];

                            if crsf::calc_crc8(payload) == crc_byte {
                                // Valid packet
                                trace!("rx: {:02x?}", payload);
                                counter!("crsf.rx.valid").increment(1);
                                if let Err(e) = sock.send(payload).await {
                                    warn!("UDP send error: {}", e);
                                }
                            } else {
                                trace!("CRC mismatch");
                                counter!("crsf.rx.crc_err").increment(1);
                            }

                            buf.drain(0..total_len);
                        } else {
                            // No sync found, clear buffer
                            buf.clear();
                            break;
                        }
                    }
                }
                Err(e) => {
                    error!("Serial read error: {}", e);
                    break;
                }
            }
        }
    });

    tokio::select! {
        _ = &mut writer_handle => error!("Writer task finished"),
        _ = &mut reader_handle => error!("Reader task finished"),
    }

    Ok(())
}
