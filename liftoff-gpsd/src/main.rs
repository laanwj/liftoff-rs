use chrono::{DateTime, Utc};
use clap::Parser;
use liftoff_lib::{geo, router_protocol};
use liftoff_lib::telemetry::{self};
use log::{debug, info, warn};
use serde_json::Value;
use std::sync::Arc;
use tokio::io::{AsyncBufReadExt, AsyncWriteExt, BufReader};
use tokio::net::{TcpListener, UdpSocket};
use tokio::time::{Duration, interval};

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Bind address for GPSD service.
    #[arg(long, default_value = "127.0.0.1:2947")]
    gpsd_bind: std::net::SocketAddr,

    /// Address of telemetry router.
    #[arg(long, default_value = "127.0.0.1:9003")]
    telemetry_addr: std::net::SocketAddr,

    /// GPS position update frequency.
    #[arg(short, long, default_value_t = 10)]
    frequency: u64,
}

// NMEA formatting helpers
fn format_nmea(body: &str) -> String {
    let mut checksum = 0u8;
    for b in body.bytes() {
        checksum ^= b;
    }
    format!("${}*{:02X}\r\n", body, checksum)
}

fn to_nmea_coord(val: f64, is_lat: bool) -> (String, char) {
    let abs_val = val.abs();
    let deg = abs_val.floor();
    let min = (abs_val - deg) * 60.0;

    let format_str = if is_lat {
        format!("{:02}{:07.4}", deg as u32, min)
    } else {
        format!("{:03}{:07.4}", deg as u32, min)
    };

    let dir = if is_lat {
        if val >= 0.0 { 'N' } else { 'S' }
    } else {
        if val >= 0.0 { 'E' } else { 'W' }
    };

    (format_str, dir)
}

fn generate_gga(time: DateTime<Utc>, lat: f64, lon: f64, alt: f64, sats: u32) -> String {
    let (lat_str, lat_dir) = to_nmea_coord(lat, true);
    let (lon_str, lon_dir) = to_nmea_coord(lon, false);
    let time_str = time.format("%H%M%S.%3f");

    // $GPGGA,hhmmss.ss,llll.ll,a,yyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
    let body = format!(
        "GPGGA,{},{},{},{},{},1,{:02},0.9,{:.1},M,46.9,M,,",
        time_str, lat_str, lat_dir, lon_str, lon_dir, sats, alt
    );
    format_nmea(&body)
}

fn generate_gga_nofix(time: DateTime<Utc>) -> String {
    let time_str = time.format("%H%M%S.%3f");

    // $GPGGA,hhmmss.ss,,,,,0,00,99.99,,,,,,*hh
    let body = format!("GPGGA,{},,,,,0,00,99.99,,,,,,", time_str);
    format_nmea(&body)
}

fn generate_rmc(time: DateTime<Utc>, lat: f64, lon: f64, speed_knots: f64, course: f64) -> String {
    let (lat_str, lat_dir) = to_nmea_coord(lat, true);
    let (lon_str, lon_dir) = to_nmea_coord(lon, false);
    let time_str = time.format("%H%M%S.%3f");
    let date_str = time.format("%d%m%y");

    // $GPRMC,hhmmss.ss,A,llll.ll,a,yyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
    let body = format!(
        "GPRMC,{},A,{},{},{},{},{:.1},{:.1},{},,,A",
        time_str, lat_str, lat_dir, lon_str, lon_dir, speed_knots, course, date_str
    );
    format_nmea(&body)
}

fn generate_rmc_nofix(time: DateTime<Utc>) -> String {
    let time_str = time.format("%H%M%S.%3f");
    let date_str = time.format("%d%m%y");

    // $GPRMC,hhmmss.ss,V,,,,,,,ddmmyy,,*hh
    let body = format!("GPRMC,{},V,,,,,,,{},,", time_str, date_str);
    format_nmea(&body)
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();

    info!("Starting liftoff-gpsd on {}", args.gpsd_bind);

    // Telemetry Receiver logic
    let sock_tel = UdpSocket::bind("0.0.0.0:0").await?;
    info!("Connecting telemetry to {}", args.telemetry_addr);
    sock_tel.connect(&args.telemetry_addr).await?;

    // Connect to telemetry socket (or send keepalive to router)
    let sock_tel = Arc::new(sock_tel);
    let sock_tel_rx = sock_tel.clone();

    // Shared state for latest telemetry
    let shared_state = Arc::new(std::sync::RwLock::new(None));
    let tx = shared_state.clone();
    let rx = shared_state.clone();

    // Keepalive
    tokio::spawn(async move {
        let mut interval = interval(router_protocol::KEEPALIVE_INTERVAL);
        loop {
            interval.tick().await;
            if let Err(e) = sock_tel.send(&[router_protocol::Opcode::Register as u8]).await {
                warn!("Failed to send keepalive: {}", e);
            }
        }
    });

    // Telemetry Reader
    let config_format = vec![
        "Timestamp".to_string(),
        "Position".to_string(),
        "Attitude".to_string(),
        "Velocity".to_string(),
        "Gyro".to_string(),
        "Input".to_string(),
        "Battery".to_string(),
        "MotorRPM".to_string(),
    ];

    tokio::spawn(async move {
        let mut buf = [0u8; 4096];
        loop {
            match sock_tel_rx.recv(&mut buf).await {
                Ok(len) => {
                    if let Ok(packet) = telemetry::parse_packet(&buf[0..len], &config_format) {
                        if let Ok(mut lock) = tx.write() {
                            // Store packet with timestamp
                            *lock = Some((std::time::Instant::now(), packet));
                        }
                    }
                }
                Err(e) => warn!("Telemetry error: {}", e),
            }
        }
    });

    // TCP Listener for GPSD clients
    let listener = TcpListener::bind(&args.gpsd_bind).await?;

    loop {
        let (mut socket, addr) = listener.accept().await?;
        info!("Accepted connection from {}", addr);
        let rx = rx.clone();
        let freq = args.frequency;

        tokio::spawn(async move {
            let (reader, mut writer) = socket.split();
            let mut reader = BufReader::new(reader);

            // Send banner
            let banner = r#"{"class":"VERSION","release":"2.93","rev":"2010-03-30T12:18:17", "proto_major":3,"proto_minor":2}"#;
            writer
                .write_all(format!("{}\n", banner).as_bytes())
                .await
                .ok();

            // Read ?WATCH command (terminated by ;)
            let mut line_bytes = Vec::new();
            if let Ok(_) = reader.read_until(b';', &mut line_bytes).await {
                let line_raw = String::from_utf8_lossy(&line_bytes);
                let line = line_raw.trim();

                if line.starts_with("?WATCH=") && line.ends_with(';') {
                    let json_str = &line[7..line.len() - 1]; // Strip ?WATCH= and ;

                    let valid = if let Ok(val) = serde_json::from_str::<Value>(json_str) {
                        let enable = val.get("enable").and_then(|v| v.as_bool()).unwrap_or(false);
                        let nmea = val.get("nmea").and_then(|v| v.as_bool()).unwrap_or(false);
                        let raw = val.get("raw").and_then(|v| v.as_bool()).unwrap_or(false);

                        enable && nmea && raw
                    } else {
                        false
                    };

                    if valid {
                        let mut interval = interval(Duration::from_millis(1000 / freq));
                        loop {
                            interval.tick().await;

                            let packet_data = if let Ok(lock) = rx.read() {
                                lock.clone()
                            } else {
                                None
                            };

                            let time = Utc::now();
                            let mut sentences = Vec::<String>::new();
                            let mut have_fix = false;
                            if let Some((recv_time, ref pkt)) = packet_data {
                                if recv_time.elapsed() < Duration::from_secs(10) {
                                    debug!("in {:?}", pkt);
                                    if let (Some(pos), Some(att), Some(vel)) =
                                        (pkt.position, pkt.attitude, pkt.velocity)
                                    {
                                        let (lon, lat, alt) = geo::gps_from_coord(
                                            &[pos[0] as f64, pos[1] as f64, pos[2] as f64],
                                            (0.0, 0.0),
                                        );

                                        // Speed
                                        let vel2d = (vel[0].powi(2) + vel[2].powi(2)).sqrt() as f64; // m/s
                                        let knots = vel2d * 1.94384;
                                        let course = geo::quat2heading(
                                            att[0] as f64,
                                            att[1] as f64,
                                            att[2] as f64,
                                            att[3] as f64,
                                        )
                                        .to_degrees();
                                        let course =
                                            if course < 0.0 { course + 360.0 } else { course };

                                        sentences.push(generate_gga(time, lat, lon, alt, 8));
                                        sentences.push(generate_rmc(time, lat, lon, knots, course));
                                        have_fix = true;
                                    }
                                }
                            }

                            if !have_fix {
                                // Send invalid GPS fix
                                sentences.push(generate_gga_nofix(time));
                                sentences.push(generate_rmc_nofix(time));
                            }

                            for sentence_out in sentences {
                                debug!("out {}", sentence_out);
                                writer.write_all(sentence_out.as_bytes()).await.ok();
                            }
                        }
                    } else {
                        warn!("Invalid WATCH command: {}", line);
                    }
                }
            }
        });
    }
}
