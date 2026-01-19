use clap::Parser;
use evdev::uinput::VirtualDevice;
use evdev::{AbsoluteAxisCode, AttributeSet, InputId, KeyCode, MiscCode, UinputAbsSetup};
use liftoff_lib::crsf::{self, PacketType};
use liftoff_lib::crsf_tx;
use liftoff_lib::router_protocol;
use liftoff_lib::telemetry::{self};
use log::{error, info, trace, warn};
use std::sync::Arc;
use std::time::Duration;
use tokio::net::UdpSocket;
use tokio::sync::Mutex;
use tokio::time::interval;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Bind address for incoming CRSF UDP packets.
    #[arg(long, default_value = "127.0.0.1:9005")]
    bind: std::net::SocketAddr,

    /// Address of telemetry router.
    #[arg(long, default_value = "127.0.0.1:9003")]
    telemetry_addr: std::net::SocketAddr,
}

const TELEMETRY_INTERVAL: Duration = Duration::from_millis(100);

// Axis constants
const AXIS_MAX: u16 = 1983; // 1984 - 1
const AXIS_MID: u16 = 992;
const AXIS_3POS_LEFT: u16 = 592;
const AXIS_3POS_RIGHT: u16 = 1392;

struct InputState {
    old_channels: [u16; 16],
    device: evdev::uinput::VirtualDevice,
}

impl InputState {
    fn new() -> std::io::Result<Self> {
        let mut keys = AttributeSet::<KeyCode>::new();
        for k in [
            KeyCode::BTN_TRIGGER,
            KeyCode::BTN_THUMB,
            KeyCode::BTN_THUMB2,
            KeyCode::BTN_TOP,
            KeyCode::BTN_TOP2,
            KeyCode::BTN_PINKIE,
            KeyCode::BTN_BASE,
            KeyCode::BTN_BASE2,
            KeyCode::BTN_BASE3,
            KeyCode::BTN_BASE4,
            KeyCode::BTN_BASE5,
            KeyCode::BTN_BASE6,
            KeyCode::new(KeyCode::BTN_BASE6.code() + 1), // 0x12d
        ] {
            keys.insert(k);
        }

        let abs_setup = UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_X,
            evdev::AbsInfo::new(0, 0, AXIS_MAX.into(), 7, 127, 0),
        );
        let abs_y = UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_Y,
            evdev::AbsInfo::new(0, 0, AXIS_MAX.into(), 7, 127, 0),
        );
        let abs_z = UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_Z,
            evdev::AbsInfo::new(0, 0, AXIS_MAX.into(), 7, 127, 0),
        );
        let abs_rx = UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_RX,
            evdev::AbsInfo::new(0, 0, AXIS_MAX.into(), 7, 127, 0),
        );
        let abs_throttle = UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_THROTTLE,
            evdev::AbsInfo::new(0, 0, AXIS_MAX.into(), 7, 127, 0),
        );
        let abs_rudder = UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_RUDDER,
            evdev::AbsInfo::new(0, 0, AXIS_MAX.into(), 7, 127, 0),
        );
        let abs_wheel = UinputAbsSetup::new(
            AbsoluteAxisCode::ABS_WHEEL,
            evdev::AbsInfo::new(0, 0, AXIS_MAX.into(), 7, 127, 0),
        );

        let mut msc_set = AttributeSet::<MiscCode>::new();
        msc_set.insert(MiscCode::MSC_SCAN);

        let device = VirtualDevice::builder()?
            .name("CRSF Joystick")
            .input_id(InputId::new(evdev::BusType::BUS_USB, 0x1209, 0x4f54, 0)) // Radiomaster Pocket vendor/product
            .with_keys(&keys)?
            .with_absolute_axis(&abs_setup)?
            .with_absolute_axis(&abs_y)?
            .with_absolute_axis(&abs_z)?
            .with_absolute_axis(&abs_rx)?
            .with_absolute_axis(&abs_throttle)?
            .with_absolute_axis(&abs_rudder)?
            .with_absolute_axis(&abs_wheel)?
            .with_msc(&msc_set)?
            .build()?;

        Ok(Self {
            old_channels: [0xffff; 16], // Different initial value to force update
            device,
        })
    }

    fn update(&mut self, channels: [u16; 16]) -> std::io::Result<()> {
        let dev = &mut self.device;
        let old = self.old_channels;

        // 0 AIL (ABS_X)
        if channels[0] != old[0] {
            dev.emit(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_X.0,
                channels[0] as i32,
            )])?;
        }
        // 1 ELE (ABS_Y)
        if channels[1] != old[1] {
            dev.emit(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_Y.0,
                channels[1] as i32,
            )])?;
        }
        // 2 THR (ABS_Z)
        if channels[2] != old[2] {
            dev.emit(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_Z.0,
                channels[2] as i32,
            )])?;
        }
        // 3 RUD (ABS_RX)
        if channels[3] != old[3] {
            dev.emit(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_RX.0,
                channels[3] as i32,
            )])?;
        }

        // 4 SD disarm/arm button(s) + ABS_THROTTLE
        if channels[4] != old[4] {
            let val = channels[4] as i32;
            dev.emit(&[
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_TRIGGER.0,
                    if channels[4] < AXIS_MID { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_THUMB.0,
                    if channels[4] >= AXIS_MID { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::ABSOLUTE.0,
                    AbsoluteAxisCode::ABS_THROTTLE.0,
                    val,
                ),
            ])?;
        }

        // 5 button SE (2POS, momentary) -> BTN_THUMB2
        if channels[5] != old[5] {
            dev.emit(&[evdev::InputEvent::new(
                evdev::EventType::KEY.0,
                KeyCode::BTN_THUMB2.0,
                if channels[5] >= AXIS_MID { 1 } else { 0 },
            )])?;
        }

        // 6 S1-pot -> ABS_RUDDER
        if channels[6] != old[6] {
            dev.emit(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_RUDDER.0,
                channels[6] as i32,
            )])?;
        }

        // 7 button SA (2POS, fixed) -> BTN_BASE6 / BTN_BASE6+1 + ABS_WHEEL
        if channels[7] != old[7] {
            let val = channels[7] as i32;
            dev.emit(&[
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_BASE6.0,
                    if channels[7] < AXIS_MID { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::new(KeyCode::BTN_BASE6.code() + 1).0,
                    if channels[7] >= AXIS_MID { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::ABSOLUTE.0,
                    AbsoluteAxisCode::ABS_WHEEL.0,
                    val,
                ),
            ])?;
        }

        // 8: RUD trim
        if channels[8] != old[8] {
            dev.emit(&[
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_TOP.0,
                    if channels[8] <= AXIS_3POS_LEFT { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_TOP2.0,
                    if channels[8] >= AXIS_3POS_RIGHT { 1 } else { 0 },
                ),
            ])?;
        }
        // 9: ELE trim
        if channels[9] != old[9] {
            dev.emit(&[
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_PINKIE.0,
                    if channels[9] <= AXIS_3POS_LEFT { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_BASE.0,
                    if channels[9] >= AXIS_3POS_RIGHT { 1 } else { 0 },
                ),
            ])?;
        }
        // 10: THR trim
        if channels[10] != old[10] {
            dev.emit(&[
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_BASE2.0,
                    if channels[10] <= AXIS_3POS_LEFT { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_BASE3.0,
                    if channels[10] >= AXIS_3POS_RIGHT {
                        1
                    } else {
                        0
                    },
                ),
            ])?;
        }
        // 11: AIL trim
        if channels[11] != old[11] {
            dev.emit(&[
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_BASE4.0,
                    if channels[11] <= AXIS_3POS_LEFT { 1 } else { 0 },
                ),
                evdev::InputEvent::new(
                    evdev::EventType::KEY.0,
                    KeyCode::BTN_BASE5.0,
                    if channels[11] >= AXIS_3POS_RIGHT {
                        1
                    } else {
                        0
                    },
                ),
            ])?;
        }

        self.old_channels = channels;
        Ok(())
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    let args = Args::parse();

    info!("Starting liftoff-input on {}", args.bind);

    let sock_crsf = Arc::new(UdpSocket::bind(&args.bind).await?);
    let sock_tel = UdpSocket::bind("0.0.0.0:0").await?; // Bind to random port for telemetry return

    // Connect telemetry socket to Liftoff/Router
    info!("Connecting telemetry to {}", args.telemetry_addr);
    sock_tel.connect(&args.telemetry_addr).await?;

    // Create uinput device
    // NOTE: This requires permission to write to /dev/uinput
    let input_state = Arc::new(Mutex::new(InputState::new()?));

    let sock_crsf_clone = sock_crsf.clone();
    let target_addr = Arc::new(Mutex::new(None));
    let target_addr_clone = target_addr.clone();

    // Task: Telemetry Keepalive and Forwarding
    let sock_tel = Arc::new(sock_tel);
    let sock_tel_rx = sock_tel.clone();

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

    // Telemetry Receive & Forward
    // We assume default configuration for now
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
        let mut next_send = tokio::time::Instant::now();
        loop {
            match sock_tel_rx.recv(&mut buf).await {
                Ok(len) => {
                    trace!("rx tel {:02x?}", &buf[0..len]);
                    let now = tokio::time::Instant::now();
                    if now >= next_send {
                        if let Ok(packet) = telemetry::parse_packet(&buf[0..len], &config_format) {
                            // Generate CRSF packets
                            let crsf_packets = crsf_tx::generate_crsf_telemetry(&packet);
                            let target = *target_addr_clone.lock().await;
                            if let Some(addr) = target {
                                for pkt in crsf_packets {
                                    trace!("tx tel {:02x?}", &pkt);
                                    if let Err(e) = sock_crsf_clone.send_to(&pkt, addr).await {
                                        warn!("Failed to forward CRSF telem: {}", e);
                                    }
                                }
                            }
                            next_send = now + TELEMETRY_INTERVAL;
                        }
                    }
                }
                Err(e) => warn!("Telemetry recv error: {}", e),
            }
        }
    });

    // Main loop: Receive CRSF RC channels
    let mut buf = [0u8; 64];
    loop {
        let (len, addr) = sock_crsf.recv_from(&mut buf).await?;

        // Update target address for return telemetry
        let mut t = target_addr.lock().await;
        if *t != Some(addr) {
            info!("New client connected: {}", addr);
            *t = Some(addr);
        }
        drop(t);

        trace!("rx crsf {:02x?}", &buf[0..len]);

        if len > 1 {
            // Check packet type.
            // remote_input_server_crsf.py: handle_crsf_packet(devices, data[0], data[1:])
            let type_byte = buf[0];
            let payload = &buf[1..len];

            if type_byte == PacketType::RcChannelsPacked as u8 {
                if let Some(channels) = crsf::unpack_channels(payload) {
                    // Check range
                    if channels.iter().any(|&c| c > AXIS_MAX) {
                        warn!("Channel out of range: {:?}", channels);
                        continue;
                    }

                    let mut state = input_state.lock().await;
                    if let Err(e) = state.update(channels) {
                        error!("Failed to update uinput: {}", e);
                    }
                } else {
                    warn!("Channels packet has wrong size");
                }
            }
        }
    }
}
