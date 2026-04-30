use clap::Parser;
use evdev::uinput::VirtualDevice;
use evdev::{AbsoluteAxisCode, AttributeSet, InputId, KeyCode, MiscCode, UinputAbsSetup};
use telemetry_lib::crsf::{self, CrsfPacket};
use telemetry_lib::crsf_custom;
use telemetry_lib::crsf_tx;
use telemetry_lib::simstate::{self, BatteryPacket, DamagePacket, SimstatePacket};
use telemetry_lib::telemetry::{self};
use telemetry_lib::topics;
use log::{error, info, trace, warn};
use metrics::{Unit, counter, describe_counter};
use metrics_exporter_tcp::TcpBuilder;
use std::sync::Arc;
use std::time::Duration;
use tokio::net::UdpSocket;
use tokio::sync::{Mutex, Notify};
use zenoh::Config;

#[derive(Parser, Debug)]
#[command(author, version, about, long_about = None)]
struct Args {
    /// Bind address for simulator telemetry UDP.
    #[arg(long, default_value = "127.0.0.1:9001")]
    sim_bind: std::net::SocketAddr,

    /// Bind address for the liftoff-simstate-bridge UDP stream
    /// (per-prop damage + battery telemetry from the BepInEx plugin).
    #[arg(long, default_value = "127.0.0.1:9020")]
    simstate_bind: std::net::SocketAddr,

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
    #[arg(long, default_value = "127.0.0.1:5002")]
    metrics_tcp_bind: std::net::SocketAddr,
}

const TELEMETRY_INTERVAL: Duration = Duration::from_millis(100);
const DAMAGE_HEARTBEAT_INTERVAL: Duration = Duration::from_secs(1);

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
        let mut events = Vec::<evdev::InputEvent>::new();
        let dev = &mut self.device;
        let old = self.old_channels;

        // 0 AIL (ABS_X)
        if channels[0] != old[0] {
            events.extend(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_X.0,
                channels[0] as i32,
            )]);
        }
        // 1 ELE (ABS_Y)
        if channels[1] != old[1] {
            events.extend(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_Y.0,
                channels[1] as i32,
            )]);
        }
        // 2 THR (ABS_Z)
        if channels[2] != old[2] {
            events.extend(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_Z.0,
                channels[2] as i32,
            )]);
        }
        // 3 RUD (ABS_RX)
        if channels[3] != old[3] {
            events.extend(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_RX.0,
                channels[3] as i32,
            )]);
        }

        // 4 SD disarm/arm button(s) + ABS_THROTTLE
        if channels[4] != old[4] {
            let val = channels[4] as i32;
            events.extend(&[
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
            ]);
        }

        // 5 button SE (2POS, momentary) -> BTN_THUMB2
        if channels[5] != old[5] {
            events.extend(&[evdev::InputEvent::new(
                evdev::EventType::KEY.0,
                KeyCode::BTN_THUMB2.0,
                if channels[5] >= AXIS_MID { 1 } else { 0 },
            )]);
        }

        // 6 S1-pot -> ABS_RUDDER
        if channels[6] != old[6] {
            events.extend(&[evdev::InputEvent::new(
                evdev::EventType::ABSOLUTE.0,
                AbsoluteAxisCode::ABS_RUDDER.0,
                channels[6] as i32,
            )]);
        }

        // 7 button SA (2POS, fixed) -> BTN_BASE6 / BTN_BASE6+1 + ABS_WHEEL
        if channels[7] != old[7] {
            let val = channels[7] as i32;
            events.extend(&[
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
            ]);
        }

        // 8: RUD trim
        if channels[8] != old[8] {
            events.extend(&[
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
            ]);
        }
        // 9: ELE trim
        if channels[9] != old[9] {
            events.extend(&[
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
            ]);
        }
        // 10: THR trim
        if channels[10] != old[10] {
            events.extend(&[
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
            ]);
        }
        // 11: AIL trim
        if channels[11] != old[11] {
            events.extend(&[
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
            ]);
        }

        self.old_channels = channels;

        if !events.is_empty() {
            counter!("input.uinput.update").increment(1);
            dev.emit(&events)?;
        }
        Ok(())
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::init();
    let args = Args::parse();

    info!("Starting liftoff-input");

    if args.metrics_tcp {
        let builder = TcpBuilder::new().listen_address(args.metrics_tcp_bind);
        builder
            .install()
            .expect("failed to install metrics TCP exporter");
    }

    describe_counter!("input.crsf.rx", Unit::Count, "CRSF packets received");
    describe_counter!(
        "input.crsf.rx_rc_channels",
        Unit::Count,
        "CRSF RC_CHANNELS packets received"
    );
    describe_counter!(
        "input.telemetry.rx",
        Unit::Count,
        "Telemetry packets received"
    );
    describe_counter!(
        "input.telemetry.tx",
        Unit::Count,
        "CRSF telemetry packets sent"
    );
    describe_counter!(
        "input.uinput.update",
        Unit::Count,
        "Updates to virtual input device"
    );
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
    describe_counter!(
        "simstate.damage.rx",
        Unit::Count,
        "Damage UDP packets received from simstate-bridge"
    );
    describe_counter!(
        "simstate.battery.rx",
        Unit::Count,
        "Battery UDP packets received from simstate-bridge"
    );
    describe_counter!(
        "simstate.parse_error",
        Unit::Count,
        "Malformed simstate UDP packets"
    );

    // Zenoh session
    let mut config = Config::default();
    config.insert_json5("mode", &format!(r#""{}""#, args.zenoh_mode))?;
    if let Some(ref endpoint) = args.zenoh_connect {
        config.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))?;
    }

    let session = zenoh::open(config).await?;

    let tel_topic = topics::topic(&args.zenoh_prefix, topics::TELEMETRY);
    let crsf_tel_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_TELEMETRY);
    let crsf_rc_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_RC);
    let crsf_rc_ap_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_RC_AUTOPILOT);
    let damage_topic = topics::topic(&args.zenoh_prefix, topics::DAMAGE);
    let battery_topic = topics::topic(&args.zenoh_prefix, topics::BATTERY);

    info!("Subscribing to: {}", tel_topic);
    info!("Publishing on: {}", crsf_tel_topic);
    info!("Subscribing to: {} (manual)", crsf_rc_topic);
    info!("Subscribing to: {} (autopilot)", crsf_rc_ap_topic);
    info!("Publishing on: {} (simstate damage)", damage_topic);
    info!("Publishing on: {} (simstate battery)", battery_topic);

    let crsf_tel_publisher = session.declare_publisher(crsf_tel_topic).await?;
    let tel_subscriber = session.declare_subscriber(&tel_topic).await?;
    let rc_subscriber = session.declare_subscriber(&crsf_rc_topic).await?;
    let rc_ap_subscriber = session.declare_subscriber(&crsf_rc_ap_topic).await?;
    let damage_publisher = session.declare_publisher(damage_topic).await?;
    let battery_publisher = session.declare_publisher(battery_topic).await?;

    // Shared latest battery snapshot from the simstate UDP stream. The CRSF
    // generation task reads from here when building BatterySensor + Voltages
    // packets so they carry real current and per-cell voltage instead of
    // just voltage+percent from the sim's standard telemetry.
    let battery_state: Arc<Mutex<Option<BatteryPacket>>> = Arc::new(Mutex::new(None));

    // Shared latest damage snapshot and a Notify to trigger an immediate
    // CRSF damage frame when damage state changes.
    let damage_state: Arc<Mutex<Option<DamagePacket>>> = Arc::new(Mutex::new(None));
    let damage_notify: Arc<Notify> = Arc::new(Notify::new());

    // simstate-bridge UDP listener: forwards raw bytes to the corresponding
    // Zenoh topic and updates the shared battery snapshot.
    let simstate_sock = UdpSocket::bind(args.simstate_bind).await?;
    info!("Bridge: simstate-bridge UDP on {}", args.simstate_bind);
    {
        let battery_state = battery_state.clone();
        let damage_state = damage_state.clone();
        let damage_notify = damage_notify.clone();
        tokio::spawn(async move {
            let mut buf = [0u8; 1024];
            loop {
                match simstate_sock.recv_from(&mut buf).await {
                    Ok((len, _addr)) => {
                        let payload = &buf[..len];
                        match simstate::parse_packet(payload) {
                            Ok(SimstatePacket::Damage(dmg)) => {
                                counter!("simstate.damage.rx").increment(1);
                                // Check if damage values actually changed.
                                let changed = {
                                    let mut guard = damage_state.lock().await;
                                    let changed = guard
                                        .as_ref()
                                        .map(|prev| prev.damage != dmg.damage || prev.flags != dmg.flags)
                                        .unwrap_or(true);
                                    *guard = Some(dmg);
                                    changed
                                };
                                if changed {
                                    damage_notify.notify_one();
                                }
                                if let Err(e) = damage_publisher.put(payload).await {
                                    warn!("Failed to publish damage: {}", e);
                                }
                            }
                            Ok(SimstatePacket::Battery(bat)) => {
                                counter!("simstate.battery.rx").increment(1);
                                *battery_state.lock().await = Some(bat);
                                if let Err(e) = battery_publisher.put(payload).await {
                                    warn!("Failed to publish battery: {}", e);
                                }
                            }
                            Err(e) => {
                                counter!("simstate.parse_error").increment(1);
                                warn!("simstate parse error: {} (len={})", e, len);
                            }
                        }
                    }
                    Err(e) => {
                        error!("simstate UDP recv error: {}", e);
                    }
                }
            }
        });
    }

    // Bridge task: receive sim UDP telemetry and publish to Zenoh
    let bridge_publisher = session.declare_publisher(tel_topic.clone()).await?;
    let sock = UdpSocket::bind(args.sim_bind).await?;
    info!("Bridge: simulator telemetry on {}", args.sim_bind);
    tokio::spawn(async move {
        let mut buf = [0u8; 4096];
        loop {
            match sock.recv_from(&mut buf).await {
                Ok((len, _addr)) => {
                    trace!("rx sim {} bytes", len);
                    counter!("bridge.packet.rx").increment(1);
                    if let Err(e) = bridge_publisher.put(&buf[..len]).await {
                        warn!("Failed to publish sim telemetry: {}", e);
                    } else {
                        counter!("bridge.packet.tx").increment(1);
                    }
                }
                Err(e) => {
                    error!("UDP recv error: {}", e);
                }
            }
        }
    });

    // Create uinput device
    // NOTE: This requires permission to write to /dev/uinput
    let input_state = Arc::new(Mutex::new(InputState::new()?));

    // Telemetry format config
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

    // Task: Receive raw telemetry from bridge, convert to CRSF, publish.
    // Also listens for damage-change notifications to send an immediate
    // damage frame, and includes a 1 Hz damage heartbeat.
    let crsf_tel_pub = crsf_tel_publisher;
    let crsf_battery_state = battery_state.clone();
    let crsf_damage_state = damage_state.clone();
    let crsf_damage_notify = damage_notify.clone();
    tokio::spawn(async move {
        let mut next_send = tokio::time::Instant::now();
        let mut next_damage_heartbeat = tokio::time::Instant::now();

        /// Publish a single CRSF frame, logging and counting on success.
        async fn send_frame(
            pub_: &zenoh::pubsub::Publisher<'_>,
            pkt: &[u8],
        ) {
            trace!("tx crsf tel {} bytes", pkt.len());
            if let Err(e) = pub_.put(pkt).await {
                warn!("Failed to publish CRSF telem: {}", e);
            } else {
                counter!("input.telemetry.tx").increment(1);
            }
        }

        loop {
            tokio::select! {
                // Normal telemetry path: rate-limited to TELEMETRY_INTERVAL.
                result = tel_subscriber.recv_async() => {
                    match result {
                        Ok(sample) => {
                            let payload = sample.payload().to_bytes();
                            trace!("rx tel {} bytes", payload.len());
                            counter!("input.telemetry.rx").increment(1);
                            let now = tokio::time::Instant::now();
                            if now >= next_send {
                                if let Ok(packet) =
                                    telemetry::parse_packet(&payload, &config_format)
                                {
                                    let bat_snapshot = crsf_battery_state.lock().await.clone();
                                    let crsf_packets =
                                        crsf_tx::generate_crsf_telemetry(&packet, bat_snapshot.as_ref());
                                    for pkt in &crsf_packets {
                                        send_frame(&crsf_tel_pub, pkt).await;
                                    }

                                    // Include damage heartbeat at 1 Hz alongside
                                    // the normal telemetry batch.
                                    if now >= next_damage_heartbeat {
                                        let dmg_snapshot = crsf_damage_state.lock().await.clone();
                                        if let Some(frame) = dmg_snapshot.and_then(|d| crsf_custom::build_damage_packet(&d)) {
                                            send_frame(&crsf_tel_pub, &frame).await;
                                        }
                                        next_damage_heartbeat = now + DAMAGE_HEARTBEAT_INTERVAL;
                                    }

                                    next_send = now + TELEMETRY_INTERVAL;
                                }
                            }
                        }
                        Err(e) => {
                            warn!("Telemetry subscriber error: {}", e);
                            break;
                        }
                    }
                }

                // Immediate damage path: fires when damage state changes.
                _ = crsf_damage_notify.notified() => {
                    let dmg_snapshot = crsf_damage_state.lock().await.clone();
                    if let Some(frame) = dmg_snapshot.and_then(|d| crsf_custom::build_damage_packet(&d)) {
                        send_frame(&crsf_tel_pub, &frame).await;
                    }
                    // Reset heartbeat timer so we don't double-send.
                    next_damage_heartbeat = tokio::time::Instant::now() + DAMAGE_HEARTBEAT_INTERVAL;
                }
            }
        }
    });

    // Mux state: track manual radio presence and SA switch position
    let manual_timeout = Duration::from_millis(500);
    let mut last_manual_time: Option<tokio::time::Instant> = None;
    let mut last_manual_ch7: u16 = 0; // SA switch, low = manual
    let mut active_source = "none";

    // Main loop: Mux between manual (crsf/rc) and autopilot (crsf/rc/autopilot) frames
    loop {
        let (payload, source) = tokio::select! {
            result = rc_subscriber.recv_async() => {
                match result {
                    Ok(sample) => (sample.payload().to_bytes().to_vec(), "manual"),
                    Err(e) => {
                        error!("RC subscriber error: {}", e);
                        break;
                    }
                }
            }
            result = rc_ap_subscriber.recv_async() => {
                match result {
                    Ok(sample) => (sample.payload().to_bytes().to_vec(), "autopilot"),
                    Err(e) => {
                        error!("RC autopilot subscriber error: {}", e);
                        break;
                    }
                }
            }
        };

        trace!("rx crsf ({}) {:02x?}", source, &*payload);
        counter!("input.crsf.rx").increment(1);

        if let Some(CrsfPacket::RcChannelsPacked(channels)) =
            crsf::parse_packet_check(&payload)
        {
            counter!("input.crsf.rx_rc_channels").increment(1);
            if channels.channels.iter().any(|&c| c > AXIS_MAX) {
                warn!("Channel out of range: {:?}", channels.channels);
                continue;
            }

            // Update manual tracking state
            if source == "manual" {
                last_manual_time = Some(tokio::time::Instant::now());
                last_manual_ch7 = channels.channels[7];
            }

            // Determine selected source from mux state
            let manual_active = last_manual_time
                .map(|t| t.elapsed() < manual_timeout)
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

            // Apply frame if it matches the selected source
            if source == selected {
                let mut state = input_state.lock().await;
                if let Err(e) = state.update(channels.channels) {
                    error!("Failed to update uinput: {}", e);
                }
            }
        }
    }

    session.close().await?;
    Ok(())
}
