//! Zenoh I/O worker for the simulator.
//!
//! Subscribes to RC topics, routes incoming CRSF frames by packet type
//! (RC → latest-wins slots, anything else → command queue), and
//! publishes outbound telemetry frames on a single topic.
//!
//! Threading model:
//! - All Zenoh I/O runs on a dedicated OS thread with a minimal
//!   multi-thread Tokio runtime (single worker thread).
//! - The Godot physics thread interacts exclusively through lock-free
//!   `Arc<ArcSwap<…>>` slots and an `Arc<Mutex<Vec<…>>>` command queue.
//!   No blocking calls on the physics thread.
//! - Shutdown: setting the `AtomicBool` flag causes the worker to exit
//!   its event loop; the thread is `join()`ed in `Drop`.

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Mutex;
use std::thread::{self, JoinHandle};
use std::time::{Duration, Instant};

use arc_swap::ArcSwap;
use log::{info, warn};
use telemetry_lib::crsf::{self, CrsfPacket};

use crate::crsf_io_trait::{CrsfIo, RcFrame, RC_STREAM_AUTOPILOT, RC_STREAM_DIRECT};

/// Lock-free, single-slot "latest value or empty" cell.
type Slot<T> = Arc<ArcSwap<Option<T>>>;

fn new_slot<T>() -> Slot<T> {
    Arc::new(ArcSwap::from_pointee(None))
}

/// An RC sample stored in a slot: raw CRSF bytes + arrival time.
#[derive(Debug, Clone)]
struct RcSlotEntry {
    data: Vec<u8>,
    received_at: Instant,
}

/// Configuration for one ZenohBus instance.
#[derive(Debug, Clone)]
pub struct ZenohBusConfig {
    /// Topic to subscribe for direct RC commands.
    pub rc_topic: String,
    /// Topic to subscribe for autopilot RC commands.
    pub autopilot_topic: String,
    /// Topic to publish telemetry frames on.
    pub telemetry_topic: String,
    /// Zenoh session mode: `"peer"` or `"client"`.
    pub mode: String,
    /// Optional explicit endpoint (e.g. `"tcp/127.0.0.1:7447"`).
    pub connect: Option<String>,
}

impl Default for ZenohBusConfig {
    fn default() -> Self {
        Self {
            rc_topic: "sim0/crsf/rc".to_string(),
            autopilot_topic: "sim0/crsf/rc/autopilot".to_string(),
            telemetry_topic: "sim0/crsf/telemetry".to_string(),
            mode: "client".to_string(),
            connect: None,
        }
    }
}

/// Handle to a Zenoh I/O worker. Implements `CrsfIo`.
pub struct ZenohBus {
    /// Latest direct RC frame.
    direct_rx: Slot<RcSlotEntry>,
    /// Latest autopilot RC frame.
    autopilot_rx: Slot<RcSlotEntry>,
    /// Queue of non-RC command frames.
    command_queue: Arc<Mutex<VecDeque<Vec<u8>>>>,
    /// Outbound telemetry frames buffered for the worker to publish.
    telemetry_tx: Arc<Mutex<VecDeque<Vec<u8>>>>,

    shutdown: Arc<AtomicBool>,
    worker: Option<JoinHandle<()>>,
}

impl ZenohBus {
    /// Spawn the worker thread. Returns immediately; the Zenoh session
    /// opens asynchronously inside the worker.
    pub fn spawn(cfg: ZenohBusConfig) -> Self {
        let direct_rx = new_slot::<RcSlotEntry>();
        let autopilot_rx = new_slot::<RcSlotEntry>();
        let command_queue: Arc<Mutex<VecDeque<Vec<u8>>>> =
            Arc::new(Mutex::new(VecDeque::new()));
        let telemetry_tx: Arc<Mutex<VecDeque<Vec<u8>>>> =
            Arc::new(Mutex::new(VecDeque::new()));

        let shutdown = Arc::new(AtomicBool::new(false));

        let direct_rx_w = direct_rx.clone();
        let autopilot_rx_w = autopilot_rx.clone();
        let command_queue_w = command_queue.clone();
        let telemetry_tx_w = telemetry_tx.clone();
        let shutdown_w = shutdown.clone();

        let thread_name = format!("gss-zenoh-{}", cfg.rc_topic);
        let worker = thread::Builder::new()
            .name(thread_name.clone())
            .spawn(move || {
                let runtime = match tokio::runtime::Builder::new_multi_thread()
                    .worker_threads(1)
                    .enable_all()
                    .thread_name(format!("{thread_name}-rt"))
                    .build()
                {
                    Ok(rt) => rt,
                    Err(e) => {
                        warn!("ZenohBus: failed to build tokio runtime: {e}");
                        return;
                    }
                };

                runtime.block_on(async move {
                    if let Err(e) = run(
                        cfg,
                        direct_rx_w,
                        autopilot_rx_w,
                        command_queue_w,
                        telemetry_tx_w,
                        shutdown_w,
                    )
                    .await
                    {
                        warn!("ZenohBus worker exited with error: {e}");
                    }
                });
            })
            .expect("failed to spawn zenoh worker thread");

        Self {
            direct_rx,
            autopilot_rx,
            command_queue,
            telemetry_tx,
            shutdown,
            worker: Some(worker),
        }
    }
}

impl CrsfIo for ZenohBus {
    fn poll_rc(&self) -> Vec<RcFrame> {
        let mut frames = Vec::with_capacity(2);
        if let Some(entry) = self.direct_rx.load_full().as_ref() {
            frames.push(RcFrame {
                stream_id: RC_STREAM_DIRECT,
                data: entry.data.clone(),
                received_at: entry.received_at,
            });
        }
        if let Some(entry) = self.autopilot_rx.load_full().as_ref() {
            frames.push(RcFrame {
                stream_id: RC_STREAM_AUTOPILOT,
                data: entry.data.clone(),
                received_at: entry.received_at,
            });
        }
        frames
    }

    fn poll_commands(&self) -> Vec<Vec<u8>> {
        let mut queue = self.command_queue.lock().unwrap();
        queue.drain(..).collect()
    }

    fn send_telemetry(&self, data: &[u8]) {
        let mut queue = self.telemetry_tx.lock().unwrap();
        queue.push_back(data.to_vec());
    }

    fn shutdown(&self) {
        self.shutdown.store(true, Ordering::SeqCst);
    }
}

impl Drop for ZenohBus {
    fn drop(&mut self) {
        self.shutdown.store(true, Ordering::SeqCst);
        if let Some(h) = self.worker.take()
            && let Err(e) = h.join()
        {
            warn!("ZenohBus worker join failed: {e:?}");
        }
    }
}

async fn run(
    cfg: ZenohBusConfig,
    direct_rx: Slot<RcSlotEntry>,
    autopilot_rx: Slot<RcSlotEntry>,
    command_queue: Arc<Mutex<VecDeque<Vec<u8>>>>,
    telemetry_tx: Arc<Mutex<VecDeque<Vec<u8>>>>,
    shutdown: Arc<AtomicBool>,
) -> Result<(), String> {
    let mut zcfg = zenoh::Config::default();
    zcfg.insert_json5("mode", &format!(r#""{}""#, cfg.mode))
        .map_err(|e| format!("zenoh mode insert: {e:?}"))?;
    if let Some(ref endpoint) = cfg.connect {
        zcfg.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))
            .map_err(|e| format!("zenoh endpoint insert: {e:?}"))?;
    }
    let session = zenoh::open(zcfg)
        .await
        .map_err(|e| format!("zenoh open: {e}"))?;

    info!("ZenohBus: subscribing to rc={}", cfg.rc_topic);
    info!("ZenohBus: subscribing to autopilot={}", cfg.autopilot_topic);
    info!("ZenohBus: publishing to telemetry={}", cfg.telemetry_topic);

    let direct_sub = session
        .declare_subscriber(&cfg.rc_topic)
        .await
        .map_err(|e| format!("subscribe rc: {e}"))?;
    let autopilot_sub = session
        .declare_subscriber(&cfg.autopilot_topic)
        .await
        .map_err(|e| format!("subscribe autopilot: {e}"))?;
    let tel_pub = session
        .declare_publisher(&cfg.telemetry_topic)
        .await
        .map_err(|e| format!("declare tel publisher: {e}"))?;

    // Publish interval: drain telemetry buffer at 100 Hz max to avoid
    // starving subscriptions, while keeping latency low.
    let mut publish_interval = tokio::time::interval(Duration::from_millis(10));
    publish_interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

    loop {
        if shutdown.load(Ordering::SeqCst) {
            info!("ZenohBus: shutdown requested");
            break;
        }

        tokio::select! {
            biased;
            _ = tokio::time::sleep(Duration::from_millis(50)) => {
                if shutdown.load(Ordering::SeqCst) { break; }
            }
            res = direct_sub.recv_async() => {
                if let Ok(sample) = res {
                    let payload = sample.payload().to_bytes();
                    handle_inbound(payload.as_ref(), &direct_rx, &command_queue);
                }
            }
            res = autopilot_sub.recv_async() => {
                if let Ok(sample) = res {
                    let payload = sample.payload().to_bytes();
                    handle_inbound(payload.as_ref(), &autopilot_rx, &command_queue);
                }
            }
            _ = publish_interval.tick() => {
                drain_telemetry(&tel_pub, &telemetry_tx).await;
            }
        }
    }

    Ok(())
}

/// Route an inbound CRSF frame: RcChannelsPacked goes to the
/// latest-wins slot; anything else is pushed to the command queue.
fn handle_inbound(
    payload: &[u8],
    rc_slot: &Slot<RcSlotEntry>,
    command_queue: &Arc<Mutex<VecDeque<Vec<u8>>>>,
) {
    // Verify CRC before routing.
    let Some(packet) = crsf::parse_packet_check(payload) else {
        return;
    };
    match packet {
        CrsfPacket::RcChannelsPacked(_) => {
            rc_slot.store(Arc::new(Some(RcSlotEntry {
                data: payload.to_vec(),
                received_at: Instant::now(),
            })));
        }
        _ => {
            if let Ok(mut queue) = command_queue.lock() {
                queue.push_back(payload.to_vec());
            }
        }
    }
}

/// Drain buffered telemetry frames and publish each one.
async fn drain_telemetry(
    tel_pub: &zenoh::pubsub::Publisher<'_>,
    telemetry_tx: &Arc<Mutex<VecDeque<Vec<u8>>>>,
) {
    let frames: Vec<Vec<u8>> = {
        let mut queue = telemetry_tx.lock().unwrap();
        queue.drain(..).collect()
    };
    for f in frames {
        if let Err(e) = tel_pub.put(f).await {
            warn!("ZenohBus publish failed: {e}");
        }
    }
}
