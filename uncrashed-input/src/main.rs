//! File-IPC → Zenoh bridge for the Uncrashed UE4SS Lua telemetry mod.
//!
//! Pairs with `uncrashed-telemetry-mod/`, a Lua mod that runs inside the
//! Uncrashed game process and overwrites a single binary file with the
//! latest `UCFV` packet on every game tick. This bridge:
//!
//! 1. Polls the IPC file on a fixed cadence.
//! 2. Reads `wire::PACKET_SIZE` bytes; short reads (only seen during the
//!    pre-first-write window) are skipped silently.
//! 3. Parses via [`wire::parse`].
//! 4. Skips Zenoh publication when the sequence number hasn't changed
//!    (poll rate > tick rate, no new data).
//! 5. Otherwise: publishes liftoff-flavoured CRSF telemetry frames on
//!    `{prefix}/crsf/telemetry` — standard frames from
//!    `liftoff_lib::crsf_tx::generate_crsf_telemetry` plus the custom
//!    0x42 per-prop damage frame from `liftoff_lib::crsf_custom`.
mod wire;

use std::io::SeekFrom;
use std::time::Duration;

use clap::Parser;
use liftoff_lib::{crsf_custom, crsf_tx, topics};
use log::{debug, error, info, warn};
use tokio::fs::File;
use tokio::io::{AsyncReadExt, AsyncSeekExt};
use zenoh::Config;

#[derive(Parser, Debug)]
#[command(version, about)]
struct Args {
    /// Path to the IPC file the Lua mod overwrites every tick. The default
    /// matches the Lua mod's `Z:\tmp\uncrashed-telemetry.bin` (Wine maps
    /// `Z:` to the host filesystem root).
    #[arg(long, default_value = "/tmp/uncrashed-telemetry.bin")]
    input_file: String,

    /// Poll interval, milliseconds. The mod ticks at ~60 Hz (16 ms);
    /// matching that catches every fresh frame without busy-spinning.
    #[arg(long, default_value = "16")]
    poll_ms: u64,

    /// Zenoh connect endpoint (e.g. `tcp/127.0.0.1:7447`). Optional.
    #[arg(long)]
    zenoh_connect: Option<String>,

    /// Zenoh session mode: `peer` or `client`.
    #[arg(long, default_value = "peer")]
    zenoh_mode: String,

    /// Topic prefix; defaults match the rest of the workspace.
    #[arg(long, default_value_t = topics::DEFAULT_PREFIX.to_string())]
    zenoh_prefix: String,
}

#[tokio::main(flavor = "multi_thread", worker_threads = 1)]
async fn main() -> Result<(), Box<dyn std::error::Error + Send + Sync>> {
    env_logger::init();
    let args = Args::parse();

    info!("Starting uncrashed-input");
    info!("IPC file: {}", args.input_file);
    info!("Poll interval: {} ms", args.poll_ms);

    let mut config = Config::default();
    config.insert_json5("mode", &format!(r#""{}""#, args.zenoh_mode))?;
    if let Some(ref endpoint) = args.zenoh_connect {
        config.insert_json5("connect/endpoints", &format!(r#"["{}"]"#, endpoint))?;
    }
    let session = zenoh::open(config).await?;

    let crsf_topic = topics::topic(&args.zenoh_prefix, topics::CRSF_TELEMETRY);
    info!("Publishing CRSF telemetry on: {crsf_topic}");
    let crsf_pub = session.declare_publisher(crsf_topic).await?;

    let mut interval = tokio::time::interval(Duration::from_millis(args.poll_ms));
    interval.set_missed_tick_behavior(tokio::time::MissedTickBehavior::Skip);

    let mut buf = vec![0u8; wire::PACKET_SIZE];
    let mut frames_seen: u64 = 0;
    let mut last_seq: Option<u32> = None;
    let mut parse_errors: u64 = 0;
    let mut file: Option<File> = None;
    let mut missing_logged = false;
    // Position-differentiation state for synthesising linear velocity.
    // Uncrashed exposes no usable instantaneous velocity reading from
    // UE4SS Lua, so we estimate it from frame-to-frame position deltas.
    // Stored in liftoff-convention metres (matching `telemetry.position`)
    // so the differentiated vector is in m/s downstream.
    let mut last_pos_m: Option<[f32; 3]> = None;
    let mut last_ts_us: Option<u64> = None;

    loop {
        interval.tick().await;

        // Open lazily on first successful poll; keep the handle for the
        // process lifetime. The mod also keeps its handle open after init,
        // so once both sides are connected the per-tick cost is one
        // pread/pwrite syscall each.
        if file.is_none() {
            match File::open(&args.input_file).await {
                Ok(f) => {
                    info!("opened {}", args.input_file);
                    file = Some(f);
                    missing_logged = false;
                }
                Err(e) if e.kind() == std::io::ErrorKind::NotFound => {
                    if !missing_logged {
                        info!(
                            "input file {} not present yet; waiting for the mod to write it",
                            args.input_file
                        );
                        missing_logged = true;
                    }
                    continue;
                }
                Err(e) => {
                    error!("open {} failed: {e}", args.input_file);
                    continue;
                }
            }
        }
        let f = file.as_mut().unwrap();

        // Snapshot read of the whole packet from offset 0. If the read
        // returns fewer than PACKET_SIZE bytes, the mod hasn't written
        // its first packet yet (e.g. we connected during init); skip and
        // try again. Once the mod is up, every read is a full envelope.
        if let Err(e) = f.seek(SeekFrom::Start(0)).await {
            error!("seek failed: {e} — re-opening on next poll");
            file = None;
            continue;
        }
        let n = match f.read(&mut buf).await {
            Ok(n) => n,
            Err(e) => {
                error!("read failed: {e} — re-opening on next poll");
                file = None;
                continue;
            }
        };
        let pkt = match wire::parse(&buf[..n]) {
            Ok(p) => p,
            Err(wire::ParseError::LengthMismatch) => {
                // Pre-first-write window only — file exists but the mod's
                // init() hasn't completed its first PACKET_SIZE write yet.
                continue;
            }
            Err(e) => {
                parse_errors = parse_errors.saturating_add(1);
                debug!(
                    "parse error: {e} (first byte 0x{:02x})",
                    buf.first().copied().unwrap_or(0)
                );
                continue;
            }
        };

        // Sequence dedup: if the mod hasn't ticked since our last read,
        // the file content is unchanged; skip the Zenoh publish so
        // downstream consumers don't see synthetic duplicates.
        if last_seq == Some(pkt.sequence) {
            continue;
        }
        last_seq = Some(pkt.sequence);

        let mut telemetry = pkt.into_telemetry();

        // Synthesise velocity from position deltas. First tick (no prior
        // sample) and timestamp regressions (mod restart resets the
        // monotonic baseline) leave velocity at None.
        if let (Some(prev_pos), Some(prev_ts), Some(cur_pos)) =
            (last_pos_m, last_ts_us, telemetry.position)
        {
            if pkt.timestamp_us > prev_ts {
                let dt_s = (pkt.timestamp_us - prev_ts) as f64 / 1_000_000.0;
                if dt_s > 1e-4 {
                    telemetry.velocity = Some([
                        ((cur_pos[0] - prev_pos[0]) as f64 / dt_s) as f32,
                        ((cur_pos[1] - prev_pos[1]) as f64 / dt_s) as f32,
                        ((cur_pos[2] - prev_pos[2]) as f64 / dt_s) as f32,
                    ]);
                }
            }
        }
        last_pos_m = telemetry.position;
        last_ts_us = Some(pkt.timestamp_us);

        let battery = pkt.to_battery_packet();
        let mut crsf_frames = crsf_tx::generate_crsf_telemetry(&telemetry, battery.as_ref());

        // Custom CRSF damage frame (type 0x42) — same channel as the rest
        // of the telemetry so subscribers don't have to merge two streams.
        if let Some(dmg) = pkt.to_damage_packet() {
            if let Some(frame) = crsf_custom::build_damage_packet(&dmg) {
                crsf_frames.push(frame);
            }
        }

        for frame in &crsf_frames {
            if let Err(e) = crsf_pub.put(frame.as_slice()).await {
                warn!("zenoh put on {}: {e}", topics::CRSF_TELEMETRY);
            }
        }

        frames_seen += 1;
        if frames_seen <= 3 || frames_seen % 600 == 0 {
            // Everything in this log line is the raw value the Lua mod
            // received from the game — no unit conversions, no axis
            // remapping, no swizzles. Useful for confirming what the
            // game is actually exposing.
            //
            // - pos:   UE cm, (X fwd, Y right, Z up)
            // - vel:   UE cm/s, same axes (zero — see ComponentVelocity note)
            // - gyro:  FRotator deg/s, (Pitch, Yaw, Roll)
            // - dmg:   Props_1..N order, 0=healthy → 1=destroyed
            let dmg_str = match pkt.damage.as_ref() {
                Some(v) => format!("{:?}", v),
                None => "—".to_string(),
            };
            let bat_str = if pkt.has_battery() {
                format!(
                    "cells={} vpc={:.3}V I={:.2}A used={:.0}/{}mAh",
                    pkt.cell_count,
                    pkt.voltage_per_cell,
                    pkt.current_amps,
                    pkt.charge_used_mah,
                    pkt.capacity_mah,
                )
            } else {
                "—".to_string()
            };
            info!(
                "frame #{frames_seen} seq={} flags=0x{:04x} armed={} ground={} crashed={} pos=[{:.1},{:.1},{:.1}] vel=[{:.2},{:.2},{:.2}] gyro=[P={:.2}, Y={:.2}, R={:.2}] inputs=[T={:.2}, Y={:.2}, P={:.2}, R={:.2}] battery={} dmg={} → {} CRSF frames (parse_err={})",
                pkt.sequence,
                pkt.flags,
                pkt.armed(),
                pkt.on_ground(),
                pkt.crashed(),
                pkt.position[0],
                pkt.position[1],
                pkt.position[2],
                pkt.velocity[0],
                pkt.velocity[1],
                pkt.velocity[2],
                pkt.gyro[0],
                pkt.gyro[1],
                pkt.gyro[2],
                pkt.inputs[0],
                pkt.inputs[1],
                pkt.inputs[2],
                pkt.inputs[3],
                bat_str,
                dmg_str,
                crsf_frames.len(),
                parse_errors,
            );
        }
    }
}
