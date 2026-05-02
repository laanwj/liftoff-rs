//! `GodotInputInterface` — Godot Node providing local keyboard-driven CRSF I/O.
//!
//! Produces RC channel frames from Godot's `Input` singleton (same
//! mappings as the original `StickStub`). Reset is encoded on CRSF
//! channel 14. Telemetry received via `send_telemetry` is currently a
//! no-op but available for future use (e.g. local HUD rendering).
//!
//! Add as a child of `DroneController` to enable keyboard piloting.
//! Only one drone in the scene should have this node.

use std::sync::Mutex;
use std::time::Instant;

use godot::classes::{INode, Node};
use godot::prelude::*;

use telemetry_lib::crsf::{self, CrsfPacket, RcChannelsPacked, device_address};

use crate::crsf_io_trait::{CrsfIo, RcFrame, RC_STREAM_DIRECT};
use crate::input_stub::StickStub;

/// CRSF channel used to encode the reset command.
const RESET_CHANNEL: usize = 14;
const RESET_ACTIVE: u16 = 1983;
const RESET_INACTIVE: u16 = 0;

/// CRSF 11-bit axis constants.
const AXIS_MID: u16 = 992;
const AXIS_MAX: u16 = 1983;

/// Inner state implementing `CrsfIo`. Separated from the Godot node
/// so it can satisfy `Send + Sync` (Base<Node> is not Send/Sync).
pub struct GodotInput {
    sticks: Mutex<StickStub>,
    last_poll: Mutex<Option<Instant>>,
}

impl GodotInput {
    fn new(throttle_rate: f32) -> Self {
        let mut sticks = StickStub::default();
        sticks.throttle_rate = throttle_rate;
        Self {
            sticks: Mutex::new(sticks),
            last_poll: Mutex::new(None),
        }
    }
}

impl CrsfIo for GodotInput {
    fn poll_rc(&self) -> Vec<RcFrame> {
        let now = Instant::now();
        let dt = {
            let mut last = self.last_poll.lock().unwrap();
            let dt = last.map(|t| now.duration_since(t).as_secs_f32()).unwrap_or(0.0);
            *last = Some(now);
            dt
        };

        let mut sticks = self.sticks.lock().unwrap();
        let rc = sticks.poll(dt);
        let reset = sticks.consume_reset();

        let channels = rc_input_to_channels(&rc, reset);
        let packed = RcChannelsPacked { channels };
        let Some(data) = crsf::build_packet(
            device_address::RADIO_TRANSMITTER,
            &CrsfPacket::RcChannelsPacked(packed),
        ) else {
            return Vec::new();
        };

        vec![RcFrame {
            stream_id: RC_STREAM_DIRECT,
            data,
            received_at: now,
        }]
    }

    fn poll_commands(&self) -> Vec<Vec<u8>> {
        Vec::new()
    }

    fn send_telemetry(&self, _data: &[u8]) {
        // No-op for now. Future: feed to HUD overlay.
    }
}

#[derive(GodotClass)]
#[class(base=Node)]
pub struct GodotInputInterface {
    base: Base<Node>,

    /// Throttle ramp rate in units-per-second.
    #[export]
    throttle_rate: f32,

    /// Inner CrsfIo implementation (created on ready).
    inner: Option<GodotInput>,
}

#[godot_api]
impl INode for GodotInputInterface {
    fn init(base: Base<Node>) -> Self {
        Self {
            base,
            throttle_rate: 0.6,
            inner: None,
        }
    }

    fn ready(&mut self) {
        self.inner = Some(GodotInput::new(self.throttle_rate));
    }
}

impl GodotInputInterface {
    /// Access the underlying `CrsfIo` implementation. Returns `None`
    /// before `ready()` has been called.
    pub fn crsf_io(&self) -> Option<&dyn CrsfIo> {
        self.inner.as_ref().map(|io| io as &dyn CrsfIo)
    }
}

/// Convert decoded stick values back to raw CRSF 11-bit channels.
fn rc_input_to_channels(rc: &crate::rc_input::RcInput, reset: bool) -> [u16; 16] {
    let mut ch = [AXIS_MID; 16];

    // AETRA layout: ch0=roll, ch1=pitch, ch2=throttle, ch3=yaw, ch4=arm
    ch[0] = signed_to_raw(rc.roll);
    ch[1] = signed_to_raw(rc.pitch);
    ch[2] = unsigned_to_raw(rc.throttle);
    ch[3] = signed_to_raw(rc.yaw);
    ch[4] = if rc.arm { AXIS_MAX } else { 0 };

    // ch6 = RC mux (SA switch)
    ch[6] = if rc.rc_mux_auto { AXIS_MAX } else { 0 };

    // ch14 = reset trigger
    ch[RESET_CHANNEL] = if reset { RESET_ACTIVE } else { RESET_INACTIVE };

    ch
}

/// Map a signed [-1, 1] stick value to an 11-bit CRSF channel value.
fn signed_to_raw(val: f32) -> u16 {
    let mid = AXIS_MID as f32;
    let half_span = (AXIS_MAX - AXIS_MID) as f32;
    ((mid + val * half_span) as u16).clamp(0, AXIS_MAX)
}

/// Map an unsigned [0, 1] stick value to an 11-bit CRSF channel value.
fn unsigned_to_raw(val: f32) -> u16 {
    (val * AXIS_MAX as f32).clamp(0.0, AXIS_MAX as f32) as u16
}
