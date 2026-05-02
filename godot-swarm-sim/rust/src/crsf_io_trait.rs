//! Abstract CRSF I/O interface trait.
//!
//! Provides a transport-agnostic boundary between the drone simulation
//! and whatever carries CRSF frames in and out (Zenoh pub/sub, local
//! keyboard input, a future HUD, serial forwarding, etc.).
//!
//! Implementations handle their own threading and networking. All trait
//! methods are non-blocking when called from the physics thread.

use std::time::Instant;

/// Identifies an inbound RC stream within a single interface.
pub type RcStreamId = u8;

/// Direct RC commands targeted at this drone.
pub const RC_STREAM_DIRECT: RcStreamId = 0;
/// Autopilot-generated RC commands for this drone.
pub const RC_STREAM_AUTOPILOT: RcStreamId = 1;

/// An inbound RC frame with its stream identity and arrival timestamp.
#[derive(Debug, Clone)]
pub struct RcFrame {
    /// Which logical stream produced this frame.
    pub stream_id: RcStreamId,
    /// Raw CRSF wire bytes (sync + length + type + payload + CRC).
    pub data: Vec<u8>,
    /// When this frame was received by the interface.
    pub received_at: Instant,
}

/// Abstract CRSF I/O interface.
///
/// RC input uses "latest wins" semantics — at most one frame per stream
/// is retained between polls. Non-RC commands use queue semantics — no
/// frames are dropped between polls.
///
/// Telemetry output is fire-and-forget: the caller hands raw CRSF frame
/// bytes to `send_telemetry` and the interface decides how to deliver
/// them (publish on Zenoh, render in a HUD, etc.).
pub trait CrsfIo: Send + Sync {
    /// Poll for the latest RC frame(s). Returns at most one frame per
    /// stream ID that has fresh data since the last poll. Non-blocking.
    fn poll_rc(&self) -> Vec<RcFrame>;

    /// Drain all pending non-RC command frames received since the last
    /// call. Queue semantics — nothing is dropped. Non-blocking.
    fn poll_commands(&self) -> Vec<Vec<u8>>;

    /// Send a raw CRSF telemetry frame out through this interface.
    /// Non-blocking; may internally buffer for later transmission.
    fn send_telemetry(&self, data: &[u8]);

    /// Graceful shutdown. Called when the owning node exits the tree.
    /// Default implementation does nothing.
    fn shutdown(&self) {}
}
