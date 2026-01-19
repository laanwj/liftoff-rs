use std::time::Duration;

/// Re-send a Register command with this duration in between.
pub const KEEPALIVE_INTERVAL: Duration = Duration::from_secs(30);

/// Telemetry router command opcode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Opcode {
    /// Register a client. This opcode doubles as keep-alive.
    Register = 0x00,
    /// Unregister a client.
    Unregister = 0x01,
}

impl Opcode {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x00 => Some(Self::Register),
            0x01 => Some(Self::Unregister),
            _ => None,
        }
    }
}

