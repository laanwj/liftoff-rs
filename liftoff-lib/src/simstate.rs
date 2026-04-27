//! Parser for the `liftoff-simstate-bridge` UDP wire format.
//!
//! Two packet kinds share one UDP port; each starts with a 4-byte ASCII tag
//! the consumer dispatches on. See `liftoff-simstate-bridge/README.md` for the
//! authoritative byte-level spec.
use byteorder::{ByteOrder, LittleEndian};
use serde::{Deserialize, Serialize};

/// Per-propeller damage packet (`LFDM`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct DamagePacket {
    pub version: u16,
    pub flags: u16,
    /// Plugin-monotonic milliseconds since plugin load.
    pub timestamp_ms: u64,
    /// Raw `Propeller.DamageState` floats in MotorRPM order: LF, RF, LB, RB.
    /// 0.0 = healthy, increases with damage (typically capped at 1.0 = broken).
    pub damage: Vec<f32>,
}

impl DamagePacket {
    pub const TAG: &'static [u8; 4] = b"LFDM";
    pub const FLAG_KILLED: u16 = 0x0001;
    pub const FLAG_CRASHED: u16 = 0x0002;
    pub const FLAG_NO_DRONE: u16 = 0x0004;
    pub const HEADER_SIZE: usize = 20;
    pub const MAX_PROPS: usize = 8;

    pub fn killed(&self) -> bool {
        self.flags & Self::FLAG_KILLED != 0
    }
    pub fn crashed(&self) -> bool {
        self.flags & Self::FLAG_CRASHED != 0
    }
    pub fn no_drone(&self) -> bool {
        self.flags & Self::FLAG_NO_DRONE != 0
    }
}

/// Battery telemetry packet (`LFBT`). Fixed 40 bytes on the wire.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BatteryPacket {
    pub version: u16,
    pub flags: u16,
    /// Plugin-monotonic milliseconds since plugin load (same clock as DamagePacket).
    pub timestamp_ms: u64,
    /// `BatteryPart.NrOfCells`; 0 if the drone has no battery part.
    pub cell_count: u8,
    /// Total pack voltage, V.
    pub voltage: f32,
    /// Per-cell voltage, V. The sim does not model per-cell variation, so
    /// this is the same value the in-game HUD displays for "voltage per cell".
    pub voltage_per_cell: f32,
    /// Instantaneous current draw, A.
    pub current_amps: f32,
    /// Accumulated charge drawn, Ah. Multiply by 1000 for mAh.
    pub charge_drawn_ah: f32,
    /// Remaining capacity, 0.0 – 1.0.
    pub percentage: f32,
}

impl BatteryPacket {
    pub const TAG: &'static [u8; 4] = b"LFBT";
    pub const FLAG_NO_DRAINER: u16 = 0x0001;
    pub const FLAG_NO_DRONE: u16 = 0x0002;
    pub const PACKET_SIZE: usize = 40;

    pub fn no_drainer(&self) -> bool {
        self.flags & Self::FLAG_NO_DRAINER != 0
    }
    pub fn no_drone(&self) -> bool {
        self.flags & Self::FLAG_NO_DRONE != 0
    }

    /// True when the float fields are meaningful (drone present and battery
    /// sim active). When false, all floats and `cell_count` are zero.
    pub fn has_data(&self) -> bool {
        !self.no_drainer() && !self.no_drone()
    }
}

/// Either packet kind. Use [`parse_packet`] to dispatch by tag.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub enum SimstatePacket {
    Damage(DamagePacket),
    Battery(BatteryPacket),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    TooShort,
    UnknownTag,
    UnsupportedVersion,
    LengthMismatch,
    PropCountTooLarge,
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParseError::TooShort => write!(f, "packet shorter than tag"),
            ParseError::UnknownTag => write!(f, "unknown packet tag"),
            ParseError::UnsupportedVersion => write!(f, "unsupported version"),
            ParseError::LengthMismatch => write!(f, "packet length doesn't match header"),
            ParseError::PropCountTooLarge => write!(f, "prop_count exceeds maximum"),
        }
    }
}

impl std::error::Error for ParseError {}

/// Dispatch a raw datagram to the appropriate parser based on its 4-byte tag.
pub fn parse_packet(data: &[u8]) -> Result<SimstatePacket, ParseError> {
    if data.len() < 4 {
        return Err(ParseError::TooShort);
    }
    match &data[0..4] {
        b"LFDM" => parse_damage(data).map(SimstatePacket::Damage),
        b"LFBT" => parse_battery(data).map(SimstatePacket::Battery),
        _ => Err(ParseError::UnknownTag),
    }
}

pub fn parse_damage(data: &[u8]) -> Result<DamagePacket, ParseError> {
    if data.len() < DamagePacket::HEADER_SIZE {
        return Err(ParseError::TooShort);
    }
    if &data[0..4] != DamagePacket::TAG.as_slice() {
        return Err(ParseError::UnknownTag);
    }
    let version = LittleEndian::read_u16(&data[4..6]);
    if version != 1 {
        return Err(ParseError::UnsupportedVersion);
    }
    let flags = LittleEndian::read_u16(&data[6..8]);
    let timestamp_ms = LittleEndian::read_u64(&data[8..16]);
    let prop_count = data[16] as usize;
    // bytes 17..20 are pad
    if prop_count > DamagePacket::MAX_PROPS {
        return Err(ParseError::PropCountTooLarge);
    }
    let expected = DamagePacket::HEADER_SIZE + 4 * prop_count;
    if data.len() != expected {
        return Err(ParseError::LengthMismatch);
    }
    let mut damage = Vec::with_capacity(prop_count);
    for i in 0..prop_count {
        let off = DamagePacket::HEADER_SIZE + 4 * i;
        damage.push(LittleEndian::read_f32(&data[off..off + 4]));
    }
    Ok(DamagePacket {
        version,
        flags,
        timestamp_ms,
        damage,
    })
}

pub fn parse_battery(data: &[u8]) -> Result<BatteryPacket, ParseError> {
    if data.len() != BatteryPacket::PACKET_SIZE {
        return Err(ParseError::LengthMismatch);
    }
    if &data[0..4] != BatteryPacket::TAG.as_slice() {
        return Err(ParseError::UnknownTag);
    }
    let version = LittleEndian::read_u16(&data[4..6]);
    if version != 1 {
        return Err(ParseError::UnsupportedVersion);
    }
    let flags = LittleEndian::read_u16(&data[6..8]);
    let timestamp_ms = LittleEndian::read_u64(&data[8..16]);
    let cell_count = data[16];
    // bytes 17..20 are pad
    let voltage = LittleEndian::read_f32(&data[20..24]);
    let voltage_per_cell = LittleEndian::read_f32(&data[24..28]);
    let current_amps = LittleEndian::read_f32(&data[28..32]);
    let charge_drawn_ah = LittleEndian::read_f32(&data[32..36]);
    let percentage = LittleEndian::read_f32(&data[36..40]);
    Ok(BatteryPacket {
        version,
        flags,
        timestamp_ms,
        cell_count,
        voltage,
        voltage_per_cell,
        current_amps,
        charge_drawn_ah,
        percentage,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Round-trip a damage packet matching what the C# plugin emits.
    #[test]
    fn parse_damage_roundtrip() {
        // 4-prop quad, two healthy + two broken, CRASHED flag set.
        let mut buf = Vec::new();
        buf.extend_from_slice(b"LFDM");
        buf.extend_from_slice(&1u16.to_le_bytes()); // version
        buf.extend_from_slice(&DamagePacket::FLAG_CRASHED.to_le_bytes());
        buf.extend_from_slice(&12345u64.to_le_bytes()); // ts_ms
        buf.push(4); // prop_count
        buf.extend_from_slice(&[0, 0, 0]); // pad
        for v in [0.0_f32, 0.42, 1.0, 1.0] {
            buf.extend_from_slice(&v.to_le_bytes());
        }

        let pkt = parse_damage(&buf).unwrap();
        assert_eq!(pkt.version, 1);
        assert!(pkt.crashed());
        assert!(!pkt.killed());
        assert_eq!(pkt.timestamp_ms, 12345);
        assert_eq!(pkt.damage, vec![0.0, 0.42, 1.0, 1.0]);
    }

    #[test]
    fn parse_damage_no_drone_zero_props() {
        let mut buf = Vec::new();
        buf.extend_from_slice(b"LFDM");
        buf.extend_from_slice(&1u16.to_le_bytes());
        buf.extend_from_slice(&DamagePacket::FLAG_NO_DRONE.to_le_bytes());
        buf.extend_from_slice(&0u64.to_le_bytes());
        buf.extend_from_slice(&[0, 0, 0, 0]); // prop_count + pad

        let pkt = parse_damage(&buf).unwrap();
        assert!(pkt.no_drone());
        assert!(pkt.damage.is_empty());
    }

    #[test]
    fn parse_battery_roundtrip() {
        let mut buf = Vec::new();
        buf.extend_from_slice(b"LFBT");
        buf.extend_from_slice(&1u16.to_le_bytes());
        buf.extend_from_slice(&0u16.to_le_bytes()); // no flags
        buf.extend_from_slice(&54321u64.to_le_bytes());
        buf.push(4); // cell_count
        buf.extend_from_slice(&[0, 0, 0]); // pad
        buf.extend_from_slice(&15.4_f32.to_le_bytes()); // voltage
        buf.extend_from_slice(&3.85_f32.to_le_bytes()); // voltage per cell
        buf.extend_from_slice(&22.5_f32.to_le_bytes()); // current
        buf.extend_from_slice(&0.25_f32.to_le_bytes()); // ah drawn
        buf.extend_from_slice(&0.78_f32.to_le_bytes()); // percentage
        assert_eq!(buf.len(), BatteryPacket::PACKET_SIZE);

        let pkt = parse_battery(&buf).unwrap();
        assert_eq!(pkt.cell_count, 4);
        assert_eq!(pkt.voltage, 15.4);
        assert_eq!(pkt.voltage_per_cell, 3.85);
        assert_eq!(pkt.current_amps, 22.5);
        assert_eq!(pkt.charge_drawn_ah, 0.25);
        assert_eq!(pkt.percentage, 0.78);
        assert!(pkt.has_data());
    }

    #[test]
    fn parse_battery_no_drainer() {
        let mut buf = vec![0u8; BatteryPacket::PACKET_SIZE];
        buf[0..4].copy_from_slice(b"LFBT");
        buf[4..6].copy_from_slice(&1u16.to_le_bytes());
        buf[6..8].copy_from_slice(&BatteryPacket::FLAG_NO_DRAINER.to_le_bytes());

        let pkt = parse_battery(&buf).unwrap();
        assert!(pkt.no_drainer());
        assert!(!pkt.has_data());
        assert_eq!(pkt.voltage, 0.0);
    }

    #[test]
    fn dispatch_by_tag() {
        let mut dmg = Vec::new();
        dmg.extend_from_slice(b"LFDM");
        dmg.extend_from_slice(&1u16.to_le_bytes());
        dmg.extend_from_slice(&[0, 0]);
        dmg.extend_from_slice(&[0; 8]);
        dmg.extend_from_slice(&[0, 0, 0, 0]);
        match parse_packet(&dmg).unwrap() {
            SimstatePacket::Damage(_) => {}
            _ => panic!("expected Damage variant"),
        }

        let mut bat = vec![0u8; BatteryPacket::PACKET_SIZE];
        bat[0..4].copy_from_slice(b"LFBT");
        bat[4..6].copy_from_slice(&1u16.to_le_bytes());
        match parse_packet(&bat).unwrap() {
            SimstatePacket::Battery(_) => {}
            _ => panic!("expected Battery variant"),
        }
    }

    #[test]
    fn rejects_unknown_tag() {
        let buf = b"XXXXextra".to_vec();
        assert!(matches!(parse_packet(&buf), Err(ParseError::UnknownTag)));
    }

    #[test]
    fn rejects_runt() {
        assert!(matches!(parse_packet(b"LFD"), Err(ParseError::TooShort)));
    }
}
