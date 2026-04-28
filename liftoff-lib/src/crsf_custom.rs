//! Custom (non-standard) CRSF packet types.
//!
//! These frame types are not part of the official CRSF specification.
//! They use unallocated type IDs in the extended-header range and are
//! decoded on the radio side by a companion LUA script for EdgeTX.

use crate::crsf::{CrsfPacket, build_packet, device_address};
use crate::simstate::DamagePacket;

const SOURCE_ADDRESS: u8 = device_address::FLIGHT_CONTROLLER;

// ---------------------------------------------------------------------------
// 0x42 – Per-rotor damage telemetry
// ---------------------------------------------------------------------------

/// Per-rotor damage telemetry (custom CRSF extended frame 0x42).
///
/// Sent as an extended-header frame with dest=RADIO_TRANSMITTER,
/// origin=FLIGHT_CONTROLLER so EdgeTX forwards it to the LUA queue
/// via `crossfireTelemetryPop()`.
#[derive(Debug, Clone, PartialEq)]
pub struct Damage {
    /// Status flags (bit 0: killed, bit 1: crashed, bit 2: no drone).
    pub flags: u8,
    /// Per-rotor health values, 0–10000 (representing 0.00%–100.00%).
    /// 10000 = fully healthy, 0 = destroyed.
    pub health: Vec<u16>,
}

/// Serialise a [`Damage`] payload into an already-started CRSF frame buffer.
///
/// The caller has already pushed address + length-placeholder + type byte;
/// this function appends the extended-header dest/origin and the payload
/// fields.  Returns `None` if there are too many rotors (>8).
pub(crate) fn build_damage_payload(frame: &mut Vec<u8>, dmg: &Damage) -> Option<()> {
    // Extended header: dest + origin
    frame.push(device_address::RADIO_TRANSMITTER);
    frame.push(device_address::FLIGHT_CONTROLLER);
    frame.push(dmg.flags);
    let n = dmg.health.len();
    if n > 8 {
        return None;
    }
    frame.push(n as u8);
    for &h in &dmg.health {
        frame.extend_from_slice(&h.to_be_bytes());
    }
    Some(())
}

/// Parse a [`Damage`] packet from the data slice *after* the type byte.
///
/// For extended-header frames `data[0]` is the destination address and
/// `data[1]` is the origin address; the actual payload starts at `data[2]`.
pub(crate) fn parse_damage_payload(data: &[u8]) -> Option<Damage> {
    // data[0]=dest, data[1]=origin, data[2]=flags, data[3]=n_rotors
    if data.len() < 4 {
        return None;
    }
    let flags = data[2];
    let n = data[3] as usize;
    if n > 8 || data.len() < 4 + n * 2 {
        return None;
    }
    let mut health = Vec::with_capacity(n);
    for i in 0..n {
        let off = 4 + i * 2;
        health.push(u16::from_be_bytes([data[off], data[off + 1]]));
    }
    Some(Damage { flags, health })
}

/// Build a CRSF Damage packet (custom type 0x42) from a [`DamagePacket`].
///
/// Health values are mapped from the sim's `[0.0, 1.0]` (where 1.0 = healthy)
/// to wire `u16` values `[0, 10000]` representing `0.00%` to `100.00%`.
pub fn build_damage_packet(dmg: &DamagePacket) -> Option<Vec<u8>> {
    let mut flags: u8 = 0;
    if dmg.killed() {
        flags |= 0x01;
    }
    if dmg.crashed() {
        flags |= 0x02;
    }
    if dmg.no_drone() {
        flags |= 0x04;
    }
    let health: Vec<u16> = dmg
        .damage
        .iter()
        .map(|&v| (v.clamp(0.0, 1.0) * 10000.0) as u16)
        .collect();
    let damage = Damage { flags, health };
    build_packet(SOURCE_ADDRESS, &CrsfPacket::Damage(damage))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crsf::{PacketType, frame_check_crc, parse_packet, parse_packet_check};

    #[test]
    fn damage_roundtrip() {
        let dmg = Damage {
            flags: 0x02, // crashed
            health: vec![10000, 7500, 0, 10000],
        };
        let packet = CrsfPacket::Damage(dmg.clone());
        let built = build_packet(SOURCE_ADDRESS, &packet).unwrap();

        // Extended header: addr(1) + len(1) + type(1) + dest(1) + orig(1)
        //   + flags(1) + n(1) + 4*2 health + CRC(1) = 16 bytes
        assert_eq!(built.len(), 16);
        assert_eq!(built[2], PacketType::Damage as u8);
        // dest = RADIO_TRANSMITTER, origin = FLIGHT_CONTROLLER
        assert_eq!(built[3], device_address::RADIO_TRANSMITTER);
        assert_eq!(built[4], device_address::FLIGHT_CONTROLLER);

        assert!(frame_check_crc(&built));
        let parsed = parse_packet_check(&built).unwrap();
        if let CrsfPacket::Damage(p) = parsed {
            assert_eq!(p.flags, dmg.flags);
            assert_eq!(p.health, dmg.health);
        } else {
            panic!("Round trip failed for Damage");
        }
    }

    #[test]
    fn damage_empty() {
        let dmg = Damage {
            flags: 0x04, // no drone
            health: vec![],
        };
        let built = build_packet(SOURCE_ADDRESS, &CrsfPacket::Damage(dmg.clone())).unwrap();
        assert!(frame_check_crc(&built));
        let parsed = parse_packet_check(&built).unwrap();
        if let CrsfPacket::Damage(p) = parsed {
            assert_eq!(p.flags, 0x04);
            assert!(p.health.is_empty());
        } else {
            panic!("Round trip failed for empty Damage");
        }
    }

    #[test]
    fn damage_overflow() {
        // More than 8 rotors should fail
        let dmg = Damage {
            flags: 0,
            health: vec![100; 9],
        };
        assert!(build_packet(SOURCE_ADDRESS, &CrsfPacket::Damage(dmg)).is_none());
    }

    #[test]
    fn build_damage_from_simstate() {
        let sim_dmg = DamagePacket {
            version: 1,
            flags: DamagePacket::FLAG_CRASHED,
            timestamp_ms: 0,
            damage: vec![1.0, 0.5, 0.0, 1.0],
        };
        let frame = build_damage_packet(&sim_dmg).unwrap();
        assert!(frame_check_crc(&frame));
        let parsed = parse_packet(&frame).unwrap();
        if let CrsfPacket::Damage(p) = parsed {
            assert_eq!(p.flags, 0x02); // crashed
            assert_eq!(p.health, vec![10000, 5000, 0, 10000]);
        } else {
            panic!("expected Damage packet");
        }
    }
}
