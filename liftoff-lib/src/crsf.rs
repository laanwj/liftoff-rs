use crc::{CRC_8_DVB_S2, Crc};
use num_enum::TryFromPrimitive;

pub const CRC8_DVB_S2: Crc<u8> = Crc::<u8>::new(&CRC_8_DVB_S2);

/// CRSF maximum frame size including address, length and CRC bytes.
pub const MAX_FRAME_SIZE: usize = 64;

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, TryFromPrimitive)]
pub enum PacketType {
    Gps = 0x02,
    Vario = 0x07,
    BatterySensor = 0x08,
    BaroAlt = 0x09,
    Airspeed = 0x0A,
    Heartbeat = 0x0B,
    Rpm = 0x0C,
    Temp = 0x0D,
    Voltages = 0x0E,
    VideoTransmitter = 0x0F,
    LinkStatistics = 0x14,
    RcChannelsPacked = 0x16,
    LinkStatisticsRx = 0x1C,
    LinkStatisticsTx = 0x1D,
    Attitude = 0x1E,
    FlightMode = 0x21,
    DeviceInfo = 0x29,
    ConfigRead = 0x2C,
    ConfigWrite = 0x2D,
    RadioId = 0x3A,
}

/// CRSF device addresses. These double as sync byte.
pub mod device_address {
    pub const BROADCAST: u8 = 0x00;
    pub const FLIGHT_CONTROLLER: u8 = 0xC8;
    pub const VTX: u8 = 0xCE;
    pub const RADIO_TRANSMITTER: u8 = 0xEA;
    pub const CRSF_RECEIVER: u8 = 0xEC;
    pub const CRSF_TRANSMITTER: u8 = 0xEE;
}

#[derive(Debug, Clone)]
pub struct Attitude {
    pub pitch: i16, // Radians * 1e4
    pub roll: i16,  // Radians * 1e4
    pub yaw: i16,   // Radians * 1e4
}

#[derive(Debug, Clone)]
pub struct Gps {
    pub lat: i32,     // deg * 1e7
    pub lon: i32,     // deg * 1e7
    pub speed: u16,   // km/h * 10
    pub heading: u16, // deg * 100
    pub alt: u16,     // m + 1000
    pub sats: u8,
}

#[derive(Debug, Clone)]
pub struct Battery {
    pub voltage: u16,  // dV (0.1V)
    pub current: u16,  // dA (0.1A)
    pub capacity: u32, // mAh
    pub remaining: u8, // %
}

#[derive(Debug, Clone)]
pub struct Vario {
    pub vertical_speed: i16, // dm/s
}

#[derive(Debug, Clone)]
pub struct FlightMode {
    pub mode: String,
}

#[derive(Debug, Clone)]
pub struct BaroAlt {
    pub alt: u16, // m + 1000
    pub vertical_speed: u8,
}

#[derive(Debug, Clone)]
pub struct Airspeed {
    pub speed: u16, // km/h * 10
}

#[derive(Debug, Clone)]
pub struct Rpm {
    pub source_id: u8,
    pub rpms: Vec<u32>,
}

#[derive(Debug, Clone)]
pub struct RcChannelsPacked {
    pub channels: [u16; 16],
}

#[derive(Debug, Clone)]
pub enum CrsfPacket {
    Attitude(Attitude),
    Gps(Gps),
    Battery(Battery),
    Vario(Vario),
    FlightMode(FlightMode),
    BaroAlt(BaroAlt),
    Airspeed(Airspeed),
    Rpm(Rpm),
    RcChannelsPacked(RcChannelsPacked),
    Unknown(PacketType), // Keep Unknown for parsing existing unknown packets
}

pub fn us_to_ticks(us: u16) -> u16 {
    // (x - 1500) * 8 / 5 + 992
    ((us as i32 - 1500) * 8 / 5 + 992) as u16
}

pub fn ticks_to_us(ticks: u16) -> u16 {
    // (x - 992) * 5 / 8 + 1500
    ((ticks as i32 - 992) * 5 / 8 + 1500) as u16
}

pub fn calc_crc8(data: &[u8]) -> u8 {
    CRC8_DVB_S2.checksum(data)
}

/// Unpack CRSF 11-bit channels from a byte buffer.
/// Expects 22 bytes of channel data (16 channels * 11 bits = 176 bits = 22 bytes).
fn unpack_channels(data: &[u8]) -> Option<[u16; 16]> {
    let mut channels = [0u16; 16];
    if data.len() < 22 {
        return None;
    }

    let mut src_shift = 0;
    let mut ptr = 0;

    for i in 0..16 {
        let mut value = (data[ptr] as u16) >> src_shift;
        ptr += 1;

        let mut value_bits_left = 11 - 8 + src_shift;
        value = (value | ((data[ptr] as u16) << (11 - value_bits_left))) & 0x7ff;

        if value_bits_left >= 8 {
            ptr += 1;
            value_bits_left -= 8;
            if value_bits_left > 0 {
                value = (value | ((data[ptr] as u16) << (11 - value_bits_left))) & 0x7ff;
            }
        }
        src_shift = value_bits_left;
        channels[i] = value;
    }
    Some(channels)
}

/// Pack 16x 11-bit channels into 22 bytes.
fn pack_channels(channels: &[u16; 16]) -> Option<[u8; 22]> {
    let mut buf = [0u8; 22];
    let mut dest_shift = 0;
    let mut ptr = 0;

    for &ch in channels {
        if ch > 0x7ff {
            return None;
        }
        let mut value = ch;
        let mut bits_to_write = 11;

        while bits_to_write > 0 {
            let space_in_byte = 8 - dest_shift;
            let write_bits = bits_to_write.min(space_in_byte);

            buf[ptr] |= ((value as u8) & (((1u16 << write_bits) - 1) as u8)) << dest_shift;

            value >>= write_bits;
            bits_to_write -= write_bits;
            dest_shift += write_bits;

            if dest_shift == 8 {
                dest_shift = 0;
                ptr += 1;
            }
        }
    }
    Some(buf)
}

pub fn build_packet(address: u8, packet: &CrsfPacket) -> Option<Vec<u8>> {
    let mut frame = Vec::with_capacity(MAX_FRAME_SIZE);
    frame.push(address); // Address/sync byte
    frame.push(0x00); // Length: fill in later
    match packet {
        CrsfPacket::Attitude(att) => {
            frame.push(PacketType::Attitude as u8);
            frame.extend_from_slice(&att.pitch.to_be_bytes());
            frame.extend_from_slice(&att.roll.to_be_bytes());
            frame.extend_from_slice(&att.yaw.to_be_bytes());
        }
        CrsfPacket::Gps(gps) => {
            frame.push(PacketType::Gps as u8);
            frame.extend_from_slice(&gps.lat.to_be_bytes());
            frame.extend_from_slice(&gps.lon.to_be_bytes());
            frame.extend_from_slice(&gps.speed.to_be_bytes());
            frame.extend_from_slice(&gps.heading.to_be_bytes());
            frame.extend_from_slice(&gps.alt.to_be_bytes()); // alt + 1000
            frame.push(gps.sats);
        }
        CrsfPacket::Battery(bat) => {
            frame.push(PacketType::BatterySensor as u8);
            frame.extend_from_slice(&bat.voltage.to_be_bytes());
            frame.extend_from_slice(&bat.current.to_be_bytes());
            let cap_bytes = bat.capacity.to_be_bytes();
            if cap_bytes[0] != 0x00 {
                // Overflow
                return None;
            }
            frame.extend_from_slice(&cap_bytes[1..]); // 3 bytes
            frame.push(bat.remaining);
        }
        CrsfPacket::Vario(vario) => {
            frame.push(PacketType::Vario as u8);
            frame.extend_from_slice(&vario.vertical_speed.to_be_bytes());
        }
        CrsfPacket::FlightMode(fm) => {
            frame.push(PacketType::FlightMode as u8);
            frame.extend_from_slice(fm.mode.as_bytes());
            frame.push(0);
        }
        CrsfPacket::BaroAlt(baro) => {
            frame.push(PacketType::BaroAlt as u8);
            frame.extend_from_slice(&baro.alt.to_be_bytes());
            frame.push(baro.vertical_speed);
        }
        CrsfPacket::Airspeed(airspeed) => {
            frame.push(PacketType::Airspeed as u8);
            frame.extend_from_slice(&airspeed.speed.to_be_bytes());
        }
        CrsfPacket::Rpm(rpm) => {
            frame.push(PacketType::Rpm as u8);
            frame.push(rpm.source_id);
            for &val in &rpm.rpms {
                let bytes = val.to_be_bytes();
                if bytes[0] != 0x00 {
                    // Overflow
                    return None;
                }
                frame.extend_from_slice(&bytes[1..]); // 3 bytes
            }
        }
        CrsfPacket::RcChannelsPacked(channels) => {
            frame.push(PacketType::RcChannelsPacked as u8);
            frame.extend_from_slice(&pack_channels(&channels.channels)?);
        }
        CrsfPacket::Unknown(_pt) => {
            // Cannot build unknown packet without data
            return None;
        }
    }
    if (frame.len() + 1) > MAX_FRAME_SIZE {
        // Total frame size with CRC byte may not exceed 64.
        None
    } else {
        // Fill in length. Length includes type byte and CRC byte, but not address and length.
        frame[1] = (frame.len() - 2 + 1) as u8;
        // Add CRC. CRC is computed over type byte and data only.
        frame.push(calc_crc8(&frame[2..]));
        Some(frame)
    }
}

/// Parse CRSF packet without checking CRC.
pub fn parse_packet(frame: &[u8]) -> Option<CrsfPacket> {
    // Check length. Length byte includes type byte and CRC, but not address and length byte.
    if frame.len() < 4 || (frame[1] as usize) != (frame.len() - 2) {
        return None;
    }
    // We do not check the address byte, CRC here.
    let type_byte = frame[2];
    let data = &frame[3..frame.len() - 1];
    let packet_type = PacketType::try_from_primitive(type_byte).ok()?;

    match packet_type {
        PacketType::Attitude => {
            if data.len() < 6 {
                return None;
            }
            let pitch = i16::from_be_bytes([data[0], data[1]]);
            let roll = i16::from_be_bytes([data[2], data[3]]);
            let yaw = i16::from_be_bytes([data[4], data[5]]);
            Some(CrsfPacket::Attitude(Attitude { pitch, roll, yaw }))
        }
        PacketType::Gps => {
            if data.len() < 15 {
                return None;
            }
            let lat = i32::from_be_bytes([data[0], data[1], data[2], data[3]]);
            let lon = i32::from_be_bytes([data[4], data[5], data[6], data[7]]);
            let speed = u16::from_be_bytes([data[8], data[9]]);
            let heading = u16::from_be_bytes([data[10], data[11]]);
            let alt = u16::from_be_bytes([data[12], data[13]]);
            let sats = data[14];
            Some(CrsfPacket::Gps(Gps {
                lat,
                lon,
                speed,
                heading,
                alt,
                sats,
            }))
        }
        PacketType::BatterySensor => {
            if data.len() < 8 {
                return None;
            }
            let voltage = u16::from_be_bytes([data[0], data[1]]);
            let current = u16::from_be_bytes([data[2], data[3]]);
            let capacity = u32::from_be_bytes([0, data[4], data[5], data[6]]); // 24-bit
            let remaining = data[7];
            Some(CrsfPacket::Battery(Battery {
                voltage,
                current,
                capacity,
                remaining,
            }))
        }
        PacketType::Vario => {
            if data.len() < 2 {
                return None;
            }
            let vertical_speed = i16::from_be_bytes([data[0], data[1]]);
            Some(CrsfPacket::Vario(Vario { vertical_speed }))
        }
        PacketType::FlightMode => {
            // Null-terminated string
            let mode = String::from_utf8_lossy(data)
                .trim_matches(char::from(0))
                .to_string();
            Some(CrsfPacket::FlightMode(FlightMode { mode }))
        }
        PacketType::BaroAlt => {
            if data.len() < 3 {
                return None;
            }
            let alt = u16::from_be_bytes([data[0], data[1]]);
            let vertical_speed = data[2];
            Some(CrsfPacket::BaroAlt(BaroAlt {
                alt,
                vertical_speed,
            }))
        }
        PacketType::Airspeed => {
            if data.len() < 2 {
                return None;
            }
            let speed = u16::from_be_bytes([data[0], data[1]]);
            Some(CrsfPacket::Airspeed(Airspeed { speed }))
        }
        PacketType::Rpm => {
            if data.len() < 1 {
                return None;
            }
            let source_id = data[0];
            let mut rpms = Vec::new();
            let mut i = 1;
            while i + 3 <= data.len() {
                let val = u32::from_be_bytes([0, data[i], data[i + 1], data[i + 2]]);
                rpms.push(val);
                i += 3;
            }
            Some(CrsfPacket::Rpm(Rpm { source_id, rpms }))
        }
        PacketType::RcChannelsPacked => {
            let channels = unpack_channels(data)?;
            Some(CrsfPacket::RcChannelsPacked(RcChannelsPacked { channels }))
        }
        _ => Some(CrsfPacket::Unknown(packet_type)),
    }
}

/// Perform minimal CRSF packet validation and check CRC.
pub fn frame_check_crc(frame: &[u8]) -> bool {
    // Check length. Length byte includes type byte and CRC, but not address and length byte.
    if frame.len() < 4 || (frame[1] as usize) != (frame.len() - 2) {
        return false;
    }
    calc_crc8(&frame[2..frame.len() - 1]) == frame[frame.len() - 1]
}

/// Parse CRSF packet and check CRC.
pub fn parse_packet_check(frame: &[u8]) -> Option<CrsfPacket> {
    if frame_check_crc(frame) {
        parse_packet(frame)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    const SOURCE_ADDRESS: u8 = device_address::FLIGHT_CONTROLLER;

    #[test]
    fn test_pack_unpack_channels() {
        // Create an array of channels with varied values within 11-bit range
        let mut original_channels = [0u16; 16];
        for i in 0..16 {
            original_channels[i] = (i * 100) as u16 % 2048;
        }
        // Set some specific edge values
        original_channels[0] = 0; // Min value
        original_channels[1] = 2047; // Max value (0x7FF)
        original_channels[2] = 1024; // Mid value
        original_channels[15] = 992; // Center (1500us -> 992)

        let packed = pack_channels(&original_channels).unwrap();

        // Ensure packed buffer is 22 bytes
        assert_eq!(packed.len(), 22);

        let unpacked = unpack_channels(&packed).expect("Failed to unpack channels");

        assert_eq!(original_channels, unpacked);
    }

    #[test]
    fn test_pack_channels_overflow() {
        // Test that values > 11 bits fail
        let mut channels = [0u16; 16];
        channels[0] = 2048;
        channels[1] = 4095;

        let packed = pack_channels(&channels);
        assert_eq!(packed, None);
    }

    #[test]
    fn test_unpack_channels_short_buffer() {
        // Should return None if buffer < 22 bytes
        let data = [0u8; 21];
        assert_eq!(unpack_channels(&data), None);
    }

    #[test]
    fn test_parse_packet_attitude() {
        // Payload: Type (1), Pitch, Roll, Yaw (2 bytes each, signed, big endian, scaled by 10000)
        let pitch_rad = 1.0;
        let roll_rad = -0.5;
        let yaw_rad = 0.123;

        let pitch_raw = (pitch_rad * 10000.0) as i16;
        let roll_raw = (roll_rad * 10000.0) as i16;
        let yaw_raw = (yaw_rad * 10000.0) as i16;

        let mut payload = Vec::new();
        payload.push(SOURCE_ADDRESS);
        payload.push(8); // Length
        payload.push(PacketType::Attitude as u8);
        payload.extend_from_slice(&pitch_raw.to_be_bytes());
        payload.extend_from_slice(&roll_raw.to_be_bytes());
        payload.extend_from_slice(&yaw_raw.to_be_bytes());
        payload.push(0x00); // Dummy CRC

        match parse_packet(&payload) {
            Some(CrsfPacket::Attitude(att)) => {
                assert!(att.pitch == pitch_raw);
                assert!(att.roll == roll_raw);
                assert!(att.yaw == yaw_raw);
            }
            _ => panic!("Expected Attitude packet"),
        }
    }

    #[test]
    fn test_parse_packet_gps() {
        // Payload: Type (1), lat (i32), lon (i32), speed (u16), heading (u16), alt (u16), sats (u8)
        let lat: i32 = 525200000; // 52.52 deg
        let lon: i32 = 134050000; // 13.405 deg
        let speed: u16 = 1000; // 100.0 km/h
        let heading: u16 = 18000; // 180.00 deg
        let alt: u16 = 1500; // 500m (alt - 1000)
        let sats = 8;

        let mut payload = Vec::new();
        payload.push(SOURCE_ADDRESS);
        payload.push(17); // Length
        payload.push(PacketType::Gps as u8);
        payload.extend_from_slice(&lat.to_be_bytes());
        payload.extend_from_slice(&lon.to_be_bytes());
        payload.extend_from_slice(&speed.to_be_bytes());
        payload.extend_from_slice(&heading.to_be_bytes());
        payload.extend_from_slice(&alt.to_be_bytes());
        payload.push(sats);
        payload.push(0x00); // Dummy CRC

        match parse_packet(&payload) {
            Some(CrsfPacket::Gps(gps)) => {
                assert_eq!(gps.lat, lat);
                assert_eq!(gps.lon, lon);
                assert_eq!(gps.speed, speed);
                assert_eq!(gps.heading, heading);
                assert_eq!(gps.alt, alt);
                assert_eq!(gps.sats, sats);
            }
            _ => panic!("Expected GPS packet"),
        }
    }

    #[test]
    fn test_parse_packet_battery() {
        // Payload: Type (1), voltage (u16), current (u16), capacity (3 bytes), remaining (u8)
        let voltage: u16 = 168; // 16.8V
        let current: u16 = 50; // 5.0A
        let capacity: u32 = 1500; // mAh
        let remaining = 80; // %

        let mut payload = Vec::new();
        payload.push(SOURCE_ADDRESS);
        payload.push(10); // Length
        payload.push(PacketType::BatterySensor as u8);
        payload.extend_from_slice(&voltage.to_be_bytes());
        payload.extend_from_slice(&current.to_be_bytes());
        // Capacity is 3 bytes big endian
        let cap_bytes = capacity.to_be_bytes(); // u32 -> [u8; 4]
        payload.push(cap_bytes[1]);
        payload.push(cap_bytes[2]);
        payload.push(cap_bytes[3]);
        payload.push(remaining);
        payload.push(0x00); // Dummy CRC

        match parse_packet(&payload) {
            Some(CrsfPacket::Battery(bat)) => {
                assert_eq!(bat.voltage, voltage);
                assert_eq!(bat.current, current);
                assert_eq!(bat.capacity, capacity);
                assert_eq!(bat.remaining, remaining);
            }
            _ => panic!("Expected Battery packet"),
        }
    }

    #[test]
    fn test_parse_packet_vario() {
        // Payload: Type (1), vertical_speed (i16)
        let vspeed: i16 = -15; // -1.5 m/s
        let mut payload = Vec::new();
        payload.push(SOURCE_ADDRESS);
        payload.push(4); // Length
        payload.push(PacketType::Vario as u8);
        payload.extend_from_slice(&vspeed.to_be_bytes());
        payload.push(0x00); // Dummy CRC

        match parse_packet(&payload) {
            Some(CrsfPacket::Vario(vario)) => {
                assert_eq!(vario.vertical_speed, vspeed);
            }
            _ => panic!("Expected Vario packet"),
        }
    }

    #[test]
    fn test_parse_packet_flight_mode() {
        // Payload: Type (1), string null terminated
        let mode_str = "ACRO";
        let mut payload = Vec::new();
        payload.push(SOURCE_ADDRESS);
        payload.push((mode_str.len() + 1 + 2) as u8); // Length
        payload.push(PacketType::FlightMode as u8);
        payload.extend_from_slice(mode_str.as_bytes());
        payload.push(0); // Null terminator
        payload.push(0x00); // Dummy CRC

        match parse_packet(&payload) {
            Some(CrsfPacket::FlightMode(fm)) => {
                assert_eq!(fm.mode, "ACRO");
            }
            _ => panic!("Expected FlightMode packet"),
        }
    }

    #[test]
    fn test_parse_packet_short_payload() {
        // Test with payload too short (just type)
        let payload = [PacketType::Attitude as u8];
        assert!(parse_packet(&payload).is_none());

        let payload = [PacketType::Gps as u8];
        assert!(parse_packet(&payload).is_none());

        let payload = [PacketType::BatterySensor as u8];
        assert!(parse_packet(&payload).is_none());

        let payload = [PacketType::Vario as u8];
        assert!(parse_packet(&payload).is_none());

        // Empty payload
        let payload = [];
        assert!(parse_packet(&payload).is_none());
    }

    #[test]
    fn test_parse_packet_unknown() {
        let payload = [
            SOURCE_ADDRESS,
            5,
            PacketType::LinkStatistics as u8,
            1,
            2,
            3,
            0x00,
        ];
        match parse_packet(&payload) {
            Some(CrsfPacket::Unknown(pt)) => assert_eq!(pt, PacketType::LinkStatistics),
            _ => panic!("Expected Unknown packet"),
        }
    }

    #[test]
    fn test_build_packet_gps() {
        let gps = Gps {
            lat: 525_200_000,
            lon: 134_050_000,
            speed: 1000,
            heading: 18000,
            alt: 1500,
            sats: 8,
        };
        let packet = CrsfPacket::Gps(gps.clone());
        let built = build_packet(SOURCE_ADDRESS, &packet).unwrap();

        let parsed = parse_packet_check(&built).unwrap();
        if let CrsfPacket::Gps(p_gps) = parsed {
            assert_eq!(p_gps.lat, gps.lat);
            assert_eq!(p_gps.lon, gps.lon);
            assert_eq!(p_gps.speed, gps.speed);
            assert_eq!(p_gps.heading, gps.heading);
            assert_eq!(p_gps.alt, gps.alt);
            assert_eq!(p_gps.sats, gps.sats);
        } else {
            panic!("Round trip failed for GPS");
        }
    }

    #[test]
    fn test_build_packet_attitude() {
        let att = Attitude {
            pitch: 10000,
            roll: -5000,
            yaw: 1000,
        };
        let packet = CrsfPacket::Attitude(att.clone());
        let built = build_packet(SOURCE_ADDRESS, &packet).unwrap();

        let parsed = parse_packet_check(&built).unwrap();
        if let CrsfPacket::Attitude(p_att) = parsed {
            assert_eq!(p_att.pitch, att.pitch);
            assert_eq!(p_att.roll, att.roll);
            assert_eq!(p_att.yaw, att.yaw);
        } else {
            panic!("Round trip failed for Attitude");
        }
    }

    #[test]
    fn test_build_packet_battery() {
        let bat = Battery {
            voltage: 120,
            current: 10,
            capacity: 1000,
            remaining: 50,
        };
        let packet = CrsfPacket::Battery(bat.clone());
        let built = build_packet(SOURCE_ADDRESS, &packet).unwrap();

        let parsed = parse_packet(&built).unwrap();
        if let CrsfPacket::Battery(p_bat) = parsed {
            assert_eq!(p_bat.voltage, bat.voltage);
            assert_eq!(p_bat.current, bat.current);
            assert_eq!(p_bat.capacity, bat.capacity);
            assert_eq!(p_bat.remaining, bat.remaining);
        } else {
            panic!("Round trip failed for Battery");
        }
    }

    #[test]
    fn test_build_packet_vario() {
        let vario = Vario {
            vertical_speed: -100,
        };
        let packet = CrsfPacket::Vario(vario.clone());
        let built = build_packet(SOURCE_ADDRESS, &packet).unwrap();

        let parsed = parse_packet_check(&built).unwrap();
        if let CrsfPacket::Vario(p_vario) = parsed {
            assert_eq!(p_vario.vertical_speed, vario.vertical_speed);
        } else {
            panic!("Round trip failed for Vario");
        }
    }

    #[test]
    fn test_build_packet_flight_mode() {
        let mode = FlightMode {
            mode: "ACRO".to_string(),
        };
        let packet = CrsfPacket::FlightMode(mode.clone());
        let built = build_packet(SOURCE_ADDRESS, &packet).unwrap();

        let parsed = parse_packet_check(&built).unwrap();
        if let CrsfPacket::FlightMode(p_mode) = parsed {
            assert_eq!(p_mode.mode, mode.mode);
        } else {
            panic!("Round trip failed for FlightMode");
        }
    }

    #[test]
    fn test_build_packet_baro_alt() {
        let baro = BaroAlt {
            alt: 500,
            vertical_speed: 10,
        };
        let packet = CrsfPacket::BaroAlt(baro.clone());
        let built = build_packet(SOURCE_ADDRESS, &packet).unwrap();

        let parsed = parse_packet_check(&built).unwrap();
        if let CrsfPacket::BaroAlt(p_baro) = parsed {
            assert_eq!(p_baro.alt, baro.alt);
            assert_eq!(p_baro.vertical_speed, baro.vertical_speed);
        } else {
            panic!("Round trip failed for BaroAlt");
        }
    }

    #[test]
    fn test_build_packet_airspeed() {
        let air = Airspeed { speed: 500 };
        let packet = CrsfPacket::Airspeed(air.clone());
        let built = build_packet(SOURCE_ADDRESS, &packet).unwrap();

        let parsed = parse_packet_check(&built).unwrap();
        if let CrsfPacket::Airspeed(p_air) = parsed {
            assert_eq!(p_air.speed, air.speed);
        } else {
            panic!("Round trip failed for Airspeed");
        }
    }

    #[test]
    fn test_build_packet_rpm() {
        let rpm = Rpm {
            source_id: 1,
            rpms: vec![1000, 2000],
        };
        let packet = CrsfPacket::Rpm(rpm.clone());
        let built = build_packet(SOURCE_ADDRESS, &packet).unwrap();

        // Manual verification of built packet structure for RPM
        // Framing (4) + Source (1) + 3 bytes * 2
        assert_eq!(built.len(), 4 + 1 + 6);
        assert_eq!(built[2], PacketType::Rpm as u8);
        assert_eq!(built[3], 1);
        // 1000 = 0x0003E8 -> 00 03 E8
        assert_eq!(built[4], 0x00);
        assert_eq!(built[5], 0x03);
        assert_eq!(built[6], 0xE8);
        // 2000 = 0x0007D0 -> 00 07 D0
        assert_eq!(built[7], 0x00);
        assert_eq!(built[8], 0x07);
        assert_eq!(built[9], 0xD0);

        // RPM value overflow.
        let rpm = Rpm {
            source_id: 1,
            rpms: vec![0x1000000, 2000],
        };
        let packet = CrsfPacket::Rpm(rpm.clone());
        let built = build_packet(SOURCE_ADDRESS, &packet);
        assert_eq!(built, None);
    }

    #[test]
    fn test_build_packet_rc_channels() {
        let rc_channels = RcChannelsPacked {
            channels: [0x123, 12, 13, 510, 10, 0, 0, 0, 0, 0, 30, 0, 0, 0, 0, 0x7ff],
        };
        let packet = CrsfPacket::RcChannelsPacked(rc_channels.clone());
        let built = build_packet(SOURCE_ADDRESS, &packet).unwrap();
        assert_eq!(built.len(), 4 + 22);
        assert_eq!(built[2], PacketType::RcChannelsPacked as u8);

        let parsed = parse_packet_check(&built).unwrap();
        if let CrsfPacket::RcChannelsPacked(p_rc) = parsed {
            assert_eq!(p_rc.channels, rc_channels.channels);
        } else {
            panic!("Round trip failed for RcChannelsPacked");
        }

        // Channel value overflow.
        let rc_channels = RcChannelsPacked {
            channels: [0xfff, 0, 13, 510, 10, 0, 0, 0, 0, 0, 30, 0, 0, 0, 0, 0x7ff],
        };
        let packet = CrsfPacket::RcChannelsPacked(rc_channels.clone());
        let built = build_packet(SOURCE_ADDRESS, &packet);
        assert_eq!(built, None);
    }
}
