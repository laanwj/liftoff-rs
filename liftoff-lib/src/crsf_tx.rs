use crate::crsf::PacketType;
use crate::geo;
use crate::telemetry::TelemetryPacket;

pub fn generate_crsf_telemetry(rec: &TelemetryPacket) -> Vec<Vec<u8>> {
    let mut packets = Vec::new();

    // GPS
    if let (Some(pos), Some(att), Some(vel)) = (rec.position, rec.attitude, rec.velocity) {
        let (lon, lat, alt) =
            geo::gps_from_coord(&[pos[0] as f64, pos[1] as f64, pos[2] as f64], (0.0, 0.0));
        let hdg = geo::quat2heading(att[0] as f64, att[1] as f64, att[2] as f64, att[3] as f64);
        let mut hdg_deg = hdg.to_degrees();
        if hdg_deg < 0.0 {
            hdg_deg += 360.0;
        }

        let vel2d = (vel[0].powi(2) + vel[2].powi(2)).sqrt();

        // GPS Packet
        let mut pkt = Vec::new();
        pkt.push(PacketType::Gps as u8);
        pkt.extend_from_slice(&((lat * 10_000_000.0) as i32).to_be_bytes()); // Lat
        pkt.extend_from_slice(&((lon * 10_000_000.0) as i32).to_be_bytes()); // Lon
        pkt.extend_from_slice(&((vel2d * 3.6 * 10.0) as u16).to_be_bytes()); // Speed
        pkt.extend_from_slice(&((hdg_deg * 100.0) as u16).to_be_bytes()); // Heading
        pkt.extend_from_slice(&((alt + 1000.0) as u16).to_be_bytes()); // Alt
        pkt.push(1); // Sats
        packets.push(pkt);
    }

    // Battery
    if let Some(bat) = rec.battery {
        let mut pkt = Vec::new();
        pkt.push(PacketType::BatterySensor as u8);
        pkt.extend_from_slice(&((bat[1] * 10.0) as i16).to_be_bytes()); // Voltage
        pkt.extend_from_slice(&(0i16).to_be_bytes()); // Current
        pkt.extend_from_slice(&[0, 0, 0]); // Capacity
        pkt.push((bat[0] * 100.0) as u8); // Remaining
        packets.push(pkt);
    }

    // Vario
    if let Some(vel) = rec.velocity {
        let mut pkt = Vec::new();
        pkt.push(PacketType::Vario as u8);
        pkt.extend_from_slice(&((vel[1] * 100.0) as i16).to_be_bytes());
        packets.push(pkt);
    }

    // Attitude
    if let Some(att) = rec.attitude {
        let (pitch, roll, yaw) =
            geo::quat2eulers(att[0] as f64, att[1] as f64, att[2] as f64, att[3] as f64);
        let mut pkt = Vec::new();
        pkt.push(PacketType::Attitude as u8);
        pkt.extend_from_slice(&((pitch * 10000.0) as i16).to_be_bytes());
        pkt.extend_from_slice(&((roll * 10000.0) as i16).to_be_bytes());
        pkt.extend_from_slice(&((yaw * 10000.0) as i16).to_be_bytes());
        packets.push(pkt);
    }

    // Baro Alt
    if let Some(pos) = rec.position {
        let (_lon, _lat, alt) =
            geo::gps_from_coord(&[pos[0] as f64, pos[1] as f64, pos[2] as f64], (0.0, 0.0));
        let mut pkt = Vec::new();
        pkt.push(PacketType::BaroAlt as u8);

        let mut alt_packed = ((alt * 10.0) as i32) + 10000;
        if alt_packed < 0 {
            alt_packed = 0;
        } else if alt_packed > 0x7fff {
            alt_packed = 0x8000 | (alt as i32).min(0x7fff);
        }

        pkt.extend_from_slice(&(alt_packed as u16).to_be_bytes());
        pkt.push(0); // Vertical speed (packed, ignored)
        packets.push(pkt);
    }

    // Airspeed
    if let Some(vel) = rec.velocity {
        let vel3d = (vel[0].powi(2) + vel[1].powi(2) + vel[2].powi(2)).sqrt();
        let mut pkt = Vec::new();
        pkt.push(PacketType::Airspeed as u8);
        pkt.extend_from_slice(&((vel3d * 3.6 * 10.0) as u16).to_be_bytes());
        packets.push(pkt);
    }

    // RPM
    if let Some(rpms) = &rec.motor_rpm {
        let mut pkt = Vec::new();
        pkt.push(PacketType::Rpm as u8);
        pkt.push(0); // Source ID
        for rpm in rpms {
            let val = *rpm as u32;
            let bytes = val.to_be_bytes(); // 4 bytes
            pkt.extend_from_slice(&bytes[1..4]); // Take last 3 bytes
        }
        packets.push(pkt);
    }

    packets
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::telemetry::TelemetryPacket;

    #[test]
    fn test_generate_crsf_telemetry_empty() {
        let rec = TelemetryPacket {
            timestamp: None,
            position: None,
            attitude: None,
            velocity: None,
            gyro: None,
            input: None,
            battery: None,
            motor_rpm: None,
        };
        let packets = generate_crsf_telemetry(&rec);
        assert!(packets.is_empty());
    }

    #[test]
    fn test_generate_crsf_telemetry_full() {
        let rec = TelemetryPacket {
            timestamp: Some(123.45),
            position: Some([10.0, 100.0, 20.0]), // x, z(alt), y
            attitude: Some([0.0, 0.0, 0.0, 1.0]), // Identity quaternion
            velocity: Some([10.0, 0.0, 0.0]), // 10m/s X velocity
            gyro: None,
            input: None,
            battery: Some([0.5, 12.0]), // 50%, 12V
            motor_rpm: Some(vec![1000.0, 2000.0]),
        };

        let packets = generate_crsf_telemetry(&rec);
        assert!(!packets.is_empty());

        // Check for specific packet types
        let packet_types: Vec<u8> = packets.iter().map(|p| p[0]).collect();
        assert!(packet_types.contains(&(PacketType::Gps as u8)));
        assert!(packet_types.contains(&(PacketType::BatterySensor as u8)));
        assert!(packet_types.contains(&(PacketType::Vario as u8))); // Generated from velocity
        assert!(packet_types.contains(&(PacketType::Attitude as u8)));
        assert!(packet_types.contains(&(PacketType::BaroAlt as u8))); // Generated from position
        assert!(packet_types.contains(&(PacketType::Airspeed as u8))); // Generated from velocity
        assert!(packet_types.contains(&(PacketType::Rpm as u8)));
    }
}
