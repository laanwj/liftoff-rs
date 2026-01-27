use crate::crsf::{self, CrsfPacket, build_packet};
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
        let gps = crsf::Gps {
            lat: (lat * 10_000_000.0) as i32,
            lon: (lon * 10_000_000.0) as i32,
            speed: (vel2d * 3.6 * 10.0) as u16,
            heading: (hdg_deg * 100.0) as u16,
            alt: (alt + 1000.0) as u16,
            sats: 1,
        };
        packets.push(build_packet(&CrsfPacket::Gps(gps)).unwrap());
    }

    // Battery
    if let Some(bat) = rec.battery {
        let battery = crsf::Battery {
            voltage: (bat[1] * 10.0) as u16,
            current: 0,
            capacity: 0,
            remaining: (bat[0] * 100.0) as u8,
        };
        packets.push(build_packet(&CrsfPacket::Battery(battery)).unwrap());
    }

    // Vario
    if let Some(vel) = rec.velocity {
        let vario = crsf::Vario {
            vertical_speed: (vel[1] * 100.0) as i16,
        };
        packets.push(build_packet(&CrsfPacket::Vario(vario)).unwrap());
    }

    // Attitude
    if let Some(att) = rec.attitude {
        let (pitch, roll, yaw) =
            geo::quat2eulers(att[0] as f64, att[1] as f64, att[2] as f64, att[3] as f64);
        let attitude = crsf::Attitude {
            pitch: (pitch * 10_000.0) as i16,
            roll: (roll * 10_000.0) as i16,
            yaw: (yaw * 10_000.0) as i16,
        };
        packets.push(build_packet(&CrsfPacket::Attitude(attitude)).unwrap());
    }

    // Baro Alt
    if let Some(pos) = rec.position {
        let (_lon, _lat, alt) =
            geo::gps_from_coord(&[pos[0] as f64, pos[1] as f64, pos[2] as f64], (0.0, 0.0));

        let mut alt_packed = ((alt * 10.0) as i32) + 10000;
        if alt_packed < 0 {
            alt_packed = 0;
        } else if alt_packed > 0x7fff {
            alt_packed = 0x8000 | (alt as i32).min(0x7fff);
        }

        let baro = crsf::BaroAlt {
            alt: alt_packed as u16,
            vertical_speed: 0,
        };
        packets.push(build_packet(&CrsfPacket::BaroAlt(baro)).unwrap());
    }

    // Airspeed
    if let Some(vel) = rec.velocity {
        let vel3d = (vel[0].powi(2) + vel[1].powi(2) + vel[2].powi(2)).sqrt();
        let airspeed = crsf::Airspeed {
            speed: (vel3d * 3.6 * 10.0) as u16,
        };
        packets.push(build_packet(&CrsfPacket::Airspeed(airspeed)).unwrap());
    }

    // RPM
    if let Some(rpms) = &rec.motor_rpm {
        let rpm = crsf::Rpm {
            source_id: 0,
            rpms: rpms.iter().map(|&r| r as u32).collect(),
        };
        packets.push(build_packet(&CrsfPacket::Rpm(rpm)).unwrap());
    }

    packets
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::crsf::PacketType;
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
            position: Some([10.0, 100.0, 20.0]),  // x, z(alt), y
            attitude: Some([0.0, 0.0, 0.0, 1.0]), // Identity quaternion
            velocity: Some([10.0, 0.0, 0.0]),     // 10m/s X velocity
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
