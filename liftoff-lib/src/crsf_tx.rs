use crate::crsf::{self, CrsfPacket, build_packet};
use crate::geo;
use crate::telemetry::TelemetryPacket;

const SOURCE_ADDRESS: u8 = crsf::device_address::FLIGHT_CONTROLLER;

fn build_gps_packet(rec: &TelemetryPacket) -> Option<Vec<u8>> {
    let position = rec.position?;
    let attitude = rec.attitude?;
    let velocity = rec.velocity?;

    let (lon, lat, alt) = geo::gps_from_coord(
        &[position[0] as f64, position[1] as f64, position[2] as f64],
        (0.0, 0.0),
    );
    let hdg = geo::quat2heading(
        attitude[0] as f64,
        attitude[1] as f64,
        attitude[2] as f64,
        attitude[3] as f64,
    );
    let mut hdg_deg = hdg.to_degrees();
    if hdg_deg < 0.0 {
        hdg_deg += 360.0;
    }

    let vel2d = (velocity[0].powi(2) + velocity[2].powi(2)).sqrt();

    let gps = crsf::Gps {
        lat: (lat * 10_000_000.0) as i32,
        lon: (lon * 10_000_000.0) as i32,
        speed: (vel2d * 3.6 * 10.0) as u16,
        heading: (hdg_deg * 100.0) as u16,
        alt: (alt + 1000.0) as u16,
        sats: 1,
    };
    build_packet(SOURCE_ADDRESS, &CrsfPacket::Gps(gps))
}

fn build_battery_packet(rec: &TelemetryPacket) -> Option<Vec<u8>> {
    let bat = rec.battery?;
    let battery = crsf::Battery {
        voltage: (bat[1] * 10.0) as u16,
        current: 0,
        capacity: 0,
        remaining: (bat[0] * 100.0) as u8,
    };
    build_packet(SOURCE_ADDRESS, &CrsfPacket::Battery(battery))
}

fn build_vario_packet(rec: &TelemetryPacket) -> Option<Vec<u8>> {
    let velocity = rec.velocity?;
    let vario = crsf::Vario {
        vertical_speed: (velocity[1] * 100.0) as i16,
    };
    build_packet(SOURCE_ADDRESS, &CrsfPacket::Vario(vario))
}

fn build_attitude_packet(rec: &TelemetryPacket) -> Option<Vec<u8>> {
    let attitude = rec.attitude?;
    let (pitch, roll, yaw) = geo::quat2eulers(
        attitude[0] as f64,
        attitude[1] as f64,
        attitude[2] as f64,
        attitude[3] as f64,
    );
    let att = crsf::Attitude {
        pitch: (pitch * 10_000.0) as i16,
        roll: (roll * 10_000.0) as i16,
        yaw: (yaw * 10_000.0) as i16,
    };
    build_packet(SOURCE_ADDRESS, &CrsfPacket::Attitude(att))
}

fn build_baro_alt_packet(rec: &TelemetryPacket) -> Option<Vec<u8>> {
    let position = rec.position?;
    let (_lon, _lat, alt) = geo::gps_from_coord(
        &[position[0] as f64, position[1] as f64, position[2] as f64],
        (0.0, 0.0),
    );

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
    build_packet(SOURCE_ADDRESS, &CrsfPacket::BaroAlt(baro))
}

fn build_airspeed_packet(rec: &TelemetryPacket) -> Option<Vec<u8>> {
    let velocity = rec.velocity?;
    let vel3d = (velocity[0].powi(2) + velocity[1].powi(2) + velocity[2].powi(2)).sqrt();
    let airspeed = crsf::Airspeed {
        speed: (vel3d * 3.6 * 10.0) as u16,
    };
    build_packet(SOURCE_ADDRESS, &CrsfPacket::Airspeed(airspeed))
}

fn build_rpm_packet(rec: &TelemetryPacket) -> Option<Vec<u8>> {
    let rpms = rec.motor_rpm.as_ref()?;
    let rpm = crsf::Rpm {
        source_id: 0,
        rpms: rpms.iter().map(|&r| r as u32).collect(),
    };
    build_packet(SOURCE_ADDRESS, &CrsfPacket::Rpm(rpm))
}

pub fn generate_crsf_telemetry(rec: &TelemetryPacket) -> Vec<Vec<u8>> {
    let mut packets = Vec::new();
    packets.extend(build_gps_packet(rec));
    packets.extend(build_battery_packet(rec));
    packets.extend(build_vario_packet(rec));
    packets.extend(build_attitude_packet(rec));
    packets.extend(build_baro_alt_packet(rec));
    packets.extend(build_airspeed_packet(rec));
    packets.extend(build_rpm_packet(rec));
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
        let packet_types: Vec<u8> = packets.iter().map(|p| p[2]).collect();
        assert!(packet_types.contains(&(PacketType::Gps as u8)));
        assert!(packet_types.contains(&(PacketType::BatterySensor as u8)));
        assert!(packet_types.contains(&(PacketType::Vario as u8))); // Generated from velocity
        assert!(packet_types.contains(&(PacketType::Attitude as u8)));
        assert!(packet_types.contains(&(PacketType::BaroAlt as u8))); // Generated from position
        assert!(packet_types.contains(&(PacketType::Airspeed as u8))); // Generated from velocity
        assert!(packet_types.contains(&(PacketType::Rpm as u8)));
    }
}
