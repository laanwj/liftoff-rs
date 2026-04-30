use crate::crsf::{self, CrsfPacket, build_packet};
use crate::geo;
use crate::simstate::BatteryPacket;
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

    let speed_kmh = vel2d as f64 * 3.6;
    let gps = crsf::Gps::from_values(lat, lon, alt, speed_kmh, hdg_deg, 1)?;
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

/// Build a CRSF BatterySensor packet from the simstate-bridge `BatteryPacket`,
/// which carries the full set of fields the standard sim telemetry stream
/// doesn't expose: instantaneous current draw and accumulated mAh drawn.
fn build_battery_packet_from_lfbt(bat: &BatteryPacket) -> Option<Vec<u8>> {
    if !bat.has_data() {
        return None;
    }
    let battery = crsf::Battery {
        voltage: (bat.voltage * 10.0) as u16,
        current: (bat.current_amps * 10.0) as u16,
        capacity: (bat.charge_drawn_ah * 1000.0) as u32,
        remaining: (bat.percentage * 100.0).clamp(0.0, 255.0) as u8,
    };
    build_packet(SOURCE_ADDRESS, &CrsfPacket::Battery(battery))
}

/// Build a CRSF Voltages packet (per-cell voltages) from the simstate-bridge
/// `BatteryPacket`. The sim does not model per-cell variation, so all cells
/// report the same `voltage_per_cell` value.
fn build_voltages_packet_from_lfbt(bat: &BatteryPacket) -> Option<Vec<u8>> {
    if !bat.has_data() || bat.cell_count == 0 {
        return None;
    }
    let mv = (bat.voltage_per_cell * 1000.0).clamp(0.0, u16::MAX as f32) as u16;
    let voltages = crsf::Voltages {
        source_id: 0,
        voltages_mv: vec![mv; bat.cell_count as usize],
    };
    build_packet(SOURCE_ADDRESS, &CrsfPacket::Voltages(voltages))
}

fn build_vario_packet(rec: &TelemetryPacket) -> Option<Vec<u8>> {
    let velocity = rec.velocity?;
    let vario = crsf::Vario::from_ms(velocity[1] as f64)?;
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
    let att = crsf::Attitude::from_radians(pitch, roll, yaw)?;
    build_packet(SOURCE_ADDRESS, &CrsfPacket::Attitude(att))
}

fn build_baro_alt_packet(rec: &TelemetryPacket) -> Option<Vec<u8>> {
    let position = rec.position?;
    let (_lon, _lat, alt) = geo::gps_from_coord(
        &[position[0] as f64, position[1] as f64, position[2] as f64],
        (0.0, 0.0),
    );
    let baro = crsf::BaroAlt::from_values(alt, 0.0)?;
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

/// Build the full CRSF telemetry packet set for a single sample.
///
/// `battery_lfbt`, when provided, takes precedence for the BatterySensor packet
/// (giving real current and mAh-drawn instead of the standard telemetry's
/// voltage+percent only) and additionally produces a per-cell Voltages packet.
pub fn generate_crsf_telemetry(
    rec: &TelemetryPacket,
    battery_lfbt: Option<&BatteryPacket>,
) -> Vec<Vec<u8>> {
    let mut packets = Vec::new();
    packets.extend(build_gps_packet(rec));
    // Prefer LFBT when it has valid data; fall back to the standard
    // telemetry's voltage+percentage if the battery sim is off
    // (NO_DRAINER) or there's no current drone.
    let lfbt_battery = battery_lfbt.and_then(build_battery_packet_from_lfbt);
    if lfbt_battery.is_some() {
        packets.extend(lfbt_battery);
        packets.extend(battery_lfbt.and_then(build_voltages_packet_from_lfbt));
    } else {
        packets.extend(build_battery_packet(rec));
    }
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
        let packets = generate_crsf_telemetry(&rec, None);
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

        let packets = generate_crsf_telemetry(&rec, None);
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
        // No LFBT supplied → no Voltages packet.
        assert!(!packet_types.contains(&(PacketType::Voltages as u8)));
    }

    #[test]
    fn test_generate_crsf_telemetry_with_lfbt_battery() {
        let rec = TelemetryPacket {
            timestamp: Some(1.0),
            position: None,
            attitude: None,
            velocity: None,
            gyro: None,
            input: None,
            // Standard-stream battery would normally provide voltage+percent
            // only; assert we still prefer the LFBT data when both are present.
            battery: Some([0.5, 12.0]),
            motor_rpm: None,
        };
        let lfbt = BatteryPacket {
            version: 1,
            flags: 0, // has data
            timestamp_ms: 0,
            cell_count: 4,
            voltage: 15.4,
            voltage_per_cell: 3.85,
            current_amps: 22.5,
            charge_drawn_ah: 0.25,
            percentage: 0.78,
        };

        let packets = generate_crsf_telemetry(&rec, Some(&lfbt));
        let packet_types: Vec<u8> = packets.iter().map(|p| p[2]).collect();
        assert!(packet_types.contains(&(PacketType::BatterySensor as u8)));
        assert!(packet_types.contains(&(PacketType::Voltages as u8)));

        // Round-trip the BatterySensor packet to confirm LFBT values were used.
        let bat_frame = packets
            .iter()
            .find(|p| p[2] == PacketType::BatterySensor as u8)
            .expect("battery packet present");
        match crsf::parse_packet(bat_frame).unwrap() {
            CrsfPacket::Battery(b) => {
                assert_eq!(b.voltage, 154); // 15.4 V × 10
                assert_eq!(b.current, 225); // 22.5 A × 10
                assert_eq!(b.capacity, 250); // 0.25 Ah × 1000 = 250 mAh
                assert_eq!(b.remaining, 78);
            }
            _ => panic!("expected BatterySensor"),
        }
    }

    #[test]
    fn test_lfbt_no_data_falls_back_to_telemetry() {
        let rec = TelemetryPacket {
            timestamp: None,
            position: None,
            attitude: None,
            velocity: None,
            gyro: None,
            input: None,
            battery: Some([0.5, 12.0]),
            motor_rpm: None,
        };
        // NO_DRAINER flag set → has_data() == false → no LFBT-sourced packets.
        let lfbt = BatteryPacket {
            version: 1,
            flags: BatteryPacket::FLAG_NO_DRAINER,
            timestamp_ms: 0,
            cell_count: 0,
            voltage: 0.0,
            voltage_per_cell: 0.0,
            current_amps: 0.0,
            charge_drawn_ah: 0.0,
            percentage: 0.0,
        };
        let packets = generate_crsf_telemetry(&rec, Some(&lfbt));
        let packet_types: Vec<u8> = packets.iter().map(|p| p[2]).collect();
        // Without LFBT data we fall back to the standard-telemetry
        // BatterySensor (voltage+percent only). No Voltages packet.
        assert!(packet_types.contains(&(PacketType::BatterySensor as u8)));
        assert!(!packet_types.contains(&(PacketType::Voltages as u8)));
    }
}
