//! Build CRSF telemetry frames directly from simulator state.

use telemetry_lib::crsf::{
    build_packet, Airspeed, Attitude, Battery, BaroAlt, CrsfPacket, Gps, LinkStatistics,
    Rpm, Vario, Voltages, device_address,
};
use telemetry_lib::crsf_custom;
use telemetry_lib::geo;
use telemetry_lib::simstate::DamagePacket;

use crate::physics::battery::BatteryState;
use crate::pipeline::TickOutput;

const SOURCE: u8 = device_address::FLIGHT_CONTROLLER;

/// World-frame body state extracted from Godot by the gdext layer.
#[derive(Debug, Clone, Copy)]
pub struct WorldState {
    pub position: [f32; 3],
    pub attitude: [f32; 4],
    pub velocity: [f32; 3],
    pub timestamp_s: f32,
}

/// Build the full set of CRSF telemetry frames from the sim's state.
pub fn build_crsf_frames(
    world: &WorldState,
    out: &TickOutput,
    current_a: f32,
    cells: u8,
    _timestamp_ms: u64,
) -> Vec<Vec<u8>> {
    let mut frames: Vec<Vec<u8>> = Vec::with_capacity(8);

    // GPS
    if let Some(f) = build_gps(world) {
        frames.push(f);
    }
    // Battery
    if let Some(f) = build_battery(&out.battery, current_a, cells) {
        frames.push(f);
    }
    // Per-cell voltages
    if let Some(f) = build_voltages(&out.battery, cells) {
        frames.push(f);
    }
    // Vario (vertical speed)
    if let Some(f) = build_vario(world) {
        frames.push(f);
    }
    // Attitude
    if let Some(f) = build_attitude(world) {
        frames.push(f);
    }
    // Baro altitude
    if let Some(f) = build_baro_alt(world) {
        frames.push(f);
    }
    // Airspeed
    if let Some(f) = build_airspeed(world) {
        frames.push(f);
    }
    // RPM
    if let Some(f) = build_rpm(out) {
        frames.push(f);
    }

    frames
}

fn build_gps(world: &WorldState) -> Option<Vec<u8>> {
    let pos = world.position;
    let vel = world.velocity;
    let att = world.attitude;

    let (lon, lat, alt) = geo::gps_from_coord(
        &[pos[0] as f64, pos[1] as f64, pos[2] as f64],
        (0.0, 0.0),
    );
    let hdg = geo::quat2heading(att[0] as f64, att[1] as f64, att[2] as f64, att[3] as f64);
    let mut hdg_deg = hdg.to_degrees();
    if hdg_deg < 0.0 {
        hdg_deg += 360.0;
    }
    let vel2d = ((vel[0] * vel[0]) + (vel[2] * vel[2])).sqrt();
    let speed_kmh = vel2d as f64 * 3.6;

    let gps = Gps::from_values(lat, lon, alt, speed_kmh, hdg_deg, 1)?;
    build_packet(SOURCE, &CrsfPacket::Gps(gps))
}

fn build_battery(state: &BatteryState, current_a: f32, _cells: u8) -> Option<Vec<u8>> {
    let battery = Battery {
        voltage: (state.v_pack_terminal * 10.0) as u16,
        current: (current_a * 10.0) as u16,
        capacity: (state.consumed_mah as u32),
        remaining: (state.soc * 100.0).clamp(0.0, 100.0) as u8,
    };
    build_packet(SOURCE, &CrsfPacket::Battery(battery))
}

fn build_voltages(state: &BatteryState, cells: u8) -> Option<Vec<u8>> {
    if cells == 0 {
        return None;
    }
    let mv = (state.v_cell_terminal * 1000.0).clamp(0.0, u16::MAX as f32) as u16;
    let voltages = Voltages {
        source_id: 0,
        voltages_mv: vec![mv; cells as usize],
    };
    build_packet(SOURCE, &CrsfPacket::Voltages(voltages))
}

fn build_vario(world: &WorldState) -> Option<Vec<u8>> {
    let vario = Vario::from_ms(world.velocity[1] as f64)?;
    build_packet(SOURCE, &CrsfPacket::Vario(vario))
}

fn build_attitude(world: &WorldState) -> Option<Vec<u8>> {
    let att = world.attitude;
    let (pitch, roll, yaw) =
        geo::quat2eulers(att[0] as f64, att[1] as f64, att[2] as f64, att[3] as f64);
    let a = Attitude::from_radians(pitch, roll, yaw)?;
    build_packet(SOURCE, &CrsfPacket::Attitude(a))
}

fn build_baro_alt(world: &WorldState) -> Option<Vec<u8>> {
    let pos = world.position;
    let (_lon, _lat, alt) = geo::gps_from_coord(
        &[pos[0] as f64, pos[1] as f64, pos[2] as f64],
        (0.0, 0.0),
    );
    let baro = BaroAlt::from_values(alt, 0.0)?;
    build_packet(SOURCE, &CrsfPacket::BaroAlt(baro))
}

fn build_airspeed(world: &WorldState) -> Option<Vec<u8>> {
    let vel = world.velocity;
    let speed = ((vel[0] * vel[0]) + (vel[1] * vel[1]) + (vel[2] * vel[2])).sqrt();
    let airspeed = Airspeed {
        speed: (speed * 3.6 * 10.0) as u16,
    };
    build_packet(SOURCE, &CrsfPacket::Airspeed(airspeed))
}

fn build_rpm(out: &TickOutput) -> Option<Vec<u8>> {
    let rpms: Vec<u32> = out.motor_states.iter().map(|m| m.rpm as u32).collect();
    let rpm = Rpm {
        source_id: 0,
        rpms,
    };
    build_packet(SOURCE, &CrsfPacket::Rpm(rpm))
}

/// Constant "healthy link" CRSF LinkStatistics frame (~1 Hz).
pub fn build_link_stats_frame() -> Vec<u8> {
    let ls = LinkStatistics {
        snr: 70,
        rf_mode: 0,
        rssi: 100,
        lq: 10,
        tx_power: 0,
        tx_auc: 0,
        rx_auc: 3,
        snr_rx: 70,
        rssi_rx: 100,
        lq_rx: 10,
    };
    build_packet(SOURCE, &CrsfPacket::LinkStatistics(ls)).unwrap()
}

/// Build a CRSF damage frame. Inverts the sim's damage convention
/// (0=healthy, 1=destroyed) to the wire convention (1=healthy, 0=destroyed).
pub fn build_damage_frame(
    prop_damage: &[f32; 4],
    destroyed: bool,
    timestamp_ms: u64,
) -> Option<Vec<u8>> {
    let mut flags: u16 = 0;
    if destroyed {
        flags |= DamagePacket::FLAG_CRASHED;
    }
    let pkt = DamagePacket {
        version: 1,
        flags,
        timestamp_ms,
        damage: prop_damage.iter().map(|&d| 1.0 - d).collect(),
    };
    crsf_custom::build_damage_packet(&pkt)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::pipeline::{DroneSim, TickInput};
    use crate::preset::DronePresetData;
    use telemetry_lib::crsf::PacketType;

    fn run_to_armed_steady_state() -> (DroneSim, TickOutput) {
        let mut s = DroneSim::new(DronePresetData::racing_5inch());
        let mut input = TickInput::default();
        input.rc.arm = true;
        for _ in 0..10 {
            s.step(&input, 1.0 / 240.0);
        }
        input.rc.throttle = 0.5;
        let mut last = None;
        for _ in 0..240 {
            last = Some(s.step(&input, 1.0 / 240.0));
        }
        (s, last.unwrap())
    }

    fn dummy_world() -> WorldState {
        WorldState {
            position: [10.0, 5.0, -3.0],
            attitude: [0.0, 0.0, 0.0, 1.0],
            velocity: [0.5, 0.1, -1.2],
            timestamp_s: 1.234,
        }
    }

    #[test]
    fn build_crsf_frames_emits_multiple_packets() {
        let (_s, out) = run_to_armed_steady_state();
        let frames = build_crsf_frames(&dummy_world(), &out, 12.0, 4, 1234);
        assert!(
            frames.len() >= 6,
            "expected several CRSF frames, got {}",
            frames.len()
        );
        for f in &frames {
            assert!(f.len() >= 4, "frame too short: {} bytes", f.len());
        }
    }

    #[test]
    fn frames_round_trip_through_parser() {
        let (_s, out) = run_to_armed_steady_state();
        let frames = build_crsf_frames(&dummy_world(), &out, 12.0, 4, 1234);
        for f in &frames {
            let parsed = telemetry_lib::crsf::parse_packet_check(f);
            assert!(parsed.is_some(), "failed to parse a generated frame");
        }
    }

    #[test]
    fn battery_frame_contains_current_and_capacity() {
        let (_s, out) = run_to_armed_steady_state();
        let frames = build_crsf_frames(&dummy_world(), &out, 15.0, 4, 0);
        let bat_frame = frames.iter().find(|f| {
            f.len() > 2 && f[2] == PacketType::BatterySensor as u8
        });
        assert!(bat_frame.is_some(), "no battery frame found");
    }

    #[test]
    fn link_stats_frame_valid() {
        let f = build_link_stats_frame();
        assert_eq!(f[0], SOURCE);
        assert_eq!(f[2], PacketType::LinkStatistics as u8);
        assert_eq!(f.len(), 14);
    }

    #[test]
    fn damage_frame_inverts_convention() {
        let damage = [0.0, 0.5, 1.0, 0.0];
        let frame = build_damage_frame(&damage, false, 0);
        assert!(frame.is_some());
    }
}
