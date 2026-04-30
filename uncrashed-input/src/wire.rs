//! `UCFV` wire-format parser for the Uncrashed → uncrashed-input UDP stream.
//!
//! The authoritative byte-level spec lives in
//! `uncrashed-telemetry-mod/include/wire.h` (the C++ mod's view of the same
//! struct). Keep that file and this one in lock-step when bumping the
//! version.
//!
//! Style follows `liftoff_lib::simstate`: 4-byte ASCII tag, little-endian
//! header, optional variable tail. The `UCFV` packet packs the entire
//! per-tick telemetry in one datagram so the receiver sees a coherent
//! snapshot (no LFDM/LFBT-style per-aspect demux).
use byteorder::{ByteOrder, LittleEndian};
use liftoff_lib::simstate::DamagePacket;
use liftoff_lib::telemetry::TelemetryPacket;

/// Tag at offset 0..4 in every `UCFV` datagram.
pub const TAG: &[u8; 4] = b"UCFV";

/// Wire-format version. Bump when adding/removing fields.
pub const VERSION: u16 = 2;

/// Fixed-header size (everything before the damage tail).
pub const HEADER_SIZE: usize = 108;

/// Maximum prop count we ever expect to see. Matches `DamagePacket::MAX_PROPS`
/// so the meaningful prefix of a UCFV damage block re-emits as `LFDM`
/// 1-for-1.
pub const MAX_PROPS: usize = 8;

/// Total wire size: `HEADER_SIZE` plus a `MAX_PROPS × f32` damage block.
/// Damage slots beyond `prop_count` are garbage; only `[0, prop_count)`
/// should be exposed to consumers, and only when `HAS_DAMAGE` is set.
pub const PACKET_SIZE: usize = HEADER_SIZE + 4 * MAX_PROPS;

/// Output-index → Uncrashed-`Props_*`-index mapping for the per-prop
/// damage array. Liftoff/CRSF dashboard convention is
/// `[LF, RF, LB, RB]` (left-front, right-front, left-back, right-back);
/// Uncrashed's `Props_1..Props_4` were determined empirically to be
/// `[RB, LB, RF, LF]`, so the mapping is a full reversal of the first
/// four slots. The trailing slots stay identity in case a future drone
/// build uses more than four props (no calibration done for those yet).
pub const PROP_ORDER: [usize; MAX_PROPS] = [3, 2, 1, 0, 4, 5, 6, 7];

// ─── flag bits ──────────────────────────────────────────────────────────────
pub const FLAG_ON_GROUND:   u16 = 1 << 0;
pub const FLAG_ARMED:       u16 = 1 << 1;
pub const FLAG_HAS_BATTERY: u16 = 1 << 2;
pub const FLAG_HAS_DAMAGE:  u16 = 1 << 3;
pub const FLAG_CRASHED:     u16 = 1 << 4;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParseError {
    UnknownTag,
    UnsupportedVersion,
    LengthMismatch,
    PropCountTooLarge,
}

impl std::fmt::Display for ParseError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            ParseError::UnknownTag => write!(f, "unknown packet tag (expected UCFV)"),
            ParseError::UnsupportedVersion => write!(f, "unsupported UCFV version"),
            ParseError::LengthMismatch => write!(f, "packet length is not PACKET_SIZE"),
            ParseError::PropCountTooLarge => write!(f, "prop_count exceeds MAX_PROPS"),
        }
    }
}

impl std::error::Error for ParseError {}

/// One per-tick telemetry frame from the Uncrashed C++ mod.
///
/// Units stay in the wire's native frame (UE4 cm + cm/s + deg/s); the
/// receiver normalises into liftoff conventions in [`UcfvPacket::into_telemetry`].
#[derive(Debug, Clone, PartialEq)]
pub struct UcfvPacket {
    pub version: u16,
    pub flags: u16,
    /// Microseconds since mod load, monotonic. Use `QueryPerformanceCounter`
    /// in the mod (matches `LFDM::timestamp_ms` semantics, just finer grain).
    pub timestamp_us: u64,
    pub sequence: u32,
    /// UE4 world position, cm. Axis convention: X forward, Y right, Z up.
    pub position: [f32; 3],
    /// UE4 quaternion (X, Y, Z, W). Pass-through.
    pub attitude_quat: [f32; 4],
    /// UE4 velocity, cm/s, in the same axis convention as position.
    pub velocity: [f32; 3],
    /// Body-frame angular rates, deg/s. Order: pitch, roll, yaw — to match
    /// `TelemetryPacket::gyro`.
    /// Angular velocity in **FRotator order** (Pitch, Yaw, Roll), deg/s,
    /// as Uncrashed's `Angular speed drone` UPROPERTY exposes it. The
    /// receiver reorders to liftoff/CRSF's (pitch, roll, yaw) convention
    /// in `into_telemetry`.
    pub gyro: [f32; 3],
    /// Stick inputs (throttle, yaw, pitch, roll). Range -1..1 unitless;
    /// throttle is 0..1 conventionally. Matches `TelemetryPacket::input`.
    pub inputs: [f32; 4],
    /// Raw battery fields from `BP_MyController_C.S_BatteryState`.
    /// Meaningful only when `FLAG_HAS_BATTERY` is set; otherwise garbage.
    /// All transformations (pack voltage, percent remaining, Ah scaling)
    /// happen in the Rust receiver — the wire carries the game's values
    /// verbatim.
    pub voltage_per_cell: f32, // currentvoltagepercellv (V)
    pub current_amps: f32,     // CurrentAmpdrainA (A)
    pub charge_used_mah: f32,  // Currentmah (mAh used so far)
    pub capacity_mah: i32,     // Capacitymah (pack capacity)
    pub cell_count: u8,        // cellcount
    /// Per-prop damage values exactly as Uncrashed reports them: **0.0
    /// healthy → 1.0 destroyed**, in `Props_1..Props_N` order. The
    /// receiver inverts (to liftoff/CRSF's health convention) and applies
    /// `PROP_ORDER` when building the [`DamagePacket`] for downstream
    /// consumers — see [`UcfvPacket::to_damage_packet`]. `Some` iff
    /// `FLAG_HAS_DAMAGE` is set.
    pub damage: Option<Vec<f32>>,
}

impl UcfvPacket {
    pub fn on_ground(&self) -> bool { self.flags & FLAG_ON_GROUND != 0 }
    pub fn armed(&self) -> bool { self.flags & FLAG_ARMED != 0 }
    pub fn crashed(&self) -> bool { self.flags & FLAG_CRASHED != 0 }
    pub fn has_battery(&self) -> bool { self.flags & FLAG_HAS_BATTERY != 0 }
}

/// Parse a `PACKET_SIZE`-byte UCFV envelope.
///
/// When `HAS_DAMAGE` is clear, the damage block bytes are garbage and we
/// surface `damage = None`; when set, we surface the meaningful
/// `[0, prop_count)` prefix.
pub fn parse(data: &[u8]) -> Result<UcfvPacket, ParseError> {
    if data.len() != PACKET_SIZE {
        return Err(ParseError::LengthMismatch);
    }
    if &data[0..4] != TAG.as_slice() {
        return Err(ParseError::UnknownTag);
    }
    let version = LittleEndian::read_u16(&data[4..6]);
    if version != VERSION {
        return Err(ParseError::UnsupportedVersion);
    }
    let flags = LittleEndian::read_u16(&data[6..8]);
    let timestamp_us = LittleEndian::read_u64(&data[8..16]);
    let sequence = LittleEndian::read_u32(&data[16..20]);

    let position = read_f32x3(&data[20..32]);
    let attitude_quat = read_f32x4(&data[32..48]);
    let velocity = read_f32x3(&data[48..60]);
    let gyro = read_f32x3(&data[60..72]);
    let inputs = read_f32x4(&data[72..88]);

    let voltage_per_cell = LittleEndian::read_f32(&data[88..92]);
    let current_amps     = LittleEndian::read_f32(&data[92..96]);
    let charge_used_mah  = LittleEndian::read_f32(&data[96..100]);
    let capacity_mah     = LittleEndian::read_i32(&data[100..104]);
    let cell_count       = data[104];
    let prop_count       = data[105] as usize;
    // bytes 106..108 are pad
    if prop_count > MAX_PROPS {
        return Err(ParseError::PropCountTooLarge);
    }

    let damage = (flags & FLAG_HAS_DAMAGE != 0).then(|| {
        (0..prop_count)
            .map(|i| LittleEndian::read_f32(&data[HEADER_SIZE + 4 * i..HEADER_SIZE + 4 * (i + 1)]))
            .collect()
    });

    Ok(UcfvPacket {
        version,
        flags,
        timestamp_us,
        sequence,
        position,
        attitude_quat,
        velocity,
        gyro,
        inputs,
        voltage_per_cell,
        current_amps,
        charge_used_mah,
        capacity_mah,
        cell_count,
        damage,
    })
}

fn read_f32x3(buf: &[u8]) -> [f32; 3] {
    [
        LittleEndian::read_f32(&buf[0..4]),
        LittleEndian::read_f32(&buf[4..8]),
        LittleEndian::read_f32(&buf[8..12]),
    ]
}
fn read_f32x4(buf: &[u8]) -> [f32; 4] {
    [
        LittleEndian::read_f32(&buf[0..4]),
        LittleEndian::read_f32(&buf[4..8]),
        LittleEndian::read_f32(&buf[8..12]),
        LittleEndian::read_f32(&buf[12..16]),
    ]
}

impl UcfvPacket {
    /// Convert into the workspace-shared `TelemetryPacket`.
    ///
    /// Coordinate convention: UE4 ships with X-forward, Y-right, Z-up; the
    /// liftoff `TelemetryPacket::position` is `[x, alt, z]` (index 1 = altitude
    /// per `liftoff_lib::geo::coord_from_gps`). We pull UE4's vertical axis
    /// (`Z_ue`) into slot 1 and let the horizontal pair (`X_ue`, `Y_ue`)
    /// sit in slots 0 and 2.
    ///
    /// Attitude quaternion: passed through as `(X, Y, Z, W)`. The handedness
    /// vs. liftoff is empirical and may need flipping after the first flight
    /// test — see the calibration note in `uncrashed.md`.
    ///
    /// Battery is left at `None` here — full battery info reaches CRSF via
    /// the LFBT path; see [`UcfvPacket::to_battery_packet`].
    pub fn into_telemetry(&self) -> TelemetryPacket {
        let pos_m = [
            self.position[0] / 100.0,
            self.position[2] / 100.0, // UE Z-up → liftoff altitude slot
            self.position[1] / 100.0,
        ];

        // Wire stores gyro as FRotator (Pitch, Yaw, Roll) deg/s; liftoff
        // expects (pitch, roll, yaw) — swap slots 1 and 2.
        let gyro_liftoff = [self.gyro[0], self.gyro[2], self.gyro[1]];

        TelemetryPacket {
            timestamp: Some(self.timestamp_us as f32 / 1_000_000.0),
            position: Some(pos_m),
            attitude: Some(self.attitude_quat),
            // Velocity is filled in by the receiver loop via position
            // differentiation across ticks — the game exposes no usable
            // instantaneous velocity reading.
            velocity: None,
            gyro: Some(gyro_liftoff),
            input: Some(self.inputs),
            battery: None,
            motor_rpm: None,
        }
    }

    /// Build a [`liftoff_lib::simstate::BatteryPacket`] (LFBT-style) from
    /// the raw battery fields the wire carries. Returns `None` when
    /// `HAS_BATTERY` is clear (battery sim off / data not yet valid).
    ///
    /// The LFBT path through `crsf_tx::generate_crsf_telemetry` populates
    /// the standard CRSF battery frame **and** the custom per-cell
    /// voltages frame with the correct values:
    /// - voltage_dV  = pack voltage = cell_count × per-cell voltage
    /// - current_dA  = `current_amps`
    /// - capacity    = `charge_used_mah / 1000` Ah
    /// - remaining%  = (capacity_mah − used_mah) / capacity_mah
    pub fn to_battery_packet(&self) -> Option<liftoff_lib::simstate::BatteryPacket> {
        if !self.has_battery() {
            return None;
        }
        if self.cell_count == 0 || self.capacity_mah <= 0 {
            return None;
        }
        let used = self.charge_used_mah.max(0.0);
        let capacity = self.capacity_mah as f32;
        let percentage = ((capacity - used) / capacity).clamp(0.0, 1.0);
        Some(liftoff_lib::simstate::BatteryPacket {
            version: 1,
            flags: 0,
            timestamp_ms: self.timestamp_us / 1_000,
            cell_count: self.cell_count,
            voltage: self.voltage_per_cell * self.cell_count as f32,
            voltage_per_cell: self.voltage_per_cell,
            current_amps: self.current_amps,
            charge_drawn_ah: used / 1_000.0,
            percentage,
        })
    }

    /// Build a [`DamagePacket`] (liftoff-lib's simstate damage struct) from
    /// this UCFV frame. Returns `None` when the source packet has no
    /// damage data. Consumed by `crsf_custom::build_damage_packet` to emit
    /// the 0x42 CRSF damage frame.
    ///
    /// Two conversions vs. the raw wire data:
    /// - Inverts Uncrashed's damage values (0=healthy → 1=destroyed) into
    ///   liftoff/CRSF health values (1=healthy → 0=broken).
    /// - Applies the [`PROP_ORDER`] permutation so output index `i` is
    ///   sourced from Uncrashed's `Props_{PROP_ORDER[i] + 1}`.
    pub fn to_damage_packet(&self) -> Option<DamagePacket> {
        let raw = self.damage.as_ref()?;
        let n = raw.len();
        if n > DamagePacket::MAX_PROPS {
            return None;
        }
        let mut flags: u16 = 0;
        if self.crashed() {
            flags |= DamagePacket::FLAG_CRASHED;
        }
        // We deliberately do NOT forward FLAG_KILLED — Uncrashed has no kill
        // notion modelled.
        let health: Vec<f32> = (0..n)
            .map(|i| {
                let src = PROP_ORDER[i];
                let dmg = if src < n { raw[src] } else { 0.0 };
                (1.0 - dmg).clamp(0.0, 1.0)
            })
            .collect();
        Some(DamagePacket {
            version: 1,
            flags,
            // simstate's timestamp is ms; truncate from us.
            timestamp_ms: self.timestamp_us / 1_000,
            damage: health,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn build_packet(prop_count: u8, has_damage: bool) -> Vec<u8> {
        // Test-only emitter: produces a wire image consumers should accept.
        let mut buf = Vec::new();
        buf.extend_from_slice(TAG);
        buf.extend_from_slice(&VERSION.to_le_bytes());

        let mut flags: u16 = FLAG_ARMED | FLAG_HAS_BATTERY;
        if has_damage { flags |= FLAG_HAS_DAMAGE | FLAG_CRASHED; }
        buf.extend_from_slice(&flags.to_le_bytes());

        buf.extend_from_slice(&12_345_678u64.to_le_bytes()); // timestamp_us
        buf.extend_from_slice(&42u32.to_le_bytes()); // sequence

        // position cm: 100, 200, 300 → expect 1, 3, 2 in liftoff frame
        for v in [100.0_f32, 200.0, 300.0] {
            buf.extend_from_slice(&v.to_le_bytes());
        }
        // attitude
        for v in [0.0_f32, 0.0, 0.0, 1.0] {
            buf.extend_from_slice(&v.to_le_bytes());
        }
        // velocity slot — wire carries [up_speed_ms, total_kmh, unused]
        for v in [3.0_f32, 60.0, 0.0] {
            buf.extend_from_slice(&v.to_le_bytes());
        }
        // gyro deg/s
        for v in [1.5_f32, -2.5, 3.5] {
            buf.extend_from_slice(&v.to_le_bytes());
        }
        // inputs
        for v in [0.7_f32, 0.0, 0.1, -0.1] {
            buf.extend_from_slice(&v.to_le_bytes());
        }
        // battery: per-cell V, current A, used mAh, capacity mAh, cell_count
        buf.extend_from_slice(&4.10_f32.to_le_bytes());     // voltage_per_cell
        buf.extend_from_slice(&12.5_f32.to_le_bytes());     // current_amps
        buf.extend_from_slice(&300.0_f32.to_le_bytes());    // charge_used_mah
        buf.extend_from_slice(&1500_i32.to_le_bytes());     // capacity_mah
        buf.push(6_u8);                                     // cell_count
        buf.push(prop_count);
        buf.extend_from_slice(&[0, 0]);                     // pad

        // Fill the MAX_PROPS damage slots in Uncrashed's native convention
        // (0.0 = healthy → 1.0 = destroyed). When HAS_DAMAGE is clear or
        // i ≥ prop_count, the slot is garbage from the receiver's
        // perspective and the test seeds it with 0.0.
        for i in 0..MAX_PROPS {
            let v: f32 = if has_damage && i < prop_count as usize {
                if i == 0 { 1.0 } else { 0.0 }   // first prop destroyed
            } else {
                0.0
            };
            buf.extend_from_slice(&v.to_le_bytes());
        }
        assert_eq!(buf.len(), PACKET_SIZE);
        buf
    }

    #[test]
    fn roundtrip_full_quad() {
        let buf = build_packet(4, true);
        let pkt = parse(&buf).unwrap();
        assert_eq!(pkt.version, VERSION);
        assert_eq!(pkt.sequence, 42);
        assert!(pkt.armed());
        assert!(pkt.crashed());
        // Wire carries Uncrashed's raw damage values (0=healthy, 1=destroyed)
        assert_eq!(pkt.damage.as_ref().unwrap().len(), 4);
        assert_eq!(pkt.damage.as_ref().unwrap()[0], 1.0); // first prop destroyed
        assert_eq!(pkt.damage.as_ref().unwrap()[1], 0.0); // healthy
    }

    #[test]
    fn position_cm_to_meter_and_axis_remap() {
        let buf = build_packet(0, false);
        let pkt = parse(&buf).unwrap();
        let tel = pkt.into_telemetry();
        // UE input was (100, 200, 300) cm → liftoff [X=1.0, alt=3.0, Z=2.0] m
        let pos = tel.position.unwrap();
        assert!((pos[0] - 1.0).abs() < 1e-6);
        assert!((pos[1] - 3.0).abs() < 1e-6);
        assert!((pos[2] - 2.0).abs() < 1e-6);
    }

    #[test]
    fn velocity_left_for_receiver_to_synthesise() {
        // The wire's velocity slot is unused; the receiver loop
        // populates `telemetry.velocity` via position differentiation.
        let buf = build_packet(0, false);
        let pkt = parse(&buf).unwrap();
        assert!(pkt.into_telemetry().velocity.is_none());
    }

    #[test]
    fn battery_packet_built_from_raw_fields() {
        // build_packet seeds vpc=4.10, used_mah=300, capacity_mah=1500,
        // cells=6, current=12.5. After translation:
        //   pack voltage  = 6 * 4.10 = 24.6 V
        //   percentage    = (1500 - 300) / 1500 = 0.8
        //   charge_drawn  = 300 / 1000 = 0.3 Ah
        let buf = build_packet(0, false);
        let pkt = parse(&buf).unwrap();
        let bat = pkt.to_battery_packet().expect("HAS_BATTERY set");
        assert_eq!(bat.cell_count, 6);
        assert!((bat.voltage_per_cell - 4.10).abs() < 1e-6);
        assert!((bat.voltage - 24.6).abs() < 1e-4);
        assert!((bat.current_amps - 12.5).abs() < 1e-6);
        assert!((bat.charge_drawn_ah - 0.3).abs() < 1e-6);
        assert!((bat.percentage - 0.8).abs() < 1e-6);
    }

    #[test]
    fn battery_packet_none_when_flag_clear() {
        let mut buf = build_packet(0, false);
        let flags = LittleEndian::read_u16(&buf[6..8]) & !FLAG_HAS_BATTERY;
        buf[6..8].copy_from_slice(&flags.to_le_bytes());
        let pkt = parse(&buf).unwrap();
        assert!(pkt.to_battery_packet().is_none());
    }

    #[test]
    fn telemetry_battery_is_none_so_lfbt_path_used() {
        // We always leave TelemetryPacket.battery = None — battery info
        // reaches CRSF via the LFBT-style builder.
        let buf = build_packet(0, false);
        let pkt = parse(&buf).unwrap();
        assert!(pkt.into_telemetry().battery.is_none());
    }

    #[test]
    fn damage_packet_export_for_crsf_frame() {
        // build_packet seeds Uncrashed-raw `[1.0, 0.0, 0.0, 0.0]` — first
        // prop (Uncrashed Props_1, physically RB) destroyed. After the
        // [3, 2, 1, 0] PROP_ORDER reversal, the destroyed slot lands in
        // output index 3 (dashboard's RB slot), and inversion makes the
        // healthy ones 1.0.
        let buf = build_packet(4, true);
        let pkt = parse(&buf).unwrap();
        let dmg = pkt.to_damage_packet().expect("packet has damage");
        assert_eq!(dmg.damage, vec![1.0, 1.0, 1.0, 0.0]);
        assert!(dmg.crashed());
        assert!(!dmg.killed());
        // timestamp_us 12_345_678 → ms = 12_345
        assert_eq!(dmg.timestamp_ms, 12_345);
    }

    #[test]
    fn damage_packet_applies_prop_order_swizzle() {
        // Distinct raw damage per slot so any swizzle is detectable.
        // Input order = Uncrashed [Props_1, Props_2, Props_3, Props_4]
        //             = physically [RB,     LB,      RF,      LF].
        // Output order per dashboard convention = [LF, RF, LB, RB]
        //                = Uncrashed [Props_4, Props_3, Props_2, Props_1].
        // PROP_ORDER = [3, 2, 1, 0] performs that permutation; inversion
        // turns each (1 - raw[src]).
        let mut buf = build_packet(0, false);
        let flags = LittleEndian::read_u16(&buf[6..8]) | FLAG_HAS_DAMAGE;
        buf[6..8].copy_from_slice(&flags.to_le_bytes());
        buf[96] = 4;
        let raw = [0.10_f32, 0.40, 0.70, 1.00];
        for (i, v) in raw.iter().enumerate() {
            let off = HEADER_SIZE + 4 * i;
            buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
        }
        let pkt = parse(&buf).unwrap();
        let dmg = pkt.to_damage_packet().expect("packet has damage");
        // After swizzle: [raw[3], raw[2], raw[1], raw[0]] = [1.00, 0.70, 0.40, 0.10]
        // After invert:  [1 - 1.00, 1 - 0.70, 1 - 0.40, 1 - 0.10]
        //              = [0.0, 0.30, 0.60, 0.90]
        // Use approx equality to dodge f32 representation of 0.3/0.6/0.9.
        let expected = [0.0_f32, 0.30, 0.60, 0.90];
        for (got, want) in dmg.damage.iter().zip(expected.iter()) {
            assert!((got - want).abs() < 1e-6, "{got} vs {want}");
        }
    }

    #[test]
    fn damage_packet_none_when_no_damage_flag() {
        let buf = build_packet(0, false);
        let pkt = parse(&buf).unwrap();
        assert!(pkt.to_damage_packet().is_none());
    }

    #[test]
    fn rejects_short_buffer() {
        assert!(matches!(parse(&[0; 10]), Err(ParseError::LengthMismatch)));
    }

    #[test]
    fn no_damage_flag_ignores_garbage_slots() {
        // Garbage in the damage block must not surface as Some(damage)
        // when the flag is clear.
        let buf = build_packet(0, false);
        let pkt = parse(&buf).unwrap();
        assert!(pkt.damage.is_none());
    }

    #[test]
    fn damage_flag_truncates_to_prop_count() {
        // Wire always carries MAX_PROPS damage slots; only the leading
        // prop_count of them are exposed.
        let buf = build_packet(4, true);
        let pkt = parse(&buf).unwrap();
        let dmg = pkt.damage.as_ref().unwrap();
        assert_eq!(dmg.len(), 4);
    }

    #[test]
    fn rejects_wrong_tag() {
        let mut buf = build_packet(0, false);
        buf[0] = b'X';
        assert!(matches!(parse(&buf), Err(ParseError::UnknownTag)));
    }

    #[test]
    fn rejects_length_mismatch() {
        let buf = build_packet(4, true);
        let truncated = &buf[..buf.len() - 4];
        assert!(matches!(parse(truncated), Err(ParseError::LengthMismatch)));
    }

    #[test]
    fn rejects_prop_count_too_large() {
        let mut buf = build_packet(0, false);
        // prop_count is at byte 105 in the v2 layout.
        buf[105] = (MAX_PROPS + 1) as u8;
        assert!(matches!(parse(&buf), Err(ParseError::PropCountTooLarge)));
    }
}
