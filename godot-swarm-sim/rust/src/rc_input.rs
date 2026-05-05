//! Decoded RC input with semantic types.
//!
//! This is the **only** representation the FC pipeline sees. Raw CRSF
//! channels and µs values never leak past the decode boundary. The
//! decode step (`decode_rc`) takes raw `[u16; 16]` channels plus a
//! `ChannelMap` configuration and produces an `RcInput`.
//!
//! The `ChannelMap` is a flight-controller / sim concern (not shared
//! with the rest of the liftoff-rs workspace). It lives here rather
//! than in `telemetry-lib`.

/// Decoded value for a single channel. Returned for both the fixed FC
/// roles and the custom game-specific channels.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DecodedValue {
    /// Channel not mapped / unused.
    None,
    /// Continuous axis value. Range depends on decode type: `[-1, 1]`
    /// for signed, `[0, 1]` for unsigned.
    Axis(f32),
    /// Two-position switch.
    Switch(bool),
    /// Multi-position switch (3POS, 6POS, etc.). 0-indexed position.
    Position(u8),
}

impl Default for DecodedValue {
    fn default() -> Self {
        Self::None
    }
}

impl DecodedValue {
    pub fn as_axis(self) -> f32 {
        match self {
            Self::Axis(v) => v,
            Self::Switch(b) => if b { 1.0 } else { 0.0 },
            Self::Position(p) => p as f32,
            Self::None => 0.0,
        }
    }

    pub fn as_bool(self) -> bool {
        match self {
            Self::Switch(b) => b,
            Self::Axis(v) => v > 0.5,
            Self::Position(p) => p > 0,
            Self::None => false,
        }
    }
}

/// Fully-decoded RC input. The FC pipeline consumes only this.
///
/// Fixed fields cover the roles the sim's flight controller always
/// needs. The `custom` array carries decoded values for game-specific
/// channels (camera tilt, take-photo, mode select, etc.) that
/// GDScript consumes — the Rust sim pipeline ignores them.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct RcInput {
    // ── Fixed FC roles ──────────────────────────────────────────
    /// Roll command. −1 = full left, +1 = full right.
    pub roll: f32,
    /// Pitch command. −1 = full nose-down, +1 = full nose-up.
    pub pitch: f32,
    /// Throttle command. 0 = idle, 1 = full.
    pub throttle: f32,
    /// Yaw command. −1 = full left, +1 = full right.
    pub yaw: f32,
    /// Arm switch. `true` = pilot wants the drone armed.
    pub arm: bool,
    /// RC-mux switch. `true` = autopilot has control; `false` = manual.
    pub rc_mux_auto: bool,
    /// Reset/respawn trigger. `true` = pilot requests a respawn.
    pub reset: bool,

    // ── Custom game-specific channels ───────────────────────────
    /// Up to 16 custom decoded values. GDScript reads by index; the
    /// sim pipeline ignores them. Unused slots are `DecodedValue::None`.
    pub custom: [DecodedValue; 16],
}

impl Default for RcInput {
    fn default() -> Self {
        Self {
            roll: 0.0,
            pitch: 0.0,
            throttle: 0.0,
            yaw: 0.0,
            arm: false,
            rc_mux_auto: false,
            reset: false,
            custom: [DecodedValue::None; 16],
        }
    }
}

// ─── Channel map configuration ──────────────────────────────────────

/// How to decode one raw 11-bit channel.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChannelDecode {
    /// Signed axis: midpoint → 0, extremes → ±1.
    AxisSigned,
    /// Unsigned axis: 0 → 0.0, max → 1.0.
    AxisUnsigned,
    /// Two-position switch: below midpoint → false, at/above → true.
    Switch2Pos,
    /// N-position switch. Splits the 0..AXIS_MAX range into `n`
    /// equal bands; returns the 0-indexed band the value falls in.
    SwitchNPos(u8),
    /// Not mapped.
    Ignore,
}

/// Which semantic role a channel fills.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ChannelRole {
    Roll,
    Pitch,
    Throttle,
    Yaw,
    Arm,
    RcMux,
    /// Reset/respawn trigger.
    Reset,
    /// Custom game-specific channel. The `u8` is the index into
    /// `RcInput.custom[]` (0..15).
    Custom(u8),
    /// Not mapped.
    Ignore,
}

/// One slot in the channel map.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ChannelMapping {
    pub role: ChannelRole,
    pub decode: ChannelDecode,
}

impl Default for ChannelMapping {
    fn default() -> Self {
        Self {
            role: ChannelRole::Ignore,
            decode: ChannelDecode::Ignore,
        }
    }
}

/// Maps raw CRSF channels (0-indexed) to semantic roles + decode types.
#[derive(Debug, Clone, PartialEq)]
pub struct ChannelMap {
    pub channels: [ChannelMapping; 16],
}

impl ChannelMap {
    /// Standard AETRA layout matching the rest of the workspace.
    pub fn aetra_default() -> Self {
        let mut channels = [ChannelMapping::default(); 16];
        channels[0] = ChannelMapping { role: ChannelRole::Roll, decode: ChannelDecode::AxisSigned };
        channels[1] = ChannelMapping { role: ChannelRole::Pitch, decode: ChannelDecode::AxisSigned };
        channels[2] = ChannelMapping { role: ChannelRole::Throttle, decode: ChannelDecode::AxisUnsigned };
        channels[3] = ChannelMapping { role: ChannelRole::Yaw, decode: ChannelDecode::AxisSigned };
        channels[4] = ChannelMapping { role: ChannelRole::Arm, decode: ChannelDecode::Switch2Pos };
        channels[5] = ChannelMapping { role: ChannelRole::Reset, decode: ChannelDecode::Switch2Pos };
        channels[6] = ChannelMapping { role: ChannelRole::RcMux, decode: ChannelDecode::Switch2Pos };
        Self {
            channels,

        }
    }
}

impl Default for ChannelMap {
    fn default() -> Self {
        Self::aetra_default()
    }
}

// ─── Decode function ────────────────────────────────────────────────

/// CRSF 11-bit channel constants.
const AXIS_MAX: u16 = 1983;
const AXIS_MID: u16 = 992;

/// Decode raw 16-channel CRSF frame into a fully-typed `RcInput`.
pub fn decode_rc(channels: &[u16; 16], map: &ChannelMap) -> RcInput {
    let mut input = RcInput::default();

    for (i, mapping) in map.channels.iter().enumerate() {
        let raw = channels[i];
        let decoded = decode_value(raw, mapping.decode);

        match mapping.role {
            ChannelRole::Roll => input.roll = decoded.as_axis(),
            ChannelRole::Pitch => input.pitch = decoded.as_axis(),
            ChannelRole::Throttle => input.throttle = decoded.as_axis(),
            ChannelRole::Yaw => input.yaw = decoded.as_axis(),
            ChannelRole::Arm => input.arm = decoded.as_bool(),
            ChannelRole::RcMux => input.rc_mux_auto = decoded.as_bool(),
            ChannelRole::Reset => input.reset = decoded.as_bool(),
            ChannelRole::Custom(idx) => {
                let slot = idx as usize;
                if slot < 16 {
                    input.custom[slot] = decoded;
                }
            }
            ChannelRole::Ignore => {}
        }
    }

    input
}

fn decode_value(raw: u16, decode: ChannelDecode) -> DecodedValue {
    match decode {
        ChannelDecode::AxisSigned => DecodedValue::Axis(decode_signed(raw)),
        ChannelDecode::AxisUnsigned => DecodedValue::Axis(decode_unsigned(raw)),
        ChannelDecode::Switch2Pos => DecodedValue::Switch(raw >= AXIS_MID),
        ChannelDecode::SwitchNPos(n) => {
            if n == 0 {
                return DecodedValue::Position(0);
            }
            let clamped = raw.min(AXIS_MAX) as f32;
            let band = (clamped / (AXIS_MAX as f32 + 1.0) * n as f32).floor() as u8;
            DecodedValue::Position(band.min(n - 1))
        }
        ChannelDecode::Ignore => DecodedValue::None,
    }
}

fn decode_signed(raw: u16) -> f32 {
    let c = raw.min(AXIS_MAX) as i32 - AXIS_MID as i32;
    let span = (AXIS_MAX as i32 - AXIS_MID as i32).max(1);
    (c as f32 / span as f32).clamp(-1.0, 1.0)
}

fn decode_unsigned(raw: u16) -> f32 {
    (raw.min(AXIS_MAX) as f32 / AXIS_MAX as f32).clamp(0.0, 1.0)
}

// ─── Convenience: build RcInput directly from keyboard (for stubs) ──

impl RcInput {
    /// Build from stick values directly (used by the keyboard stub and
    /// autofly profiles). Arm and rc_mux are set explicitly.
    pub fn from_sticks(
        roll: f32,
        pitch: f32,
        throttle: f32,
        yaw: f32,
        arm: bool,
        rc_mux_auto: bool,
    ) -> Self {
        Self {
            roll: roll.clamp(-1.0, 1.0),
            pitch: pitch.clamp(-1.0, 1.0),
            throttle: throttle.clamp(0.0, 1.0),
            yaw: yaw.clamp(-1.0, 1.0),
            arm,
            rc_mux_auto,
            reset: false,
            custom: [DecodedValue::None; 16],
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mid_channels() -> [u16; 16] {
        [AXIS_MID; 16]
    }

    #[test]
    fn centred_sticks() {
        let mut ch = mid_channels();
        ch[2] = 0; // throttle low
        let input = decode_rc(&ch, &ChannelMap::default());
        assert_eq!(input.roll, 0.0);
        assert_eq!(input.pitch, 0.0);
        assert_eq!(input.yaw, 0.0);
        assert!(input.throttle < 0.01);
    }

    #[test]
    fn arm_below_mid_false() {
        let mut ch = mid_channels();
        ch[4] = AXIS_MID - 1;
        assert!(!decode_rc(&ch, &ChannelMap::default()).arm);
    }

    #[test]
    fn arm_at_mid_true() {
        let mut ch = mid_channels();
        ch[4] = AXIS_MID;
        assert!(decode_rc(&ch, &ChannelMap::default()).arm);
    }

    #[test]
    fn arm_above_mid_true() {
        let mut ch = mid_channels();
        ch[4] = AXIS_MAX;
        assert!(decode_rc(&ch, &ChannelMap::default()).arm);
    }

    #[test]
    fn full_roll() {
        let mut ch = mid_channels();
        ch[0] = AXIS_MAX;
        let input = decode_rc(&ch, &ChannelMap::default());
        assert!((input.roll - 1.0).abs() < 0.01);
    }

    #[test]
    fn full_negative_pitch() {
        let mut ch = mid_channels();
        ch[1] = 0;
        let input = decode_rc(&ch, &ChannelMap::default());
        assert!(input.pitch < -0.9);
    }

    #[test]
    fn full_throttle() {
        let mut ch = mid_channels();
        ch[2] = AXIS_MAX;
        let input = decode_rc(&ch, &ChannelMap::default());
        assert!((input.throttle - 1.0).abs() < 0.01);
    }

    #[test]
    fn rc_mux_channel_6() {
        let mut ch = mid_channels();
        ch[6] = 0;
        assert!(!decode_rc(&ch, &ChannelMap::default()).rc_mux_auto);
        ch[6] = AXIS_MAX;
        assert!(decode_rc(&ch, &ChannelMap::default()).rc_mux_auto);
    }

    #[test]
    fn custom_channel() {
        let mut map = ChannelMap::aetra_default();
        map.channels[5] = ChannelMapping {
            role: ChannelRole::Custom(0),
            decode: ChannelDecode::AxisSigned,
        };
        map.channels[7] = ChannelMapping {
            role: ChannelRole::Custom(1),
            decode: ChannelDecode::Switch2Pos,
        };
        let mut ch = mid_channels();
        ch[5] = AXIS_MAX;
        ch[7] = 0;
        let input = decode_rc(&ch, &map);
        assert!((input.custom[0].as_axis() - 1.0).abs() < 0.01);
        assert!(!input.custom[1].as_bool());
    }

    #[test]
    fn switch_3pos() {
        let mut map = ChannelMap::aetra_default();
        map.channels[8] = ChannelMapping {
            role: ChannelRole::Custom(2),
            decode: ChannelDecode::SwitchNPos(3),
        };
        let mut ch = mid_channels();
        ch[8] = 0;
        assert_eq!(decode_rc(&ch, &map).custom[2], DecodedValue::Position(0));
        ch[8] = AXIS_MID;
        assert_eq!(decode_rc(&ch, &map).custom[2], DecodedValue::Position(1));
        ch[8] = AXIS_MAX;
        assert_eq!(decode_rc(&ch, &map).custom[2], DecodedValue::Position(2));
    }

    #[test]
    fn switch_6pos() {
        let mut map = ChannelMap::aetra_default();
        map.channels[9] = ChannelMapping {
            role: ChannelRole::Custom(3),
            decode: ChannelDecode::SwitchNPos(6),
        };
        let mut ch = mid_channels();
        ch[9] = 0;
        assert_eq!(decode_rc(&ch, &map).custom[3], DecodedValue::Position(0));
        ch[9] = AXIS_MAX;
        assert_eq!(decode_rc(&ch, &map).custom[3], DecodedValue::Position(5));
    }

    #[test]
    fn from_sticks_helper() {
        let input = RcInput::from_sticks(0.5, -0.3, 0.7, 0.1, true, false);
        assert!((input.roll - 0.5).abs() < 1e-6);
        assert!((input.pitch - -0.3).abs() < 1e-6);
        assert!((input.throttle - 0.7).abs() < 1e-6);
        assert!((input.yaw - 0.1).abs() < 1e-6);
        assert!(input.arm);
        assert!(!input.rc_mux_auto);
    }

    #[test]
    fn custom_map_moves_arm() {
        let mut map = ChannelMap::aetra_default();
        map.channels[4] = ChannelMapping::default(); // clear default arm
        map.channels[10] = ChannelMapping {
            role: ChannelRole::Arm,
            decode: ChannelDecode::Switch2Pos,
        };
        let mut ch = mid_channels();
        ch[4] = AXIS_MAX; // old arm slot, now ignored
        ch[10] = 0; // new arm slot, low
        assert!(!decode_rc(&ch, &map).arm);
        ch[10] = AXIS_MAX;
        assert!(decode_rc(&ch, &map).arm);
    }

    #[test]
    fn decoded_value_conversions() {
        assert_eq!(DecodedValue::None.as_axis(), 0.0);
        assert_eq!(DecodedValue::None.as_bool(), false);
        assert_eq!(DecodedValue::Axis(0.7).as_bool(), true);
        assert_eq!(DecodedValue::Axis(0.3).as_bool(), false);
        assert_eq!(DecodedValue::Switch(true).as_axis(), 1.0);
        assert_eq!(DecodedValue::Position(3).as_axis(), 3.0);
        assert_eq!(DecodedValue::Position(0).as_bool(), false);
        assert_eq!(DecodedValue::Position(1).as_bool(), true);
    }
}
