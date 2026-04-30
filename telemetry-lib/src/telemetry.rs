use byteorder::{ByteOrder, LittleEndian};
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelemetryPacket {
    pub timestamp: Option<f32>,
    pub position: Option<[f32; 3]>, // X, Y, Z (Liftoff coordinates)
    pub attitude: Option<[f32; 4]>, // X, Y, Z, W
    pub velocity: Option<[f32; 3]>, // X, Y, Z
    pub gyro: Option<[f32; 3]>,     // Pitch, Roll, Yaw
    pub input: Option<[f32; 4]>,    // Throttle, Yaw, Pitch, Roll
    pub battery: Option<[f32; 2]>,  // Percentage, Voltage
    pub motor_rpm: Option<Vec<f32>>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelemetryDescriptor {
    #[serde(rename = "EndPoint")]
    pub end_point: String,
    #[serde(rename = "StreamFormat")]
    pub stream_format: Vec<String>,
}

pub fn parse_packet(data: &[u8], format: &[String]) -> Result<TelemetryPacket, &'static str> {
    let mut ptr = 0;

    // Defaults are None
    let mut timestamp = None;
    let mut position = None;
    let mut attitude = None;
    let mut velocity = None;
    let mut gyro = None;
    let mut input = None;
    let mut battery = None;
    let mut motor_rpm = None;

    for field in format {
        match field.as_str() {
            "Timestamp" => {
                if ptr + 4 > data.len() {
                    return Err("Buffer too short");
                }
                timestamp = Some(LittleEndian::read_f32(&data[ptr..ptr + 4]));
                ptr += 4;
            }
            "Position" => {
                if ptr + 12 > data.len() {
                    return Err("Buffer too short");
                }
                let mut pos = [0.0; 3];
                for i in 0..3 {
                    pos[i] = LittleEndian::read_f32(&data[ptr + i * 4..ptr + (i + 1) * 4]);
                }
                position = Some(pos);
                ptr += 12;
            }
            "Attitude" => {
                if ptr + 16 > data.len() {
                    return Err("Buffer too short");
                }
                let mut att = [0.0; 4];
                for i in 0..4 {
                    att[i] = LittleEndian::read_f32(&data[ptr + i * 4..ptr + (i + 1) * 4]);
                }
                attitude = Some(att);
                ptr += 16;
            }
            "Velocity" => {
                if ptr + 12 > data.len() {
                    return Err("Buffer too short");
                }
                let mut vel = [0.0; 3];
                for i in 0..3 {
                    vel[i] = LittleEndian::read_f32(&data[ptr + i * 4..ptr + (i + 1) * 4]);
                }
                velocity = Some(vel);
                ptr += 12;
            }
            "Gyro" => {
                if ptr + 12 > data.len() {
                    return Err("Buffer too short");
                }
                let mut gyr = [0.0; 3];
                for i in 0..3 {
                    gyr[i] = LittleEndian::read_f32(&data[ptr + i * 4..ptr + (i + 1) * 4]);
                }
                gyro = Some(gyr);
                ptr += 12;
            }
            "Input" => {
                if ptr + 16 > data.len() {
                    return Err("Buffer too short");
                }
                let mut inp = [0.0; 4];
                for i in 0..4 {
                    inp[i] = LittleEndian::read_f32(&data[ptr + i * 4..ptr + (i + 1) * 4]);
                }
                input = Some(inp);
                ptr += 16;
            }
            "Battery" => {
                if ptr + 8 > data.len() {
                    return Err("Buffer too short");
                }
                let mut bat = [0.0; 2];
                for i in 0..2 {
                    bat[i] = LittleEndian::read_f32(&data[ptr + i * 4..ptr + (i + 1) * 4]);
                }
                battery = Some(bat);
                ptr += 8;
            }
            "MotorRPM" => {
                if ptr + 1 > data.len() {
                    return Err("Buffer too short");
                }
                let count = data[ptr] as usize;
                ptr += 1;
                if ptr + count * 4 > data.len() {
                    return Err("Buffer too short");
                }
                let mut rpms = Vec::with_capacity(count);
                for i in 0..count {
                    rpms.push(LittleEndian::read_f32(
                        &data[ptr + i * 4..ptr + (i + 1) * 4],
                    ));
                }
                motor_rpm = Some(rpms);
                ptr += count * 4;
            }
            _ => {
                return Err("Unknown field in stream format");
            }
        }
    }

    Ok(TelemetryPacket {
        timestamp,
        position,
        attitude,
        velocity,
        gyro,
        input,
        battery,
        motor_rpm,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_packet_timestamp() {
        // 123.4 as f32 le bytes: 0xcd, 0xcc, 0xf6, 0x42
        let data = [0xcd, 0xcc, 0xf6, 0x42];
        let format = vec!["Timestamp".to_string()];
        let pkt = parse_packet(&data, &format).unwrap();
        assert!(pkt.timestamp.is_some());
        assert!((pkt.timestamp.unwrap() - 123.4).abs() < 1e-4);
    }

    #[test]
    fn test_parse_packet_position() {
        // 3 floats: 1.0, 2.0, 3.0
        let mut data = Vec::new();
        data.extend_from_slice(&(1.0f32).to_le_bytes());
        data.extend_from_slice(&(2.0f32).to_le_bytes());
        data.extend_from_slice(&(3.0f32).to_le_bytes());
        let format = vec!["Position".to_string()];
        let pkt = parse_packet(&data, &format).unwrap();
        assert_eq!(pkt.position, Some([1.0, 2.0, 3.0]));
    }

    #[test]
    fn test_parse_packet_short_buffer() {
        let data = [0x00];
        let format = vec!["Timestamp".to_string()];
        let res = parse_packet(&data, &format);
        assert!(res.is_err());
    }

    #[test]
    fn test_parse_packet_unknown_field() {
        let data = [];
        let format = vec!["Unknown".to_string()];
        let res = parse_packet(&data, &format);
        assert!(res.is_err());
    }
}
