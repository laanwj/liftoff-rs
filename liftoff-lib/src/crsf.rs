use crc::{CRC_8_DVB_S2, Crc};

pub const CRC8_DVB_S2: Crc<u8> = Crc::<u8>::new(&CRC_8_DVB_S2);

use num_enum::TryFromPrimitive;

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
pub fn unpack_channels(data: &[u8]) -> Option<[u16; 16]> {
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
