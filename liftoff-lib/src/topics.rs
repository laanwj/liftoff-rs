pub const DEFAULT_PREFIX: &str = "liftoff";
pub const TELEMETRY: &str = "telemetry";
pub const CRSF_TELEMETRY: &str = "crsf/telemetry";
pub const CRSF_RC: &str = "crsf/rc";
pub const CRSF_RC_AUTOPILOT: &str = "crsf/rc/autopilot";

pub fn topic(prefix: &str, suffix: &str) -> String {
    format!("{}/{}", prefix, suffix)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_topic() {
        assert_eq!(topic("liftoff", "telemetry"), "liftoff/telemetry");
        assert_eq!(topic("drone2", "crsf/rc"), "drone2/crsf/rc");
    }
}
