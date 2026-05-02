//! RC-source mux: pick which RC frame the FC consumes this tick.
//!
//! Two potential sources:
//!
//! 1. **Direct** — RC commands targeted at this specific drone (from
//!    any interface: keyboard, Zenoh per-drone topic, etc.).
//! 2. **Autopilot** — autopilot-generated commands for this drone.
//!
//! Selection rules:
//!
//! - If the direct source has a fresh frame and the SA switch
//!   (channel 7) is **low**, the direct source wins (manual override).
//! - Otherwise, if the autopilot source is fresh, it wins.
//! - Otherwise, if the direct source is fresh (even with SA high),
//!   it wins as a fallback.
//! - Otherwise, return `None` (the caller uses a neutral/zero input).
//!
//! "Fresh" means `now − received_at ≤ FRESHNESS`. The default is
//! 500 ms.
//!
//! This module is pure logic — no Zenoh, no Godot, no time source
//! beyond what the caller passes in. Easy to unit-test exhaustively.

use std::time::{Duration, Instant};

/// Default freshness window. Frames older than this are ignored.
pub const DEFAULT_FRESHNESS: Duration = Duration::from_millis(500);

/// 0-indexed array slot for the SA switch (CRSF channel 7).
pub const SA_SWITCH_INDEX: usize = 6;
/// SA switch threshold: below this is "low" (manual override active).
pub const SA_SWITCH_LOW_THRESHOLD: u16 = 992;

/// One CRSF RC frame plus when it arrived. The `Instant` is whatever
/// time source the caller uses — we only ever compare against `now`.
#[derive(Debug, Clone, Copy)]
pub struct RcSample {
    pub channels: [u16; 16],
    pub received_at: Instant,
}

impl RcSample {
    pub fn new(channels: [u16; 16], received_at: Instant) -> Self {
        Self {
            channels,
            received_at,
        }
    }

    pub fn is_fresh(&self, now: Instant, freshness: Duration) -> bool {
        now.saturating_duration_since(self.received_at) <= freshness
    }
}

/// Identity of which source the router selected this tick.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Selected {
    Autopilot,
    Direct,
}

/// Two optional RC sources (each `None` if no frame has arrived yet).
#[derive(Debug, Clone, Copy, Default)]
pub struct RouterInputs {
    pub autopilot: Option<RcSample>,
    pub direct: Option<RcSample>,
}

#[derive(Debug, Clone, Copy)]
pub struct RouterOutput {
    pub channels: [u16; 16],
    pub source: Selected,
}

#[derive(Debug, Clone, Copy)]
pub struct RouterConfig {
    /// Frames older than this are treated as not-arrived.
    pub freshness: Duration,
}

impl Default for RouterConfig {
    fn default() -> Self {
        Self {
            freshness: DEFAULT_FRESHNESS,
        }
    }
}

/// Run the mux. Returns `None` if no source is fresh — the caller is
/// then expected to fall back to a neutral/zero input.
pub fn select(inputs: &RouterInputs, cfg: &RouterConfig, now: Instant) -> Option<RouterOutput> {
    // 1. Direct with SA switch low → manual override, wins immediately.
    if let Some(d) = inputs.direct
        && d.is_fresh(now, cfg.freshness)
    {
        let sa = d.channels[SA_SWITCH_INDEX];
        if sa < SA_SWITCH_LOW_THRESHOLD {
            return Some(RouterOutput {
                channels: d.channels,
                source: Selected::Direct,
            });
        }
        // SA switch high: pilot has handed control to autopilot.
        // Fall through to check autopilot first.
    }

    // 2. Autopilot.
    if let Some(a) = inputs.autopilot
        && a.is_fresh(now, cfg.freshness)
    {
        return Some(RouterOutput {
            channels: a.channels,
            source: Selected::Autopilot,
        });
    }

    // 3. Direct as fallback (SA high but no autopilot available).
    if let Some(d) = inputs.direct
        && d.is_fresh(now, cfg.freshness)
    {
        return Some(RouterOutput {
            channels: d.channels,
            source: Selected::Direct,
        });
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    const AXIS_MAX: u16 = 1983;
    const AXIS_MID: u16 = 992;

    fn frame(sa: u16) -> [u16; 16] {
        let mut c = [AXIS_MID; 16];
        c[SA_SWITCH_INDEX] = sa;
        c
    }

    #[test]
    fn nothing_fresh_returns_none() {
        let now = Instant::now();
        let r = select(&RouterInputs::default(), &RouterConfig::default(), now);
        assert!(r.is_none());
    }

    #[test]
    fn stale_direct_returns_none() {
        let now = Instant::now();
        let inputs = RouterInputs {
            direct: Some(RcSample::new(frame(0), now - Duration::from_secs(2))),
            ..Default::default()
        };
        let r = select(&inputs, &RouterConfig::default(), now);
        assert!(r.is_none());
    }

    #[test]
    fn direct_low_sa_wins_over_autopilot() {
        let now = Instant::now();
        let inputs = RouterInputs {
            direct: Some(RcSample::new(frame(0), now)), // SA low
            autopilot: Some(RcSample::new(frame(AXIS_MAX), now)),
        };
        let r = select(&inputs, &RouterConfig::default(), now).unwrap();
        assert_eq!(r.source, Selected::Direct);
    }

    #[test]
    fn direct_high_sa_yields_to_autopilot() {
        let now = Instant::now();
        let inputs = RouterInputs {
            direct: Some(RcSample::new(frame(AXIS_MAX), now)), // SA high
            autopilot: Some(RcSample::new(frame(AXIS_MID), now)),
        };
        let r = select(&inputs, &RouterConfig::default(), now).unwrap();
        assert_eq!(r.source, Selected::Autopilot);
    }

    #[test]
    fn no_direct_falls_through_to_autopilot() {
        let now = Instant::now();
        let inputs = RouterInputs {
            direct: None,
            autopilot: Some(RcSample::new(frame(AXIS_MID), now)),
        };
        let r = select(&inputs, &RouterConfig::default(), now).unwrap();
        assert_eq!(r.source, Selected::Autopilot);
    }

    #[test]
    fn direct_high_sa_no_autopilot_still_uses_direct() {
        let now = Instant::now();
        let inputs = RouterInputs {
            direct: Some(RcSample::new(frame(AXIS_MAX), now)), // SA high
            autopilot: None,
        };
        let r = select(&inputs, &RouterConfig::default(), now).unwrap();
        assert_eq!(r.source, Selected::Direct);
    }

    #[test]
    fn stale_direct_does_not_count() {
        let now = Instant::now();
        let inputs = RouterInputs {
            direct: Some(RcSample::new(frame(0), now - Duration::from_secs(2))),
            autopilot: Some(RcSample::new(frame(AXIS_MID), now)),
        };
        let r = select(&inputs, &RouterConfig::default(), now).unwrap();
        assert_eq!(r.source, Selected::Autopilot);
    }

    #[test]
    fn freshness_is_inclusive_at_boundary() {
        let now = Instant::now();
        let cfg = RouterConfig::default();
        let exactly_at_deadline = now - cfg.freshness;
        let just_past = now - cfg.freshness - Duration::from_millis(1);

        let m1 = RcSample::new(frame(0), exactly_at_deadline);
        assert!(m1.is_fresh(now, cfg.freshness));

        let m2 = RcSample::new(frame(0), just_past);
        assert!(!m2.is_fresh(now, cfg.freshness));
    }

    #[test]
    fn autopilot_overrides_stale_direct() {
        let now = Instant::now();
        let inputs = RouterInputs {
            direct: Some(RcSample::new(frame(AXIS_MAX), now - Duration::from_secs(2))),
            autopilot: Some(RcSample::new(frame(AXIS_MID), now)),
        };
        let r = select(&inputs, &RouterConfig::default(), now).unwrap();
        assert_eq!(r.source, Selected::Autopilot);
    }
}
