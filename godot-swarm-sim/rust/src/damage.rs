//! Per-prop impulse-threshold damage model.
//!
//! Each prop has a health value in `[0, 1]` where 0 = healthy and
//! 1 = destroyed (matching the convention documented in the plan and
//! used by the existing EdgeTX LUA damage indicator). A collision
//! impulse near a prop that exceeds the configurable threshold snaps
//! that prop to destroyed in one hit (hard-break, not gradual).
//!
//! Whole-drone "destroyed" state: when ≥ 3 of 4 props are broken the
//! drone is considered destroyed and should be disarmed + respawned.
//!
//! `DamageState` is owned by `DroneSim`; the gdext layer feeds in
//! collision events each tick and reads the damage values out for
//! force scaling and telemetry.

/// Per-drone damage configuration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct DamageParams {
    /// Collision impulse (N·s) at a prop that causes it to break.
    pub impulse_threshold_ns: f32,
    /// Number of broken props required for the whole drone to be
    /// declared "destroyed" (triggering auto-disarm + respawn).
    pub destroyed_prop_count: u8,
    /// Respawn cooldown — how long the drone is held in place before
    /// reset. Driven by the gdext layer's timer; stored here for the
    /// preset to carry.
    pub respawn_cooldown_s: f32,
}

impl Default for DamageParams {
    fn default() -> Self {
        Self {
            impulse_threshold_ns: 8.0,
            destroyed_prop_count: 3,
            respawn_cooldown_s: 2.0,
        }
    }
}

/// A collision event near a prop, reported by the gdext collision
/// handler. The caller resolves *which* prop is nearest the contact
/// point and passes the impulse magnitude.
#[derive(Debug, Clone, Copy)]
pub struct PropCollision {
    /// 0-based prop index (0..3 for a quad).
    pub prop_index: u8,
    /// Collision impulse magnitude (N·s).
    pub impulse_ns: f32,
}

/// Mutable per-drone damage state.
#[derive(Debug, Clone)]
pub struct DamageState {
    pub params: DamageParams,
    /// Per-prop damage in `[0, 1]`. 0 = healthy, 1 = destroyed.
    pub props: [f32; 4],
    /// True once ≥ `destroyed_prop_count` props are broken.
    pub destroyed: bool,
}

impl DamageState {
    pub fn new(params: DamageParams) -> Self {
        Self {
            params,
            props: [0.0; 4],
            destroyed: false,
        }
    }

    /// Process one or more collision events for this tick. Returns
    /// `true` if the drone transitioned to the "destroyed" state
    /// during this call (so the caller can trigger disarm + respawn).
    pub fn apply_collisions(&mut self, collisions: &[PropCollision]) -> bool {
        let was_destroyed = self.destroyed;
        for c in collisions {
            let i = c.prop_index as usize;
            if i >= 4 {
                continue;
            }
            if c.impulse_ns >= self.params.impulse_threshold_ns {
                self.props[i] = 1.0;
            }
        }
        self.destroyed = self.broken_count() >= self.params.destroyed_prop_count as usize;
        // Only report the *transition* to destroyed, not repeated calls after.
        !was_destroyed && self.destroyed
    }

    /// Number of props currently at damage = 1.0 (broken).
    pub fn broken_count(&self) -> usize {
        self.props.iter().filter(|&&d| d >= 1.0).count()
    }

    /// True if any prop has non-zero damage.
    pub fn has_damage(&self) -> bool {
        self.props.iter().any(|&d| d > 0.0)
    }

    /// Reset to fully healthy. Used on respawn.
    pub fn reset(&mut self) {
        self.props = [0.0; 4];
        self.destroyed = false;
    }

    /// Per-prop health as a percentage (100 = healthy, 0 = destroyed).
    /// Matches the EdgeTX damage indicator convention (Hp1..Hp4 in %).
    pub fn health_percent(&self) -> [u8; 4] {
        [
            ((1.0 - self.props[0]) * 100.0).round() as u8,
            ((1.0 - self.props[1]) * 100.0).round() as u8,
            ((1.0 - self.props[2]) * 100.0).round() as u8,
            ((1.0 - self.props[3]) * 100.0).round() as u8,
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn state() -> DamageState {
        DamageState::new(DamageParams::default())
    }

    #[test]
    fn starts_healthy() {
        let s = state();
        assert_eq!(s.props, [0.0; 4]);
        assert!(!s.destroyed);
        assert!(!s.has_damage());
        assert_eq!(s.broken_count(), 0);
        assert_eq!(s.health_percent(), [100, 100, 100, 100]);
    }

    #[test]
    fn below_threshold_no_damage() {
        let mut s = state();
        let collisions = vec![PropCollision {
            prop_index: 0,
            impulse_ns: 5.0, // below 8.0 threshold
        }];
        let destroyed = s.apply_collisions(&collisions);
        assert!(!destroyed);
        assert_eq!(s.props[0], 0.0);
    }

    #[test]
    fn at_threshold_breaks_prop() {
        let mut s = state();
        let collisions = vec![PropCollision {
            prop_index: 1,
            impulse_ns: 8.0,
        }];
        let destroyed = s.apply_collisions(&collisions);
        assert!(!destroyed); // 1 broken, need 3
        assert_eq!(s.props[1], 1.0);
        assert_eq!(s.broken_count(), 1);
        assert!(s.has_damage());
        assert_eq!(s.health_percent(), [100, 0, 100, 100]);
    }

    #[test]
    fn above_threshold_breaks_prop() {
        let mut s = state();
        let destroyed = s.apply_collisions(&[PropCollision {
            prop_index: 2,
            impulse_ns: 20.0,
        }]);
        assert!(!destroyed);
        assert_eq!(s.props[2], 1.0);
    }

    #[test]
    fn three_broken_triggers_destroyed() {
        let mut s = state();
        let collisions: Vec<PropCollision> = (0..3)
            .map(|i| PropCollision {
                prop_index: i,
                impulse_ns: 10.0,
            })
            .collect();
        let destroyed = s.apply_collisions(&collisions);
        assert!(destroyed);
        assert!(s.destroyed);
    }

    #[test]
    fn four_broken_also_destroyed() {
        let mut s = state();
        let collisions: Vec<PropCollision> = (0..4)
            .map(|i| PropCollision {
                prop_index: i,
                impulse_ns: 10.0,
            })
            .collect();
        let destroyed = s.apply_collisions(&collisions);
        assert!(destroyed);
    }

    #[test]
    fn destroyed_transition_fires_only_once() {
        let mut s = state();
        let c: Vec<PropCollision> = (0..3)
            .map(|i| PropCollision {
                prop_index: i,
                impulse_ns: 10.0,
            })
            .collect();
        let first = s.apply_collisions(&c);
        assert!(first);
        // Second call with same state: already destroyed → returns false.
        let second = s.apply_collisions(&[PropCollision {
            prop_index: 3,
            impulse_ns: 10.0,
        }]);
        assert!(!second);
    }

    #[test]
    fn reset_clears_all_damage() {
        let mut s = state();
        s.apply_collisions(&[
            PropCollision { prop_index: 0, impulse_ns: 10.0 },
            PropCollision { prop_index: 1, impulse_ns: 10.0 },
            PropCollision { prop_index: 2, impulse_ns: 10.0 },
        ]);
        assert!(s.destroyed);
        s.reset();
        assert!(!s.destroyed);
        assert_eq!(s.props, [0.0; 4]);
        assert_eq!(s.broken_count(), 0);
    }

    #[test]
    fn out_of_range_prop_index_ignored() {
        let mut s = state();
        let destroyed = s.apply_collisions(&[PropCollision {
            prop_index: 99,
            impulse_ns: 100.0,
        }]);
        assert!(!destroyed);
        assert!(!s.has_damage());
    }

    #[test]
    fn custom_threshold() {
        let params = DamageParams {
            impulse_threshold_ns: 2.0,
            destroyed_prop_count: 2,
            respawn_cooldown_s: 1.0,
        };
        let mut s = DamageState::new(params);
        s.apply_collisions(&[
            PropCollision { prop_index: 0, impulse_ns: 2.0 },
            PropCollision { prop_index: 3, impulse_ns: 3.0 },
        ]);
        assert!(s.destroyed);
    }

    #[test]
    fn idempotent_on_already_broken_prop() {
        let mut s = state();
        s.apply_collisions(&[PropCollision { prop_index: 0, impulse_ns: 10.0 }]);
        assert_eq!(s.props[0], 1.0);
        // Hitting it again doesn't change anything.
        s.apply_collisions(&[PropCollision { prop_index: 0, impulse_ns: 10.0 }]);
        assert_eq!(s.props[0], 1.0);
        assert_eq!(s.broken_count(), 1);
    }
}
