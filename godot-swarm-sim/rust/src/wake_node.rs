//! `WakeFieldNode` — gdext class holding the shared `WakeField`.
//!
//! Added to the scene tree once (by `world.gd`). Each `DroneController`
//! grabs a reference via the node path `../WakeField` and calls:
//! - `sample(pos, own_origin)` in its per-tick read phase
//! - `contribute(column_data)` in its per-tick write phase
//!
//! The world script calls `swap()` once per physics frame after all
//! drones have stepped, flipping the double buffer.

use godot::classes::{INode, Node};
use godot::prelude::*;

use crate::physics::wake::{WakeColumn, WakeField};

#[derive(GodotClass)]
#[class(base=Node)]
pub struct WakeFieldNode {
    base: Base<Node>,
    field: WakeField,
}

#[godot_api]
impl INode for WakeFieldNode {
    fn init(base: Base<Node>) -> Self {
        Self {
            base,
            field: WakeField::default_for_racing(),
        }
    }
}

#[godot_api]
impl WakeFieldNode {
    /// Swap the double-buffer. Called by the world script once per
    /// physics frame, after all drones have stepped. This makes the
    /// current tick's contributions visible to the next tick's samples.
    #[func]
    pub fn swap(&mut self) {
        self.field.swap();
    }

    /// Current number of wake columns in the read buffer (diagnostic).
    #[func]
    pub fn column_count(&self) -> i64 {
        self.field.column_count() as i64
    }

    /// Set the spatial-hash cell size (metres). Only affects future
    /// contributions, not already-stored columns.
    #[func]
    pub fn set_cell_size(&mut self, size: f32) {
        self.field.cell_size = size.max(0.1);
    }
}

impl WakeFieldNode {
    /// Direct access for `DroneController` (same process, no Godot
    /// method call overhead). The caller must hold a `&mut Gd<Self>`
    /// to call this — we bind_mut() at the call site.
    pub fn field_mut(&mut self) -> &mut WakeField {
        &mut self.field
    }
}
