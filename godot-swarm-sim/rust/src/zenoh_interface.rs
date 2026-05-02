//! `ZenohIOInterface` — Godot Node that wraps a Zenoh-based `CrsfIo`
//! implementation. Add as a child of `DroneController` to enable
//! Zenoh pub/sub transport for CRSF frames.
//!
//! All topic names are explicit exported properties — no magic
//! derivation from drone IDs or prefixes.

use godot::classes::{INode, Node};
use godot::prelude::*;

use crate::crsf_io_trait::CrsfIo;
use crate::zenoh_io::{ZenohBus, ZenohBusConfig};

#[derive(GodotClass)]
#[class(base=Node)]
pub struct ZenohIOInterface {
    base: Base<Node>,

    /// Topic subscribed for direct RC commands (e.g. "sim0/crsf/rc").
    #[export]
    rc_topic: GString,

    /// Topic subscribed for autopilot RC commands (e.g. "sim0/crsf/rc/autopilot").
    #[export]
    autopilot_topic: GString,

    /// Topic for publishing outbound telemetry (e.g. "sim0/crsf/telemetry").
    #[export]
    telemetry_topic: GString,

    /// Zenoh session mode: "peer" or "client".
    #[export]
    zenoh_mode: GString,

    /// Optional explicit Zenoh endpoint (e.g. "tcp/127.0.0.1:7447").
    /// Leave empty for default peer discovery.
    #[export]
    zenoh_connect: GString,

    /// Internal bus handle, created on ready().
    bus: Option<ZenohBus>,
}

#[godot_api]
impl INode for ZenohIOInterface {
    fn init(base: Base<Node>) -> Self {
        Self {
            base,
            rc_topic: GString::from("sim0/crsf/rc"),
            autopilot_topic: GString::from("sim0/crsf/rc/autopilot"),
            telemetry_topic: GString::from("sim0/crsf/telemetry"),
            zenoh_mode: GString::from("peer"),
            zenoh_connect: GString::new(),
            bus: None,
        }
    }

    fn ready(&mut self) {
        let connect = {
            let s = self.zenoh_connect.to_string();
            if s.is_empty() { None } else { Some(s) }
        };
        let cfg = ZenohBusConfig {
            rc_topic: self.rc_topic.to_string(),
            autopilot_topic: self.autopilot_topic.to_string(),
            telemetry_topic: self.telemetry_topic.to_string(),
            mode: self.zenoh_mode.to_string(),
            connect,
        };
        self.bus = Some(ZenohBus::spawn(cfg));
    }

    fn exit_tree(&mut self) {
        if let Some(bus) = self.bus.take() {
            bus.shutdown();
            drop(bus);
        }
    }
}

impl ZenohIOInterface {
    /// Access the underlying `CrsfIo` implementation. Returns `None`
    /// before `ready()` has been called.
    pub fn crsf_io(&self) -> Option<&dyn CrsfIo> {
        self.bus.as_ref().map(|b| b as &dyn CrsfIo)
    }
}
