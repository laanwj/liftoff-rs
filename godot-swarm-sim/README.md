# godot-swarm-sim

Multi-drone FPV simulator on Godot 4 + Jolt physics, with the inner sim
loop in Rust via [godot-rust](https://godot-rust.github.io/) (`gdext`).
See [`../doc/godot-swarm-sim-plan.md`](../doc/godot-swarm-sim-plan.md)
for the full design plan.

## Status

N drones fly simultaneously in a shared outdoor scene with inter-drone
wake interaction, pluggable CRSF I/O interfaces, and a multi-view camera
system (single active-drone full-screen or tiled √N×√N grid, switchable
at runtime).

What's working:

- Curve-shaped motor (RPM-from-throttle/voltage), 1st-order spool-up,
  ESC idle floor.
- LiPo discharge + sag battery (`V = V_oc(SOC) − I·R`), brownout fade.
- Quadratic per-prop thrust (`T ∝ ω²`) with damage and ground-effect
  multipliers and per-prop reaction torque (correct sign per
  handedness).
- Per-axis quadratic body drag.
- Per-prop ground-effect raycast.
- Betaflight-style "Actual" per-axis rates plus throttle mid/expo curve.
- Three rate-PIDs (roll, pitch, yaw) with anti-windup decay, no-kick
  derivative, integrator and output clamps.
- X-quad mixer matrix with airmode-style anti-saturation.
- AUX1 arming gate with throttle-low safety and force-disarm
  (requires arm switch to cycle low before re-arming).
- `DronePreset` and `ActualRatesPreset` GDExtension `Resource`
  subclasses (inspector-editable, persisted as `.tres`).
- FPV camera with per-preset tilt, FOV, and offset; live-tunable via
  `set_camera_tilt_deg` / `set_camera_fov_deg` from GDScript.
- Outdoor map: ground plane, sky environment, sun, three pylon
  obstacles.
- Spawn / respawn — Enter resets the drone to its scene-load pose with
  battery refilled, PID state cleared, and the arming gate requiring a
  switch cycle before re-arming.
- Headless mode is a first-class CI tool, not deferred:
  `GSS_AUTOQUIT_S=<sec>` quits after N sim seconds, `GSS_AUTOFLY=hover`
  / `climb` injects a canned 3-phase autofly profile (settle →
  arm-with-throttle-low → ramp throttle).
- **Abstract CRSF I/O** — transport-agnostic `CrsfIo` trait defines the
  boundary between the sim and whatever carries CRSF frames:
  - `poll_rc()` — latest-wins RC frames tagged with a stream ID
    (direct or autopilot).
  - `poll_commands()` — queued non-RC CRSF command frames.
  - `send_telemetry()` — raw CRSF bytes outward.
  - Concrete implementations are Godot Node children of
    DroneController, discovered at ready-time.
- **`ZenohIOInterface`** (Godot Node) — Zenoh pub/sub transport for CRSF
  frames. Exported properties for all topic names (no magic derivation).
  Spawns a dedicated OS thread with a Tokio runtime; lock-free ArcSwap
  slots for RC in, Mutex-guarded queue for telemetry out. Incoming
  frames are routed by CRSF packet type: `RcChannelsPacked` → ArcSwap
  slot, anything else → command queue.
- **`GodotInputInterface`** (Godot Node) — local keyboard input encoded as
  real CRSF wire bytes. Reads Godot `Input` singleton, produces
  `RcChannelsPacked` frames with the standard AETRA channel layout.
- **DroneController** discovers `CrsfIo` children at ready-time, polls
  all of them each tick, feeds frames through the input router, and
  broadcasts rate-limited telemetry (~20 Hz) to all interfaces.
- **Input router** — pure-Rust mux selects between direct and autopilot
  RC sources by freshness and SA-switch position. Falls back to
  neutral input when no source is fresh.
- **RC decode** — `ChannelMap` maps raw CRSF channels to semantic roles
  (Roll, Pitch, Throttle, Yaw, Arm, RcMux, Reset, Custom). The FC
  pipeline only sees the decoded `RcInput`, never raw channel values.
- **Multi-drone + wake field:**
  - N drones from `drones.json` with per-drone Zenoh topics.
  - `WakeFieldNode`: shared double-buffered wake field with coarse 3D
    spatial hash. Each drone samples inflow from others' downwash
    before computing thrust, and contributes its own wake columns
    after. `world.gd` calls `swap()` once per physics frame.
- **Multi-view camera system:**
  - `camera_director.gd`: creates per-drone SubViewports dynamically,
    syncs their cameras to each drone's FPV camera every frame.
  - **Single mode** (default): one full-screen viewport showing the
    active drone's FPV view.
  - **Tiled mode** (F1): dynamic √N×√N grid showing all drones
    simultaneously.
  - Number keys (0–9, Shift+N for 10+N) switch which drone's FPV is
    shown.

## Layout

```
godot-swarm-sim/
├── README.md                       # this file
├── build.sh                        # cargo build → stage cdylib into addons/
├── smoke-test.py                   # cargo test + headless climb smoke run (no Zenoh)
├── zenoh-smoke-test.py             # end-to-end Zenoh RC → fly → telemetry test
├── godot/                          # Godot 4 project root
│   ├── project.godot               # 240 Hz physics, Jolt, Mobile renderer
│   ├── drones.json                 # fleet config (N drones, spawn poses, presets)
│   ├── scenes/
│   │   ├── world.tscn              # outdoor map, WakeFieldNode, CameraDirector
│   │   └── drone.tscn              # drone prefab (RigidBody3D + ZenohIOInterface + props + camera)
│   ├── scripts/
│   │   ├── world.gd                # spawn N drones, configure per-drone topics, add GodotInputInterface to active drone
│   │   └── camera_director.gd      # SubViewport grid, single/tiled mode
│   ├── presets/
│   │   └── racing_5inch.tres       # default 5" racing drone preset
│   └── addons/
│       └── godot-swarm-sim/
│           ├── godot-swarm-sim.gdextension
│           └── libgodot_swarm_sim.so   # produced by build.sh, gitignored
└── rust/                           # Cargo workspace member
    ├── Cargo.toml                  # cdylib + rlib, godot 0.5
    └── src/
        ├── lib.rs                  # gdext entry, module registration
        ├── arming.rs               # AUX-channel arming gate with rearm safety
        ├── preset.rs               # DronePresetData (pure Rust) + defaults
        ├── pipeline.rs             # per-tick orchestrator (DroneSim)
        ├── rc_input.rs             # RcInput, ChannelMap, decode_rc
        ├── crsf_io.rs              # build CRSF telemetry frames from sim state
        ├── crsf_io_trait.rs        # CrsfIo trait, RcFrame, stream ID constants
        ├── input_router.rs         # RC-source mux (direct/autopilot)
        ├── damage.rs               # per-prop damage + destroyed state
        ├── fc/
        │   ├── rates.rs            # Actual rates + throttle curve
        │   ├── pid.rs              # rate PID with anti-windup decay
        │   ├── mixer.rs            # X-quad mixer
        │   └── mode.rs             # FlightMode trait + AcroMode
        ├── physics/
        │   ├── motor.rs            # CurveMotor (idle floor, brownout)
        │   ├── battery.rs          # SagBattery (V_oc(SOC) − I·R)
        │   ├── thrust.rs           # CurveThrust (T ∝ ω², damage, ground)
        │   ├── drag.rs             # per-axis quadratic body drag
        │   ├── ground.rs           # ground-effect multiplier from AGL
        │   └── wake.rs             # WakeField double-buffer + spatial hash
        ├── drone.rs                # gdext class: DroneController : RigidBody3D
        ├── zenoh_interface.rs      # gdext ZenohIOInterface : Node
        ├── godot_input_interface.rs # gdext GodotInputInterface : Node
        ├── zenoh_io.rs             # ZenohBus: Tokio thread + ArcSwap (internal)
        ├── input_stub.rs           # StickStub: keyboard poll (internal to IOInterface)
        ├── wake_node.rs            # gdext WakeFieldNode: shared wake field holder
        └── preset_resource.rs      # gdext Resource subclasses for .tres
```

## Build & run

### Build

```bash
./godot-swarm-sim/build.sh           # debug build (default)
./godot-swarm-sim/build.sh release   # release build
```

### Tests (Rust unit tests)

```bash
cargo test -p godot-swarm-sim
```

159 tests covering rates, PID, mixer, motor, battery, thrust, drag,
ground effect, arming (including rearm-after-force-disarm), damage, the
per-tick orchestrator, pipeline behaviour, CRSF telemetry frame
generation + round-trip, the RC channel decode layer, and the input
router (freshness, SA switch mux).

### End-to-end smoke tests

```bash
./godot-swarm-sim/smoke-test.py         # cargo test + headless autofly (no Zenoh)
./godot-swarm-sim/zenoh-smoke-test.py   # Zenoh RC in → fly → telemetry out
```

`smoke-test.py` runs `cargo test`, builds the cdylib, then runs Godot
headless with the canned `climb` autofly profile. Asserts the drone
reaches at least 5 m of altitude in 4 s and that the gdext extension
loaded.

`zenoh-smoke-test.py` (requires `zenoh` Python package) opens a Zenoh
peer session, subscribes to `sim0/crsf/telemetry`, spawns Godot
headless (with `GSS_NO_LOCAL_IO=1` to suppress keyboard input), and
publishes a 3-phase RC sequence (settle → arm → climb) on
`sim0/crsf/rc`. Asserts the drone climbed via Zenoh-driven input, and
that telemetry frames flowed back on the output topic.

Outer-layer test runners are written in Python — anything that's not
pure Rust math or GDScript goes here. No ad-hoc shell scripts.

### Run interactively

```bash
godot --path godot-swarm-sim/godot
```

#### Controls

##### Flight (active drone)

| Action             | Keys                    | Notes |
|--------------------|-------------------------|-------|
| Roll right / left  | `D` / `A`              | Binary ±1 while held |
| Pitch down / up    | `W` / `S`              | W = nose down (forward flight) |
| Throttle up / down | `Shift` / `Ctrl`       | Held — throttle stays where you leave it (non-self-centring, like a real radio) |
| Yaw right / left   | `E` / `Q`              | |
| Arm / disarm       | `Space`                | Toggle — arms on first press if throttle is low |
| Respawn            | `Enter`                | Resets active drone to spawn pose, clears damage + battery; arm switch must cycle before re-arming |

##### Camera / multi-view

| Action                  | Keys                  | Notes |
|-------------------------|-----------------------|-------|
| Toggle single ↔ tiled   | `F1`                  | Single = one full-screen FPV; Tiled = √N×√N grid of all drones |
| Set active drone 0–9    | `0`–`9`               | Switches which drone's FPV is shown |
| Set active drone 10–19  | `Shift` + `0`–`9`     | Same as above for drone IDs ≥ 10 |

##### I/O interface configuration

Keyboard input is provided by a `GodotInputInterface` child node added to
the active drone by `world.gd`. Only one drone gets this node — other
drones are driven exclusively via their `ZenohIOInterface`.

When a Zenoh publisher feeds RC frames on the drone's `rc_topic`, the
input router selects the freshest source based on the SA-switch mux
rules. The keyboard still works as a fallback if no Zenoh frame has
arrived within the 500 ms freshness window.

The scene tree for each drone looks like:

```
Drone (DroneController : RigidBody3D)
├── ZenohIO (ZenohIOInterface)      ← topics configured per drone
├── LocalIO (GodotInputInterface)      ← only on the keyboard-piloted drone
├── CollisionShape3D
├── BodyMesh, PropMarkers, FpvCamera
```

### Run headless (CI / scripted)

```bash
GSS_AUTOQUIT_S=4 GSS_AUTOFLY=climb godot --headless --path godot-swarm-sim/godot
```

`GSS_AUTOFLY` values:
- `hover` — settle, then ramp throttle to ~50 % stick.
- `climb` — settle, then ramp throttle to ~70 % stick.

`GSS_NO_LOCAL_IO=1` suppresses adding a `GodotInputInterface` to the active
drone (for pure-Zenoh test runs).

On a fresh checkout the project may need a one-time import:

```bash
godot --headless --path godot-swarm-sim/godot --import
```

(Godot 4.6.2 currently SIGSEGVs on shutdown after a headless `--import`
even when the import itself succeeds; the `.godot/` cache is written
correctly. Subsequent runs without `--import` work cleanly.)

## Versions pinned

- Godot **4.6.2** (`compatibility_minimum = 4.4`)
- godot-rust `gdext` **0.5.2** (`features = ["api-4-4"]`)
- Rust edition **2024**, stable toolchain

### What's correct now

- **Prop marker positions** (`±0.074 m` in X and Z) match the preset's
  `RACING_5INCH_PROPS` offsets. Forces are applied at these exact
  positions, AGL raycasts fire from here, and damage mapping uses
  these to find the "nearest prop" to a collision.
- **Collision shape**: a single `BoxShape3D` (16×4×16 cm) centred on the
  body — roughly the carbon-fibre frame footprint of a 210 mm
  wheelbase 5" quad. Adequate for drone-on-drone and drone-on-pylon
  collisions.
- **Mass** (0.6 kg) and **inertia tensor** (`[0.0035, 0.005, 0.0035]`)
  are physically realistic for the size/weight class.
- **Prop discs are visual only** (no collision shape) — correct because
  real props break on contact rather than deflecting objects, and the
  damage system handles that via impulse threshold.

### What to improve with real art

The current drone scene (`drone.tscn`) uses **placeholder art**: a flat
box for the frame and cylinder discs for the props (colour-coded red
for CW, blue for CCW). The physics wiring is all in place and
referenced from the preset's body-frame geometry — swapping in real
meshes requires no code changes, just replacing the MeshInstance3D
sub-resources.

- **Mesh**: a low-poly model with X-frame arms, motor bells at the prop
  positions, a battery lump on the underside, and a camera housing at
  the front. Match the arm geometry to the preset's prop offsets so
  the visual lines up with where forces are applied.
- **Compound collision shape**: replace the single box with per-arm
  capsules + a central plate (`CompoundShape3D`). This gives cleaner
  data to the "nearest prop to impact" damage mapping — a pylon
  hitting one arm doesn't trigger damage on the opposite prop.
- **Prop spin animation**: rotate the `CylinderMesh` prop discs around
  their local Y axis proportional to `motor_states[i].rpm`. Purely
  visual; the physics doesn't care.
- **Multiple presets**: different meshes for `freestyle_5inch.tres`,
  `cinewhoop_3inch.tres`, etc. — each with matching collision shapes
  and prop offsets from its preset.

None of the above requires changes to the Rust code or the physics
pipeline. The force application, AGL raycasts, wake contributions, and
damage detection all reference `DronePresetData.props[i].offset` and
the body's world transform — they're mesh-agnostic.

## Potential TODOs

- Headless `--no-throttle` for faster-than-realtime runs.
- KV-drop curves (compressibility + prop-loading effects on motor KV) —
  the preset fields are designed but the motor model currently uses a
  static `loading_factor`. Wiring the curves in is a v1 fidelity raise
  inside `CurveMotor`, no architecture changes needed.
- Benchmark 16 drones at 240 Hz; characterise scaling to 32/64.
- Optional: fisheye camera toggle, second example map, OSD overlay
  (battery %, throttle, attitude indicator per viewport).
- Replace placeholder box/cylinder meshes with proper 3D models
  (see "3D models and collision geometry" above).
- HUD as a `CrsfIo` interface: receives telemetry for drones and
  renders it in the UI.
