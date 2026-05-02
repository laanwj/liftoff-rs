# godot-swarm-sim — design plan

A multi-drone FPV simulator built on Godot 4.4+ with Jolt physics, Rust
inner-loop physics via [godot-rust](https://godot-rust.github.io/)
(`gdext`), and Zenoh pub/sub I/O wire-compatible with the rest of this
workspace.

This document captures the design decisions and phased delivery plan for
the project. Authoring lives at `liftoff-rs/godot-swarm-sim/`.

## Goals

- N simultaneous drones in one Godot scene, colliding with each other
  and the world. **The first release targets up to ~16 drones** as the
  shakedown number, but 16 is not a hard limit — no fixed-size buffers,
  hardcoded indices, or `[N=16]` arrays in the design. Going higher
  (32, 64, …) should be a matter of CPU/GPU headroom, not source
  changes. Per-drone state, ID assignment, Zenoh topic plumbing, and
  the wake-field spatial hash are all sized at runtime from
  `drones.json`.
- Realistic-enough physics — mid-fidelity to start (RPM-from-curve,
  battery sag, curve-shaped thrust, per-axis quadratic drag, ground
  effect, inter-drone wake), swappable to higher-fidelity blade-element
  later via trait-object substitution.
- RC input directly from Zenoh (`{prefix}{id}/crsf/rc` per drone, plus a
  configurable manual-RC topic for the local pilot).
- CRSF telemetry output directly to Zenoh
  (`{prefix}{id}/crsf/telemetry`), wire-compatible with the existing
  `telemetry-dashboard`, `autopilot`, `crsf-gpsd`, `mavlink-bridge`
  stack.
- Per-drone FPV cameras; single active-drone full-screen view by
  default, with F-key toggles for 2×2 / 4×4 tile layouts.
- Hot inner loops in Rust; Godot/GDScript only for scene wiring, UI,
  event handling, and viewport tiling.

## Decisions in one place

| Area | Choice |
|---|---|
| Workspace placement | `liftoff-rs/godot-swarm-sim/` (Cargo workspace member, reuses `telemetry-lib` by path) |
| Binding | `godot-rust` **gdext** (Godot 4 native, GDExtension-based) |
| Engine + physics | Godot **4.4+** with **Jolt** as the rigid-body solver |
| Renderer | **Mobile Vulkan** (`rendering_method="mobile"`) — lighter pipeline than Forward+, better suited to many simultaneous viewports |
| Languages | Rust for inner sim loop and FC; GDScript only for scene wiring, UI, viewport tiling |
| Physics tick | **240 Hz** fixed-step |
| Initial fidelity | **Mid (curve-shaped)** — RPM-from-throttle/voltage curve, `V = V_oc − I·R` battery, curve-shaped thrust, per-axis quadratic body drag, per-prop ground effect |
| Inter-drone aero | **Day-1 wake field**, single global, double-buffered, **coarse 3D spatial hash** for neighbour lookups |
| Topic-naming | **`{prefix}{id}/crsf/...`** — per-drone Zenoh prefix `sim0`, `sim1`, … `simN` |
| Local joystick | Sim subscribes directly to a configurable manual-RC topic (default `sim/crsf/rc`), routes to the active drone |
| Default view | Single active-drone, F-keys toggle 2×2 / 4×4 tile modes |
| Camera | Pinhole, FOV ~120° (fisheye toggle later) |
| Damage | Per-prop impulse-threshold hard-break |
| Drone preset | **Godot Resource (`.tres`)** — `DronePreset` subclass of `Resource` registered via gdext; inspector-editable, scenes carry the reference |
| Headless mode | **Day-1 (Phase 0)** — `godot --headless` + `GSS_AUTOQUIT_S` env var; `GSS_AUTOFLY=hover\|climb` for canned input profiles. Used continuously for CI smoke tests of the gdext layer (extension loads, scene boots, drone arms and flies, telemetry within bounds) |
| Initial world | **Open outdoor terrain** — heightmap + sky + sun + scattered cylindrical obstacles |

## Architecture

```
┌────────────────────────────────────────────────────────────────────────────┐
│                       Godot 4.4+ project (godot/)                          │
│ ┌────────────────────────────────────────────────────────────────────────┐ │
│ │  World (scene root)                                                    │ │
│ │  ├── Terrain (StaticBody3D + HeightMapShape3D)                         │ │
│ │  ├── Sky (Environment + procedural sun)                                │ │
│ │  ├── Obstacles (StaticBody3D × M)                                      │ │
│ │  ├── DroneN (RigidBody3D + Jolt) ×N                                    │ │
│ │  │     ├── CollisionShape3D                                            │ │
│ │  │     ├── Marker3D × 4 (prop positions, body-frame)                   │ │
│ │  │     ├── FpvCamera (Camera3D; tilt/FOV/offset from preset)           │ │
│ │  │     ├── DronePreset (Resource ref → .tres)                          │ │
│ │  │     └── DroneController  ◀── gdext class (Rust)                     │ │
│ │  ├── WakeFieldNode (gdext class, autoload-style) ─ global, double-buf  │ │
│ │  ├── ZenohBus (gdext class, Tokio runtime on its own thread)           │ │
│ │  └── CameraDirector (GDScript) → SubViewportContainer grid             │ │
│ └────────────────────────────────────────────────────────────────────────┘ │
└────────────────────────────────────────────────────────────────────────────┘
        ▲                                          │
        │ {prefix}/crsf/rc (active drone manual)   │ {prefix}{id}/crsf/telemetry
        │ {prefix}{id}/crsf/rc                     │ {prefix}{id}/damage
        │ {prefix}{id}/crsf/rc/autopilot           │ {prefix}{id}/battery
        ▼                                          ▼
        ┌────────────────────────────────────────────────────────┐
        │                Zenoh (UDP peer, default)               │
        └────────────────────────────────────────────────────────┘
```

## Per-tick step (240 Hz, per drone)

```
 1. sensor    — imu = SensorModel.read(truth_gyro_accel, dt)            (default: pass-through)
 2. fc        — rates → PID → mixer → 4 motor commands (clamped, idle-floored)
                · brownout_factor from battery scales the commands
 3. arming    — AUX channel + threshold gate; if disarmed, motor commands → 0
 4. motor     — rpm = curve(motor_cmd, V_cell), 1st-order spool-up
 5. env_in    — for each prop: v_env = Environment.sample_velocity(prop_world_pos)
 6. wake_in   — for each prop: v_inflow = WakeField.sample(prop_world_pos, prop_axis)
                v_local = body_vel_at(prop) + v_inflow + v_env
 7. inflow    — adj = InflowModel.adjust(prop_ctx)                       (default: identity)
                v_local += adj.v_local_delta
 8. thrust    — T = thrust_curve(rpm, |v_local · prop_axis|, motor_cmd)
                  · damage_factor(prop_damage)
                  · ground_effect(prop_AGL_raycast)
                  · adj.thrust_scale
 9. forces    — apply_force_at_position(T · prop_axis, prop_world_pos)
                apply_torque(reaction_torque(rpm, handedness))
10. drag      — v_rel = body_vel − Environment.sample_velocity(body_world_pos)
                F_drag = per-axis ½·ρ·v_rel²·Cd·A (+ optional induced/lift later)
                apply_central_force(F_drag)
11. ext_force — for each registered ExternalForce: f, τ at body
                apply_force_at_position / apply_torque
12. wake_out  — for each prop: WakeField.contribute(WakeColumn { origin, axis, v_h, … })
13. battery   — Σ I_motor → battery.step(dt) → V_cell_next, brownout_factor_next
14. crsf_io   — push attitude/gps/battery/damage/link-stats into ZenohBus outbox SPSC
```

After all drones step, a coordinator call swaps the WakeField
double-buffer.

## Why these choices

### Engine: Godot 4.4+ with Jolt

- Free, MIT-style licensed, friendly to redistribution.
- Jolt is the default rigid-body solver from 4.4 onwards: modern, fast,
  stable contacts, holds up at high physics rates better than Bullet or
  the legacy Godot Physics.
- Forces-on-rigid-body is exactly what the per-tick step calls for: we
  accumulate forces and torques per tick and let the engine integrate.

### Languages

- **Rust for the inner loop** (FC, mixer, PID, per-prop thrust, motor,
  battery, drag, wake) via `gdext`. At 240 Hz × 16 drones × 4 props ≈
  15 000 force-vector computes per second; GDScript would not survive
  that. Rust also lets us reuse `telemetry-lib` directly.
- **GDScript for frontend glue** — scene tree wiring, camera director,
  viewport tiling, on-screen OSD, menu, keyboard switching of "active"
  drone.
- No C# — keeps the toolchain to two languages.

### Physics rate

- **240 Hz** fixed-step. Conservative; safer at 16 drones on weaker
  hardware than 500 Hz; FC will feel a bit snappier than UE4-default but
  not as crisp as a 1 kHz Betaflight box. Tunable via project settings.

### Rust ↔ Godot data flow

- One `DroneController` `gdext` class per drone, attached as a child of
  each `RigidBody3D`.
- Zenoh I/O lives in a Rust gdext class (`ZenohBus`) running its own
  Tokio runtime on a dedicated thread. RC frames and telemetry pass
  between the Tokio thread and per-drone controllers via lock-free SPSC
  ringbuffers.
- Godot's `_physics_process` calls into Rust each tick; Rust never
  blocks on Zenoh I/O.

## Multi-drone design

| Concern | Approach |
|---|---|
| Spawn | `world.gd` reads `drones.json` (count, spawn poses, drone preset) and instances `Drone.tscn` N times |
| Per-drone identity | Stable `drone_id: u16` (0..N-1) baked into node name and Zenoh topic prefix. `u16` chosen so 16-bit fields don't artificially cap the swarm — the design has no inherent ceiling, only the CPU/GPU one |
| Topics | `{prefix}{id}/crsf/rc` for RC in, `{prefix}{id}/crsf/telemetry` for telemetry out, `{prefix}{id}/damage`, `{prefix}{id}/battery` |
| Local joystick | `--manual-rc-topic <topic>` (default `sim/crsf/rc`) feeds the active drone |
| Inter-drone collisions | Free from Jolt; each drone is a `RigidBody3D` |
| Inter-drone aerodynamics | Day-1 wake field with coarse spatial hash (see below) |

## Inter-drone aero — wake field

A pragmatic approximation based on momentum theory: each prop emits a
`WakeColumn` representing its downwash, and other props sample the field
to get inflow velocity. Double-buffered so iteration order doesn't
matter.

```rust
pub struct WakeColumn {
    pub origin: Vec3,         // world-frame prop position
    pub axis:   Vec3,         // unit, points downwash direction (away from prop face)
    pub v_h:    f32,          // induced velocity at disc, √(T / (2ρA))
    pub radius: f32,          // ≈ prop radius
    pub axial_decay: f32,     // length scale for downwash decay along axis
}

pub struct WakeField {
    grid:       AHashMap<IVec3, SmallVec<[u32; 8]>>, // cell → column indices (write side)
    columns:    Vec<WakeColumn>,                     // write side
    cell_size:  f32,                                 // ≈ 2 × max axial_decay (default 10 m)
    snapshot:   Vec<WakeColumn>,                     // last-tick read side
    snap_grid:  AHashMap<IVec3, SmallVec<[u32; 8]>>, // last-tick read side
}

fn sample(snapshot: &[WakeColumn], snap_grid: &AHashMap<IVec3, _>,
          cell_size: f32, pos: Vec3, _axis: Vec3) -> Vec3 {
    let cell = (pos / cell_size).floor().as_ivec3();
    let mut acc = Vec3::ZERO;
    for dx in -1..=1 { for dy in -1..=1 { for dz in -1..=1 {
        if let Some(idxs) = snap_grid.get(&(cell + IVec3::new(dx, dy, dz))) {
            for &i in idxs {
                let w = &snapshot[i as usize];
                let r = pos - w.origin;
                let r_axial = r.dot(w.axis);
                if r_axial < 0.0 { continue; } // sample is upstream of disc
                let r_radial = (r - w.axis * r_axial).length();
                let f_axial  = (-r_axial / w.axial_decay).exp();
                let f_radial = (-(r_radial / w.radius).powi(2)).exp();
                acc += w.axis * (-w.v_h * f_axial * f_radial);
            }
        }
    }}}
    acc
}
```

For 16 drones × 4 props at 240 Hz across a 100×100×100 m world with 10 m
cells: ~64 columns spread over ~64 cells; each sample touches ~3-5
columns; ~64 × 4 = ~256 samples × 5 columns ≈ 1300 evals/tick ≈
300 k/sec. Plenty of headroom; cell size is tunable as drones cluster.

The `WakeModel` trait abstracts the kernel so we can swap the Gaussian
form for a cylindrical-step or a Glauert-corrected form later without
touching callers.

## Environment & external forces (extension points)

Wind, gusts, turbulence, thermals, region-of-effect zones, tethers,
scripted pushes — none of these are in v0, but the architecture is
shaped so any of them can be added later without touching the per-drone
core. Two extension points carry the weight:

- **`EnvironmentField`** — a velocity field sampled at world points.
  Both per-prop inflow (step 3, `env_in`) and body-level drag (step 7)
  query it to derive air-relative velocity. v0 ships
  `NullEnvironment` (returns zero); a `ConstantWind` is a one-screen
  follow-up; a height-varying field, OU/Dryden gust process, or
  per-zone wind volumes from the map all slot in behind the same
  trait.

  Once any `EnvironmentField` is plugged in, the existing drag and
  thrust paths react correctly with no further changes — wind drag
  emerges from `body_vel − env_vel` going into `PerAxisQuadraticDrag`,
  and prop loading shifts because `v_local` at each prop now includes
  the local wind sample. This is exactly the kind of effect that gets
  more interesting as the swarm grows: a 16-drone formation flying
  through a gust front behaves visibly differently from a single
  drone, and the wake field already couples the swarm together.

- **`ExternalForce`** — a per-drone registry of force/torque
  contributors that bypass the air model entirely (tethers, magnetic
  fields, scripted impulses, choreography "puppet strings", missile
  thrust). v0 ships an empty registry; consumers register impls at
  scene load.

A `CompositeEnvironment` will let multiple sources stack (e.g. base
wind + turbulence overlay + per-zone modifier) without the per-drone
controller knowing how the field was built. Likewise the
`ExternalForce` registry is just a `Vec<Box<dyn ExternalForce>>` per
drone.

Both hooks are in the per-tick step from day 1 (steps 3, 7, 8) — they
just call into no-op default implementations until something useful is
plugged in. This means turning wind on later is a matter of writing the
`EnvironmentField` impl and replacing the singleton, with zero changes
to `DroneController`, `ThrustModel`, or `DragModel`.

## Physics fidelity ladder

Coded against flexible interfaces from day 1, with the v0 implementation
in the middle of each axis:

| Phenomenon | v0 (default) | v1 stretch | v2 stretch |
|---|---|---|---|
| Thrust | `T = thrust_curve(rpm, V_local, throttle) · damage · ground_eff`, applied per-prop at offset | Same shape, tunable curves | Blade-element + actuator-disc, `Cl(α)` LUT, induced-velocity solve |
| Counter-torque | Per-rotor reaction torque around prop axis | Same | Same |
| Motor | RPM-from-curve(throttle, V_cell), 1st-order spool-up | + KV-drop curves (compressibility, loading) | Full BLDC: V/R, EMF, current, K_t·(I−I_no_load) |
| Battery | `V = V_oc(SOC) − I · R_internal`, per-cell discharge curve | + thermal LP filter, R rises with discharge | + LiPo/Li-ion preset switch |
| Body drag | Per-axis quadratic `½ρv²·Cd·A` in body frame | + induced drag from total thrust | + body lift |
| Ground effect | Per-prop AGL raycast, ×1.0..1.5 multiplier within ~1 prop diameter | Per-prop wake reduction near ground | — |
| Ceiling effect | None (`SurfaceProximityModel` trait wired with no-op for "up") | Per-prop AGL-up raycast, thrust attenuation | — |
| Wind / environment | `NullEnvironment` (trait wired, no-op) | `ConstantWind` impl | `LayeredWind` / `GustyWind` / `ZoneWind` impls behind same trait |
| VRS / propwash | None (`InflowModel` hook in place; default returns `v_local` unchanged) | Soft propwash via curve on prop's axial-descent component | Full positive-feedback VRS model with lateral-velocity escape |
| Damage | Per-prop `[0,1]` health, hard impulse-threshold break | Continuous deform | — |
| Battery brownout | Hard cutoff: when `V_cell < cutoff_v` motors lose authority (multiplier scales linearly to 0 over a small dV band) | + LP-filtered cutoff for natural feel | — |
| Sensor noise | None (`SensorModel` trait wired, default = pass-through truth) | Gyro Gaussian noise + propwash-correlated noise | Full IMU model with bias drift, RPM-filter, Betaflight notch chain |
| Inter-drone aero | Gaussian wake field, coarse spatial hash | Cylindrical-step kernel | Glauert-corrected wake |

### Trait shapes

```rust
pub struct PropCtx<'a> {
    pub rpm: f32,
    pub throttle: f32,
    pub v_local_world: Vec3,           // body vel + wake sample + environment sample at prop
    pub prop_world_pos: Vec3,
    pub prop_axis_world: Vec3,
    pub air_density: f32,
    pub prop_damage: f32,              // 0 = healthy, 1 = destroyed
    pub agl: Option<f32>,              // ground raycast result
    pub preset: &'a PropPreset,
}

pub trait ThrustModel   { fn compute(&self, ctx: &PropCtx) -> PropForceTorque; }
pub trait MotorModel    { fn step(&mut self, throttle: f32, dt: f32, v_cell: f32) -> MotorState; }
pub trait BatteryModel  { fn step(&mut self, current_a: f32, dt: f32) -> BatteryState; }
pub trait DragModel     { fn compute(&self, vel_body_relative: Vec3, attitude: Quat, air_density: f32) -> Wrench; }
pub trait WakeModel     { fn sample(&self, world_pos: Vec3, axis: Vec3) -> Vec3;
                          fn contribute(&mut self, col: WakeColumn); }

/// Per-prop axial-flow side-effects: VRS, propwash, ceiling effect, etc.
/// Called between wake sample and thrust to perturb v_local or scale thrust.
pub trait InflowModel   { fn adjust(&self, ctx: &PropCtx) -> InflowAdjustment; }
//                        InflowAdjustment { v_local_delta: Vec3, thrust_scale: f32 }

/// Sensor model: convert truth-state gyro/accel into what the FC sees.
/// Default = pass-through; future = noise + filters + bias drift.
pub trait SensorModel   { fn read(&mut self, truth: &ImuTruth, dt: f32) -> ImuReading; }

/// Velocity-field environment (wind, gusts, thermals, region-of-effect zones, etc.).
/// Sampled once per body and once per prop per tick to derive relative airspeed.
/// Multiple sources can be stacked behind a `CompositeEnvironment`.
pub trait EnvironmentField {
    fn sample_velocity(&self, world_pos: Vec3, t: f64) -> Vec3;
}

/// Direct external force on a drone body (e.g. tether, magnetic field,
/// scripted "push", missile thrust). Distinct from EnvironmentField, which
/// only changes the air-relative velocity that thrust/drag see.
pub trait ExternalForce {
    fn compute(&self, body: &BodyState, t: f64) -> Wrench;  // (force, torque) at body
}
```

Default v0 implementations:

- `ThrustModel`: `CurveThrust` — `T = thrust_curve(rpm, v_local·axis, throttle)`
- `MotorModel`: `CurveMotor` — `rpm = throttle_curve(throttle, v_cell)`, 1st-order spool-up. Includes a configurable ESC-side `idle_throttle_floor` (motor minimum) and `motor_command_clamp` (0..1 with optional brownout scaling).
- `BatteryModel`: `SagBattery` — `V = V_oc(SOC) − I·R`. Reports a `brownout_factor` in [0,1] from the cutoff band (`cutoff_v − cutoff_band` → 0, `cutoff_v` → 1) that the motor model multiplies into its output, so a dying battery degrades thrust naturally.
- `DragModel`: `PerAxisQuadraticDrag` — operates on `body_vel − env_vel`, so wind drag works automatically once an `EnvironmentField` is plugged in
- `WakeModel`: `GaussianWake`
- `InflowModel`: `IdentityInflow` (no adjustment) — VRS / propwash / ceiling effect all defer to v1
- `SensorModel`: `TruthSensor` (truth-state IMU pass-through) — gyro noise / RPM filter defer to v1
- `EnvironmentField`: `NullEnvironment` (returns `Vec3::ZERO`) — no wind in v0
- `ExternalForce`: empty registry

Future impls slot in cleanly:

- `ThrustModel`: `MixerOutputThrust` (level 1), `BladeElementThrust` (level 3)
- `MotorModel`: `ImplicitMotor`, `BldcMotor`
- `BatteryModel`: `ScalarSocBattery`
- `InflowModel`: `PropwashCurveInflow` (curve on axial descent), `VrsInflow` (positive-feedback descent stall), `CeilingEffectInflow` (per-prop AGL-up raycast)
- `SensorModel`: `NoisyImu` (Gaussian gyro noise + bias drift), `BetaflightFiltered` (notch + RPM filter chain)
- `EnvironmentField`: `ConstantWind`, `LayeredWind` (height-varying), `GustyWind` (Dryden / OU process), `TurbulenceField`, `ZoneWind` (per-region volumes from the map)
- `ExternalForce`: `Tether`, `ScriptedImpulse`, `ChoreographyForce` (for show flying)

## Flight controller

- Per-axis **"Actual" rates** (Betaflight-style) for roll / pitch / yaw:
  three numbers per axis — `center_sensitivity` (deg/s of commanded
  angular rate at low stick deflection, per unit stick), `max_rate`
  (deg/s at full stick), and `expo` (curve shape blending between the
  two). The other historical rate-curve variants (Betaflight
  RC-Rate/Super-Rate, RaceFlight, KISS, Quick) are deliberately not
  implemented; Actual rates are what modern pilots use and they map
  cleanly onto the two endpoints + curvature that matter.

  ```
  // sketch of the Actual rates evaluation per axis:
  let s   = stick_deflection;            // [-1, +1]
  let abs = s.abs();
  let lin = abs * center_sensitivity;
  let cur = max_rate * (abs.powf(1.0 + 8.0 * expo));
  let rate = s.signum() * lerp(lin, cur, abs);
  ```

- **Throttle curve** (lives in the same `ActualRatesPreset` as the
  per-axis rates, since it's also pilot-set stick shaping): two
  parameters, `throttle_mid` (stick position at which throttle output
  equals 0.5; default 0.5 = linear) and `throttle_expo` (curvature
  around the mid point; positive softens response near mid for finer
  hover control, negative steepens it). Same shape Betaflight uses;
  same two knobs the in-game settings menu will expose. Evaluated as a
  smooth piecewise cubic blend of `input` with `(input − mid)³` around
  `mid`.

- Three rate-PIDs (roll, pitch, yaw) — the inner loop. v0 has no outer
  loops; everything assumes Acro.
- Standard X-quad mixer matrix:
  ```
  [ motor_0 ]     [ +1  +1  −1  +1 ]   [ roll ]
  [ motor_1 ]  =  [ −1  +1  +1  +1 ] · [ pitch ]
  [ motor_2 ]     [ −1  −1  −1  +1 ]   [ yaw ]
  [ motor_3 ]     [ +1  −1  +1  +1 ]   [ throttle ]
  ```
- Anti-windup decay (`I = I · 0.98 + new_error · dt`) — simple and
  effective.

### Flight modes: Acro only in v0

v0 implements **Acro (rate) mode only**. The PIDs control body angular
rate directly; stick deflection commands a desired angular rate (deg/s)
via the Actual rates curves; releasing the sticks does *not* level the
drone. This is what racing/freestyle pilots fly, what the existing
`crsf-joystick` chain already produces, and what every test in the
workspace assumes.

Other Betaflight modes (Angle, Horizon, AltHold, PosHold) are not in
scope for v0. The architecture leaves them implementable as future
work without disturbing what's there:

- A `FlightMode` trait is wired into the FC pipeline as a no-op
  identity transform on the stick command, with a single
  `AcroMode` impl in v0:

  ```rust
  pub trait FlightMode {
      /// Convert raw stick command + truth state into the rate-PID setpoint
      /// (deg/s on each of roll/pitch/yaw). For Acro this is just the rates
      /// curve output; for Angle/Horizon it would be the output of an outer
      /// attitude PID; for PosHold the output of an outer position PID.
      fn rate_setpoint(&mut self, sticks: &SticksAetra, truth: &BodyState, dt: f32)
          -> Vec3;
  }
  ```

- AUX2 (CRSF channel 6) is reserved for future mode select but unused
  in v0 — whatever the radio sends there is ignored. When other modes
  land, AUX2 will pick between registered `FlightMode` impls.
- The rate PIDs, mixer, arming, and force pipeline are unchanged
  regardless of which `FlightMode` is active. All other modes are
  simply different ways to compute the rate setpoint that the inner
  PIDs track.

This means everything below the `FlightMode` boundary — physics,
mixer, motors, telemetry, swarm wake — can assume "stick → rate" and
not care which mode produced it.

### FC refinements deferred (no hooks needed)

These are internal Betaflight tuning features that can be added later
without touching the architecture: feedforward (derivative of stick
input), D-min (low-stick D-gain reduction), anti-gravity (I-term boost
on rapid throttle changes), RPM filter (motor-RPM-driven gyro notch).
None require new traits — they're all variations on the PID/mixer
pipeline. Likely added as configurable knobs on the existing FC during
a "real-FC realism" pass.

## Operational concerns

The bits a sim needs that the physics primer doesn't cover but every
shipped FPV sim solves somehow.

### RC channel ordering

Standard **AETRA** layout, 1-indexed in CRSF / radio parlance
(0-indexed in the `[u16; 16]` array on the wire):

| CRSF ch | Array idx | Letter | Purpose |
|---|---|---|---|
| 1 | 0 | A | Aileron — roll  |
| 2 | 1 | E | Elevator — pitch |
| 3 | 2 | T | Throttle |
| 4 | 3 | R | Rudder — yaw |
| 5 | 4 | A | AUX1 — **arm/disarm** |
| 6 | 5 |   | AUX2 — reserved for future flight-mode select; ignored in v0 (Acro only) |
| 7 | 6 |   | AUX3 — reserved (e.g. RC-mux SA-switch handoff, mirrors existing convention) |
| 8+ | 7+ |   | further AUX, unused by sim core |

This matches how `crsf-joystick` already lays out the channels and how
`crsf-forward` produces them, so the existing radio chain feeds the sim
without remapping.

### Arming

The drone starts disarmed; motor commands are forced to zero
regardless of throttle. Arming is gated by AUX1 per the AETRA layout
above:

- Default: **CRSF channel 5 (AUX1)** crossing 1500 µs going up arms,
  going down disarms.
- Channel index and threshold are configurable in `DronePreset`
  (`arm_aux_channel`, `arm_threshold_us`) for non-standard radio
  setups; the default values match AETRA.
- A safety gate also requires throttle (channel 3) to be at or below
  `arm_throttle_max_us` (default 1100 µs) at arm time —
  Betaflight-standard, prevents arming with throttle up.
- Disarming is also forced on whole-drone failure (impulse-threshold
  break or sustained battery brownout).

### Motor idle / minimum command

Real ESCs spin motors at a non-zero idle floor to prevent desync.
`MotorModel` exposes `idle_throttle_floor: f32` (typical 0.05) — the
output `motor_cmd` is `max(idle_floor, mixer_out) · brownout_factor`
when armed, `0` when disarmed.

### Battery brownout

When `V_cell` approaches the configured cutoff, motor authority
degrades smoothly rather than cliff-edge:

```
brownout_factor = clamp((V_cell − cutoff_v) / cutoff_band, 0.0, 1.0)
motor_cmd_effective = motor_cmd · brownout_factor
```

A drained battery feels under-powered before it dies completely. At
`brownout_factor == 0` for a sustained interval, the FC disarms.
Defaults: `cutoff_v = 3.3 V/cell`, `cutoff_band = 0.2 V`.

### Crash, respawn, and reset

- Per-prop damage hits 1.0 (broken) when collision impulse exceeds
  `crash_impulse_threshold_n_s` near that prop's offset.
- Whole-drone "destroyed" state: when ≥ 3 of 4 props are broken, or a
  body-mid hit exceeds a higher whole-drone threshold.
- On destroy: drone is held in place for `respawn_cooldown_s`
  (default 2.0), then teleported to its spawn pose with battery
  topped up and damage cleared. Configurable per drone.
- Manual reset (key bound in `input_router.gd`) triggers the same
  reset flow without a cooldown.

### Faking CRSF link statistics

`telemetry-dashboard` and other consumers expect RSSI / LQ / SNR in
the link-stats CRSF frame. v0 publishes constants
(`rssi_dbm = −60`, `lq = 100`, `snr_db = 10`) so the dashboard shows a
healthy link. Future: distance- and obstacle-modulated link quality
to model RC range loss when drones fly far from a notional ground
station.

### Sensor model placeholder

The `SensorModel` trait sits between truth-state physics and the FC.
v0 ships `TruthSensor` (pass-through). This keeps the option open to
later add Gaussian gyro noise, accel noise, bias drift, the Betaflight
notch + RPM-filter chain — without changing how the FC reads the IMU.

## CRSF I/O — wire compatibility

### Input topics

- **Per-drone**: `{prefix}{id}/crsf/rc` and
  `{prefix}{id}/crsf/rc/autopilot`. Decoded with
  `telemetry_lib::crsf::CrsfPacket`.
- **Active drone manual override**: a configurable `--manual-rc-topic`
  (default `sim/crsf/rc`) that the currently-active drone subscribes to
  in addition to its own per-id topic. SA-switch handoff (channel 7)
  works as in `crsf-joystick`.

### Output topics

Per-drone CRSF telemetry frames at the same cadence as the existing
`liftoff-input` bridge:

- `{prefix}{id}/crsf/telemetry` — attitude (50 Hz), GPS (10 Hz), battery
  (5 Hz), link statistics (1 Hz).
- `{prefix}{id}/damage` — per-prop damage frames (custom CRSF type
  `0x42`, identical to `crsf_custom`); EdgeTX LUA receives them
  unchanged.
- `{prefix}{id}/battery` — extended battery telemetry (per-cell V,
  current draw, mAh used) when the curve-based battery model has the
  data.

### CRSF channel mapping

The mapping is currently hardcoded in `crsf-joystick/src/lib.rs`. The
plan is to extract it into `telemetry-lib::crsf::rc_mapping` so both
`crsf-joystick` and `godot-swarm-sim` consume the same definition.

## Topic plan

| Topic | Direction | Example |
|---|---|---|
| `{prefix}{id}/crsf/rc` | sim ← autopilot or remote pilot | `sim0/crsf/rc` |
| `{prefix}{id}/crsf/rc/autopilot` | sim ← autopilot | `sim0/crsf/rc/autopilot` |
| `{prefix}{id}/crsf/telemetry` | sim → world | `sim0/crsf/telemetry` |
| `{prefix}{id}/damage` | sim → world | `sim0/damage` |
| `{prefix}{id}/battery` | sim → world | `sim0/battery` |
| `--manual-rc-topic` | sim ← local manual radio | default `sim/crsf/rc`, configurable |

CLI examples for downstream services after launch:

- `telemetry-dashboard --zenoh-prefix sim0`
- `autopilot --zenoh-prefix sim2`
- `crsf-gpsd --zenoh-prefix sim0`
- `mavlink-bridge --zenoh-prefix sim0`

## Camera / viewport

- Per-drone `Camera3D` (child of the drone). Pinhole projection in v0;
  fisheye toggle in a later phase.
- **FPV camera angle is a first-class configurable parameter**, not a
  build-time constant. Racing/freestyle drones tilt the camera upward
  so that when the drone is pitched forward to fly, the horizon stays
  in view. Typical real-world values: 0–10° for cinematic, 25–35° for
  freestyle, 35–55° for racing. Configured via:
  - **`fpv_camera_tilt_deg`** in `DronePreset` (per-preset default).
    Applied as a fixed rotation on the camera's local X-axis at scene
    load.
  - **`fpv_camera_fov_deg`** in `DronePreset` (default ~120°).
  - **`fpv_camera_offset`** in `DronePreset` — body-frame `Vector3`
    for the camera's mount position (so different frames can place
    the cam differently relative to the centre of mass).
  - **Live adjustment**: per-drone tilt and FOV exposed via
    `DroneController` setters (gdext `#[func]`) and bindable to
    keyboard / settings UI; persisted back to the preset resource on
    save. This matches what every shipped FPV sim offers — pilots
    have strong personal preferences and need to tune cam angle to
    match their real-life setup. Crucially, the angle is **not
    static** — it is always either preset-driven or live-tunable.
  - **Future: RC-bound tilt control** (not v0, but architecturally
    open). Tilt could be driven by a CRSF AUX *axis* (e.g. AUX3 as a
    proportional pot) or by a *button pair* (one channel for
    tilt-up, one for tilt-down, momentary). The setter API on
    `DroneController` already accepts arbitrary updates per tick, so
    wiring CRSF input to it is a small addition in `crsf_io.rs` plus
    a `camera_tilt_source` enum on the preset (`Static` | `KeyboardOnly`
    | `RcAxis(channel)` | `RcButtons(up_ch, down_ch, rate_deg_s)`).
  - **Optional shake**: a small placeholder `camera_shake_amount` float
    in the preset that future audio/vibration models can drive. Zero
    in v0.
- A single `CameraDirector` GDScript node manages a `SubViewportContainer`
  grid:
  - **Single mode** (default): one full-screen viewport bound to the
    active drone's camera.
  - **Tile mode**: dynamic √N×√N grid (rounded up) computed from the
    actual drone count at scene load — e.g. 2×2 for 4 drones, 4×4 for
    16, 6×6 for 36, etc. No hardcoded layout sizes.
  - F1/F2 cycle through layout presets (single / tile-current /
    tile-bigger); number keys 0..9 plus modifier (e.g. Shift+digit for
    +10, Ctrl+digit for +20) set active drone, scaling beyond ten
    drones without source changes.
- OSD rendered as a `CanvasLayer` overlay per viewport (battery %,
  throttle, attitude indicator).
- Audio listener follows the active drone in single mode; in tile mode
  it stays on the last-active drone or a free-fly camera.

## DronePreset as a Godot Resource

Registered via gdext as a `Resource` subclass with `@export`-style
fields. Authored in the Godot inspector, persisted as `.tres`, scenes
carry the reference.

```rust
#[derive(GodotClass)]
#[class(base=Resource)]
pub struct DronePreset {
    // Body
    #[var] mass_kg: f32,
    #[var] inertia_diag: Vector3,           // diag(Ixx, Iyy, Izz)
    #[var] cd_per_axis: Vector3,            // body-frame drag coefs (front, side, top)
    #[var] area_per_axis: Vector3,          // projected areas

    // Props (4× X-quad)
    #[var] prop_offsets: Array<Vector3>,    // body-frame positions
    #[var] prop_axes: Array<Vector3>,       // body-frame thrust axes
    #[var] prop_handedness: PackedInt32Array, // ±1 per prop
    #[var] prop_diameter_m: f32,

    // Power
    #[var] motor_kv: f32,
    #[var] motor_idle_throttle_floor: f32,      // ESC idle (typical 0.05)
    #[var] battery_cells: i32,
    #[var] battery_internal_r_mohm: f32,
    #[var] battery_capacity_mah: f32,
    #[var] battery_discharge_curve: Gd<Curve>,  // Godot-native Curve resource
    #[var] battery_cutoff_v_per_cell: f32,      // brownout midpoint (e.g. 3.3 V)
    #[var] battery_cutoff_band_v: f32,          // brownout fade width (e.g. 0.2 V)
    #[var] thrust_curve: Gd<Curve>,
    #[var] motor_throttle_rpm_curve: Gd<Curve>, // commanded throttle → RPM target (powertrain)
    #[var] kvdrop_machspeed_curve: Gd<Curve>,
    #[var] kvdrop_loading_curve: Gd<Curve>,
    #[var] thrust_ratio_props_curve: Gd<Curve>, // damage → thrust scale

    // FC
    #[var] pid_roll: Vector3, #[var] pid_pitch: Vector3, #[var] pid_yaw: Vector3,
    #[var] rates: Gd<ActualRatesPreset>,        // pilot stick shaping (per-axis rates + throttle curve)

    // Arming / safety
    #[var] arm_aux_channel: i32,                // 1-based CRSF AUX index (default 5 = AUX1)
    #[var] arm_threshold_us: i32,               // commanded µs threshold (default 1500)
    #[var] arm_throttle_max_us: i32,            // refuse arming above this throttle (default 1100)

    // FPV camera
    #[var] fpv_camera_tilt_deg:    f32,         // upward tilt about body X (default 30°; range typically 0–55)
    #[var] fpv_camera_fov_deg:     f32,         // pinhole vertical FOV (default 120°)
    #[var] fpv_camera_offset:      Vector3,     // body-frame mount position
    #[var] camera_shake_amount:    f32,         // placeholder, 0 in v0

    // Damage / respawn
    #[var] crash_impulse_threshold_n_s: f32,
    #[var] respawn_cooldown_s: f32,             // hold time before reset (default 2.0)
}

#[derive(GodotClass)]
#[class(base=Resource)]
pub struct ActualRatesPreset {
    // Per-axis "Actual" rates: center_sensitivity (deg/s/stick), max_rate (deg/s), expo
    #[var] roll:  Vector3,
    #[var] pitch: Vector3,
    #[var] yaw:   Vector3,
    // Throttle stick curve
    #[var] throttle_mid:  f32,                  // 0.5 = linear
    #[var] throttle_expo: f32,                  // curvature around throttle_mid
}
```

`Curve` is a Godot-native resource and is inspector-editable, perfect
for the curve-shaped throttle / KV-drop / thrust-ratio tables.

Example presets shipped in `godot/presets/`:

- `racing_5inch.tres`
- `freestyle_5inch.tres`
- `cinewhoop_3inch.tres`

## Workspace layout

```
liftoff-rs/
├── Cargo.toml                                    # add godot-swarm-sim/rust to members
├── godot-swarm-sim/
│   ├── README.md
│   ├── godot/                                    # Godot project root
│   │   ├── project.godot                         # physics_ticks_per_second=240, jolt_physics_3d
│   │   ├── addons/
│   │   │   └── godot-swarm-sim/
│   │   │       └── godot-swarm-sim.gdextension   # points to ../../rust/target/release/*.so
│   │   ├── scenes/
│   │   │   ├── world.tscn
│   │   │   ├── drone.tscn
│   │   │   ├── viewport_grid.tscn
│   │   │   └── sky.tscn
│   │   ├── presets/
│   │   │   ├── racing_5inch.tres
│   │   │   ├── freestyle_5inch.tres
│   │   │   └── cinewhoop_3inch.tres
│   │   ├── maps/
│   │   │   └── default_outdoor.tscn              # heightmap + obstacles + sky
│   │   ├── scripts/
│   │   │   ├── world.gd                          # spawn N from drones.json
│   │   │   ├── camera_director.gd                # tile / single, F-key toggles
│   │   │   ├── input_router.gd                   # keyboard, active-drone switch, camera tilt/FOV live-tune
│   │   │   └── osd.gd                            # per-viewport OSD
│   │   └── drones.json                           # default fleet config
│   └── rust/
│       ├── Cargo.toml                            # cdylib named `godot_swarm_sim`
│       └── src/
│           ├── lib.rs                            # gdext class registration
│           ├── drone.rs                          # DroneController (gdext class)
│           ├── preset.rs                         # DronePreset Resource (gdext)
│           ├── fc/
│           │   ├── mod.rs
│           │   ├── rates.rs                      # ActualRatesPreset evaluation (per-axis rates + throttle curve)
│           │   ├── pid.rs                        # rate PID (inner loop)
│           │   ├── mode.rs                       # FlightMode trait + AcroMode (Angle/Horizon/PosHold later)
│           │   └── mixer.rs                      # X-quad mixer (constant matrix)
│           ├── physics/
│           │   ├── mod.rs                        # Step orchestrator
│           │   ├── motor.rs                      # MotorModel + CurveMotor (incl. ESC idle floor, brownout scaling)
│           │   ├── battery.rs                    # BatteryModel + SagBattery (incl. brownout_factor)
│           │   ├── thrust.rs                     # ThrustModel + CurveThrust
│           │   ├── drag.rs                       # DragModel + PerAxisQuadraticDrag
│           │   ├── ground.rs                     # per-prop AGL raycast
│           │   ├── wake.rs                       # WakeField + WakeModel + GaussianWake
│           │   ├── inflow.rs                     # InflowModel trait + IdentityInflow (VRS / propwash / ceiling effect later)
│           │   ├── sensor.rs                     # SensorModel trait + TruthSensor (gyro noise + RPM filter later)
│           │   ├── env.rs                        # EnvironmentField trait + NullEnvironment + CompositeEnvironment
│           │   └── external.rs                   # ExternalForce trait + per-drone registry
│           ├── crsf_io.rs                        # CRSF telemetry encode + fake link statistics (uses telemetry-lib)
│           ├── zenoh_io.rs                       # ZenohBus + Tokio thread + SPSC
│           ├── arming.rs                         # AUX-channel arming gate, throttle-low safety
│           ├── respawn.rs                        # crash detection, cooldown, reset to spawn pose
│           └── damage.rs                         # impulse-threshold prop break
```

## Testing strategy

Three layers, in order of how much of the stack they exercise:

1. **Unit tests in pure Rust (`cargo test`)** — every math / logic
   module (rates, PID, mixer, motor, battery, thrust, drag, ground,
   arming, pipeline orchestrator) lives in plain Rust with no Godot
   types in its public API and ships with comprehensive `#[cfg(test)]`
   coverage. These run anywhere the workspace's `cargo test` runs and
   give the bulk of confidence that physics changes haven't regressed.
   Strategy: hot loops are pure functions over `f32` / `[f32; 3]`, the
   gdext layer just wires Godot inputs into those. Means future
   gdext-API churn can only break the wiring, not the math.

2. **Headless gdext smoke tests (`godot --headless`)** — the extension
   loads, the scene boots, the drone class registers and instantiates,
   the per-tick orchestrator runs at the configured physics rate.
   Driven by two env vars from Phase 0 onward:
   - `GSS_AUTOQUIT_S=<seconds>` — quit the SceneTree after N seconds
     of sim time. Used by every CI run to keep test duration bounded.
   - `GSS_AUTOFLY=hover|climb` — replace keyboard polling with a
     canned 3-phase profile (settle → arm-with-throttle-low → ramp
     stick to target). Exercises the arming gate, motor spool-up,
     thrust-to-weight, and the rigid-body integrator end-to-end
     without needing a windowed display.
   These run in CI and double as integration tests against gdext API
   churn — if a `godot` crate update breaks our wiring, the smoke
   tests fail before the drone has a chance to mis-fly.

   **Outer-layer runners are written in Python**, never in shell.
   Anything that isn't pure Rust math (Layer 1) or GDScript (engine
   glue) lives in `*.py` scripts (e.g.
   `godot-swarm-sim/smoke-test.py`). Python gives us proper
   subprocess handling, regex parsing, exit-code semantics, and easy
   cross-platform behaviour without the failure modes of bash
   pipelines.

3. **Interactive verification** — open the project in `godot`,
   keyboard-fly with the layout in `input_stub.rs`, observe feel.
   Catches things automated tests don't (PID feel, camera tilt, OSD
   readability). Documented in `godot-swarm-sim/README.md`.

Going forward, every phase that adds physics should add a Layer-1 unit
test for the new math module *and* extend the Layer-2 smoke test with
a behavioural check (e.g. Phase 3's wake field gets a smoke test
where two drones fly stacked and the lower one shows reduced thrust).

## Phased delivery

### Phase 0 — toolchain skeleton (~1 day)

Empty Godot 4.4 project with Jolt enabled; gdext "hello-world" extension
class loaded; build pipeline verified (`cargo build` → drop `.so` →
Godot loads the class). Confirm Jolt at 240 Hz with one falling cube.

**Done when**: `cargo build --release` from workspace root produces a
shared library that Godot picks up, and a stub `DroneController` prints
its physics-tick rate to the console.

### Phase 1 — single drone + outdoor terrain (~5 days)

- Heightmap terrain via `HeightMapShape3D` + sky environment + sun +
  scattered cylindrical obstacles.
- `Drone.tscn` with `RigidBody3D` (Jolt), `CollisionShape3D`, four
  `Marker3D` prop positions, `FpvCamera`, and a `DroneController` Rust
  child.
- Rust side: `CurveMotor` + `SagBattery` + `CurveThrust` (still using
  body velocity only for `V_local`, no wake yet); per-axis quadratic
  drag; per-prop ground-effect raycast; per-prop reaction torque.
- FC: rates → PID → mixer.
- `DronePreset` Resource (`.tres`) wired through; one default preset
  shipped (`racing_5inch.tres`).
- Stub direct-poll input via Godot's built-in `Input` to validate
  hover/roll/pitch/yaw feel before Zenoh is in the path.

**Done when**: a single drone hovers under joystick input, behaves
sensibly under throttle punches and direction changes, and the battery
sags realistically over a flight.

### Phase 2 — Zenoh I/O (~2 days)

- `ZenohBus` gdext class with a Tokio runtime on its own thread.
- SPSC channels between the Tokio thread and per-drone controllers.
- Subscribe `sim0/crsf/rc` + `sim0/crsf/rc/autopilot` + the configurable
  `--manual-rc-topic` (default `sim/crsf/rc`).
- Publish `sim0/crsf/telemetry`, `sim0/damage`, `sim0/battery`.
- Refactor CRSF channel mapping out of `crsf-joystick` into
  `telemetry-lib`.

**Done when**: `telemetry-dashboard --zenoh-prefix sim0` displays the
sim's telemetry without source changes, and `crsf-forward` (running with
its existing prefix) drives the active drone via the manual-RC topic.

### Phase 3 — multi-drone + spatial-hash wake field (~3 days)

- Spawn N from `drones.json` with per-id `simN` prefixes.
- Per-id `ZenohBus` channels.
- Double-buffered `WakeField` with the coarse 3D spatial hash described
  above; `GaussianWake` kernel as default.
- Per-prop wake sample feeds `V_local` for thrust; per-prop wake
  contribute after the thrust step.
- Coordinator step swaps the snapshot once per physics tick.

**Done when**: a drone visibly loses thrust flying directly under
another's downwash, and 4 simultaneously-flying drones all telemeter
correctly on their own `simN` prefixes.

### Phase 4 — multi-view + active-drone switch (~2 days)

- `CameraDirector` GDScript node, `SubViewportContainer` grid, F1/F2
  layout toggle, number keys for active drone.
- Per-viewport OSD overlay.
- Audio listener routing.

**Done when**: F1 toggles a 2×2 tile view of all drones, F2 a 4×4 view,
and number keys reassign the active drone (which receives manual RC
from the `--manual-rc-topic`).

### Phase 5 — polish & extras (~3-5 days)

- Impulse-threshold per-prop damage with crash effects (sound, particle,
  motor cut-off).
- Polish on headless mode: a `--no-throttle` option to run faster than
  realtime (useful for autopilot CI / soak tests). Headless itself is
  already supported from Phase 0 onward — `godot --headless` plus the
  `GSS_AUTOQUIT_S` and `GSS_AUTOFLY` env vars give a complete CI loop.
- KV-drop curves wired in. (Wind / gusts deliberately deferred — the
  `EnvironmentField` extension point already exists from Phase 1, so
  adding wind is a separate, post-v0 task that doesn't change Phase 5
  scope.)
- Benchmark 16 drones at 240 Hz on a representative laptop and document
  the per-tick budget; if it slips, fall back to 120 Hz with substepping.
  Also benchmark a stretch run (e.g. 32 or 64 drones) to characterise
  how cost scales — useful for sizing future releases.
- Optional: fisheye camera toggle, second example map.

**Done when**: 16 drones fly simultaneously in tile view at locked 240
Hz on the reference machine, with collisions between them and the
world, full Zenoh I/O on `sim0`..`sim15`, and per-prop damage from
crashes. Going beyond 16 should be a `drones.json` change plus enough
hardware, with no source edits required.

### Milestones

- **MVP** (single drone flying via Zenoh, dashboard sees it): end of
  Phase 2 (~8 days).
- **First-release target** (~16-drone tiled view with wake interaction;
  no hard cap in code): end of Phase 5 (~16 days).

## Compatibility with the existing stack

Downstream services consume the new sim with no source changes:

- `crsf-forward` runs as today, publishing to its configured prefix —
  the active drone consumes it via `--manual-rc-topic`.
- `autopilot --zenoh-prefix simN` controls drone N over
  `simN/crsf/rc/autopilot`.
- `telemetry-dashboard --zenoh-prefix simN` dashboards drone N.
- `crsf-gpsd --zenoh-prefix simN` serves drone N's NMEA.
- `mavlink-bridge --zenoh-prefix simN` bridges drone N to MAVLink GCSes.
- EdgeTX LUA damage script consumes `simN/damage` frames in the
  existing wire format.

`crsf-joystick` becomes optional for the local pilot — useful if you
want the sim's RC to also be visible to the OS as a uinput device (e.g.
for non-sim apps), but no longer in the path between the radio and the
active drone.

## Risks / things to watch

- **gdext + Tokio threading discipline**: Tokio runtime must live on a
  non-Godot thread; communication crosses via SPSC ringbuffers.
  Mis-shape this and Godot crashes.
- **240 Hz × ~16 drones × wake-field**: should fit comfortably (~1 ms/tick
  of CPU on modern hardware), but unconfirmed. Phase 5 benchmark is the
  gate; if it slips, drop to 120 Hz with substepping. The wake-field
  spatial-hash design scales reasonably to higher drone counts (cell
  size and bucket capacity are runtime-configurable), but very large
  swarms (≫16) may want a job-system parallel pass — left as a future
  optimization, not a v0 concern.
- **Godot Resource hot-reload across `.tres` edits**: works at
  scene-load; live editing during play needs explicit re-load logic.
  Acceptable for Phase 1; revisit if it bites.
- **CRSF channel mapping**: hardcoded today in `crsf-joystick/src/lib.rs`;
  it must be extracted into `telemetry-lib::crsf::rc_mapping` so both
  crates share it.
- **HeightMapShape3D collision performance**: Jolt handles heightmaps
  well, but terrain resolution should stay moderate (≤ 1024×1024 cells
  for a 1×1 km map).
- **gdext API stability**: still pre-1.0; minor breakage on Godot
  upgrades is possible. Pin a known-good gdext + Godot pair in
  `Cargo.toml` and `project.godot`.
- **Wake-field one-tick lag**: the double-buffer means each drone
  samples last-tick's wake. At 240 Hz this is 4.2 ms — fine for normal
  flight, may produce minor artefacts when two drones are in close
  proximity at high relative speed. Acceptable v0 tradeoff; can be
  addressed by iterating the wake/thrust solve twice if it bites.
- **Diagonal-only inertia tensor**: assumes principal axes coincide
  with body axes (true for most symmetric quads). Asymmetric craft
  (heavy battery off-centre, hex frames, side-mounted gimbals) want
  off-diagonal terms. This is an **engine limitation**, not a design
  choice: Godot's `RigidBody3D.inertia` and Jolt's body parameter
  both only accept a `Vector3` (three principal moments). They don't
  support a full 3×3 inertia matrix. The workaround for asymmetric
  builds would be to eigen-decompose the full tensor, use the
  eigenvalues as the diagonal, and rotate the collision shape by the
  eigenvector matrix so the body frame aligns with the principal
  frame. The proper fix is an upstream Jolt/Godot change to accept a
  full symmetric 3×3.

## Future-realism notes

Items the physics primer covers that aren't in v0 and don't have
extension points yet, because they're niche or high-cost to wire
through. Adding any of them later is a focused change rather than a
trait-introduction:

- **FC tuning niceties** (feedforward, D-min, anti-gravity, RPM filter)
  — all live inside the existing PID/mixer pipeline; no architecture
  changes needed.
- **Air density variation with altitude** — currently a constant in
  `PropCtx`. Trivial to make a function of altitude later.
- **Off-diagonal inertia** — engine limitation (see Risks section).
  Ideal fix: upstream Jolt/Godot to accept a full symmetric 3×3
  inertia tensor. Workaround: eigendecompose + rotate collision shape.
- **DShot quantization / PWM resolution** — invisible at v0 fidelity.
- **Gyroscopic precession from spinning rotors** — small enough to
  ignore for racing-quad scales.
- **Audio**: motor RPM → pitched whine. Phase 5 polish; Godot's
  AudioStreamPlayer3D + per-drone bus + RPM-driven pitch shift is
  enough.
- **Glauert / tilted-disc wake tracker** — only relevant once thrust
  goes to level 3 (blade-element). Defer with that.

## Out of scope for v0

- **Wind, gusts, turbulence, thermals, region-of-effect zones, tethers,
  scripted forces.** All deferred — but the `EnvironmentField` and
  `ExternalForce` extension points are wired through the per-tick step
  from day 1, with no-op default implementations. Adding any of these
  later is an isolated change: write the impl, register it, no edits to
  `DroneController`, `ThrustModel`, or `DragModel`. Particularly
  interesting once the swarm is up — a 16-drone formation moving
  through a gust front already couples through the wake field, so the
  emergent behaviour will be richer than for a single drone.
- Inter-drone aerodynamic effects beyond wake (e.g. blade-on-blade
  vortex coupling).
- Tail rotors, coaxial rotors, hex/octo configurations.
- GPS denial or sensor noise modelling.
- Networked multiplayer across machines (single-process today; Zenoh
  could enable cross-process later).
- **Non-Acro flight modes** (Angle, Horizon, AltHold, PosHold). The
  `FlightMode` trait is wired in v0 with a single `AcroMode` impl;
  AUX2 is reserved on the wire but unused. Adding self-leveling or
  position hold is a focused change behind that trait — no impact on
  physics, mixer, telemetry, or swarm code.
- Advanced flight modes (waypoint missions, return-to-home) — the
  `autopilot` crate handles those externally and feeds RC over Zenoh.
- Photorealistic art / sound design.
