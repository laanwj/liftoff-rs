# liftoff-simstate-bridge

A BepInEx 5 plugin for [Liftoff: FPV Drone Racing](https://store.steampowered.com/app/410340/) that exposes simulator state Liftoff's built-in `DroneTelemetryManager` does not put on the wire — currently per-propeller damage and battery telemetry (voltage, current, charge drawn, percentage, per-cell voltage, cell count). Both stream over UDP on the same port; consumers dispatch by the 4-byte ASCII tag at the start of each packet.

This is the only non-Rust component in the `liftoff-rs` workspace — it is a C# Unity mod, not a Cargo crate.

## What it does

- Locates the player's drone via `FlightManager.CurrentDrone` each frame, only while a flight is active.

- **Damage stream (`LFDM`):** reads the raw `float DamageState` from every `Propeller` component under that drone. Sent on every change plus a 10 Hz heartbeat (in-flight only).

- **Battery stream (`LFBT`):** reads voltage, per-cell voltage, current draw, charge drawn (Ah), and remaining percentage from the `BatteryDrainer` MonoBehaviour, plus cell count from `BatteryPart.NrOfCells`. Sent at a fixed 10 Hz cadence (in-flight only).

- Both streams share one UDP port. Filters to the local player's drone only — multiplayer ghosts are ignored.

## Building

Requires .NET SDK 6.0 or newer (8.0 recommended).

```sh
dotnet build -c Release
```

The build references `Assembly-CSharp.dll`, `UnityEngine.dll`, and `UnityEngine.CoreModule.dll` from the Liftoff installation. The default path is

```
$HOME/.local/share/Steam/steamapps/common/Liftoff
```

Override with:

```sh
dotnet build -c Release -p:LiftoffPath=/path/to/Liftoff
```

Output DLL lands in `bin/Release/LiftoffSimstateBridge.dll`.

## Installing

1. Install **BepInEx 5** (Mono build, x64) into the Liftoff folder. Download from <https://github.com/BepInEx/BepInEx/releases>; pick the `BepInEx_unix_x64_5.4.x.zip` archive on Linux or `BepInEx_win_x64_5.4.x.zip` on Windows. Extract the contents directly into the Liftoff install folder so that `BepInEx/`, `doorstop_config.ini`, and `run_bepinex.sh` (Linux) or `winhttp.dll` (Windows) sit next to `Liftoff.x86_64`.

2. Launch Liftoff once so BepInEx generates `BepInEx/plugins/`.

3. Copy `bin/Release/LiftoffSimstateBridge.dll` into `BepInEx/plugins/`.

4. On Linux, ensure Liftoff's launch command runs BepInEx's loader. The `launch.sh` shipped by Liftoff usually just execs `Liftoff.x86_64 ...`; you can either edit it to invoke `./run_bepinex.sh Liftoff.x86_64 ...` instead, or set Steam's launch options for the game to: ` ./run_bepinex.sh %command% `

5. Launch the game. Plugin status appears in `BepInEx/LogOutput.log` — look for `Liftoff Simstate Bridge ready: emitting to 127.0.0.1:9020`.

### Configuration

After the first run, BepInEx creates `BepInEx/config/com.liftoffrs.simstatebridge.cfg` with:

```ini
[Network]
TargetAddress = 127.0.0.1
TargetPort = 9020
```

Edit and restart the game to change the target.

## Wire format

Little-endian, fixed-layout UDP datagrams. One packet = one snapshot. The first 4 bytes are an ASCII tag the consumer dispatches on.

### Damage packet — tag `LFDM`

| Offset | Size | Field | Notes |
| ----:| --:| ------------ | ----------------------------------------------------------- |
| 0 | 4 | `tag` | ASCII `LFDM` (0x4C 0x46 0x44 0x4D) |
| 4 | 2 | `version` | u16, currently `1` |
| 6 | 2 | `flags` | u16 bitfield (see below) |
| 8 | 8 | `timestamp_ms` | u64, monotonic milliseconds since plugin load |
| 16 | 1 | `prop_count` | u8, number of propellers in this packet (≤ 8) |
| 17 | 3 | `pad` | always zero, reserved |
| 20 | 4·N | `damage[N]` | N × `f32`, raw `Propeller.DamageState` values, in prop order |

Total damage packet size = `20 + 4 * prop_count` bytes.

#### Damage `flags` bits

| Bit | Name | Meaning |
| -:| -------- | -------------------------------------------------- |
| 0 | `KILLED` | `FlightManager.IsKilled` — drone destroyed |
| 1 | `CRASHED` | `FlightManager.IsCrashed` — drone in crashed state |
| 2 | `NO_DRONE` | No current drone (menu, between flights) |

When `NO_DRONE` is set, `prop_count` is `0` and there are no float values.

#### `damage[N]` semantics

These are the raw values Liftoff stores internally — the plugin does no normalization. Empirically the field is `0.0` when healthy and increases as damage accumulates; the in-game HUD picks the "healthy / damaged / broken" color based on internal thresholds. The Rust consumer should threshold and/or normalize as needed; if you want to mirror the OSD coloring, you'll need to calibrate the thresholds against in-game behavior.

Propellers are reported in the order returned by Unity's `GetComponentsInChildren<Propeller>()`, which matches the order of `MotorRPM` in Liftoff's standard telemetry stream: **left front, right front, left back, right back**.

### Battery packet — tag `LFBT`

Fixed 40 bytes. Sent at 10 Hz whenever the plugin's `BatteryDrainer` reflection is set up (always, when the in-game battery sim is active for the current drone).

| Offset | Size | Field | Notes |
| ----:| --:| ---------------- | ------------------------------------------------------------------ |
| 0 | 4 | `tag` | ASCII `LFBT` (0x4C 0x46 0x42 0x54) |
| 4 | 2 | `version` | u16, currently `1` |
| 6 | 2 | `flags` | u16 bitfield (see below) |
| 8 | 8 | `timestamp_ms` | u64, monotonic milliseconds since plugin load (same clock as damage) |
| 16 | 1 | `cell_count` | u8, `BatteryPart.NrOfCells` (0 if no battery part on the drone) |
| 17 | 3 | `pad` | always zero, reserved |
| 20 | 4 | `voltage` | f32, total pack voltage (V) |
| 24 | 4 | `voltage_per_cell` | f32, per-cell voltage (V) |
| 28 | 4 | `current_amps` | f32, instantaneous current draw (A) |
| 32 | 4 | `charge_drawn_ah` | f32, accumulated charge drawn (Ah; multiply by 1000 for mAh) |
| 36 | 4 | `percentage` | f32, remaining capacity (0.0 – 1.0; multiply by 100 for percent) |

#### Battery `flags` bits

| Bit | Name | Meaning |
| -:| ---------- | -------------------------------------------------------------------------------------------------- |
| 0 | `NO_DRAINER` | No `BatteryDrainer` found on the current drone (battery sim disabled / `EnableBattery` modifier off) |
| 1 | `NO_DRONE` | No current drone (menu, between flights) |

When either flag is set, the float fields and `cell_count` are zero.

#### Battery semantics

All values are read directly from the `BatteryDrainer` MonoBehaviour. The Liftoff HUD modules display these by:

- voltage as `RoundToInt(voltage * 10)` formatted with the per-display format string ("Default" battery management) or `* 100` (DJI),

- per-cell voltage as `RoundToInt(voltage_per_cell * 100)` with `{0:F2}` format,

- current as `RoundToInt(current_amps)`,

- mAh drawn as `RoundToInt(charge_drawn_ah * 1000)`.

So `voltage` is volts, `voltage_per_cell` is volts, `current_amps` is amps, `charge_drawn_ah` is amp-hours, and `percentage` is a 0–1 fraction.

The plugin emits even when the battery sim is off (the `NO_DRAINER` flag is set in that case) so consumers can tell "plugin running, sim disabled" apart from "plugin not running".

## Cadence

- **Damage**: change-driven — a packet is sent immediately whenever any `DamageState` value differs from the last sent snapshot. A 10 Hz heartbeat re-emits the current state regardless, so the consumer can detect plugin presence and recover from packet loss.

- **Battery**: fixed 10 Hz. Battery values change every frame under load (voltage sag follows current draw); change detection would emit on essentially every frame, so a fixed cadence is simpler and equivalent.

- The plugin's per-frame tick is driven from inside `FlightManager.Update` via a Harmony postfix. **Telemetry only flows during a flight** — there are no menu / pre-flight heartbeats. The Harmony hook fires once per Unity frame (typically 60–200 Hz). Worst-case bandwidth: damage ≈ 7 KB/s (capped by frame rate), battery 0.4 KB/s (10 packets/s × 40 B).

## Caveats

- The drone class returned by `FlightManager.CurrentDrone` has an obfuscated identifier in `Assembly-CSharp.dll`. The plugin handles this by treating it as `object` and downcasting to `Component` — so a future Liftoff update that reorganizes the drone class hierarchy might break this assumption. If that happens, the log will show no propellers found.

- The `BatteryDrainer` class **and most of its members** are obfuscated (47-character identifiers made of pure `'` and `"` ASCII bytes). The plugin resolves the type at startup by reading the `FieldType` of `DroneHUDModule.AmpTotal.batteryDrainer` (a clean named field), and looks up each getter by its raw byte sequence — encoded as hex constants (`DrainerVoltageNameHex`, etc.) in `Plugin.cs`. If a Liftoff update changes those getters, the log will show "Battery getter resolution: only N/5 found" and the corresponding fields will be zero. Recovery: re-run the IL inspection on `DroneHUDModule.AmpTotal::Tick`, `BatteryVoltagePerCell::Tick`, `BatteryMilliAmpHourDrawn::Tick`, `BatteryManagementDefault::Tick` to identify the new method names, and update the hex constants.

- Built and tested against Liftoff `1.7.1`. Major game updates may rename `FlightManager`, `Propeller`, or `DamageState`; rebuild against the new `Assembly-CSharp.dll` if so.

- BepInEx is a third-party mod loader. Do not run with EAC-protected multiplayer (Liftoff does not currently use anti-cheat, but Lugus could add one in a future update).

## Future: generalized event stream

The damage stream above is fundamentally a *state* feed — per-frame floats, snapshot-style. Several other things in Liftoff are inherently *discrete events* (a thing happened at time T) and don't fit naturally into the same wire format. A second channel for those would extend this plugin without overloading the damage packet.

### Events worth exposing

Ranked roughly by usefulness for autopilot/scripting:

- **Race / lap / checkpoint** — the highest-value addition. Liftoff exposes `onCountDownStart` / `onCountDownTick` / `onCountDownFinished` / `onFinishReached` as proper events on the relevant manager. Per-checkpoint passage and lap completion do **not** have public event accessors; they're computed inside `DroneCheckpointDelegator` and `RaceTriggerCheckpoint`, plus the `LapCounter` / `LapTimer` HUD modules. Getting them out cleanly likely needs a Harmony patch on the checkpoint trigger entry (or polling the lap counter state). With these, you can script autopilot test runs against the built-in race tracks and measure objective performance (lap times, gate-pass timing).

- **Drone lifecycle** — `onPlayerDroneBuilt` / `onPlayerDroneArmed` / `onPlayerDroneDestroyed` / `onDroneResetStart` / `onDroneResetDone` on `FlightManager`. The damage stream's `KILLED` / `NO_DRONE` flags partly cover this, but discrete events let an autopilot reset its own state deterministically on respawn instead of polling for flag transitions.

- **Crash / kill discrete events** — `onDroneCrashed` / `onDroneKilled`. Already represented as flags here, but events let you count occurrences (crashes per session, time between crashes) without edge-detecting a flag stream.

- **Per-propeller collision** — `Propeller.onPropellerCollision`. Fires on every micro-bump, so probably more noise than signal; only worth wiring up if you specifically want a "perception of surface contact" signal.

### Suggested transport: WebSocket

UDP datagrams worked well for the damage stream because the payload is fixed-shape, high-rate, and tolerant of loss. Events are the opposite — variable shape (different fields per event type), low-rate but **must not be lost** (a missed lap-completion is a missing data point you can't reconstruct), and consumers want them in order.

A WebSocket server in the plugin (Velocidrone-style — they expose their telemetry over a WebSocket JSON API) fits this well:

- TCP underneath gives ordered, reliable delivery for free.

- Built-in framing means each event is one message; no header/length juggling.

- JSON payloads scale to heterogeneous event types without a versioned binary schema per event.

- Fan-out is easy: multiple consumers (Rust pipeline, a debug HTML page, a logger) can connect to the same port concurrently. Compare to UDP where you either pick one destination or rely on broadcast/multicast.

- The Rust side can use `tokio-tungstenite` and pipe each event onto a Zenoh `{prefix}/events` topic.

The cost is a bit more weight in the BepInEx plugin (a managed WebSocket library, e.g. `WebSocketSharp` or `Fleck`, both small and BepInEx-compatible). The damage stream can stay on UDP unchanged; events are a separate channel.

### Sketch of an event payload

```json
{
  "ts_ms": 12345,
  "type": "checkpoint_passed",
  "lap": 1,
  "checkpoint_index": 3,
  "checkpoint_total": 8,
  "elapsed_ms": 8420
}
```

Each event has a `type` discriminator and a `ts_ms` (same monotonic clock as the damage stream so the two can be correlated). Other fields depend on type. A `hello` event on connect could announce supported event types so consumers can negotiate.
