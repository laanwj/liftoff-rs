# uncrashed-telemetry-mod

A [UE4SS](https://github.com/UE4SS-RE/RE-UE4SS) **Lua** mod for [Uncrashed:
FPV Drone Simulator](https://store.steampowered.com/app/1682970/Uncrashed__FPV_Drone_Simulator/)
that emits per-tick drone telemetry as a fixed-size binary file. Pairs
with the `uncrashed-input` Rust crate in this workspace, which polls the
file and republishes the stream onto Zenoh.

## What it does

On every UE4 game tick (~60 Hz):

- Locates the player drone (`BP_FPVDrone_Pawn_C`) and player controller
  (`BP_MyController_C`) via UE4SS's `FindFirstOf`.
- Reads transform + physics velocity + angular velocity from the pawn's
  root component.
- Reads raw stick axes (`ThrottleAxis_Raw` etc.) from the controller.
- Reads `PropsDamage` `TArray<float>`, `Armed` bool, `Battery` sub-component
  voltage from the pawn.
- Packs a [`UCFV` v1](#wire-format) packet, pads to `UCFV_PACKET_SIZE`
  bytes, and rewrites `Z:\tmp\uncrashed-telemetry.bin` in place (Wine
  maps `Z:` to the Linux root).

The file handle is opened once at mod load with `setvbuf("no")` and
stays open for the mod's lifetime; per-tick I/O is just `seek(0)` +
`write(UCFV_PACKET_SIZE)`. The file size is constant, so the receiver
never observes a partially-written frame except in the brief window
before the very first write completes.

Class names + property FNames were locked down by Phase 1 pak recon — see
`uncrashed.md` in the project memory for the full table.

## Why file IPC and not UDP?

Lua 5.4's stdlib has `io.*` but no networking. Sending UDP from Lua needs
LuaSocket, a native C module that has to be built/bundled per Lua-version
+ platform combo. File IPC sidesteps that — every Lua install can write
to a file, and Wine maps the Linux root filesystem to drive `Z:`, so a
Win-side `io.open("Z:\\tmp\\...")` writes to the host's `/tmp/...` which
the receiver reads natively on Linux. No bundled binaries; no LuaSocket
ABI matching; one fewer install step.

The file is a fixed-size envelope (`UCFV_PACKET_SIZE` bytes) overwritten
in place every tick, so the on-disk footprint is bounded and the
receiver doesn't need to handle rotation or stream framing.

## Install

UE4SS ships a prebuilt runtime; no compilation needed.

1. **Install UE4SS v3.0.1.** Download
   [`UE4SS_v3.0.1.zip`](https://github.com/UE4SS-RE/RE-UE4SS/releases/tag/v3.0.1)
   and extract its **contents** into
   `<game>/Uncrashed/Binaries/Win64/`. Confirm `UE4SS.dll`,
   `dwmapi.dll`, and `Mods/` sit next to `Uncrashed-Win64-Shipping.exe`.

2. **Set the DLL override.** In Steam, right-click *Uncrashed → Properties
   → Launch options*:

   ```
   WINEDLLOVERRIDES="dwmapi=n,b" %command%
   ```

3. **Drop in this mod.** Copy this directory to
   `<game>/Uncrashed/Binaries/Win64/Mods/UncrashedTelemetry/`. Result:

   ```
   Mods/
     UncrashedTelemetry/
       scripts/
         main.lua
   ```

4. **Enable the mod.** Insert this line into
   `<game>/Uncrashed/Binaries/Win64/Mods/mods.txt` *before* the
   `; Built-in keybinds` comment:

   ```
   UncrashedTelemetry : 1
   ```

5. **Start the receiver:**

   ```sh
   cargo run -p uncrashed-input -- --zenoh-mode peer
   ```

   Default IPC path is `/tmp/uncrashed-telemetry.bin`; override with
   `--input-file` if needed. Default poll rate matches the mod's 60 Hz
   tick (16 ms); override with `--poll-ms`.

6. Launch the game **offline**. Anti-cheat is not a goal here — running
   this during multiplayer is at your own risk.

7. Sanity-check telemetry is flowing:

   ```sh
   # File size matches the spec's UCFV_PACKET_SIZE (see Wire format below)
   stat -c '%n %s' /tmp/uncrashed-telemetry.bin
   xxd -l 16 /tmp/uncrashed-telemetry.bin   # first 4 bytes = "UCFV"
   ```

## Wire format

`UCFV` v1 — single source of truth: this section. Mirrors live in
`scripts/main.lua` (mod side) and `uncrashed-input/src/wire.rs` (receiver).

Sizes are derived; the canonical computation is:

```
UCFV_HEADER_SIZE = 100
UCFV_MAX_PROPS   = 8
UCFV_PACKET_SIZE = UCFV_HEADER_SIZE + 4 * UCFV_MAX_PROPS
```

Adding header fields or changing `UCFV_MAX_PROPS` reflows the size and
the spec below automatically.

```
UCFV v1, little-endian, UCFV_PACKET_SIZE bytes total

byte 0..4    "UCFV"
byte 4..6    u16 version = 1
byte 6..8    u16 flags  (ON_GROUND, ARMED, HAS_BATTERY, HAS_DAMAGE, CRASHED)
byte 8..16   u64 timestamp_us  (os.clock based, monotonic since mod load)
byte 16..20  u32 sequence
byte 20..32  3×f32 position cm   (UE: X fwd, Y right, Z up)
byte 32..48  4×f32 attitude quat (UE quaternion, X Y Z W)
byte 48..60  3×f32 velocity cm/s (same axes as position)
byte 60..72  3×f32 gyro deg/s    (pitch, roll, yaw)
byte 72..88  4×f32 inputs        (throttle, yaw, pitch, roll)
byte 88..96  2×f32 battery       (percent_0_to_1, voltage_V)
byte 96      u8  prop_count       (≤ UCFV_MAX_PROPS)
byte 97..100 u8[3] pad
byte 100..(100 + 4 * UCFV_MAX_PROPS)
             UCFV_MAX_PROPS × f32 damage   (Uncrashed convention: 0.0 healthy → 1.0 destroyed)
```

The wire carries Uncrashed's `PropsDamage` values exactly as the game
exposes them, in `Props_1..Props_N` order. The receiver inverts to
liftoff/CRSF health convention (1.0 healthy) and applies a `PROP_ORDER`
permutation to map Uncrashed's prop numbering onto the
`[LF, RF, LB, RB]` order the dashboard expects.

Receivers expose the leading `prop_count` damage floats when
`HAS_DAMAGE` is set; the remaining slots, or all of them when the
flag is clear, are garbage.

`HAS_DAMAGE` means *"damage info is present in this packet"*, not *"the
drone is damaged"* — the values inside can range from all 1.0 (healthy)
to 0.0 (broken). When the bit is clear the receiver leaves the damage
slots in the CRSF stream unset.

## Calibration notes

These are best-effort settings the first flight test will refine:

- **Quaternion handedness**: UE4 is left-handed; if the dashboard horizon
  banks the wrong way, sign-flip one component in
  `uncrashed-input/src/wire.rs::into_telemetry`.
- **Gyro axis order**: `main.lua` reorders UE's (roll=X, pitch=Y, yaw=Z)
  to liftoff's (pitch, roll, yaw). If the rates look swapped, fix here.
- **PropsDamage indexing**: `main.lua` reads `dmg[i]` 1-indexed Lua-style.
  UE4SS's TArray Lua wrapper sometimes prefers `:get(i)` 0-indexed. If
  every prop shows the same value, switch indexing.
- **Battery percentage**: not currently populated — `voltageDrop` and
  cell-count math TBD.
