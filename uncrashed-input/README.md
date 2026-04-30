# uncrashed-input

File-IPC → Zenoh bridge for the [Uncrashed: FPV Drone Simulator](https://store.steampowered.com/app/1682970/Uncrashed__FPV_Drone_Simulator/)
UE4SS Lua telemetry mod (`../uncrashed-telemetry-mod/`).

Uncrashed has no built-in telemetry interface — its developer confirmed
this in [a Steam forum thread](https://steamcommunity.com/app/1682970/discussions/0/4635986990492354275/).
We work around the gap with a small UE4SS Lua mod that overwrites a
fixed-size binary file (default `/tmp/uncrashed-telemetry.bin`,
`wire::PACKET_SIZE` bytes) with the latest `UCFV` packet on every game
tick. This crate polls that file and publishes one stream onto Zenoh:

- `{prefix}/crsf/telemetry` — CRSF binary frames (matches `liftoff-input`,
  `velocidrone-input`), built via `telemetry_lib::crsf_tx::generate_crsf_telemetry`
  plus `telemetry_lib::crsf_custom::build_damage_packet` for the per-prop
  damage 0x42 frame.

## Running

```sh
RUST_LOG=info cargo run -p uncrashed-input -- --zenoh-mode peer
```

Defaults: poll `/tmp/uncrashed-telemetry.bin` every 16 ms, publish on
`liftoff/crsf/telemetry`. Override with `--input-file`, `--poll-ms`,
`--zenoh-prefix`.

## Wire format

`UCFV` v1 — see `src/wire.rs` for the full table. `PACKET_SIZE`-byte
layout: `HEADER_SIZE`-byte header followed by `MAX_PROPS × float`
damage slots (1.0 healthy → 0.0 broken, matching `LFDM` semantics).
The receiver exposes the leading `prop_count` slots when `HAS_DAMAGE`
is set; the rest of the damage block is garbage.

## Steady-state I/O

Both sides hold the file handle open for the lifetime of the process:

- The mod opens once with `"wb"` (truncate-once-at-startup), calls
  `setvbuf("no")` to disable buffering, then per tick does
  `seek(0)` + `write(PACKET_SIZE)`. One syscall per tick. The file
  size stabilises at `PACKET_SIZE` from the first write onwards and
  never changes.
- The receiver opens lazily on the first poll that finds the file,
  then per poll does `seek(0)` + `read(PACKET_SIZE)` against that
  handle. One syscall per poll.

The receiver short-read check (skip if `read < PACKET_SIZE`) only
matters if the receiver opens the file before the mod's `init()` has
finished its first write — the brief startup window where the file
exists but is still 0 bytes long. Once both sides are running, every
read is a full envelope.

## Sequence dedup

The receiver tracks the last-published `sequence` field. When polling
faster than the mod ticks (or when the mod is paused on a menu), the
file content is unchanged across reads; we'd otherwise spam Zenoh with
duplicate packets. Skipping unchanged sequences keeps downstream
consumers idle when nothing's actually happening.

## Coordinate conversions

UE4 ships with X-forward, Y-right, Z-up cm. The `into_telemetry()`
conversion in `wire.rs`:

- divides position and velocity by 100 (cm → m),
- maps UE4's vertical axis into liftoff's altitude slot
  (`TelemetryPacket::position[1]`, per `telemetry_lib::geo::coord_from_gps`),
- passes the attitude quaternion through unchanged.

The quaternion handedness vs. liftoff/velocidrone is **empirical** — the
first flight test will tell us whether it needs a sign-flip on one axis.
If the dashboard horizon banks the wrong way, that's the calibration to
chase.

## Testing without the game

`cargo test -p uncrashed-input` exercises the wire parser end-to-end,
including the `to_damage_packet` conversion that feeds the CRSF custom
0x42 frame builder.
