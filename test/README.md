# Test helpers

Small standalone utilities for exercising parts of the pipeline without
having to run the full sim + receiver + radio stack.

## `inject_damage.py`

Publishes a synthetic CRSF damage frame (type `0x42`) onto the same Zenoh
topic (`{prefix}/crsf/telemetry`) that `liftoff-input` uses, so anything
downstream — `crsf-forward`, `telemetry-dashboard`, the EdgeTX `dmg*` Lua
scripts on the radio — sees it as a regular frame from the sim. Useful
when iterating on damage rendering without actually crashing a drone.

```
# Single shot, full health
./inject_damage.py 100 100 100 100

# Liftoff quad, RB rotor destroyed (LF, RF, LB, RB ordering)
./inject_damage.py 100 80 60 0

# Repeat at 5 Hz so consumers keep seeing fresh data (e.g. the EdgeTX
# scripts mark data stale after ~3 s); Ctrl-C to stop
./inject_damage.py --rate 5 100 50 25 0

# Status flags
./inject_damage.py --crashed 0 0 0 0
./inject_damage.py --no-drone

# Talk to a remote Zenoh router instead of using peer discovery
./inject_damage.py --zenoh-connect tcp/192.168.1.10:7447 100 100 100 100
```

Requires `pip install zenoh`. See `./inject_damage.py --help` for the
full set of options.

The frame layout and CRC algorithm are documented in
[`telemetry-lib/src/crsf_custom.rs`](../telemetry-lib/src/crsf_custom.rs)
and [`telemetry-lib/src/crsf.rs`](../telemetry-lib/src/crsf.rs); the
Python implementation is verified against the Rust round-trip test
vector and the standard CRC-8/DVB-S2 check value.
