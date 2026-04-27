#!/usr/bin/env python3
"""Test client for the liftoff-simstate-bridge UDP stream.

Listens on the configured port, decodes both packet kinds (LFDM = damage,
LFBT = battery), and prints a one-line summary per state change. Heartbeats
that re-confirm the same state are suppressed; one ping every 5 s confirms
the stream is alive.

Usage:
    ./test_client.py                # listen on 0.0.0.0:9020
    ./test_client.py --port 9020
    ./test_client.py --bind 127.0.0.1 --port 9020 --raw
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import time

DAMAGE_TAG = b"LFDM"
BATTERY_TAG = b"LFBT"

DAMAGE_HEADER_FMT = "<4sHHQBBBB"
DAMAGE_HEADER_SIZE = struct.calcsize(DAMAGE_HEADER_FMT)
assert DAMAGE_HEADER_SIZE == 20

BATTERY_FMT = "<4sHHQBBBB5f"
BATTERY_SIZE = struct.calcsize(BATTERY_FMT)
assert BATTERY_SIZE == 40

DAMAGE_FLAG_KILLED = 0x0001
DAMAGE_FLAG_CRASHED = 0x0002
DAMAGE_FLAG_NO_DRONE = 0x0004

BATTERY_FLAG_NO_DRAINER = 0x0001
BATTERY_FLAG_NO_DRONE = 0x0002


def decode_damage_flags(flags: int) -> str:
    names = []
    if flags & DAMAGE_FLAG_KILLED:
        names.append("KILLED")
    if flags & DAMAGE_FLAG_CRASHED:
        names.append("CRASHED")
    if flags & DAMAGE_FLAG_NO_DRONE:
        names.append("NO_DRONE")
    extra = flags & ~(DAMAGE_FLAG_KILLED | DAMAGE_FLAG_CRASHED | DAMAGE_FLAG_NO_DRONE)
    if extra:
        names.append(f"unknown=0x{extra:04x}")
    return ",".join(names) if names else "-"


def decode_battery_flags(flags: int) -> str:
    names = []
    if flags & BATTERY_FLAG_NO_DRAINER:
        names.append("NO_DRAINER")
    if flags & BATTERY_FLAG_NO_DRONE:
        names.append("NO_DRONE")
    extra = flags & ~(BATTERY_FLAG_NO_DRAINER | BATTERY_FLAG_NO_DRONE)
    if extra:
        names.append(f"unknown=0x{extra:04x}")
    return ",".join(names) if names else "-"


def parse(packet: bytes):
    if len(packet) < 4:
        raise ValueError(f"runt packet ({len(packet)} bytes)")
    tag = packet[:4]
    if tag == DAMAGE_TAG:
        return parse_damage(packet)
    if tag == BATTERY_TAG:
        return parse_battery(packet)
    raise ValueError(f"unknown tag {tag!r}")


def parse_damage(packet: bytes):
    if len(packet) < DAMAGE_HEADER_SIZE:
        raise ValueError(f"runt damage packet ({len(packet)} bytes)")
    tag, version, flags, ts_ms, prop_count, p1, p2, p3 = struct.unpack_from(
        DAMAGE_HEADER_FMT, packet, 0
    )
    expected = DAMAGE_HEADER_SIZE + 4 * prop_count
    if len(packet) != expected:
        raise ValueError(
            f"damage length mismatch: got {len(packet)}, expected {expected} for prop_count={prop_count}"
        )
    damage = list(struct.unpack_from(f"<{prop_count}f", packet, DAMAGE_HEADER_SIZE))
    return {
        "kind": "damage",
        "version": version,
        "flags": flags,
        "ts_ms": ts_ms,
        "prop_count": prop_count,
        "damage": damage,
    }


def parse_battery(packet: bytes):
    if len(packet) != BATTERY_SIZE:
        raise ValueError(f"battery length mismatch: got {len(packet)}, expected {BATTERY_SIZE}")
    (
        tag, version, flags, ts_ms,
        cell_count, p1, p2, p3,
        voltage, voltage_per_cell, amps, ah_drawn, percentage,
    ) = struct.unpack(BATTERY_FMT, packet)
    return {
        "kind": "battery",
        "version": version,
        "flags": flags,
        "ts_ms": ts_ms,
        "cell_count": cell_count,
        "voltage": voltage,
        "voltage_per_cell": voltage_per_cell,
        "amps": amps,
        "ah_drawn": ah_drawn,
        "percentage": percentage,
    }


def fmt_damage(values: list[float]) -> str:
    return "[" + ", ".join(f"{v:7.3f}" for v in values) + "]"


def damage_key(msg) -> tuple:
    return (msg["flags"], msg["prop_count"], tuple(msg["damage"]))


def battery_key(msg) -> tuple:
    # Quantize floats so tiny per-frame ripple doesn't trigger a print every packet.
    def q(x: float) -> int:
        return int(round(x * 100))
    return (
        msg["flags"], msg["cell_count"],
        q(msg["voltage"]), q(msg["voltage_per_cell"]),
        q(msg["amps"]), q(msg["ah_drawn"] * 1000),  # quantize Ah at 0.01 mAh resolution
        q(msg["percentage"] * 100),
    )


def fmt_damage_line(msg, uptime_s: float) -> str:
    return (
        f"t+{uptime_s:7.2f}s  DMG  flags={decode_damage_flags(msg['flags']):<20} "
        f"props={msg['prop_count']}  damage={fmt_damage(msg['damage'])}"
    )


def fmt_battery_line(msg, uptime_s: float) -> str:
    return (
        f"t+{uptime_s:7.2f}s  BAT  flags={decode_battery_flags(msg['flags']):<14} "
        f"cells={msg['cell_count']}  V={msg['voltage']:6.2f} (per_cell={msg['voltage_per_cell']:5.3f})  "
        f"A={msg['amps']:6.2f}  drawn={msg['ah_drawn']*1000:7.1f} mAh  pct={msg['percentage']*100:5.1f}%"
    )


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    ap.add_argument("--bind", default="0.0.0.0", help="interface to bind (default 0.0.0.0)")
    ap.add_argument("--port", type=int, default=9020, help="UDP port (default 9020)")
    ap.add_argument("--raw", action="store_true", help="print every packet, not just state changes")
    args = ap.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.bind, args.port))
    print(f"listening on {args.bind}:{args.port} (Ctrl-C to stop)")

    last_keys: dict[str, tuple] = {}
    last_print_ts = 0.0
    pkt_count = 0
    counts = {"damage": 0, "battery": 0}
    # The plugin's ts_ms is monotonic from plugin load, so it resets when the
    # game restarts. We rebase on the first packet AND on any large backwards
    # jump (game restart while the test client is still running).
    plugin_baseline_ms: int | None = None
    last_ts_ms: int = 0
    REBASE_BACKWARD_THRESHOLD_MS = 5_000

    try:
        while True:
            packet, addr = sock.recvfrom(2048)
            pkt_count += 1
            try:
                msg = parse(packet)
            except ValueError as e:
                print(f"[{addr[0]}:{addr[1]}] parse error: {e}  raw={packet.hex()}")
                continue

            kind = msg["kind"]
            counts[kind] += 1
            ts_ms = msg["ts_ms"]

            if plugin_baseline_ms is None:
                plugin_baseline_ms = ts_ms
                print(f"first packet from {addr[0]}:{addr[1]} — kind={kind} version={msg['version']}")
            elif ts_ms + REBASE_BACKWARD_THRESHOLD_MS < last_ts_ms:
                # Plugin clock went backwards by a meaningful amount → game
                # restarted. Rebase so uptime doesn't show negative.
                print(f"plugin clock reset detected (was {last_ts_ms} ms, now {ts_ms} ms) — rebasing.")
                plugin_baseline_ms = ts_ms
                last_keys.clear()
            last_ts_ms = ts_ms

            if kind == "damage":
                key = damage_key(msg)
            else:
                key = battery_key(msg)

            uptime_s = (ts_ms - (plugin_baseline_ms or 0)) / 1000.0

            if not args.raw and last_keys.get(kind) == key:
                # heartbeat — print a summary every 5 s so the user knows it's alive
                now = time.monotonic()
                if now - last_print_ts >= 5.0:
                    print(f"  [hb] dmg={counts['damage']} bat={counts['battery']} pkts (no state change)")
                    last_print_ts = now
                continue

            last_keys[kind] = key
            last_print_ts = time.monotonic()
            print(fmt_damage_line(msg, uptime_s) if kind == "damage" else fmt_battery_line(msg, uptime_s))
    except KeyboardInterrupt:
        print(f"\nstopped after {pkt_count} packets ({counts['damage']} damage, {counts['battery']} battery)")
        return 0


if __name__ == "__main__":
    sys.exit(main())
