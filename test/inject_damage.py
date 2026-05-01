#!/usr/bin/env python3
"""Inject a synthetic CRSF damage frame (type 0x42) into Zenoh.

Useful for exercising the EdgeTX dmgbw / dmgfig / dmgcol scripts without
having to actually crash a drone in the sim. The frame is published on the
same Zenoh topic that liftoff-input uses (`{prefix}/crsf/telemetry`), so
crsf-forward picks it up and forwards it over the ELRS link to the radio.

Examples:

    # Single shot, full health, 4 rotors
    ./inject_damage.py 100 100 100 100

    # Liftoff quad with LF/RF/LB/RB ordering, RB destroyed
    ./inject_damage.py 100 80 60 0

    # Repeat at 5 Hz so the radio keeps showing the values (otherwise the
    # dmg* scripts will mark the data stale after ~3 s); Ctrl-C to stop.
    ./inject_damage.py --rate 5 100 50 25 0

    # With status flags
    ./inject_damage.py --crashed 0 0 0 0
    ./inject_damage.py --no-drone

    # Talk to a remote Zenoh router instead of using peer discovery
    ./inject_damage.py --zenoh-connect tcp/192.168.1.10:7447 100 100 100 100

Requires the `zenoh` Python package (`pip install zenoh`).
"""
from __future__ import annotations

import argparse
import json
import signal
import sys
import time

try:
    import zenoh
except ImportError:
    print(
        "error: this script requires the `zenoh` Python package.\n"
        "       install with `pip install zenoh`.",
        file=sys.stderr,
    )
    sys.exit(2)

# CRSF wire-format constants -- see telemetry-lib/src/crsf.rs and
# telemetry-lib/src/crsf_custom.rs.
SYNC_FLIGHT_CONTROLLER = 0xC8
DEST_RADIO_TRANSMITTER = 0xEA
ORIG_FLIGHT_CONTROLLER = 0xC8
PACKET_TYPE_DAMAGE = 0x42

FLAG_KILLED = 0x01
FLAG_CRASHED = 0x02
FLAG_NO_DRONE = 0x04

MAX_ROTORS = 8


def crc8_dvb_s2(data: bytes) -> int:
    """CRC-8/DVB-S2: polynomial 0xD5, init 0x00, no reflection, no xor-out.

    Matches the `crc::CRC_8_DVB_S2` parameters used on the Rust side.
    """
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def build_damage_frame(percentages: list[float], flags: int) -> bytes:
    """Build a complete CRSF damage frame (sync..CRC) from per-rotor %.

    `percentages` is a list of 0..MAX_ROTORS floats in [0.0, 100.0]. They
    are clamped and converted to the wire format (u16 BE in [0, 10000]).
    """
    if not 0 < len(percentages) <= MAX_ROTORS:
        raise ValueError(
            f"need 1..{MAX_ROTORS} rotor percentages, got {len(percentages)}"
        )

    health_u16 = bytearray()
    for pct in percentages:
        clamped = max(0.0, min(100.0, pct))
        h = int(round(clamped * 100.0))  # 0..10000
        health_u16 += h.to_bytes(2, "big")

    # Frame layout: [type] [dest] [orig] [flags] [n] [health...]
    # The "length" byte covers everything from type through the CRC.
    payload = bytearray()
    payload.append(PACKET_TYPE_DAMAGE)
    payload.append(DEST_RADIO_TRANSMITTER)
    payload.append(ORIG_FLIGHT_CONTROLLER)
    payload.append(flags & 0xFF)
    payload.append(len(percentages))
    payload += health_u16

    crc = crc8_dvb_s2(bytes(payload))
    length_field = len(payload) + 1  # +1 for the CRC byte

    frame = bytearray()
    frame.append(SYNC_FLIGHT_CONTROLLER)
    frame.append(length_field)
    frame += payload
    frame.append(crc)
    return bytes(frame)


def parse_flags(args: argparse.Namespace) -> int:
    flags = 0
    if args.killed:
        flags |= FLAG_KILLED
    if args.crashed:
        flags |= FLAG_CRASHED
    if args.no_drone:
        flags |= FLAG_NO_DRONE
    return flags


def make_zenoh_config(args: argparse.Namespace) -> zenoh.Config:
    config = zenoh.Config()
    config.insert_json5("mode", json.dumps(args.zenoh_mode))
    if args.zenoh_connect:
        config.insert_json5(
            "connect/endpoints", json.dumps([args.zenoh_connect])
        )
    return config


def main() -> int:
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument(
        "percentages",
        nargs="*",
        type=float,
        help=(
            f"Per-rotor health, 0-100 (1..{MAX_ROTORS} values). For Liftoff: "
            "LF RF LB RB. Omit when using --no-drone with no rotor data."
        ),
    )
    p.add_argument(
        "--zenoh-prefix",
        default="liftoff",
        help="Zenoh topic prefix [default: liftoff]",
    )
    p.add_argument(
        "--zenoh-connect",
        default=None,
        metavar="ENDPOINT",
        help="Zenoh connect endpoint (e.g. tcp/host:7447). Omit for peer discovery.",
    )
    p.add_argument(
        "--zenoh-mode",
        default="peer",
        choices=["peer", "client"],
        help="Zenoh mode [default: peer]",
    )
    p.add_argument(
        "--rate",
        type=float,
        default=None,
        metavar="HZ",
        help="Repeat at HZ Hz until interrupted. Default: send once and exit.",
    )
    p.add_argument(
        "--killed", action="store_true", help="Set the killed flag (bit 0)"
    )
    p.add_argument(
        "--crashed", action="store_true", help="Set the crashed flag (bit 1)"
    )
    p.add_argument(
        "--no-drone",
        action="store_true",
        help="Set the no-drone flag (bit 2). Allows zero rotors.",
    )
    p.add_argument(
        "-v", "--verbose", action="store_true", help="Print each frame in hex"
    )
    args = p.parse_args()

    flags = parse_flags(args)

    # NO_DRONE may legitimately carry zero rotors. Otherwise require at least one.
    if not args.percentages:
        if not args.no_drone:
            p.error("need at least one rotor percentage (or pass --no-drone)")
        rotors: list[float] = []
    else:
        if len(args.percentages) > MAX_ROTORS:
            p.error(f"too many rotors: {len(args.percentages)} (max {MAX_ROTORS})")
        rotors = args.percentages

    # Build the frame once; it never changes during the run.
    if rotors:
        frame = build_damage_frame(rotors, flags)
    else:
        # 0-rotor frame for the no-drone case.
        payload = bytearray(
            [
                PACKET_TYPE_DAMAGE,
                DEST_RADIO_TRANSMITTER,
                ORIG_FLIGHT_CONTROLLER,
                flags & 0xFF,
                0,  # n_rotors
            ]
        )
        crc = crc8_dvb_s2(bytes(payload))
        frame = bytes([SYNC_FLIGHT_CONTROLLER, len(payload) + 1]) + bytes(payload) + bytes([crc])

    topic = f"{args.zenoh_prefix}/crsf/telemetry"

    if args.verbose:
        hex_str = " ".join(f"{b:02X}" for b in frame)
        print(f"frame ({len(frame)} bytes): {hex_str}")
        print(f"topic: {topic}")
        print(f"flags: 0x{flags:02X}, rotors: {rotors}")

    config = make_zenoh_config(args)
    session = zenoh.open(config)
    try:
        publisher = session.declare_publisher(topic)

        # Graceful Ctrl-C handling for the repeat-mode loop.
        stopping = False

        def _stop(signum, _frame):
            nonlocal stopping
            stopping = True

        signal.signal(signal.SIGINT, _stop)
        signal.signal(signal.SIGTERM, _stop)

        if args.rate is None:
            publisher.put(frame)
            print(f"sent 1 frame on {topic}")
        else:
            if args.rate <= 0:
                p.error("--rate must be > 0")
            interval = 1.0 / args.rate
            count = 0
            print(
                f"publishing on {topic} at {args.rate:g} Hz (Ctrl-C to stop)"
            )
            while not stopping:
                publisher.put(frame)
                count += 1
                # Sleep in small slices so Ctrl-C is responsive at low rates.
                deadline = time.monotonic() + interval
                while not stopping and time.monotonic() < deadline:
                    time.sleep(min(0.05, deadline - time.monotonic()))
            print(f"\nsent {count} frame(s) on {topic}")
    finally:
        session.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
