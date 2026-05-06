#!/usr/bin/env python3
"""End-to-end smoke test: drive the simulator over Zenoh.

Procedure:

1. Open a Zenoh peer session.
2. Subscribe to ``sim0/crsf/telemetry`` (CRSF telemetry frames the
   sim is supposed to publish each tick).
3. Spawn ``godot --headless`` (no autofly profile — Zenoh is the
   sole input source).
4. Continuously publish a steady-state 70%-throttle CRSF
   ``RcChannelsPacked`` frame on ``sim0/crsf/rc`` at ~50 Hz for the
   duration of the run.
5. After the run completes, assert:
   - Drone reached at least 5 m altitude (proves RC was received and
     applied through the FC).
   - At least one frame was received on the telemetry topic (proves
     outbound publish works end-to-end).
   - The drone's logs reported ``src=Direct`` at least once (proves
     the input router selected the Zenoh source over the keyboard
     fallback).
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

import zenoh

ROOT = Path(__file__).resolve().parent
GODOT_PROJECT = ROOT / "godot"

ALT_THRESHOLD_M = 5.0
HEADLESS_TIMEOUT_S = 30
AUTOQUIT_S = 5.0
PUBLISH_HZ = 50.0
ARM_PHASE_S = 0.6  # match the autofly stub's settle window

# CRSF wire constants (mirrored from telemetry-lib::crsf).
SYNC_FC = 0xC8
PACKET_TYPE_RC_CHANNELS_PACKED = 0x16
AXIS_MAX = 1983
AXIS_MID = 992

RESET_CHANNEL = 5

def fail(msg: str) -> None:
    print(f"FAIL: {msg}", file=sys.stderr)
    sys.exit(1)


def crc8_dvb_s2(data: bytes) -> int:
    poly = 0xD5
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def pack_channels(channels: list[int]) -> bytes:
    """Pack 16x 11-bit channels into 22 bytes, mirroring the encoder
    in ``telemetry_lib::crsf::pack_channels``."""
    if len(channels) != 16:
        raise ValueError("need 16 channels")
    buf = bytearray(22)
    dest_shift = 0
    ptr = 0
    for ch in channels:
        if ch < 0 or ch > 0x7FF:
            raise ValueError(f"channel {ch} out of range")
        value = ch
        bits_left = 11
        while bits_left > 0:
            space = 8 - dest_shift
            write = min(bits_left, space)
            mask = (1 << write) - 1
            buf[ptr] |= ((value & mask) << dest_shift) & 0xFF
            value >>= write
            bits_left -= write
            dest_shift += write
            if dest_shift == 8:
                dest_shift = 0
                ptr += 1
    return bytes(buf)


def build_rc_frame(channels: list[int]) -> bytes:
    """CRSF frame: sync, length, type, packed channels, CRC."""
    payload = bytes([PACKET_TYPE_RC_CHANNELS_PACKED]) + pack_channels(channels)
    crc = crc8_dvb_s2(payload)
    length = len(payload) + 1  # +1 for CRC byte
    return bytes([SYNC_FC, length]) + payload + bytes([crc])


def aetra_climb_frame(throttle_unit: float = 0.7) -> bytes:
    """Centred sticks except throttle high; AUX1 high (arm); SA-switch
    low so the manual source wins the input mux."""
    chans = [AXIS_MID] * 16
    chans[0] = AXIS_MID  # roll
    chans[1] = AXIS_MID  # pitch
    chans[2] = max(0, min(AXIS_MAX, int(throttle_unit * AXIS_MAX)))
    chans[3] = AXIS_MID  # yaw
    chans[4] = AXIS_MAX  # AUX1 high → arm
    chans[6] = 0  # SA switch low → manual override active
    chans[RESET_CHANNEL] = 0  # reset inactive
    return build_rc_frame(chans)


def aetra_pre_arm_frame() -> bytes:
    """AUX1 high but throttle low — the arming-safety check needs to
    see throttle low at the moment AUX1 transitions high."""
    chans = [AXIS_MID] * 16
    chans[2] = 0  # throttle low
    chans[4] = AXIS_MAX  # AUX1 high
    chans[6] = 0  # SA low
    chans[RESET_CHANNEL] = 0  # reset inactive
    return build_rc_frame(chans)


def aetra_settle_frame() -> bytes:
    """All centred, AUX1 low: drone disarmed."""
    chans = [AXIS_MID] * 16
    chans[2] = 0
    chans[4] = 0
    chans[6] = 0
    chans[RESET_CHANNEL] = 0  # reset inactive
    return build_rc_frame(chans)


def main() -> None:
    if shutil.which("godot") is None:
        fail("godot binary not found on PATH")

    if not (GODOT_PROJECT / ".godot").is_dir():
        fail("godot/.godot/ missing — run `smoke-test.py` first to import the project")
    if not (
        GODOT_PROJECT
        / "addons"
        / "godot-swarm-sim"
        / "libgodot_swarm_sim.so"
    ).exists():
        fail("staged cdylib missing — run ./build.sh debug first")

    # Generate a test-specific drones.json: single drone, no local IO.
    import json
    import tempfile
    test_config = {
        "prefix_base": "sim",
        "drones": [
            {
                "id": 0,
                "spawn": [0.0, 1.0, 0.0],
                "active": False,
                "preset": "res://presets/racing_5inch.tres",
            }
        ],
    }
    fd, drones_path = tempfile.mkstemp(suffix=".json")
    with os.fdopen(fd, "w") as f:
        json.dump(test_config, f)

    try:
        _run_zenoh_test(drones_path)
    finally:
        os.unlink(drones_path)


def _run_zenoh_test(drones_path: str) -> None:
    # Open Zenoh session, declare subscriber + publisher.
    session = zenoh.open(zenoh.Config())
    telemetry_count = 0
    last_payload_len = 0
    sub_lock = threading.Lock()

    def on_telemetry(sample: zenoh.Sample) -> None:
        nonlocal telemetry_count, last_payload_len
        with sub_lock:
            telemetry_count += 1
            last_payload_len = len(sample.payload.to_bytes())

    tel_sub = session.declare_subscriber("sim0/crsf/telemetry", on_telemetry)
    rc_pub = session.declare_publisher("sim0/crsf/rc")

    # Spawn Godot in the background with no autofly — Zenoh is the
    # sole input source for this run.
    env = os.environ | {
        "GSS_AUTOQUIT_S": str(AUTOQUIT_S),
        "GSS_DRONES_JSON": drones_path,
    }
    # Strip any GSS_AUTOFLY the surrounding shell may have set.
    env.pop("GSS_AUTOFLY", None)
    print(f"==> spawning godot --headless (autoquit {AUTOQUIT_S}s)")
    godot_proc = subprocess.Popen(
        ["godot", "--headless", "--path", str(GODOT_PROJECT)],
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )

    # Reader thread: collect Godot's stdout so we can grep it later,
    # and signal when Godot is ready (DroneController has initialised).
    log_lines: list[str] = []
    ready_event = threading.Event()

    def reader() -> None:
        assert godot_proc.stdout is not None
        for line in godot_proc.stdout:
            log_lines.append(line)
            print(line, end="")
            if "DroneController ready" in line:
                ready_event.set()

    rt = threading.Thread(target=reader, daemon=True)
    rt.start()

    # Wait for Godot to boot and the DroneController to be ready
    # before starting the 3-phase RC sequence. Without this, the
    # arming gate never sees the "AUX1 went high while throttle was
    # low" transition because the test is already sending full-throttle
    # frames by the time the first physics tick runs.
    print("==> waiting for DroneController ready...")
    if not ready_event.wait(timeout=15.0):
        godot_proc.terminate()
        fail("timed out waiting for DroneController ready")

    # Drive the RC topic. Three phases:
    #   1. Settle (AUX1 low, throttle low) — ensures disarmed state.
    #   2. Pre-arm (AUX1 high, throttle low) — arming gate sees the
    #      low→high AUX1 transition while throttle is safe.
    #   3. Climb (AUX1 high, throttle 70%) — fly.
    publish_started = time.monotonic()

    settle = aetra_settle_frame()
    pre_arm = aetra_pre_arm_frame()
    climb = aetra_climb_frame(0.7)

    period = 1.0 / PUBLISH_HZ
    deadline = time.monotonic() + HEADLESS_TIMEOUT_S
    try:
        while True:
            now = time.monotonic()
            if godot_proc.poll() is not None:
                break
            if now > deadline:
                godot_proc.terminate()
                break

            elapsed = now - publish_started
            if elapsed < 0.3:
                rc_pub.put(settle)
            elif elapsed < ARM_PHASE_S:
                rc_pub.put(pre_arm)
            else:
                rc_pub.put(climb)

            time.sleep(period)
    finally:
        try:
            godot_proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            godot_proc.kill()
        rt.join(timeout=2.0)
        session.close()

    log = "".join(log_lines)

    # Assertions.
    if "Initialize godot-rust" not in log:
        fail("gdext extension did not load")

    matches = re.findall(r"alt=([0-9.]+)m", log)
    if not matches:
        fail("no altitude line in godot output")
    max_alt = max(float(m) for m in matches)
    if max_alt < ALT_THRESHOLD_M:
        fail(
            f"drone reached only {max_alt:.2f} m "
            f"(expected ≥ {ALT_THRESHOLD_M:.1f} m), Zenoh RC may not have been applied"
        )

    if "src=Direct" not in log:
        fail("input router never selected the direct source — check topic naming and freshness")

    with sub_lock:
        if telemetry_count == 0:
            fail("no CRSF telemetry frames received on sim0/crsf/telemetry")
        print(
            f"received {telemetry_count} telemetry frame(s); last payload {last_payload_len} bytes"
        )

    print(f"PASS: drone climbed to {max_alt:.2f} m via Zenoh RC, telemetry flowing")


if __name__ == "__main__":
    main()
