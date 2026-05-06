#!/usr/bin/env python3
"""End-to-end smoke test for godot-swarm-sim.

1. cargo test (pure-Rust unit tests for math/sim modules)
2. cargo build (cdylib for godot)
3. godot --headless autofly run (extension loads, drone arms and climbs)
   Asserts the drone reaches a minimum altitude in the available time.

Exit non-zero on any failure; intended for CI.
"""

from __future__ import annotations

import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT.parent
GODOT_PROJECT = ROOT / "godot"

# Drone spawns at y=1.0 m; climb autofly should comfortably exceed this.
ALT_THRESHOLD_M = 5.0
HEADLESS_TIMEOUT_S = 30
AUTOQUIT_S = 4.0


def fail(msg: str) -> "subprocess.CompletedProcess[str] | None":
    print(f"FAIL: {msg}", file=sys.stderr)
    sys.exit(1)


def run(cmd: list[str], **kwargs) -> "subprocess.CompletedProcess[str]":
    """Run a command, streaming stdout/stderr through. Raise on nonzero."""
    print("$", " ".join(cmd))
    return subprocess.run(cmd, check=True, text=True, **kwargs)


def step_unit_tests() -> None:
    print("==> [1/3] cargo test -p godot-swarm-sim")
    run(
        [
            "cargo",
            "test",
            "-p",
            "godot-swarm-sim",
            "--manifest-path",
            str(WORKSPACE / "Cargo.toml"),
            "--quiet",
        ]
    )


def step_build() -> None:
    print("==> [2/3] cargo build (debug) + stage cdylib")
    run([str(ROOT / "build.sh"), "debug"])


def step_first_import_if_needed() -> None:
    if (GODOT_PROJECT / ".godot").is_dir():
        return
    print("==> first-time godot --import")
    # Godot 4.6.2 segfaults on shutdown after a headless --import even
    # when the import succeeds. Tolerate any nonzero exit.
    try:
        subprocess.run(
            ["godot", "--headless", "--path", str(GODOT_PROJECT), "--import"],
            timeout=60,
        )
    except subprocess.TimeoutExpired:
        # Even import timeout is OK if .godot/ ended up populated.
        pass
    if not (GODOT_PROJECT / ".godot").is_dir():
        fail("`godot --import` did not produce .godot/ cache")


def step_headless_autofly() -> str:
    print(f"==> [3/3] godot --headless autofly=climb ({AUTOQUIT_S} s)")
    # Generate a test-specific drones.json with active=true for drone 0.
    import tempfile
    import json
    test_config = {
        "prefix_base": "sim",
        "drones": [
            {
                "id": 0,
                "spawn": [0.0, 1.0, 0.0],
                "active": True,
                "preset": "res://presets/racing_5inch.tres",
            }
        ],
    }
    fd, drones_path = tempfile.mkstemp(suffix=".json")
    try:
        with os.fdopen(fd, "w") as f:
            json.dump(test_config, f)

        env = os.environ | {
            "GSS_AUTOQUIT_S": str(AUTOQUIT_S),
            "GSS_AUTOFLY": "climb",
            "GSS_DRONES_JSON": drones_path,
        }
        proc = subprocess.run(
            ["godot", "--headless", "--path", str(GODOT_PROJECT)],
            env=env,
            text=True,
            capture_output=True,
            timeout=HEADLESS_TIMEOUT_S,
        )
        log = proc.stdout + proc.stderr
        print(log)
        return log
    finally:
        os.unlink(drones_path)


def assert_altitude(log: str) -> None:
    matches = re.findall(r"alt=([0-9.]+)m", log)
    if not matches:
        fail("no altitude line found in headless run")
    max_alt = max(float(m) for m in matches)
    if max_alt < ALT_THRESHOLD_M:
        fail(f"drone only reached {max_alt:.2f} m (expected >= {ALT_THRESHOLD_M:.1f} m)")
    print(f"PASS: drone climbed to {max_alt:.2f} m (>= {ALT_THRESHOLD_M:.1f} m threshold)")


def assert_armed(log: str) -> None:
    if "armed=true" not in log:
        fail("controller never armed")


def assert_extension_loaded(log: str) -> None:
    if "Initialize godot-rust" not in log:
        fail("gdext extension did not load")


def main() -> None:
    if shutil.which("godot") is None:
        fail("godot binary not found on PATH")
    if shutil.which("cargo") is None:
        fail("cargo binary not found on PATH")

    step_unit_tests()
    step_build()
    step_first_import_if_needed()
    log = step_headless_autofly()

    assert_extension_loaded(log)
    assert_armed(log)
    assert_altitude(log)

    print("==> smoke test green")


if __name__ == "__main__":
    main()
