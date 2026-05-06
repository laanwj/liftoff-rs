# Systemd user services

liftoff-rs ships unit files in `systemd/` for running components as systemd user services. This keeps things out of the system scope -- no root required, and services run as your user.

## Install

Build and install the binaries, then link the unit files:

```bash
./user-install.sh
```

## Usage

Start/stop individual services:

```bash
systemctl --user start dronesim-liftoff-input
systemctl --user stop dronesim-autopilot
systemctl --user status dronesim-crsf-gpsd
journalctl --user -u dronesim-liftoff-input -f    # follow logs
```

Start/stop all services at once via the target:

```bash
systemctl --user start dronesim.target
systemctl --user stop dronesim.target
```

If you need services to run without an active login session:

```bash
loginctl enable-linger $USER
```

## Configuration

All units read an optional environment file at `~/.config/liftoff/env`. Use this to set CLI flags and environment variables without editing the unit files. Each unit expands a `DRONESIM_*_ARGS` variable, so you can add flags in the env file without touching the units:

```bash
mkdir -p ~/.config/liftoff
cat > ~/.config/liftoff/env << 'EOF'
RUST_LOG=info
DRONESIM_VELOCIDRONE_INPUT_ARGS=--zenoh-connect udp/10.0.0.5:7447 --ws-url ws://127.0.0.1:9001
DRONESIM_UNCRASHED_INPUT_ARGS=--zenoh-connect udp/10.0.0.5:7447 --input-file /tmp/uncrashed-telemetry.bin
DRONESIM_CRSF_FORWARD_ARGS=--zenoh-connect udp/10.0.0.5:7447 -p /dev/ttyACM0
DRONESIM_CRSF_JOYSTICK_ARGS=--zenoh-connect udp/10.0.0.5:7447
DRONESIM_AUTOPILOT_ARGS=--zenoh-connect udp/10.0.0.5:7447
DRONESIM_CRSF_GPSD_ARGS=--zenoh-connect udp/10.0.0.5:7447
DRONESIM_MAVLINK_BRIDGE_ARGS=--zenoh-connect udp/10.0.0.5:7447
EOF
```

Variable names: `DRONESIM_LIFTOFF_INPUT_ARGS`, `DRONESIM_VELOCIDRONE_INPUT_ARGS`, `DRONESIM_UNCRASHED_INPUT_ARGS`, `DRONESIM_CRSF_FORWARD_ARGS`, `DRONESIM_CRSF_GPSD_ARGS`, `DRONESIM_CRSF_JOYSTICK_ARGS`, `DRONESIM_AUTOPILOT_ARGS`, `DRONESIM_MAVLINK_BRIDGE_ARGS`.
