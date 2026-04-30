# Systemd user services

liftoff-rs ships unit files in `systemd/` for running components as systemd user services. This keeps things out of the system scope -- no root required, and services run as your user.

## Install

Build and install the binaries, then link the unit files:

```bash
for crate in liftoff-{input,forward,autopilot,gpsd,mavlink-bridge}; do
  cargo install --path "$crate"
done

# Optional: zenoh router (only needed for remote connections)
cargo install zenohd

# Install unit files
mkdir -p ~/.config/systemd/user
cp systemd/*.{service,target} ~/.config/systemd/user/
systemctl --user daemon-reload

# Enable services (auto-starts on login, visible in systemctl list-units)
systemctl --user enable liftoff-{input,forward,autopilot,gpsd,mavlink-bridge}
```

## Usage

Start/stop individual services:

```bash
systemctl --user start liftoff-input
systemctl --user stop autopilot
systemctl --user status crsf-gpsd
journalctl --user -u liftoff-input -f    # follow logs
```

Start/stop all services at once via the target:

```bash
systemctl --user start liftoff.target
systemctl --user stop liftoff.target
```

If you need services to run without an active login session:

```bash
loginctl enable-linger $USER
```

## Configuration

All units read an optional environment file at `~/.config/liftoff/env`. Use this to set CLI flags and environment variables without editing the unit files. Each unit expands a `LIFTOFF_*_ARGS` variable, so you can add flags in the env file without touching the units:

```bash
mkdir -p ~/.config/liftoff
cat > ~/.config/liftoff/env << 'EOF'
RUST_LOG=info
LIFTOFF_FORWARD_ARGS=--port /dev/ttyACM0 --zenoh-connect udp/10.0.0.5:7447
LIFTOFF_AUTOPILOT_ARGS=--zenoh-connect udp/10.0.0.5:7447
LIFTOFF_GPSD_ARGS=--zenoh-connect udp/10.0.0.5:7447
LIFTOFF_MAVLINK_BRIDGE_ARGS=--zenoh-connect udp/10.0.0.5:7447
EOF
```

Variable names: `LIFTOFF_INPUT_ARGS`, `LIFTOFF_FORWARD_ARGS`, `LIFTOFF_AUTOPILOT_ARGS`, `LIFTOFF_GPSD_ARGS`, `LIFTOFF_MAVLINK_BRIDGE_ARGS`.
