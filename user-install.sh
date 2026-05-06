#!/bin/bash
set -e

# Install liftoff-rs crates
for crate in liftoff-input velocidrone-input uncrashed-input crsf-{forward,gpsd,joystick} autopilot mavlink-bridge; do
  cargo install --path "$crate"
done

# Optional: zenoh router (only needed for remote connections)
cargo install zenohd

# Install unit files
mkdir -p ~/.config/systemd/user
cp systemd/*.{service,target} ~/.config/systemd/user/
systemctl --user daemon-reload

# Create env file (if necessary)
mkdir -p ~/.config/liftoff
ENV_FILE="$HOME/.config/liftoff/env"

if [ ! -f ~/.config/liftoff/env ]; then
cat > "$ENV_FILE" << 'EOF'
RUST_LOG=info
DRONESIM_VELOCIDRONE_INPUT_ARGS=--zenoh-connect udp/10.0.0.5:7447 --ws-url ws://127.0.0.1:9001
DRONESIM_UNCRASHED_INPUT_ARGS=--zenoh-connect udp/10.0.0.5:7447 --input-file /tmp/uncrashed-telemetry.bin
DRONESIM_CRSF_FORWARD_ARGS=--zenoh-connect udp/10.0.0.5:7447 -p /dev/ttyACM0
DRONESIM_CRSF_JOYSTICK_ARGS=--zenoh-connect udp/10.0.0.5:7447
DRONESIM_AUTOPILOT_ARGS=--zenoh-connect udp/10.0.0.5:7447
DRONESIM_CRSF_GPSD_ARGS=--zenoh-connect udp/10.0.0.5:7447
DRONESIM_MAVLINK_BRIDGE_ARGS=--zenoh-connect udp/10.0.0.5:7447
EOF
fi

echo "# Installation successful"
echo
echo "You can edit the configuration in $ENV_FILE."
echo
echo "To auto-starts services on login, do:"
echo "systemctl --user enable dronesim-{liftoff-input,velocidrone-input,uncrashed-input,crsf-forward,crsf-gpsd,crsf-joystick,autopilot,mavlink-bridge}"
