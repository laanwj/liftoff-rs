# Example liftoff-rs setup

## Architecture

liftoff-rs uses [Zenoh](https://zenoh.io) pub/sub for all inter-component communication. Components discover each other via Zenoh topics, with no hardcoded ports between them.

```
LOCAL                                    CLOUD
                                         Liftoff Sim
                                           | UDP :9001
                                         liftoff-input (bridge + joystick + mux)
                                           | liftoff/telemetry, liftoff/crsf/telemetry
liftoff-forward  <------- zenohd -------> (all topics)
  | serial                                 | liftoff/crsf/rc
Radio TX                                 liftoff-input -> uinput -> Sim
```

### Zenoh topics

| Topic                       | Direction         | Content                    |
|-----------------------------|-------------------|----------------------------|
| `liftoff/telemetry`         | input (internal)       | Raw sim telemetry bytes    |
| `liftoff/crsf/telemetry`   | input -> forward, autopilot, gpsd | Individual CRSF frames |
| `liftoff/crsf/rc`          | forward/autopilot -> input | CRSF RC channel frames |

### Components

| Binary              | Where    | Role                                              |
|---------------------|----------|----------------------------------------------------|
| `liftoff-input`     | Cloud    | Sim UDP bridge + CRSF joystick (uinput) + RC mux  |
| `liftoff-forward`   | Local    | Serial port (radio TX) <-> Zenoh                  |
| `liftoff-autopilot` | Local    | Autonomous flight controller                       |
| `liftoff-gpsd`      | Either   | NMEA GPS server from CRSF telemetry                |

## Cloud server setup

The cloud server runs the Liftoff simulator and needs a Zenoh router for remote clients to connect to.

### 1. Install zenohd

```bash
cargo install zenohd
```

### 2. Start services

```bash
# Zenoh router (UDP, listens for remote connections)
zenohd --listen udp/0.0.0.0:7447

# Input (sim UDP bridge + virtual joystick for sim)
liftoff-input
```

All cloud-side binaries find `zenohd` automatically via local multicast scouting -- no connection flags needed.

Make sure **UDP port 7447** is open on the cloud server's firewall.

## Local machine setup

Local binaries connect to the cloud server's Zenoh router via `--zenoh-connect`.

```bash
# Serial bridge (radio TX <-> Zenoh)
liftoff-forward --port /dev/ttyUSB0 --zenoh-connect udp/<CLOUD_IP>:7447

# Autopilot (optional)
liftoff-autopilot --target-alt 10 --zenoh-connect udp/<CLOUD_IP>:7447

# GPSD (optional)
liftoff-gpsd --zenoh-connect udp/<CLOUD_IP>:7447
```

## Multi-drone

Use `--zenoh-prefix` to isolate multiple drones on the same Zenoh network:

```bash
# Drone 1 (default prefix "liftoff")
liftoff-input

# Drone 2
liftoff-input --sim-bind 127.0.0.1:9002 --zenoh-prefix drone2
liftoff-autopilot --zenoh-prefix drone2 --zenoh-connect udp/<CLOUD_IP>:7447
```

## Common CLI flags

All binaries accept these Zenoh flags:

| Flag               | Default      | Description                                       |
|--------------------|--------------|---------------------------------------------------|
| `--zenoh-prefix`   | `liftoff`    | Topic prefix (for multi-drone)                    |
| `--zenoh-connect`  | *(none)*     | Endpoint to connect to (e.g. `udp/1.2.3.4:7447`) |
| `--zenoh-mode`     | `peer`       | Zenoh mode (`peer` or `client`)                   |
