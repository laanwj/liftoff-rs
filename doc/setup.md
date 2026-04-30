# Example liftoff-rs setup

## Architecture

liftoff-rs uses [Zenoh](https://zenoh.io) pub/sub for all inter-component communication. Components discover each other via Zenoh topics, with no hardcoded ports between them.

```
LOCAL                                    CLOUD
                                         Liftoff Sim
                                           | UDP :9001
                                         liftoff-input (sim → Zenoh bridge)
                                           | liftoff/telemetry, liftoff/crsf/telemetry
crsf-forward  <------- zenohd -------> (all topics)
  | serial                                 | liftoff/crsf/rc
Radio TX                                 crsf-joystick -> uinput -> Sim
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
| `liftoff-input`     | Cloud    | Liftoff UDP bridge + simstate consumer            |
| `crsf-joystick`     | Cloud    | CRSF RC channels → uinput virtual joystick + mux  |
| `crsf-forward`      | Local    | Serial port (radio TX) <-> Zenoh                  |
| `autopilot`         | Local    | Autonomous flight controller                       |
| `crsf-gpsd`         | Either   | NMEA GPS server from CRSF telemetry                |

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

# Liftoff sim UDP bridge → Zenoh
liftoff-input

# Virtual joystick the sim consumes (sim-agnostic; works for Liftoff,
# Velocidrone, Uncrashed)
crsf-joystick
```

All cloud-side binaries find `zenohd` automatically via local multicast scouting -- no connection flags needed.

Make sure **UDP port 7447** is open on the cloud server's firewall.

## Local machine setup

Local binaries connect to the cloud server's Zenoh router via `--zenoh-connect`.

```bash
# Serial bridge (radio TX <-> Zenoh)
crsf-forward --port /dev/ttyUSB0 --zenoh-connect udp/<CLOUD_IP>:7447

# Autopilot (optional)
autopilot --target-alt 10 --zenoh-connect udp/<CLOUD_IP>:7447

# GPSD (optional)
crsf-gpsd --zenoh-connect udp/<CLOUD_IP>:7447
```

## Multi-drone

Use `--zenoh-prefix` to isolate multiple drones on the same Zenoh network:

```bash
# Drone 1 (default prefix "liftoff")
liftoff-input

# Drone 2
liftoff-input --sim-bind 127.0.0.1:9002 --zenoh-prefix drone2
autopilot --zenoh-prefix drone2 --zenoh-connect udp/<CLOUD_IP>:7447
```

## Common CLI flags

All binaries accept these Zenoh flags:

| Flag               | Default      | Description                                       |
|--------------------|--------------|---------------------------------------------------|
| `--zenoh-prefix`   | `liftoff`    | Topic prefix (for multi-drone)                    |
| `--zenoh-connect`  | *(none)*     | Endpoint to connect to (e.g. `udp/1.2.3.4:7447`) |
| `--zenoh-mode`     | `peer`       | Zenoh mode (`peer` or `client`)                   |
