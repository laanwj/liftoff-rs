# liftoff-rs

Tools and background services for CRSF joystick and telemetry, to use the quadcopter sim "liftoff" in hardware-in-the-loop simulation:

```
 ┏━━━━━━━┓
 ┃ radio ┃
 ┗━━━▲━━━┛
     ┆
     ┆ ELRS
     ┆
┏━━━━▼━━━━━┓
┃   ELRS   ┃        ┌─────────┐     ┌───────┐
┃ receiver ◀────────▶ forward ◀─────▶ input ▶─────────┐
┗━━━━━━━━━━┛ serial └─────────┘ UDP └───▲───┘         │
             CRSF               CRSF    │             │ uevent
                                        │ UDP         │
                                   ┌────▲───┐     ╔═══▼═════╗
                                   │ router ◀─────◀ liftoff ║
                                   └────▼───┘ UDP ║   sim   ║
                                        │ UDP     ╚═════════╝
                                        │
 ─ this repository                  ┌───▼──┐       ╔══════╗
 ═ external software                │ gpsd ▶───────▶ QGIS ║
 ━ hardware                         └──────┘       ╚══════╝
```
                                                             
- `liftoff-forward`: CRSF forwarder. Forward CRSF control and telemetry between a ELRS receiver UDP sockets
- `liftoff-gpsd`: gpsd emulator for viewing liftoff telemetry in QGIS
- `liftoff-router`: Telemetry router. Receives liftoff's native UDP telemetry and broadcasts it to multiple scripts and applications
- `liftoff-input`: CRSF joystick. Receives CRSF packets over UDP and simulates a linux udev joystick. Sends back telemetry from liftoff to UDP

## Setting up liftoff's telemetry stream

Create a file `TelemetryConfiguration.json` in liftoff's game configuration directory, with the following contents:

```json
{
  "EndPoint": "127.0.0.1:9001",
  "StreamFormat": [
    "Timestamp",
    "Position",
    "Attitude",
    "Velocity",
    "Gyro",
    "Input",
    "Battery",
    "MotorRPM"
  ]
}
```

On Linux this will usually be `~/.config/unity3d/LuGus Studios/Liftoff/`. The exact path depends on the operating system and/or install location. Details can be found here: [Liftoff - Drone Telemetry](https://steamcommunity.com/sharedfiles/filedetails/?id=3160488434). This also works for Liftoff: Micro Drones.

