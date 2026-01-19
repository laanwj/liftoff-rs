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


