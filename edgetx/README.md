# EdgeTX Telemetry Scripts

LUA telemetry scripts for EdgeTX that decode custom CRSF damage telemetry
from liftoff-rs and display per-rotor health on the radio.

## What it does

liftoff-rs sends a custom CRSF frame (type `0x42`) containing per-rotor
health data. EdgeTX doesn't know this frame type, so it forwards it raw to
the LUA telemetry queue. The script picks it up with
`crossfireTelemetryPop()`, parses the payload, and registers telemetry
sensors `Hp1`..`Hp8` via `setTelemetryValue()`.

Once registered, these sensors work like any built-in telemetry sensor --
they appear in telemetry screens, can trigger logical switches, drive voice
alerts, and are readable from other LUA scripts with `getValue("Hp1")`.

## Choosing a script

| Script | Display | Radios |
|--------|---------|--------|
| `damage_bw.lua` | B&W 128x64 | Radiomaster Pocket, Zorro, Boxer, TX12, Taranis QX7, etc. |
| `damage_color.lua` | Color LCD | Radiomaster TX16S, Jumper T-Pro, Horus X10/X12, etc. |

## Installation

1. Connect your radio's SD card (USB mass storage or SD card reader).

2. Copy the appropriate script to the telemetry scripts folder:

   ```
   # For B&W radios (Pocket, Zorro, etc.)
   cp damage_bw.lua /path/to/sdcard/SCRIPTS/TELEMETRY/damage.lua

   # For color radios (TX16S, etc.)
   cp damage_color.lua /path/to/sdcard/SCRIPTS/TELEMETRY/damage.lua
   ```

   The file **must** be named `damage.lua` (or any name you like) in the
   `SCRIPTS/TELEMETRY/` folder.

3. On the radio, go to **Model Setup > Display** (or **Telemetry Screens**
   on some firmware versions) and assign a screen to the **Script** type,
   then select `damage`.

4. The script runs in the background whenever the model is active, so the
   `Hp1`..`Hp8` sensors are available even when you're not viewing the
   telemetry screen.

## Telemetry sensors

| Sensor | Description |
|--------|-------------|
| `Hp1` | Rotor 1 health (0.00% - 100.00%) |
| `Hp2` | Rotor 2 health |
| `Hp3` | Rotor 3 health |
| `Hp4` | Rotor 4 health |
| `Hp5`..`Hp8` | Rotors 5-8 (if present) |

100.00% = fully healthy, 0.00% = destroyed.

## Example: voice alert on damage

In **Model Setup > Logical Switches**, create a switch:

- Function: `a < x`
- Source: `Hp1`
- Value: `80.00` (triggers when rotor 1 drops below 80% health)

Then in **Special Functions**, assign a voice alert to that logical switch.

## Wire format

The CRSF frame uses extended-header format (type >= `0x28`):

```
Sync  Len  Type  Dest  Orig  Flags  N  Health[0] ... Health[N-1]  CRC
0xC8  var  0x42  0xEA  0xC8  u8     u8 u16 BE    ... u16 BE       u8
```

- **Dest** `0xEA` = Remote Control
- **Orig** `0xC8` = Flight Controller
- **Flags**: bit 0 = killed, bit 1 = crashed, bit 2 = no drone
- **N**: number of rotors (1-8)
- **Health**: 0-10000 (big-endian u16), representing 0.00% to 100.00%

Total frame size for a 4-rotor quad: 16 bytes.
