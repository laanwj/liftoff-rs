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

| Script | Display | Layout | Radios |
|--------|---------|--------|--------|
| `dmgbw.lua` | B&W 128x64 | Numbered rotors `1`..`N` with horizontal health bars | Radiomaster Pocket, Zorro, Boxer, TX12, Taranis QX7, etc. |
| `dmgfig.lua` | B&W 128x64 | Quad X-frame diagram with each rotor's percentage drawn next to it; rotors fill from the bottom and become a bold X at 0% | Same as above, but quadcopter-only |
| `dmgcol.lua` | Color LCD | Numbered rotors with colour-coded health bars | Radiomaster TX16S, Jumper T-Pro, Horus X10/X12, etc. |

`dmgfig.lua` is purpose-built for 4-rotor X-frame quads — it falls back
to a "use dmgbw.lua" message if the sim reports any other rotor count.
You can install both `dmgbw.lua` and `dmgfig.lua` side by side and
assign each to a different telemetry screen.

The names are deliberately short: EdgeTX requires telemetry-script
filenames (without the `.lua` extension) to be **6 characters or less**,
otherwise the script is silently filtered out of the screen-type picker.
See the [EdgeTX Lua reference](https://luadoc.edgetx.org/overview/script-types/telemetry-scripts.md).

## Installation

1. Connect your radio's SD card (USB mass storage or SD card reader).

2. Copy the appropriate script(s) to the telemetry scripts folder:

   ```
   # For B&W radios (Pocket, Zorro, etc.) -- install one or both:
   cp dmgbw.lua  /path/to/sdcard/SCRIPTS/TELEMETRY/
   cp dmgfig.lua /path/to/sdcard/SCRIPTS/TELEMETRY/

   # For color radios (TX16S, etc.)
   cp dmgcol.lua /path/to/sdcard/SCRIPTS/TELEMETRY/
   ```

3. Eject the SD card cleanly (or `sync`) and re-insert it into the radio,
   then power-cycle so EdgeTX rescans `SCRIPTS/TELEMETRY/`.

4. On the radio, go to **Model Setup > Display** (or **Telemetry Screens**
   on some firmware versions), pick an unused screen slot, set its **Type**
   to `Script`, then select `dmgbw`, `dmgfig`, or `dmgcol` from the picker.

5. The script runs in the background whenever the model is active, so the
   `Hp1`..`Hp8` sensors are available even when you're not viewing the
   telemetry screen.

After the first run a `.luac` bytecode file will appear next to each
loaded source on the SD card — that's EdgeTX's compile cache and
confirms the script was actually loaded.

## Telemetry sensors

All three scripts register the same `Hp1`..`Hp8` telemetry sensors
regardless of which one is currently displayed.

| Sensor | Description |
|--------|-------------|
| `Hp1` | Rotor 1 health (0.00% - 100.00%) — **LF** |
| `Hp2` | Rotor 2 health — **RF** |
| `Hp3` | Rotor 3 health — **LB** |
| `Hp4` | Rotor 4 health — **RB** |
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
