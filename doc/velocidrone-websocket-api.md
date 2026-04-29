# Velocidrone WebSocket API

The WebSocket API is the only documented integration surface Velocidrone exposes. Velocidrone announced the IMU stream on April 2, 2026.

## Connection

- **URL:** `ws://<host>:60003/velocidrone` — the port is `60003` and the service path is `velocidrone` (the WSServer library defaults to `ws`, but Velocidrone's scene asset overrides the SerializeField to `velocidrone`). Both are configurable in the game scene. See "Bind address" below for what `<host>` actually is — `127.0.0.1` is **not** always correct. **Connecting to any other path returns HTTP 400** with no body hint, so a wrong path looks identical to a malformed handshake.

- **Bind address — not localhost.** The server binds to a **specific** IP address rather than `0.0.0.0` or `127.0.0.1`. The address is discovered at startup by opening a UDP socket to `8.8.8.8:65530` and reading back its local endpoint, which yields the IP of whichever interface the OS would route public traffic through. Practical consequences:

  - On a typical home machine behind NAT, the server binds to the LAN IP (e.g. `192.168.x.y`). `127.0.0.1` will refuse the connection.
  - On a machine with a public IP directly on the outbound interface, the server binds to that public IP — meaning the WebSocket is exposed to anyone who can route to that address. Firewall it (`iptables -A INPUT -p tcp --dport 60003 ! -s 127.0.0.1 -j DROP` or equivalent) if you don't want strangers reading your gyro telemetry.
  - The discovery happens once at server startup; if your routing changes (VPN up/down, interface flap) you'll need to restart Velocidrone.
  - There is no setting to override this. If you absolutely need loopback-only binding, the only options are firewalling, a routing-table rewrite, or a runtime patch that overrides `_serverURL` on `UnityWSServer` before the server starts.

  To find the address Velocidrone picked, `ss -ltn | grep 60003` (Linux) or `netstat -an | findstr 60003` (Windows) on the same machine after launching the game with `Use Web Socket` enabled.

- **Single client only.** Every outbound send-path is gated on the connection count being exactly 1. A second consumer's behavior is not defined; assume one consumer at a time and design your fan-out on your own side.

- **Keepalive timeout:** 40 s (server-side) — send `{"command":"ping"}` periodically if you don't otherwise have outbound traffic.

- **Frame type — outbound is binary, carrying text JSON.** The server wraps every JSON payload in a WebSocket **binary** frame (opcode `0x2`), not text (`0x1`), even though the bytes are always UTF-8 JSON. Naive consumers that only handle text frames (e.g. `for await msg in ws: parse(msg)` against a library that yields strings only for text frames) will silently drop everything. Decode binary payloads as UTF-8 and parse them as JSON the same way you would a text frame. Inbound (client → server) commands work either way; the server's command parser converts whatever it receives to a string before dispatching.

- **Persistence:** the server survives Velocidrone scene transitions, so your connection stays up across menu/race/menu cycles. There is no need to reconnect per flight.

## Enabling the stream

Two settings, both toggleable from Velocidrone's in-game settings menu:

| Setting | Effect | Default |
| ------------------------------------------ | ----------------------------------------------------------------------- | ----- |
| **Use Web Socket** (DB key `use-web-socket`) | Brings up the WebSocket server. Race / lobby events emit when this is on. | off |
| **Web Socket IMU** (DB key `web-socket-imu`) | Additionally enables the per-frame `imu` stream. | off |

The IMU stream is **in-flight only**. There is no menu-state or pre-flight heartbeat — `imu` frames start arriving once you spawn into a flight and stop on respawn / scene change. Race events emit independently in any state where the relevant action happens.

## Message envelope

Every message — inbound or outbound — is a single JSON object. The convention:

- **Outbound messages** are dispatched by the *single top-level key* that names the event. The value is either an object with the event's fields, an array, or (in one case) a string. Consumers should switch on the top-level key.

- **Inbound commands** carry a `"command"` field whose string value names the action, plus any command-specific fields at the same level.

There is no message ID, sequence number, or correlation field. Replies to commands (e.g. `pilotlist` for `getpilots`) come back as ordinary outbound events with no reference to the request.

## Outbound messages (server → client)

### `imu` — per-frame flight state

Available only when `web-socket-imu` is enabled. Emitted at 60 Hz while the local pilot is in a flight.

```json
{
  "imu": {
    "roll":      0.123,
    "pitch":     -0.045,
    "yaw":       0.0,
    "PositionX": 12.34,
    "PositionY": 5.67,
    "PositionZ": -8.9,
    "AttitudeX": 0.0,
    "AttitudeY": 0.0,
    "AttitudeZ": 0.0,
    "AttitudeW": 1.0,
    "SpeedX":    1.2,
    "SpeedY":    0.0,
    "SpeedZ":    -3.4,
    "timestamp": 12345.678
  }
}
```

| Field | Meaning |
| ------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------ |
| `roll`, `pitch`, `yaw` | **Gyro angular rates** in degrees/second — raw gyro readings, not Euler attitude angles. The naming is misleading; treat them as `gyro.{x,y,z}`. |
| `PositionX`, `PositionY`, `PositionZ` | Drone position in Velocidrone world coordinates (Unity convention, Y is up), metres. |
| `AttitudeX`, `AttitudeY`, `AttitudeZ`, `AttitudeW` | Drone orientation as a quaternion. |
| `SpeedX`, `SpeedY`, `SpeedZ` | Linear velocity, m/s, world frame. |
| `timestamp` | Float milliseconds since the game process started. **Not** wall-clock; resets on every game launch. |

All thirteen `imu` fields are JSON numbers (subject to the locale caveat below). The IMU stream is the only outbound message type with numeric values rather than stringified numbers.

### `session` — race lobby announcement

Sent on room creation.

```json
{
  "session": {
    "playerName":   "alice",
    "sessionName":  "Friday night",
    "sceneryTitle": "Snow Track",
    "trackName":    "Loops",
    "raceLength":   "3",
    "RaceMode":     "single_class",
    "quadType":     "freestyle",
    "quadSize":     "5inch"
  }
}
```

`raceLength` is a stringified integer.

### `countdown` — race countdown ticks

Sent once per second during the race start countdown, from 5 down to 1.

```json
{ "countdown": { "countValue": "5" } }
```

`countValue` is a stringified integer.

### `FinishGate` — start/finish gate state

```json
{ "FinishGate": { "StartFinishGate": "True" } }
```

`StartFinishGate` is a stringified C# boolean — `"True"` or `"False"` with capital first letter, **not** the lowercase JSON literal.

### `racetype` — race-mode change

```json
{
  "racetype": {
    "raceMode":   "single_class",
    "raceFormat": "fastest_lap",
    "raceLaps":   "3"
  }
}
```

All three fields are strings. `raceLaps` is a stringified integer.

### `racestatus` — race state change

```json
{ "racestatus": { "raceAction": "race finished" } }
```

`raceAction` is a free-form status string. The only value confirmed in the current build is `"race finished"`; treat unknown values as opaque.

### `racedata` — per-player race state, periodic

Emitted while a race is in progress. One entry per active player, keyed by player nickname.

```json
{
  "racedata": {
    "alice": {
      "position": "1",
      "lap":      "2",
      "gate":     "5",
      "time":     "23.456",
      "finished": "False",
      "colour":   "#FF0000",
      "uid":      "12345"
    },
    "bob": { "...": "..." }
  }
}
```

| Field | Meaning |
| -------- | --------------------------------------------------------------------------------------------------- |
| `position` | Current race position, 1-indexed, stringified integer. |
| `lap` | Current lap, 1-indexed, stringified integer. |
| `gate` | Current gate within the lap, stringified integer. |
| `time` | Elapsed race time in seconds, formatted with three decimals (`F3`). Locale-sensitive — see Caveats. |
| `finished` | Stringified C# boolean: `"True"` or `"False"`. |
| `colour` | Player's HUD colour as a CSS-style hex string. |
| `uid` | Player UID, stringified. |

### `player` — player joined / state changed

```json
{
  "player": {
    "PlayerName":   "alice",
    "playerColour": "#FF0000",
    "playerFlying": "True",
    "raceManager":  "False"
  }
}
```

`playerFlying` and `raceManager` are stringified booleans.

### `spectatorChange` — spectator name change

```json
{ "spectatorChange": "alice" }
```

The value is the spectator's display name as a plain string — this is the only outbound message whose top-level value is a string rather than an object or array.

### `pilotlist` — response to `getpilots`

```json
{
  "pilotlist": [
    { "name": "alice", "uid": "12345" },
    { "name": "bob",   "uid": "67890" }
  ]
}
```

`uid` is stringified.

### `ActivateError` — pilot activation failure

```json
{ "ActivateError": { "UIDNotFound": "12345" } }
```

`UIDNotFound` is a stringified UID.

## Inbound commands (client → server)

Send JSON objects with a top-level `"command"` field plus any extras. Anything outside this table logs `"Unknown command received: <command>"` server-side and is ignored.

| `command` | Available when | Extra fields | Action |
| ------------ | -------------- | ------------------------ | ----------------------------------------------------------------- |
| `startrace` | always | — | Start the configured race (host only in multiplayer). |
| `abortrace` | always | — | Abort the current race. |
| `activate` | multiplayer host | `uids: [int, …]` | Activate listed pilots through the host gate flow. |
| `lock` | multiplayer host | — | Lock the session — no further joins. |
| `unlock` | multiplayer host | — | Unlock the session. |
| `allspectate` | multiplayer host | — | Force every connected player into spectator mode. |
| `getpilots` | multiplayer host | — | Reply asynchronously with a `pilotlist` event. |
| `cameraselect` | not in main menu | camera number field | Numpad-style camera switch. |
| `cameramode` | not in main menu | `mode: "fpv" \| "spectate"` | Switch view mode. Other values log `"Unknown camera mode: <text>"`. |
| `cameraplayer` | not in main menu | `uid: int` | Aim the camera at the player with this uid. |
| `camerareset` | not in main menu | — | Reset the camera to the local player. |
| `ping` | always | — | No-op; useful as a keepalive. |

"Multiplayer host" gating is silent — commands sent in the wrong mode are dropped without an error reply. Plan to either know your context or rely on observed effects (e.g. a `racestatus` event arriving) rather than ack semantics.

## Caveats

These are real wire-level quirks that a robust consumer should handle:

1. **Decimal separator follows the host's locale.** All JSON numbers are emitted via the default culture of the C# runtime, so on machines configured for a comma-decimal locale (de\_DE, nl\_NL, fr\_FR, pt\_BR, …) `imu` fields and `racedata.time` come out as `1,234` instead of `1.234` — invalid JSON. Workarounds:

  - Run Velocidrone with `LC_ALL=C` / `LANG=C.UTF-8` on Linux.

  - Set the OS region to "English (United States)" on Windows.

  - On the consumer side: detect a parse failure containing comma-separated numerics and either translate them or error loudly.

2. **Strings are not JSON-escaped.** Player nicknames, chat fragments, and any other free-form string are concatenated between literal `"` quotes with no handling of `"`, `\`, or control characters. In practice nicknames are sanitized by the platform layer, but a defensive parser should not crash on a stray malformed message — drop and continue.

3. **Many integer fields are stringified.** Outside the `imu` payload, expect numeric-looking values (`countValue`, `position`, `lap`, `gate`, `raceLength`, `uid`, …) to be JSON strings, not JSON numbers. Booleans similarly are `"True"` / `"False"` strings, not the JSON literals.

4. **No backpressure.** The server does not throttle or coalesce sends. A slow consumer will see TCP-level buffering and eventually a forced disconnect under write timeout. For a high-rate `imu` consumer, drain promptly and decimate on your side if you don't need 60 Hz.

5. **No reconnect signal on settings change.** Toggling `Use Web Socket` off and on again restarts the server but does not push an event; your client sees a normal close and should reconnect with backoff.

## Quick parser sketch (pseudo-Rust)

```rust
match serde_json::from_str::<Value>(&text)?.as_object()? {
    obj if obj.contains_key("imu")        => handle_imu(obj["imu"]),
    obj if obj.contains_key("racedata")   => handle_racedata(obj["racedata"]),
    obj if obj.contains_key("countdown")  => handle_countdown(obj["countdown"]),
    obj if obj.contains_key("racestatus") => handle_racestatus(obj["racestatus"]),
    // … one branch per known top-level key …
    _ => log_unknown(&text),
}
```

A consumer that only cares about `imu` can ignore everything else and simply check for that single top-level key.
