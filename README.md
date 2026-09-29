# eufy-ptz-onvif

Two small tools that make Eufy pan/tilt cameras (which have **no ONVIF and no
autotracking**) behave like PTZ cameras for [Frigate](https://frigate.video/)
and Home Assistant:

- **`onvif-eufy-shim`** — a minimal ONVIF Device/Media/PTZ server that Frigate
  can talk to. It translates `ContinuousMove`/`RelativeMove` SOAP calls into
  Home Assistant `button.press` calls on the eufy_security PTZ buttons.
- **`ptz-follow`** — a standalone daemon that subscribes to Frigate's MQTT
  events and drives the same PTZ buttons to keep a detected person centered
  in frame ("poor man's autotracking"), with dead-reckoning so it can return
  to its starting position.

## Why this exists

Eufy's pan/tilt cameras (e.g. the T8410 family) do not speak ONVIF at all —
the manufacturer confirms this publicly — and their P2P PTZ API only accepts
a `direction`, no magnitude and no position feedback: every physical move is
a fixed step (roughly a 90&deg; pan). That means:

- Frigate's PTZ UI needs *some* ONVIF endpoint to enable its buttons at all —
  `onvif-eufy-shim` fakes just enough of Device/Media/PTZ (`GetCapabilities`,
  `GetProfiles`, `ContinuousMove`, `RelativeMove`, `GetStatus`, ...) to make
  that work, mapping each move to one button press.
- Frigate's built-in autotracker expects continuous position feedback and
  absolute/relative moves it doesn't get here, so `ptz-follow` reimplements
  a much simpler version directly from Frigate's MQTT event stream: one axis
  per cycle, proportional step count based on how far off-center the target
  is, and a persisted offset so it can walk back to the starting framing
  after losing the target.

Both talk to Home Assistant's REST API to press the eufy_security PTZ
buttons — the shim also toggles the camera's on-board motion tracking off
during a manual move and restores it afterwards, since eufy's own tracking
silently swallows PTZ commands while it's active.

## Requirements

- Python 3.10+
- A Home Assistant instance with the [eufy_security integration](https://github.com/fuatakgun/eufy_security)
  exposing `button.<camera>_ptz_{left,right,up,down}` (and, for the shim,
  optionally a `switch.<camera>_motion_tracking` entity)
- `ptz-follow` also needs an MQTT broker that Frigate publishes to

```sh
pip install git+https://github.com/machado-vitor/eufy-ptz-onvif
```

## `onvif-eufy-shim`

```sh
export HASS_URL=http://127.0.0.1:8123
export HASS_TOKEN=...                     # long-lived access token
export CAMERAS=front:8999:button.front_ptz
onvif-eufy-shim
```

Point Frigate's camera config at it:

```yaml
cameras:
  front:
    onvif:
      host: 127.0.0.1
      port: 8999
```

### Configuration (environment variables)

| Variable | Meaning |
|---|---|
| `HASS_URL`, `HASS_TOKEN` | Home Assistant REST API |
| `CAMERAS` | comma separated `<name>:<port>:<button_prefix>`, e.g. `front:8999:button.front_ptz` |
| `RTSP_URL_<NAME>` | optional RTSP URL advertised in the media profile |
| `TRACKING_SWITCH_<NAME>` | HA switch for the camera's on-board motion tracking; turned off before a move, restored after `TRACKING_RESUME_S` |
| `TRACKING_RESUME_S` | seconds of idle before motion tracking is restored (default 60) |
| `DEBOUNCE_S` | collapse repeated `ContinuousMove` calls inside this window (default 3) |
| `AUTOTRACK_PAN_THRESHOLD` / `AUTOTRACK_TILT_THRESHOLD` | `RelativeMove` magnitude (FOV units, -1..1) that triggers one step (default 0.4 / 0.6) |
| `STEP_TIME_S` | how long `GetStatus` reports `MOVING` after a step (default 5) |
| `AUTOTRACK_MIN_GAP_S` | minimum seconds between autotrack-triggered steps (default 8) |

One `CAMERAS` entry per physical camera; each gets its own HTTP server on its
own port, all inside the same process.

## `ptz-follow`

```sh
export HASS_TOKEN=...
ptz-follow --ha-url http://127.0.0.1:8123 --camera front --mqtt-host 127.0.0.1
```

Run `ptz-follow --help` for the full list of tunables (deadzone, gain, step
timing, MQTT host/port, state file path). The measured constants
(`--step-fraction`, timing) were calibrated on one specific camera model —
re-measure on yours before trusting the defaults.

State (the accumulated pan/tilt offset used to walk back home) is persisted
to `$XDG_STATE_HOME/eufy-ptz-onvif/ptz_home_<camera>.json` by default so a
process restart doesn't lose track of where "home" is.

## Tests

```sh
python tests/test_ptz_follow.py
```

Offline unit tests for `ptz-follow` that stub out the HTTP call and check
*properties* (proportional step count, reversibility of the accumulated
offset, event filtering) rather than hardcoded magic numbers, so tuning the
constants doesn't break the suite.

## License

MIT
