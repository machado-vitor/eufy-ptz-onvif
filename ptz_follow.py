#!/usr/bin/env python3
"""Follow a person on a camera by driving Eufy PTZ through Home Assistant.

Frigate detects a person -> MQTT -> Eufy PTZ buttons in Home Assistant.

Eufy pan/tilt cameras have no ONVIF support (the manufacturer confirms this
publicly), so there is no native Frigate autotracking, no absolute position
and no firmware presets. The underlying protocol (device.pan_and_tilt) only
accepts a `direction`, with no magnitude: each step is atomic, roughly 4-5%
of the frame, taking about 1.5-2 seconds.

Two properties measured on a real camera drive the design:
  - steps accumulate close to linearly (e.g. 32 / 79 / 127 / 181 px for
    1..4 steps in one test), so proportional movement can be approximated by
    firing N steps;
  - the displacement is therefore reversible: counting the net steps taken
    lets the camera return to its starting framing without any firmware
    preset (dead reckoning).

Note: on at least one camera model, a "rotation speed" select entity did NOT
change the step size (min/max values differ by measurement noise, not by a
real scale factor) - do not rely on it to modulate step magnitude; check your
own camera's behaviour before assuming it will help.
"""
import argparse
import json
import os
import threading
import time
import urllib.error
import urllib.request

import paho.mqtt.client as mqtt

# --- tunables (overridable via CLI, see --help) ---------------------------
MQTT_HOST = "127.0.0.1"
MQTT_PORT = 1883
CAMERA = "front"
LABEL = "person"

DEADZONE_X = 0.15
DEADZONE_Y = 0.20

# measured: one step moves ~4.5% of the frame width on the reference camera
STEP_FRACTION = 0.045
# gain < 1: corrects most of the error without overshooting. With gain 1 the
# smallest actionable error (the deadzone, 0.15) already needs 3 steps and
# proportionality saturates at the ceiling - every move becomes the same size.
GAIN = 0.6
MAX_STEPS = 6          # ceiling per cycle: beyond this the target has already moved
STEP_INTERVAL = 1.8    # > the ~1.7s the motor takes; queued commands get dropped
MIN_SCORE = 0.6

LOST_AFTER_S = 15.0
# only return home after being still for a while, so it doesn't recenter
# mid-patrol
HOME_AFTER_LOST_S = 60.0
MAX_HOME_STEPS = 40    # safety cap on the dead-reckoning undo

HASS_URL = "http://127.0.0.1:8123"
BUTTON_PREFIX = None  # set from --camera unless overridden
OPPOSITE = {"left": "right", "right": "left", "up": "down", "down": "up"}
STATE_PATH = None  # set in main()/configure() from --state-file


def buttons_for(prefix):
    return {
        "left": f"{prefix}_left",
        "right": f"{prefix}_right",
        "up": f"{prefix}_up",
        "down": f"{prefix}_down",
    }


BUTTONS = buttons_for(f"button.{CAMERA}_ptz")

TOKEN = None  # set in main() from the HASS_TOKEN environment variable


def log(msg):
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


def press(direction):
    body = json.dumps({"entity_id": BUTTONS[direction]}).encode()
    req = urllib.request.Request(
        f"{HASS_URL}/api/services/button/press",
        data=body,
        headers={"Authorization": f"Bearer {TOKEN}",
                 "Content-Type": "application/json"},
    )
    try:
        urllib.request.urlopen(req, timeout=10).read()
        return True
    except (urllib.error.URLError, TimeoutError, OSError) as exc:
        # a failed PTZ call costs one correction, never the whole loop
        log(f"ptz {direction} FAILED: {exc}")
        return False


class Follower:
    """Tracks the accumulated displacement so it can return to the starting framing.

    offset counts net steps: +x = how many more steps right/down were taken
    than left/up. Persisted to disk because the process restarts (KeepAlive)
    and the camera does not return to its starting point on its own.
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.last_move = 0.0
        self.last_seen = 0.0
        self.active = False
        self.moves = 0
        self.homed = True
        self.offset = {"x": 0, "y": 0}
        self._load()

    def _load(self):
        try:
            with open(STATE_PATH) as fh:
                data = json.load(fh)
            self.offset = {"x": int(data.get("x", 0)), "y": int(data.get("y", 0))}
            self.homed = self.offset == {"x": 0, "y": 0}
            if not self.homed:
                log(f"resuming with offset {self.offset} (not centered)")
        except (OSError, ValueError, KeyError):
            pass

    def _save(self):
        tmp = STATE_PATH + ".tmp"
        try:
            os.makedirs(os.path.dirname(STATE_PATH) or ".", exist_ok=True)
            with open(tmp, "w") as fh:
                json.dump(self.offset, fh)
            os.replace(tmp, STATE_PATH)   # atomic: a crash mid-write can't corrupt it
        except OSError as exc:
            log(f"could not save offset: {exc}")

    def _record(self, direction, n=1):
        if direction in ("left", "right"):
            self.offset["x"] += n if direction == "right" else -n
        else:
            self.offset["y"] += n if direction == "down" else -n
        self.homed = self.offset == {"x": 0, "y": 0}
        self._save()

    def on_object(self, box, score):
        """box normalized [x, y, w, h], origin at the top-left corner."""
        now = time.time()
        with self.lock:
            if score < MIN_SCORE:
                return
            self.last_seen = now
            if not self.active:
                self.active = True
                log(f"following (score {score:.2f})")
            if now - self.last_move < STEP_INTERVAL:
                return  # motor still moving: measuring now would be a dirty reading

            cx = box[0] + box[2] / 2
            cy = box[1] + box[3] / 2
            dx = cx - 0.5
            dy = cy - 0.5

            # one axis per cycle: pan and tilt together would queue commands
            # and the camera drops the second one
            if abs(dx) > DEADZONE_X and abs(dx) >= abs(dy):
                direction, err = ("right" if dx > 0 else "left"), abs(dx)
            elif abs(dy) > DEADZONE_Y:
                direction, err = ("down" if dy > 0 else "up"), abs(dy)
            else:
                return

            # proportional: the further off-center, the more steps (the camera
            # doesn't accept a magnitude, so magnitude becomes repetition)
            steps = max(1, min(MAX_STEPS, round(err * GAIN / STEP_FRACTION)))
            self.last_move = now + (steps - 1) * STEP_INTERVAL
            self.moves += steps
            log(f"center=({cx:.2f},{cy:.2f}) err={err:.2f} -> {direction} x{steps}")

        sent = 0
        for i in range(steps):
            if not press(direction):
                break
            sent += 1
            if i < steps - 1:
                time.sleep(STEP_INTERVAL)
        if sent:
            with self.lock:
                self._record(direction, sent)

    def go_home(self):
        """Undoes the accumulated displacement, one step at a time in the opposite direction."""
        with self.lock:
            plan = []
            for axis, key in (("x", "right"), ("y", "down")):
                n = self.offset[axis]
                if n:
                    plan.append((OPPOSITE[key] if n > 0 else key, min(abs(n), MAX_HOME_STEPS)))
            if not plan:
                self.homed = True
                return
            log(f"returning home: offset {self.offset}")

        for direction, n in plan:
            for _ in range(n):
                if not press(direction):
                    log("return home aborted (PTZ call failed)")
                    return
                with self.lock:
                    self._record(direction, 1)
                time.sleep(STEP_INTERVAL)
        with self.lock:
            log(f"back home (offset {self.offset})")

    def tick(self):
        with self.lock:
            now = time.time()
            if self.active and now - self.last_seen > LOST_AFTER_S:
                self.active = False
                log(f"lost target ({self.moves} moves)")
                self.moves = 0
            needs_home = (
                not self.active
                and not self.homed
                and self.last_seen
                and now - self.last_seen > HOME_AFTER_LOST_S
            )
        if needs_home:
            self.go_home()


follower = None  # created in main() after configuration is applied


def normalize(box):
    """frigate/events sends pixels; the REST API sends 0-1. Accepting only one breaks silently."""
    if box is None or len(box) != 4:
        return None
    if max(box) > 1.5:
        return [box[0] / 800, box[1] / 448, box[2] / 800, box[3] / 448]
    return list(box)


def on_connect(client, userdata, flags, rc, properties=None):
    log(f"mqtt connected rc={rc}")
    client.subscribe("frigate/events")
    client.subscribe("frigate/tracked_object_update")


def on_message(client, userdata, msg):
    try:
        payload = json.loads(msg.payload)
    except json.JSONDecodeError:
        return

    after = payload.get("after") or payload.get("before") or payload
    if after.get("camera") != CAMERA or after.get("label") != LABEL:
        return
    if payload.get("type") == "end":
        return

    box = normalize(after.get("box"))
    if box is None:
        return
    score = after.get("score") or after.get("top_score") or 0.0
    follower.on_object(box, score)


def default_state_path(camera):
    xdg_state = os.environ.get("XDG_STATE_HOME", os.path.expanduser("~/.local/state"))
    return os.path.join(xdg_state, "eufy-ptz-onvif", f"ptz_home_{camera}.json")


def build_parser():
    p = argparse.ArgumentParser(
        description="Follow a person on a Frigate camera by driving Eufy PTZ buttons in Home Assistant.",
    )
    p.add_argument("--ha-url", default=os.environ.get("HASS_URL", HASS_URL),
                   help="Home Assistant base URL (env HASS_URL, default %(default)s)")
    p.add_argument("--camera", default=os.environ.get("PTZ_CAMERA", CAMERA),
                   help="Frigate camera name to follow (env PTZ_CAMERA, default %(default)s)")
    p.add_argument("--label", default=os.environ.get("PTZ_LABEL", LABEL),
                   help="Frigate object label to follow (env PTZ_LABEL, default %(default)s)")
    p.add_argument("--button-prefix", default=os.environ.get("PTZ_BUTTON_PREFIX"),
                   help="HA button entity prefix, e.g. button.front_ptz "
                        "(default: button.<camera>_ptz)")
    p.add_argument("--mqtt-host", default=os.environ.get("MQTT_HOST", MQTT_HOST),
                   help="MQTT broker host (env MQTT_HOST, default %(default)s)")
    p.add_argument("--mqtt-port", type=int, default=int(os.environ.get("MQTT_PORT", MQTT_PORT)),
                   help="MQTT broker port (env MQTT_PORT, default %(default)s)")
    p.add_argument("--state-file", default=os.environ.get("PTZ_STATE_FILE"),
                   help="Path to persist the accumulated pan/tilt offset "
                        "(default: $XDG_STATE_HOME/eufy-ptz-onvif/ptz_home_<camera>.json)")
    p.add_argument("--deadzone-x", type=float, default=DEADZONE_X)
    p.add_argument("--deadzone-y", type=float, default=DEADZONE_Y)
    p.add_argument("--step-fraction", type=float, default=STEP_FRACTION,
                   help="fraction of the frame one PTZ step moves (measure on your camera)")
    p.add_argument("--gain", type=float, default=GAIN)
    p.add_argument("--max-steps", type=int, default=MAX_STEPS)
    p.add_argument("--step-interval", type=float, default=STEP_INTERVAL,
                   help="seconds to wait between steps (must exceed the camera's step time)")
    p.add_argument("--min-score", type=float, default=MIN_SCORE)
    p.add_argument("--lost-after", type=float, default=LOST_AFTER_S)
    p.add_argument("--home-after-lost", type=float, default=HOME_AFTER_LOST_S)
    p.add_argument("--max-home-steps", type=int, default=MAX_HOME_STEPS)
    return p


def configure(args):
    """Apply parsed CLI args to the module-level tunables the rest of the code reads."""
    global HASS_URL, CAMERA, LABEL, BUTTON_PREFIX, BUTTONS, MQTT_HOST, MQTT_PORT
    global STATE_PATH, DEADZONE_X, DEADZONE_Y, STEP_FRACTION, GAIN, MAX_STEPS
    global STEP_INTERVAL, MIN_SCORE, LOST_AFTER_S, HOME_AFTER_LOST_S, MAX_HOME_STEPS
    global TOKEN

    HASS_URL = args.ha_url
    CAMERA = args.camera
    LABEL = args.label
    BUTTON_PREFIX = args.button_prefix or f"button.{CAMERA}_ptz"
    BUTTONS = buttons_for(BUTTON_PREFIX)
    MQTT_HOST = args.mqtt_host
    MQTT_PORT = args.mqtt_port
    STATE_PATH = args.state_file or default_state_path(CAMERA)
    DEADZONE_X = args.deadzone_x
    DEADZONE_Y = args.deadzone_y
    STEP_FRACTION = args.step_fraction
    GAIN = args.gain
    MAX_STEPS = args.max_steps
    STEP_INTERVAL = args.step_interval
    MIN_SCORE = args.min_score
    LOST_AFTER_S = args.lost_after
    HOME_AFTER_LOST_S = args.home_after_lost
    MAX_HOME_STEPS = args.max_home_steps

    try:
        TOKEN = os.environ["HASS_TOKEN"]
    except KeyError:
        raise SystemExit("missing required environment variable: HASS_TOKEN")


def main():
    global follower
    args = build_parser().parse_args()
    configure(args)
    follower = Follower()

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="ptz-follower")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_HOST, MQTT_PORT, keepalive=60)
    client.loop_start()
    log("follower running")
    try:
        while True:
            time.sleep(1)
            follower.tick()
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()


if __name__ == "__main__":
    main()
