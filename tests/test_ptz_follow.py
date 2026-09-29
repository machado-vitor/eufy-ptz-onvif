#!/usr/bin/env python3
"""PTZ follower test suite. Runs without touching the camera: press() is replaced.

Tests PROPERTIES (proportionality, reversibility), not magic numbers: the
GAIN constant can change and the test still has to hold.
Usage: python ptz_follow_test.py
"""
import importlib.util
import json
import os
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(HERE)
spec = importlib.util.spec_from_file_location("pf", os.path.join(REPO_ROOT, "ptz_follow.py"))
pf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pf)

pf.STATE_PATH = tempfile.mktemp()
pf.STEP_INTERVAL = 0
pf.TOKEN = "test-token"
pf.HASS_URL = "http://127.0.0.1:8123"
pf.BUTTONS = pf.buttons_for("button.test_ptz")

moved = []
pf.press = lambda d: (moved.append(d), True)[1]


def new_follower():
    f = pf.Follower()
    f.last_move = 0
    f.offset = {"x": 0, "y": 0}
    f.homed = True
    return f


ok = total = 0


def check(name, cond, extra=""):
    global ok, total
    total += 1
    ok += bool(cond)
    print(f"{'OK ' if cond else 'FAILED'} {name:44s} {extra}")


def steps(cx, cy=0.45, score=0.9):
    moved.clear()
    f = new_follower()
    f.on_object([cx - 0.02, cy - 0.05, 0.04, 0.10], score)
    return list(moved), f


# --- proportionality: a property, not a fixed value ---
seq = [(cx, len(steps(cx)[0])) for cx in (0.66, 0.70, 0.80, 0.90, 0.99)]
check("proportional and monotonic", all(b >= a for (_, a), (_, b) in zip(seq, seq[1:])), str([n for _, n in seq]))
check("larger error generates more steps than smaller", seq[-1][1] > seq[0][1], f"{seq[0][1]} -> {seq[-1][1]}")
check("respects MAX_STEPS", seq[-1][1] <= pf.MAX_STEPS, f"max={seq[-1][1]}")
check("minimum error triggers at least 1 step", seq[0][1] >= 1)
check("centered does not move", len(steps(0.48)[0]) == 0)
check("inside the deadzone does not move", len(steps(0.5 + pf.DEADZONE_X - 0.01)[0]) == 0)
check("outside the deadzone moves", len(steps(0.5 + pf.DEADZONE_X + 0.03)[0]) >= 1)

# --- directions ---
check("right", set(steps(0.95)[0]) == {"right"})
check("left", set(steps(0.05)[0]) == {"left"})
mv, _ = steps(0.48, cy=0.95)
check("down", set(mv) == {"down"})
mv, _ = steps(0.48, cy=0.05)
check("up", set(mv) == {"up"})

# --- reversibility (what replaces the firmware preset) ---
mv, f = steps(0.95)
n = len(mv)
check("offset records the steps taken", f.offset["x"] == n, f"offset={f.offset}")
moved.clear()
f.go_home()
check("go_home zeroes the offset", f.offset == {"x": 0, "y": 0}, f"offset={f.offset}")
check("go_home moves the same amount in the opposite direction", moved == ["left"] * n, f"{len(moved)} x left")

# diagonal: must undo both axes
f = new_follower()
f.offset = {"x": 2, "y": -3}
f.homed = False
moved.clear()
f.go_home()
check("go_home undoes both axes", f.offset == {"x": 0, "y": 0}, f"{moved}")
# x=+2 is 2 right steps (undone with left); y=-3 is 3 up steps (undone with down)
check("go_home inverts each axis", moved.count("left") == 2 and moved.count("down") == 3)

# --- persistence across restarts (KeepAlive restarts the process) ---
f = new_follower()
f.on_object([0.93, 0.40, 0.04, 0.10], 0.9)
expected = dict(f.offset)
check("offset persists to disk", pf.Follower().offset == expected, f"{expected}")
check("resumes knowing it is not centered", pf.Follower().homed is False)

# --- a failed PTZ call must not corrupt dead reckoning ---
calls = {"n": 0}
pf.press = lambda d: (calls.__setitem__("n", calls["n"] + 1), calls["n"] <= 1)[1]
f = new_follower()
f.on_object([0.95, 0.45, 0.04, 0.10], 0.9)
check("offset only counts confirmed steps", f.offset["x"] == 1, f"offset={f.offset}")
pf.press = lambda d: (moved.append(d), True)[1]

# --- filters ---
check("low score ignored", len(steps(0.95, score=0.3)[0]) == 0)
pf.follower = new_follower()


class FakeMessage:
    pass


def msg(d):
    m = FakeMessage()
    m.payload = json.dumps(d).encode()
    moved.clear()
    pf.on_message(None, None, m)
    return list(moved)


check("wrong camera ignored", msg({"type": "update", "after": {"camera": "other", "label": "person", "score": .9, "box": [700, 200, 60, 120]}}) == [])
check("wrong label ignored", msg({"type": "update", "after": {"camera": pf.CAMERA, "label": "car", "score": .9, "box": [700, 200, 60, 120]}}) == [])
check("end event ignored", msg({"type": "end", "after": {"camera": pf.CAMERA, "label": "person", "score": .9, "box": [700, 200, 60, 120]}}) == [])
check("valid event triggers", len(msg({"type": "update", "after": {"camera": pf.CAMERA, "label": "person", "score": .9, "box": [700, 200, 60, 120]}})) >= 1)

# --- normalize accepts both formats Frigate sends ---
check("normalize: pixels", pf.normalize([400, 224, 80, 80])[0] == 0.5)
check("normalize: already normalized", pf.normalize([0.5, 0.5, 0.1, 0.1])[0] == 0.5)
check("normalize: invalid box", pf.normalize([1, 2]) is None and pf.normalize(None) is None)

if os.path.exists(pf.STATE_PATH):
    os.unlink(pf.STATE_PATH)
print(f"\n{ok}/{total} cases")
sys.exit(0 if ok == total else 1)
