"""Use the parallel-jaw YAMs to carry and feed actual nylon strips."""
import numpy as np

from . import config as C
from .motion import Program, rotation

DOWN = np.diag([-1., 1., -1.])
SIDES = ("left", "right")


def target(side, point, opening):
    # Placing both tools in opposing slight yaw avoids their knuckle housings.
    return np.asarray(point), rotation("Z", C.sign(side) * np.radians(2.)) @ DOWN, opening


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    stock = {
        s: C.line_center(s) + [0., C.sign(s) * (C.GRIP_FROM_TIP - C.LINE_LENGTH / 2), 0.]
        for s in SIDES
    }
    entry = {
        s: C.hole(s) + [0., C.sign(s) * (C.GRIP_FROM_TIP + .008), .00060 if s == "left" else 0.]
        for s in SIDES
    }
    loaded = {
        s: entry[s] - [0., C.sign(s) * (.045 if s == "left" or mode != "unseated" else .015), 0.]
        for s in SIDES
    }

    def move(steps, label, points, grip, z=0.):
        p.add(steps, label, **{s: target(s, points[s] + [0, 0, z], grip) for s in SIDES})

    move(2.5, "approach the two loose nylon strips", stock, .82, .080)
    move(1.7, "lower open jaws around the two strips", stock, .42)
    move(.75, "pinch centered on each line", stock, .0)
    move(1.3, "lift the strips off their parts rests", stock, .0, .053)
    move(2.2, "bring each tip to its own eyelet", entry, .0, .044)
    move(1.75, "lower both tips to the eyelet heights", entry, .0)
    move(1.7, "align and settle both tips immediately outside the eyelets", entry, .0)
    move(2.1, "feed both ends through the gripping channels", loaded, .0)
    move(.50, "finish seating the new lines", loaded, .0)
    test = {s: loaded[s] + [0, C.sign(s) * .0025, 0] for s in SIDES}
    move(.55, "check the lines against a gentle outward pull", test, .0)
    move(.35, "release both nylon lines", test, .95)
    move(.65, "lift away from the seated lines", test, .70, .027)
    move(1.0, "withdraw the hands from the loaded head", entry, .65, .065)
    return p.moves
