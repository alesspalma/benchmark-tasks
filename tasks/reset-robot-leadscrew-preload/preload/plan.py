"""Mirror-compatible counterhold and preload relief using ordinary gripper commands."""

import numpy as np
from . import config as C
from .motion import Program, rotation


def pinch(axis, approach):
    z = np.asarray(approach) / np.linalg.norm(approach)
    x = np.asarray(axis) - np.dot(axis, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


LEFT = pinch([0., 0., 1.], [.50, -.86, 0.])
RIGHT = pinch([0., 0., -1.], [.95, .55, -.08])


def left(opening, delta=(0, 0, 0)):
    return C.CASE_GRIP + delta, LEFT, opening


def right(theta, opening, delta=(0, 0, 0)):
    return C.CAM_GRIP + delta, rotation("X", theta - C.START) @ RIGHT, opening


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    high = (-.043, .074, 0.)
    near = (-.075, -.017, .012)
    counterhold = mode != "no-brace"
    relief = C.START - np.radians(13.) if mode == "undertravel" else C.RELIEF
    if counterhold:
        p.add(2.5, "approach", left=left(1., high), right=right(C.START, .8, near))
        p.add(2.1, "place steadying jaws around nut", left=left(1.), right=right(C.START, .8, near))
        p.add(.9, "counterhold the free housing", left=left(.02))
    else:
        p.add(2.5, "approach using one arm", right=right(C.START, .8, near))
    p.add(1.4, "place jaws around cam ring", right=right(C.START, .8))
    p.add(.75, "grip ramp cam", right=right(C.START, .025))
    p.add(2.5, "turn the cam", right=right(relief, .025))
    p.add(.55, "complete the turn", right=right(relief, .025))
    p.add(.6, "release the cam", right=right(relief, .85))
    p.add(1.4, "clear the cam", right=right(relief, .85, near))
    p.add(1.8, "allow the cam to settle")
    if counterhold:
        p.add(.85, "release case", left=left(1.))
        p.add(1.1, "withdraw support fingers", left=left(1., high))
    p.add(2.5, "settled play and drag check")
    return p.moves
