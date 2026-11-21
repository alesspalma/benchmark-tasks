"""Reference test sequence with ordinary robot actions."""

import numpy as np

from . import config as C
from .motion import Program


def approach(tilt):
    tip = np.deg2rad(tilt)
    forward = np.array([np.sin(tip), 0, -np.cos(tip)])
    close = np.array([0, -1, 0])
    return np.column_stack((close, np.cross(forward, close), forward))


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves

    lock = np.array([C.CENTER_X, C.LOCK_Y, C.LOCK_Z])
    load = np.array([C.CENTER_X, C.CENTER_Y - C.LOAD_RADIUS, C.LOAD_Z])
    left_rot, right_rot = approach(26), approach(52)

    def left(offset, grip=0.0):
        if mode == "unlocked":
            return None
        return lock + np.asarray(offset), left_rot, grip

    def right(offset, grip=0.18):
        return load + np.asarray(offset), right_rot, grip

    p.add(
        2.20,
        "Approach the input lock and load handle",
        left=left((0, 0, 0.100)),
        right=right((0, 0, 0.120), 0.88),
    )
    p.add(
        1.10,
        "Place both tools at their working surfaces",
        left=left((0, 0, 0.019)),
        right=right((0, 0, 0.008), 0.88),
    )
    p.add(
        0.80,
        "Hold the input lock and grasp the spring handle",
        left=left((0, 0, -0.008)),
        right=right((0, 0, 0.008)),
    )
    p.add(1.10, "Apply negative proof torque", right=right((-0.0120, 0, 0.008)))
    p.add(1.75, "Hold the negative reading")
    p.add(0.85, "Cross the unloaded position", right=right((0.0, 0, 0.008)))
    if mode != "one_way":
        p.add(1.10, "Apply positive proof torque", right=right((0.0170, 0, 0.008)))
        p.add(1.95, "Hold the positive reading")
        p.add(0.80, "Return the loading handle to neutral", right=right((0, 0, 0.008)))
    p.add(0.55, "Let go of the spring handle", right=right((0, 0, 0.008), 0.82))
    p.add(
        1.40,
        "Release the input lock and clear the instrument",
        left=left((-0.032, 0.030, 0.120), 0.8),
        right=right((-0.036, -0.063, 0.108), 0.9),
    )
    p.add(0.65, "Leave the two readings and lost motion displayed")
    return p.moves
