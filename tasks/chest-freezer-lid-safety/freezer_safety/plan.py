"""The ordinary joint-space reference pushes down on the clear lid's upper face."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def lid_point(theta, overshoot=0.):
    normal = np.array([np.sin(theta), 0., np.cos(theta)])
    tilt = np.deg2rad(-35.)
    approach = -np.array([np.sin(tilt), 0., np.cos(tilt)])
    cross = np.cross(approach, [0., -1., 0.])
    gesture = np.column_stack(([0., -1., 0.], cross, approach))
    position = C.HINGE + rotation("Y", theta) @ [-.417, .205, C.THICKNESS]
    offset = -approach if overshoot > 0 else normal
    return position + offset * overshoot, gesture, .04


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    p.add(2.1, "approach only after the area is clear", left=lid_point(C.ANGLE_OPEN, .055))
    p.add(.9, "touch lid", left=lid_point(C.ANGLE_OPEN, -.005))
    for a in np.linspace(C.ANGLE_OPEN, 0., 10)[1:]:
        p.add(.32, "close lid under contact", left=lid_point(a, -.009))
    p.add(.45, "seat gasket", left=lid_point(0., -.008))
    p.add(.85, "release lid", left=lid_point(0., .065))
    return p.moves
