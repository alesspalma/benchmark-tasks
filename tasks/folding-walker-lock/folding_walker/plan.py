"""Ordinary contact at the two walker handgrips."""

import numpy as np

from . import config as C
from .motion import Program, rotation

TOP_VIEW = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]]) @ rotation("X", np.deg2rad(25.))
GRIP = .255


def pose(side, fold, grip=GRIP, height=0.):
    sign = C.SIDES[side]
    yaw = rotation("Z", -sign * fold)
    position = C.START + [0., sign*C.HINGE_HALF_WIDTH, height] + yaw @ [
        C.GRASP_DISTANCE, 0., C.GRIP_HEIGHT
    ]
    return position, yaw @ TOP_VIEW, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    active = tuple(C.SIDES)
    end_fold = {side: -.023 for side in active}
    if mode == "one-side":
        end_fold["right"] = np.deg2rad(8.)
    p.add(2.2, "move both wrists above the grip sleeves",
          **{s: pose(s, C.START_FOLD[s], 1., .075) for s in active})
    p.add(1.1, "descend around the grip sleeves",
          **{s: pose(s, C.START_FOLD[s], 1.) for s in active})
    p.add(.7, "close the grippers on the two tubes",
          **{s: pose(s, C.START_FOLD[s]) for s in active})
    for frac in np.linspace(1., 0., 9)[1:]:
        p.add(.36, "sweep the two frame sides outward",
              **{s: pose(s, frac*(C.START_FOLD[s] - end_fold[s]) + end_fold[s]) for s in active})
    p.add(1.1, "hold at the stops while the spring pins engage",
          **{s: pose(s, end_fold[s] - .018) for s in active})
    p.add(.6, "release the walker", **{s: pose(s, end_fold[s] + .013, 1.) for s in active})
    p.add(1.2, "raise clear of the walker", **{s: pose(s, max(0., end_fold[s]), 1., .13) for s in active})
    p.add(1.0, "let the walker stand free")
    return p.moves
