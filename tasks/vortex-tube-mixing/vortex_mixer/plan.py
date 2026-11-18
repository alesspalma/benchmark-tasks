"""Ordinary grasp, sustained downward cup load, then stable placement."""
from . import config as C
import numpy as np

from .motion import Program, rotation

SIDE_X = rotation("Z", np.deg2rad(60.)) @ np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])


def waypoints(mode="demo"):
    if mode == "hold":
        return {}
    start = C.TUBE + [0., 0., C.GRIP_HEIGHT]
    press = C.MIXER + [0., 0., C.PAD_Z + C.GRIP_HEIGHT]
    grip = .10
    action = Program()
    action.add(2.4, "approach tube", right=(start - .072 * SIDE_X[:, 2] + [0., 0., .020], SIDE_X, 1.))
    action.add(2.3, "inspect sample layers from the pickup approach")
    action.add(1.25, "center fingers on the upper tube", right=(start, SIDE_X, 1.))
    action.add(1.40, "grip tube", right=(start, SIDE_X, grip))
    action.add(1.2, "lift tube upright", right=(start + [0., 0., .115], SIDE_X, grip))
    action.add(1.65, "align tube with the mixing cup", right=(press + [0., 0., .055], SIDE_X, grip))
    load = -.012 if mode != "no-press" else .008
    action.add(1.3, "press the tube down into the cup", right=(press + [0., 0., load], SIDE_X, grip))
    action.add(4.9, "maintain cup pressure through orbital agitation")
    action.add(.85, "lift to stop the mixer", right=(press + [0., 0., .055], SIDE_X, grip))
    action.add(1.5, "bring the capped sample back to the bench", right=(start + [0., 0., .108], SIDE_X, grip))
    action.add(1.5, "stand sample upright", right=(start - [0., 0., .001], SIDE_X, grip))
    action.add(.55, "release sample", right=(start, SIDE_X, 1.))
    action.add(1.3, "withdraw from the standing sample", right=(start - .07 * SIDE_X[:, 2] + [0., 0., .05], SIDE_X, 1.))
    action.add(1.0, "leave finished sample upright")
    return action.moves
