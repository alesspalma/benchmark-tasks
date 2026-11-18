"""Score the physical tear and exposed book without scoring how it was done."""

import math

import numpy as np

from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.hold_since = None

    def evaluate(self, state):
        tray = state["objects"]["mailer"]
        pos = np.asarray(tray["pose"][:3])
        rotation = quat_xyzw_to_matrix(tray["pose"][3:])
        book = rotation.T @ (np.asarray(state["objects"]["paperback"]["pose"][:3]) - pos)
        inside = bool(abs(book[0]) < C.DEPTH / 2 and abs(book[1]) < C.WIDTH / 2
                      and 0 < book[2] < C.HEIGHT)
        upright = bool(rotation[2, 2] >= math.cos(math.radians(20.))
                       and abs(pos[2] - C.BOX[2]) < .025)
        angle = math.degrees(tray["joints"]["Crease"])
        torn = state["material"]["progress_m"] / C.TEAR_LENGTH
        opened = inside and upright and torn >= 1. and angle >= 48.
        if not opened:
            self.hold_since = None
        elif self.hold_since is None:
            self.hold_since = state["time_s"]
        success = opened and state["time_s"] - self.hold_since >= .4
        score = 0.
        if upright and inside:
            score = 1. if success else .6 if torn >= 1. else .3 if torn >= .5 else 0.
        return dict(score=score, success=bool(success), lid_deg=angle,
                    fraction_torn=torn, book_inside=inside, carton_upright=upright)
