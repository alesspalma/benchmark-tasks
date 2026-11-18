"""Observable poured, usable drink and an independently supported finish."""
import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.hold = 0

    def evaluate(self, state):
        from yam_arena.urdf import quat_xyzw_to_matrix

        drink = state["material"]
        glass = state["objects"]["glass"]
        pose = np.asarray(glass["pose"])
        upright = quat_xyzw_to_matrix(pose[3:])[2, 2] >= np.cos(np.deg2rad(5.))
        coaster = np.linalg.norm(pose[:2] - C.COASTER[:2]) <= .012 and abs(pose[2] - C.COASTER[2]) <= .004
        tap_closed = abs(state["objects"]["tap"]["joints"]["ValvePivot"]) <= .035
        stable = np.linalg.norm(glass["velocity"][:3]) < .02
        released = state["robot_clearance_m"] >= .010
        parked = upright and coaster and stable and released and tap_closed
        partial = drink["beer_ml"] >= 240. and drink["spilled_ml"] <= 25.
        good = (330. <= drink["beer_ml"] <= 425. and 16. <= drink["head_mm"] <= 34.
                and 3. <= drink["rim_gap_mm"] <= 18. and drink["spilled_ml"] <= 12.)
        self.hold = self.hold + 1 if parked and good else 0
        success = self.hold >= 12
        score = 1. if success else .5 if parked and partial else 0.
        return dict(drink, score=score, success=success, parked=bool(parked), closed=bool(tap_closed),
                    coaster=bool(coaster), upright=bool(upright), released=bool(released), hold=self.hold)
