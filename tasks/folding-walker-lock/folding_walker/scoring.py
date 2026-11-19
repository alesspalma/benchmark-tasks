"""Evaluate released lock engagement, placement, and four-foot support."""

import math

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.settled = 0

    def evaluate(self, state):
        x, y, _ = state["root"][:3]
        orientation = quat_xyzw_to_matrix(state["root"][3:])
        angle = math.atan2(orientation[1, 0], orientation[0, 0])
        spread = max(abs(q) for q in state["fold_rad"].values())
        lock = all(state["locked"].values()) and spread <= math.radians(4.)
        lock &= all(height >= 4.4 for height in state["button_mm"].values())
        stable = max(abs(p[2] - C.FLOOR) for p in state["feet"]) <= .012
        stable &= state["root_speed"] <= .022
        stable &= state["root_angular_speed"] <= math.radians(10.)
        positioned = .14 <= x <= .30 and abs(y) <= .065 and abs(angle) <= math.radians(12.)
        # The ordinary handgrips must be clear before the walker is useful.
        clear = all(
            np.linalg.norm(np.asarray(hand) - grip) >= .085
            for hand in state["hands"].values()
            for grip in state["grips"].values()
        )
        safe = lock and stable and positioned and clear
        self.settled = self.settled + 1 if safe else 0
        return {
            "score": 1. if self.settled >= 15 else 0.,
            "success": self.settled >= 15,
            "left_locked": state["locked"]["left"],
            "right_locked": state["locked"]["right"],
            "fold_degrees": {side: math.degrees(q) for side, q in state["fold_rad"].items()},
            "button_mm": state["button_mm"],
            "four_feet_stable": bool(stable),
            "positioned": bool(positioned),
            "hands_clear": bool(clear),
            "settled_steps": self.settled,
        }
