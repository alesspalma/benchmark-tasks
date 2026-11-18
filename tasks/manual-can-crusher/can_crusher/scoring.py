"""Score the unloaded compacted can and collection outcome."""

import math

import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.settled = 0
        self.peak_fold = 0.

    def evaluate(self, state):
        position = np.asarray(state["can_pose"][:3])
        height = max(state["can_height"], state["plastic_height"])
        angle = state["lever_rad"]
        x, y, z = position - C.BIN_CENTER
        self.peak_fold = max(self.peak_fold, C.CAN_HEIGHT - state["can_height"])
        in_bin = (
            abs(x) < C.BIN_DEPTH/2 - .004 and abs(y) < C.BIN_WIDTH/2 - .004
            and -.007 < z < C.BIN_HEIGHT - .027
        )
        full_compaction = height <= .035
        full = in_bin and full_compaction and abs(angle) < math.radians(12.) and state["can_speed"] < .035
        self.settled = self.settled + 1 if full else 0
        success = self.settled >= 12
        loaded = abs(x + C.BIN_CENTER[0] - C.LOAD[0]) < .022 and abs(y) < .021 and .743 < position[2] < .800
        score = 1. if success else .5 if height <= .070 and (loaded or in_bin) else .25 if loaded else 0.
        return {
            "score": score,
            "success": success,
            "can_height_mm": state["can_height"]*1000.,
            "permanent_height_mm": height*1000.,
            "lever_degrees": math.degrees(angle),
            "in_bin": bool(in_bin),
            "seated_in_cradle": bool(loaded),
            "peak_fold_mm": self.peak_fold*1000.,
        }
