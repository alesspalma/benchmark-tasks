"""Grade contrast of the actual modeled image and mechanical lock preload."""

from math import degrees

import numpy as np

from . import config as C
from . import optics


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.stable_s = 0.0

    def evaluate(self, state, elapsed):
        camera = state["objects"]["camera"]
        joints = camera["joints"]
        angle = joints["FocusHelix"]
        lock = joints["LockScrew"]
        advance = joints["LockAdvance"]
        contrast = optics.contrast(angle)
        sharp = contrast >= C.FULL_CONTRAST
        locked = advance >= 0.000109 and lock >= C.LOCK_ANGLE - 0.12
        clear = all(
            np.linalg.norm(np.asarray(p) - C.FOCUS_ORIGIN) > 0.105
            for hand in state["hands"].values()
            for p in hand["points"]
        )
        self.stable_s = self.stable_s + elapsed if sharp and locked and clear else 0.0
        success = self.stable_s >= 0.60
        if success:
            score = 1.0
        elif sharp:
            score = 0.55
        elif contrast >= C.PARTIAL_CONTRAST:
            score = 0.25
        else:
            score = 0.0
        return {
            "success": bool(success),
            "score": score,
            "line_pair_contrast": round(contrast, 4),
            "focus_angle_deg": round(degrees(angle), 3),
            "lock_screw_deg": round(degrees(lock), 3),
            "thread_advance_mm": round(1000 * advance, 3),
            "focus_secure": bool(locked),
            "grippers_clear": bool(clear),
            "stable_s": round(self.stable_s, 3),
        }
