"""Force–displacement dough response and a crimp and release outcome score."""

from math import degrees, radians

import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.press_force = 0.0
        self.wet_contact_time = 0.0
        self.sealed = False
        self.folded_once = False
        self.stable_s = 0.0
        self.peak_force = 0.0
        self.registered = False
        self.seam_torn = False

    def sample(self, press, pastry, fold, dough_fold, dt):
        offset = np.asarray(pastry[:3]) - np.asarray(press[:3])
        self.registered = (
            np.linalg.norm(offset[:2]) < 0.010 and -0.009 < offset[2] < 0.014
        )
        touches_rim = self.registered and dough_fold > radians(171.0)
        self.press_force = max(0.0, fold - radians(169.0)) * 5.6 / C.SEAM_RADIUS if touches_rim else 0.0
        self.peak_force = max(self.peak_force, self.press_force)
        self.folded_once |= (
            self.registered and fold >= radians(155.0) and dough_fold >= radians(158.0)
        )
        if not self.sealed and not self.seam_torn:
            if self.press_force >= C.MIN_CRIMP_FORCE and dough_fold >= radians(174.0):
                self.wet_contact_time += dt
            else:
                self.wet_contact_time = 0.0
            self.sealed = self.wet_contact_time >= C.MIN_SEAL_DWELL
        elif self.sealed and dough_fold < radians(163.0):
            self.seam_torn = True
            self.sealed = False
        return -self.press_force * C.SEAM_RADIUS

    def evaluate(self, state, elapsed):
        press = state["objects"]["press"]
        pastry = state["objects"]["pastry"]
        position = np.asarray(pastry["pose"][:3])
        stationary = np.linalg.norm(pastry["velocity"][:3]) < 0.025
        angle = -pastry["joints"]["DoughFold"]
        gap = C.SEAM_RADIUS * 2.0 * np.sin(abs(C.FOLDED - angle) / 2.0)
        intact = self.sealed and not self.seam_torn and gap <= 0.003
        offset = position - np.asarray(press["pose"][:3])
        in_well = np.linalg.norm(offset[:2]) <= 0.014 and 0.0 <= offset[2] <= 0.016
        open_press = -press["joints"]["Fold"] <= radians(24)
        clear = all(
            min(np.linalg.norm(np.asarray(p) - position), np.linalg.norm(np.asarray(p) - press["pose"][:3])) > 0.093
            for hand in state["hands"].values()
            for p in hand["points"]
        )
        stable = intact and open_press and in_well and stationary and clear
        self.stable_s = self.stable_s + elapsed if stable else 0.0
        success = self.stable_s >= 0.6
        score = 1.0 if success else 0.65 if intact else 0.25 if self.folded_once else 0.0
        return {
            "success": bool(success),
            "score": score,
            "press_fold_deg": round(degrees(-press["joints"]["Fold"]), 3),
            "dough_fold_deg": round(degrees(angle), 3),
            "current_crimp_force_n": round(self.press_force, 3),
            "peak_crimp_force_n": round(self.peak_force, 3),
            "seam_gap_mm": round(1000 * gap, 3),
            "sealed_sectors": C.SECTOR_COUNT if intact else 0,
            "open_press": bool(open_press),
            "in_lower_well": bool(in_well),
            "clear_hands": bool(clear),
            "stable_s": round(self.stable_s, 3),
        }
