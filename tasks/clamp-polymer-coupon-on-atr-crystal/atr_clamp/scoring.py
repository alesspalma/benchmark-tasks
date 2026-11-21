"""Grade contact footprint, normal pressure and an unsupported finish."""

from math import cos, degrees, pi, radians

import numpy as np
from scipy.spatial.transform import Rotation

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.stable_s = 0.0
        self.contact_reference = None

    def evaluate(self, state, elapsed):
        objects = state["objects"]
        pose = np.asarray(objects["coupon"]["pose"])
        axes = Rotation.from_quat(pose[3:]).as_matrix()
        local = axes.T @ (C.CRYSTAL - pose[:3])
        margin = float(min(C.COUPON_SIZE[:2] / 2 - np.abs(local[:2])))
        flat = abs(axes[2, 2]) >= cos(radians(3.0))
        flush = abs(pose[2] - C.CRYSTAL[2] - 0.5 * C.COUPON_SIZE[2]) < 0.0011
        covers = flat and flush and margin > 0.0075
        press = objects["instrument"]["joints"]
        knob = press["PressureKnob"]
        anvil = press["PressureAnvil"]
        displacement = C.LEAD * min(0.0, knob) / (2 * pi)
        load = min(C.NOMINAL_LOAD, max(0.0, C.CUP_STIFFNESS * (anvil - displacement)))
        gap = C.TIP_BOTTOM + anvil - (pose[2] + C.COUPON_SIZE[2] / 2)
        touching = covers and abs(gap) < 0.0008
        clear = all(
            np.linalg.norm(np.asarray(point) - C.CRYSTAL) > 0.076
            and np.linalg.norm(np.asarray(point) - C.KNOB) > 0.059
            for hand in state["hands"].values()
            for point in hand["points"]
        )
        loaded = touching and load >= C.FULL_LOAD and clear
        if not loaded:
            self.contact_reference = None
            self.stable_s = 0.0
        elif self.contact_reference is None:
            self.contact_reference = pose[:3].copy()
            self.stable_s = 0.0
        drift = (
            0.0
            if self.contact_reference is None
            else float(np.linalg.norm(pose[:3] - self.contact_reference))
        )
        retained = loaded and drift <= 0.0025
        if retained:
            self.stable_s += elapsed
        elif loaded:
            self.contact_reference = pose[:3].copy()
            self.stable_s = 0.0
        success = self.stable_s >= 0.6
        score = 1.0 if success else (0.5 if covers else 0.0)
        return {
            "success": bool(success),
            "score": score,
            "coupon_covers_contact": bool(covers),
            "coverage_margin_mm": round(margin * 1000, 3),
            "pressure_n": round(load, 2),
            "press_contacts_coupon": bool(touching),
            "tip_gap_mm": round(gap * 1000, 3),
            "knob_degrees": round(degrees(knob), 3),
            "retained": bool(retained),
            "finish_drift_mm": round(drift * 1000, 3),
            "hands_clear": bool(clear),
            "stable_s": round(self.stable_s, 3),
        }
