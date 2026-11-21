"""Judge actual seating on the retained swivel after releasing both arms."""

import numpy as np

from .detent import frame


class Scorer:
    def reset(self):
        self.stable_s = 0.0

    def evaluate(self, state, elapsed):
        objects = state["objects"]
        swivel = np.asarray(objects["chair"]["links"]["Swivel"])
        old = np.asarray(objects["old_cap"]["pose"])
        new = np.asarray(objects["new_cap"]["pose"])
        seat, axes, orientation, _, _, gaps = frame(swivel, new)
        max_gap = np.max(np.abs(gaps @ axes), axis=0)
        aligned = np.linalg.norm(max_gap[:2]) <= 0.0026
        flush = (
            aligned and max_gap[2] <= 0.0018 and orientation[:, 2] @ axes[:, 2] > 0.986
        )
        old_removed = np.linalg.norm(old[:3] - seat) > 0.045 and not any(
            state["clips"]["old_cap"]
        )
        retained = bool(flush and all(state["clips"]["new_cap"]))
        clear = all(
            np.linalg.norm(np.asarray(point) - seat) > 0.056
            for arm in state["hands"].values()
            for point in arm["points"]
        )
        positive = old_removed and retained and clear
        self.stable_s = self.stable_s + elapsed if positive else 0.0
        success = self.stable_s >= 0.60
        return {
            "success": bool(success),
            "score": 1.0 if success else 0.35 if old_removed else 0.0,
            "old_removed": bool(old_removed),
            "new_flush": bool(flush),
            "new_retained": retained,
            "new_radial_error_mm": round(float(np.linalg.norm(max_gap[:2])) * 1000, 3),
            "new_largest_rim_gap_mm": round(float(max_gap[2]) * 1000, 3),
            "new_clips": sum(state["clips"]["new_cap"]),
            "old_clips": sum(state["clips"]["old_cap"]),
            "swivel_angle_deg": round(
                float(np.rad2deg(objects["chair"]["joints"]["GlideSwivel"])), 2
            ),
            "stable_s": round(self.stable_s, 3),
            "both_clear": bool(clear),
        }

    def __init__(self):
        self.reset()
