"""Read native tube shape, cam closure and spring compression for a water check."""

import math

import numpy as np

from . import config as C


def inspect_tube(links, lane):
    points = np.array([links[f"Link{i}"][:3] for i in range(C.LINKS)])
    points = points[np.argsort(points[:, 1])]
    y = points[:, 1]
    sample_y = C.HEAD[1] + np.array([-0.046, -0.022, 0, 0.022, 0.046])
    x, z = [np.interp(sample_y, y, points[:, axis]) for axis in (0, 2)]
    error_mm = 1e3 * np.max(np.abs(x - C.HEAD[0] - lane))
    height_mm = 1e3 * np.max(np.abs(z - C.TABLE - C.BED_Z - C.TUBE_R))
    spans = y[0] < sample_y[0] - 0.03 and y[-1] > sample_y[-1] + 0.03
    return bool(spans and error_mm < 4.1 and height_mm < 5.0), float(error_mm), float(height_mm)


class PumpCheck:
    def __init__(self):
        self.volumes = np.zeros(2)
        self.settled = 0

    def observe(self, state):
        joints = state["objects"]["head"]["joints"]
        angle = float(joints["CamClose"])
        closed = abs(angle) < math.radians(4)
        shafts = [state["objects"][n]["links"] for n in C.TUBE_NAMES]
        direct = [inspect_tube(shaft, lane) for shaft, lane in zip(shafts, C.LANES)]
        swapped = [inspect_tube(shaft, lane) for shaft, lane in zip(shafts[::-1], C.LANES)]
        aligned = max((direct, swapped), key=lambda checks: (sum(c[0] for c in checks), -sum(c[1] for c in checks)))
        compressed = [float(joints[f"Compression{i}"]) for i in range(2)]
        clear = all(
            all(np.linalg.norm(np.asarray(p) - np.array([C.HEAD[0], C.HEAD[1], C.TABLE + C.BED_Z])) > 0.095
                for p in hand["points"])
            for hand in state["hands"].values()
        )
        active = np.array([closed and ok and 0.0012 < travel < 0.0065 for (ok, _, _), travel in zip(aligned, compressed)])
        self.settled = self.settled + 1 if clear and active.any() else 0
        if self.settled > 10:
            # The evaluator's guarded water check uses 1.7 mL/rev at 24 rpm.
            self.volumes += active * (1.7 * 24 / 60 / 20)
        passed = active & (self.volumes >= 1.0)
        score = float(0.5 * passed.sum()) if clear else 0.0
        return {"score": score, "success": score == 1.0, "valve_angle_deg": math.degrees(angle),
                "track_lateral_error_mm": [c[1] for c in aligned], "track_height_error_mm": [c[2] for c in aligned],
                "occlusion_mm": [1e3 * x for x in compressed], "water_ml": self.volumes.tolist(), "hands_clear": clear}
