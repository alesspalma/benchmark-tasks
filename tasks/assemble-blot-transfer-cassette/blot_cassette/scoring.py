"""Reduced-order air response driven by native roller/body measurements."""

import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.exposure = np.zeros(len(C.GAPS))
        self.last_spin = None
        self.last_y = None
        self.hold = 0

    def evaluate(self, state):
        cassette = state["objects"]["cassette"]
        roller = state["objects"]["roller"]
        shear = cassette["joints"]["Shear"]
        compression = max(0., -cassette["joints"]["Compression"])
        closed = cassette["joints"]["Closing"] < -.123
        axle = np.asarray(roller["links"]["Drum"][:3])
        spin = roller["joints"]["Spin"]
        dy = abs(axle[1] - self.last_y) if self.last_y is not None else 0.
        turn = abs(spin - self.last_spin) if self.last_spin is not None else 0.
        self.last_spin, self.last_y = spin, axle[1]
        layer_z = cassette["links"]["Layer"][2] + .007
        rolling = (
            .00055 <= compression <= .0048
            and abs(axle[2] - C.ROLLER_R - layer_z) <= .0008
            and abs(shear) <= .004
            and dy >= .00015
            and abs(axle[1] - C.ORIGIN[1] - shear) <= .042
            and 0.10 * dy <= C.ROLLER_R * turn <= 3.5 * max(dy, .00035)
        )
        if rolling:
            for k, (gx, gy, radius) in enumerate(C.GAPS):
                if (
                    abs(axle[0] - (C.ORIGIN[0] + gx)) < C.ROLLER_WIDTH / 2 - radius
                    and abs(axle[1] - (C.ORIGIN[1] + shear + gy)) <= radius
                ):
                    self.exposure[k] += min(dy, C.ROLLER_R * turn)
        cleared = [distance >= .004 for distance in self.exposure]
        aligned = abs(shear) <= .003
        self.hold = self.hold + 1 if closed and aligned and all(cleared) else 0
        complete = self.hold >= 10
        score = 1. if complete else .25 * sum(cleared)
        return {
            "score": score, "success": complete, "gaps_cleared": cleared,
            "weighted_roll_mm": (1000 * self.exposure).round(4).tolist(),
            "laminate_shear_mm": 1000 * shear, "compression_mm": 1000 * compression,
            "lid_travel_mm": -1000 * cassette["joints"]["Closing"],
            "roller_xyz": axle.tolist(), "roll_rad": spin, "rolling": bool(rolling),
        }
