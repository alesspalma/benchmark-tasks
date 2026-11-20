"""Evaluate physical lace routing, retained instep compression, and release."""

import numpy as np

from . import config as c


class Scorer:
    def reset(self):
        self.dwell = 0.

    def evaluate(self, state, elapsed):
        obj = state["objects"]["boot"]
        joints = obj["joints"]
        coords = obj["links"]
        routes = {}
        jaw_deflections = {}
        for side, direction in c.SIDES.items():
            opposite = "L" if side == "R" else "R"
            points = np.array([coords[f"{side}Lace{i:02d}"][:3] for i in range(c.LINK_COUNT)])
            lock = c.LOCK * [1., -direction, 1.] + [0., joints["Flap"+opposite], 0.]
            frame = c.lock_axes(-direction)
            opening = joints["Jaw"+opposite]
            jaw_deflections[opposite] = opening
            relative = (points-lock) @ frame
            relative[:, 1] -= .5*opening
            seats = np.flatnonzero((np.abs(relative)<[.009, .003, .0045]).all(axis=1))
            routes[side] = bool(.00035 < opening < .004 and any(
                k >= 2 and k < len(points)-2 and
                relative[k-2, 0] > relative[k, 0]+.007 and
                relative[-1, 0] < -.012
                for k in seats
            ))
        slack = abs(joints["FlapL"])+abs(joints["FlapR"])
        clear = all(
            np.linalg.norm(np.asarray(point)-np.array([.416, 0., .866+c.BOOT_RISE])) >= .105
            for hand in state["hands"].values() for point in hand["points"]
        )
        retained = all(routes.values()) and slack <= .010 and clear
        self.dwell = self.dwell+elapsed if retained else 0.
        success = self.dwell >= 1.
        return {
            "score": 1. if success else .35*sum(routes.values()),
            "success": success, "routed": routes,
            "panel_opening_m": slack, "jaw_deflections_m": jaw_deflections,
            "hands_clear": clear, "retained_s": self.dwell,
        }
