"""Evaluate physical lace routing, retained instep compression, and release."""

import numpy as np

from . import config as c


class Scorer:
    def reset(self):
        self.dwell = 0.
        self.initial_gap = None

    def evaluate(self, state, elapsed):
        obj = state["objects"]["boot"]
        joints = obj["joints"]
        coords = obj["links"]
        routes = {}
        jaw_deflections = {}
        for side, direction in c.SIDES.items():
            opposite = "L" if side == "R" else "R"
            points = np.array([coords[f"{side}Lace{i:02d}"][:3] for i in range(c.LINK_COUNT)])
            lock = c.lock_center(opposite) + [0., joints["Flap"+opposite], 0.]
            frame = c.lock_axes(-direction)
            opening = joints["Jaw"+opposite]
            jaw_deflections[opposite] = opening
            relative = (points-lock) @ frame
            relative[:, 1] -= .5*opening
            seats = np.flatnonzero((np.abs(relative[:, :2]) < [.009, .003]).all(axis=1)
                                   & (relative[:, 2] > -.0045)
                                   & (relative[:, 2] < c.THROAT_TOP[opposite]+c.LACE_RADIUS+.0002))
            routes[side] = bool(.00035 < opening < .004 and any(
                k >= 2 and k < len(points)-2 and
                relative[k-2, 0] > relative[k, 0]+.007 and
                relative[-1, 0] < -.012
                for k in seats
            ))
        slack = abs(joints["FlapL"])+abs(joints["FlapR"])
        gap = 2*c.LOCK[1] + slack
        if self.initial_gap is None:
            self.initial_gap = gap
        tightening = self.initial_gap-gap
        cord = np.array([pose[:3] for name, pose in coords.items()
                         if name.startswith(("LLace", "RLace", "Bridge"))])
        instep = np.array([.416, 0., .866+c.BOOT_RISE])
        clear = True
        for hand in state["hands"].values():
            points = np.asarray(hand["points"])
            clear &= (hand["opening_m"] >= .025
                      and np.all(np.linalg.norm(points-instep, axis=1) >= .105)
                      and np.all(np.linalg.norm(points[:, None, :]-cord, axis=2) >= .020+c.LACE_RADIUS))
        retained = all(routes.values()) and gap <= .077 and tightening >= .006 and clear
        self.dwell = self.dwell+elapsed if retained else 0.
        success = self.dwell >= 1.
        return {
            "score": 1. if success else .35*sum(routes.values()),
            "success": success, "routed": routes,
            "panel_opening_m": slack, "hook_gap_m": float(gap),
            "tightening_m": float(tightening), "jaw_deflections_m": jaw_deflections,
            "hands_clear": clear, "retained_s": self.dwell,
        }
