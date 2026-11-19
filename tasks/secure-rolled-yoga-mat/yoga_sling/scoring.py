"""Native, symmetric retention evidence from the sling and mat poses."""

import numpy as np
from yam_arena.ik import quat_xyzw_to_matrix

from . import config as c


class Scorer:
    def reset(self):
        self.hold = 0
        self.proved = False

    def evaluate(self, state):
        mat = state["objects"]["mat"]
        sling = state["objects"]["sling"]
        center = np.asarray(mat["pose"][:3])
        axis = quat_xyzw_to_matrix(np.asarray(mat["pose"][3:]))[:, 1]
        band_states = {}
        for name in ("Left", "Right"):
            top = np.asarray(sling["links"][name + "BandTop"])
            displacement = top[:3] - center
            y = float(displacement @ axis)
            transverse = float(np.linalg.norm(displacement - axis * y))
            alignment = float(abs(quat_xyzw_to_matrix(top[3:])[:, 1] @ axis))
            springback = mat["joints"]["Springback"]
            band_states[name] = {"axial_m": y, "radial_m": transverse,
                                 "parallel": alignment, "springback_m": springback,
                                 "retains": bool(.160 <= abs(y) <= .265 and transverse < .021
                                                 and alignment > .91 and springback < .009)}
        signed = [band_states[n]["axial_m"] for n in ("Left", "Right")]
        n = sum(int(d["retains"]) for d in band_states.values())
        if n == 2 and signed[0] * signed[1] >= 0:
            n = 1
        lifting = n == 2 and center[2] > c.MAT_Z + .035
        self.hold = self.hold + 1 if lifting else 0
        self.proved = self.proved or self.hold >= 12
        released = all(np.linalg.norm(np.asarray(hand["points"][0]) - center) > .11
                       for hand in state["hands"].values())
        success = n == 2 and self.proved and released and center[2] < c.MAT_Z + .021
        return {"score": 1. if success else n * .25, "success": success,
                "load_lift_proved": self.proved, "band_states": band_states,
                "mat_height_m": float(center[2])}
