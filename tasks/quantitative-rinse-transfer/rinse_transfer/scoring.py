"""Observable recovered dye fraction, rinse budget, and terminal placement."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Scorer:
    def reset(self):
        self.stable = 0.0

    def evaluate(self, state):
        mat = state["material"]
        parked = True
        for name, home in (("beaker", C.BEAKER), ("bottle", C.BOTTLE)):
            obj = state["objects"][name]
            pose = np.asarray(obj["pose"])
            rotation = quat_xyzw_to_matrix(pose[3:])
            parked &= (
                np.linalg.norm(pose[:2] - home[:2]) < 0.045
                and abs(pose[2] - C.TOP) < 0.006
                and rotation[2, 2] > 0.985
                and np.linalg.norm(obj["velocity"][:3]) < 0.022
            )
        ends = [np.asarray(pt) for h in state["hands"].values() for pt in h["points"]]
        clear = all(
            np.linalg.norm(np.asarray(center) + [0, 0, 0.04] - p) > 0.050
            for center in (C.BEAKER, C.BOTTLE, C.FUNNEL)
            for p in ends
        )
        self.stable = self.stable + 0.05 if parked and clear else 0.0
        recovered = mat["recovered_dye"]
        valid = mat["spilled_ml"] <= 3.0 and mat["rinse_ml"] <= 25.0
        full = valid and recovered >= 0.90 and self.stable >= 0.40
        score = 1.0 if full else (0.5 if valid and recovered >= 0.65 else 0.0)
        return dict(
            score=score,
            success=bool(full),
            parked=bool(parked),
            clear=bool(clear),
            **mat,
        )
