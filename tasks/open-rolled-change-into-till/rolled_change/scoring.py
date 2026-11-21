"""Score actual nickel containment and disposal of the emptied wrapper."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def inside_coin(pose):
    center = np.asarray(pose[:3])
    axis = quat_xyzw_to_matrix(pose[3:])[:, 1]
    half = C.COIN_THICKNESS / 2
    ext = C.COIN_RADIUS * np.sqrt(np.maximum(0, 1 - axis**2)) + half * abs(axis)
    low = [C.BIN_X[0], C.BIN_Y[0], C.BIN_FLOOR - 0.001]
    high = [C.BIN_X[1], C.BIN_Y[1], C.BIN_TOP]
    return bool(
        np.all(center - ext >= np.asarray(low) - 0.0007)
        and np.all(center + ext <= np.asarray(high) + 0.0007)
    )


def inside_scrap(pose):
    center = np.asarray(pose[:3])
    axis = quat_xyzw_to_matrix(pose[3:])[:, 1]
    ext = C.PAPER_RADIUS * np.sqrt(np.maximum(0, 1 - axis**2))
    ext += C.PAPER_LENGTH / 2 * abs(axis)
    low = [C.SCRAP_X[0], C.SCRAP_Y[0], C.SCRAP_FLOOR - 0.001]
    high = [C.SCRAP_X[1], C.SCRAP_Y[1], C.SCRAP_TOP]
    return bool(
        np.all(center - ext >= np.asarray(low) - 0.0007)
        and np.all(center + ext <= np.asarray(high) + 0.0007)
    )


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.stable_s = 0.0
        self.settled_centers = None

    def evaluate(self, state, elapsed):
        objects = state["objects"]
        count = sum(inside_coin(objects[name]["pose"]) for name in C.COINS)
        discarded = sum(inside_scrap(objects[name]["pose"]) for name in C.PAPER_NAMES)
        torn = state["torn"]
        if torn and count == 40 and discarded == 2:
            centers = np.array(
                [objects[name]["pose"][:3] for name in (*C.COINS, *C.PAPER_NAMES)]
            )
            # Use the observable position over a dwell. Thin coins at rest can
            # show brief solver velocity spikes from submillimeter contacts.
            if self.settled_centers is None or np.any(
                np.linalg.norm(centers - self.settled_centers, axis=1) > 0.002
            ):
                self.settled_centers = centers
                self.stable_s = 0.0
            else:
                self.stable_s += elapsed
        else:
            self.reset()
        success = self.stable_s + 1e-9 >= 0.6
        score = 0.0
        if torn:
            score = 0.2 + 0.6 * count / 40
        if success:
            score = 1.0
        return {
            "success": bool(success),
            "score": round(score, 4),
            "wrapper_torn": bool(torn),
            "torn_at_drawer_edge": state["torn_at_edge"],
            "max_seam_extension_mm": round(1000 * state["max_seam_strain_m"], 3),
            "coins_in_nickel_bin": count,
            "coins_not_in_nickel_bin": 40 - count,
            "wrapper_halves_in_scrap": discarded,
            "stable_s": round(self.stable_s, 3),
        }
