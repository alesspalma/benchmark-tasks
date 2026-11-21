"""Visible roll support, paper length at the outlet, and closed cover."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def at_seat(roll):
    pose = np.asarray(roll["pose"])
    rot = quat_xyzw_to_matrix(pose[3:])
    return bool(
        np.all(np.abs(pose[:3] - C.SEAT) <= (0.025, 0.019, 0.022))
        and abs(rot[1, 1]) >= 0.972
    )


def tip_from(roll):
    p = np.asarray(roll["links"]["FreeEdge"])
    r = quat_xyzw_to_matrix(p[3:])
    return p[:3] - C.PAPER_LENGTH * r[:, 2]


def routed_tip(roll, engaged):
    """Map the physically paid-out length onto the paper's visible guide path."""
    if not engaged:
        return tip_from(roll)
    amount = max(0.0, -roll["joints"]["Travel"])
    if amount < C.NIP_TO_OUTLET:
        mouth_z = C.NIP_Z + 0.016
        z = mouth_z + (C.OUTLET_Z - mouth_z) * amount / C.NIP_TO_OUTLET
    else:
        z = C.OUTLET_Z - (amount - C.NIP_TO_OUTLET)
    return np.array([C.NIP_X, roll["pose"][1], z])


class Scorer:
    def reset(self):
        self.final_s = 0.0

    def evaluate(self, state, elapsed):
        roll = state["objects"]["roll"]
        m = state["objects"]["mechanism"]["joints"]
        seated = at_seat(roll) and np.linalg.norm(roll["velocity"][:3]) < 0.028
        tip = routed_tip(roll, state["engaged"])
        outlet = abs(tip[0] - C.NIP_X) <= 0.021 and abs(tip[1] - C.SEAT[1]) <= 0.033
        exposed = max(0.0, C.OUTLET_Z - tip[2]) if seated and outlet else 0.0
        hinge_closed = abs(m["Hinge"]) <= 0.037
        hands = np.array(
            [point for hand in state["hands"].values() for point in hand["points"][:2]]
        )
        working = (
            (hands[:, 0] >= C.COVER_HINGE[0] - 0.081)
            & (hands[:, 0] <= 0.690)
            & (np.abs(hands[:, 1] - C.SEAT[1]) <= 0.182)
            & (hands[:, 2] >= C.OUTLET_Z - 0.156)
            & (hands[:, 2] <= C.COVER_HINGE[2] + C.COVER_HEIGHT + 0.043)
        )
        clear = not bool(working.any())
        score = 0.0
        if seated:
            score = 0.25
        if exposed >= 0.025:
            score = 0.50
        if exposed >= 0.070:
            score = 0.75
        passed = exposed >= 0.070 and hinge_closed and state["latched"] and clear
        self.final_s = self.final_s + elapsed if passed else 0.0
        if self.final_s + 1e-8 >= 0.5:
            score = 1.0
        return {
            "score": score,
            "success": score == 1.0,
            "roll_seated": seated,
            "paper_tip_mm": (tip * 1000).tolist(),
            "visible_tail_mm": 1000 * exposed,
            "cover_angle_deg": float(np.rad2deg(m["Hinge"])),
            "cover_latched": state["latched"],
            "paddle_mm": 1000 * m["Stroke"],
            "payout_mm": -1000 * roll["joints"]["Travel"],
            "nip_engaged": state["engaged"],
            "hands_clear": clear,
            "final_s": self.final_s,
        }
