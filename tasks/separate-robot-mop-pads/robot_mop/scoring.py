"""Inspection of native part positions and released sorting outcomes."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.hold = 0.0

    def evaluate(self, state, elapsed):
        objects = state["objects"]
        chassis = objects["turnover"]
        angle = float(np.degrees(chassis["joints"]["Turn"]))
        body = np.asarray(chassis["links"]["Chassis"])
        axes = quat_xyzw_to_matrix(body[3:])
        inverted = axes[2, 2] <= -np.cos(np.radians(8.0))
        hands = np.asarray(
            [p for arm in state["hands"].values() for p in arm["points"]]
        )
        parts = {}
        for side, mount_y in (("L", C.PAD_Y), ("R", -C.PAD_Y)):
            carrier = np.asarray(objects[side + "Carrier"]["pose"][:3])
            fabric = np.asarray(
                [
                    objects[side + "Fabric" + end]["pose"][:3]
                    for end in ("Front", "Back")
                ]
            )
            cloth = fabric.mean(axis=0)
            dock = body[:3] + axes @ [C.PAD_X, mount_y, C.PLATE_Z]
            off = np.linalg.norm(carrier - dock) > 0.046
            peel = off and np.linalg.norm(cloth - carrier) > 0.068
            hard_sorted = off and self.in_tray(carrier, C.CARRIER_TRAY_X)
            soft_sorted = peel and all(self.in_tray(p, C.CLOTH_TRAY_X) for p in fabric)
            still = all(
                np.linalg.norm(np.asarray(objects[name]["velocity"])[:3]) < 0.029
                for name in (
                    side + "Carrier",
                    side + "FabricFront",
                    side + "FabricBack",
                )
            )
            clear = (
                np.linalg.norm(
                    hands[:, None, :] - np.vstack([carrier, fabric]), axis=2
                ).min()
                > 0.069
            )
            parts[side] = {
                "carrier_off": bool(off),
                "peeled": bool(peel),
                "hard_sorted": bool(hard_sorted),
                "cloth_sorted": bool(soft_sorted),
                "released": bool(still and clear),
            }
        complete = bool(
            inverted
            and all(
                p["hard_sorted"] and p["cloth_sorted"] and p["released"]
                for p in parts.values()
            )
        )
        self.hold = self.hold + elapsed if complete else 0.0
        score = 0.15 if inverted else 0.0
        for info in parts.values():
            score += 0.15 * info["carrier_off"] + 0.15 * info["peeled"]
        success = self.hold >= 0.5 - 1e-9
        return {
            "score": 1.0 if success else round(score, 2),
            "success": success,
            "turnover_deg": angle,
            "inverted": bool(inverted),
            "parts": parts,
            "released_s": self.hold,
        }

    @staticmethod
    def in_tray(point, center_x):
        x, yy, z = point
        if not C.SORT_Z - 0.003 <= z <= C.SORT_Z + 0.042:
            return False
        if center_x == C.CLOTH_TRAY_X:
            return (
                abs(x - center_x) <= C.CLOTH_TRAY_WIDTH / 2 - 0.069
                and abs(yy) <= C.CLOTH_TRAY_LENGTH / 2 - 0.068
            )
        return abs(x - center_x) <= 0.058 and abs(abs(yy) - C.SORT_Y) <= 0.046
