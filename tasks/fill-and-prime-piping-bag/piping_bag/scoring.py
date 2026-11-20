"""Score the mass released by a loaded blade passing through the pinched cuff."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def transform(pose):
    return np.asarray(pose[:3]), quat_xyzw_to_matrix(pose[3:])


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.transferred = np.zeros(C.SEGMENTS, dtype=bool)
        self.inserted = np.zeros(C.SEGMENTS, dtype=bool)
        self.previous_height = None
        self.finish_started = None

    def evaluate(self, state):
        spatula = state["objects"]["spatula"]
        bag = state["objects"]["bag"]
        origin, bag_rotation = transform(bag["links"]["Bagshell"])
        blade, blade_rotation = transform(spatula["links"]["Blade"])
        cuff_near, _ = transform(bag["links"]["Near"])
        cuff_far, _ = transform(bag["links"]["Far"])
        mouth = (cuff_far + cuff_near) / 2
        width = float(np.dot(cuff_far-cuff_near, bag_rotation[:, 1]))
        loads = [
            C.CUFF_K * abs(bag["joints"]["NearFlex"]),
            C.CUFF_K * abs(bag["joints"]["FarFlex"]),
        ]
        blade_force = C.FLEX_K * abs(spatula["joints"]["BladeFlex"])
        transverse = abs(np.dot(blade_rotation[:, 2], bag_rotation[:, 1]))
        blade_loaded_face = np.sign(np.dot(blade_rotation[:, 2], bag_rotation[:, 1]))
        flex_inward = np.sign(spatula["joints"]["BladeFlex"]) < 0
        points = np.column_stack((C.ICING_XS, np.zeros(C.SEGMENTS),
                                  np.full(C.SEGMENTS, C.BLADE_Z)))
        local = (blade + points @ blade_rotation.T - mouth) @ bag_rotation
        x_good = np.abs(local[:, 0]) < C.CUFF_HALF_LENGTH-C.BLADE_WIDTH/2+.006
        y_good = np.abs(local[:, 1]) < max(width/2+.003, .014)
        self.inserted |= x_good & y_good & (local[:, 2] < -.018)
        if self.previous_height is not None:
            # The loaded face moves outward past the pinched fold. Actual native
            # collar closure, blade loading and travel all have to coincide.
            pulling = local[:, 2] - self.previous_height > .00016
            wiping = (
                width < .018 and min(loads) > 1.5
                and 1.05 < blade_force < 22. and transverse > .80
                and flex_inward
            )
            if wiping:
                at_lip = np.abs(local[:, 2] - .009) < .012
                loaded_face_beside_lip = local[:, 1] * blade_loaded_face > -.007
                self.transferred |= (
                    self.inserted & x_good & y_good & pulling & at_lip
                    & loaded_face_beside_lip
                )
        self.previous_height = local[:, 2].copy()

        n = int(np.count_nonzero(self.transferred))
        cup = np.array(state["objects"]["cup"]["pose"][:3])
        bag_standing = (
            np.linalg.norm(origin[:2] - cup[:2]) < .029
            and np.dot(bag_rotation[:, 2], [0., 0., 1.]) > .94
            and C.TABLE + .145 < origin[2] < C.TABLE + .224
            and np.linalg.norm(bag["velocity"][:3]) < .025
        )
        pose, _ = transform(spatula["links"]["Handle"])
        handle = pose + transform(spatula["links"]["Handle"])[1] @ C.HANDLE_GRIP
        tool_parked = (
            np.linalg.norm(handle[:2] - (C.SPATULA + C.TOOL_START_R @ C.HANDLE_GRIP)[:2]) < .047
            and C.TABLE + .010 < handle[2] < C.TABLE + .043
            and np.linalg.norm(spatula["velocity"][:3]) < .025
        )
        hands_clear = all(
            min(np.linalg.norm(np.asarray(p)-origin), np.linalg.norm(np.asarray(p)-handle)) > .065
            for hand in state["hands"].values() for p in hand["points"]
        )
        complete = n >= 7 and bag_standing and tool_parked and hands_clear
        if complete:
            if self.finish_started is None:
                self.finish_started = state["time_s"]
        else:
            self.finish_started = None
        held = (state["time_s"] - self.finish_started) if self.finish_started is not None else 0.
        score = 1. if held >= .999 else .8 * n / C.SEGMENTS
        return {
            "score": score, "success": score == 1., "icing_in_bag_g": C.ICING_MASS_G*n/C.SEGMENTS,
            "transferred_segments": n, "inserted_segments": int(self.inserted.sum()),
            "cuff_gap_mm": width*1e3, "cuff_load_n": loads, "blade_load_n": blade_force,
            "blade_flex_mm": spatula["joints"]["BladeFlex"]*1e3,
            "blade_in_cuff": local.tolist(), "bag_standing": bool(bag_standing),
            "tool_parked": bool(tool_parked), "hands_clear": bool(hands_clear), "rest_s": held,
        }
