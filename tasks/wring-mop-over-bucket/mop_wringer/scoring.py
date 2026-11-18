"""Mass-conserving approximation of a cone squeezing water from rayon."""
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.caught = 0.
        self.spill = 0.
        self.previous_angle = None
        self.peak_squeeze = 0.
        self.highest_force = 0.
        self.hold_s = 0.
        self.last_outflow_g = 0.
        self.initial_compression = None

    def evaluate(self, state):
        mop = state["objects"]["mop"]
        bucket = state["objects"]["bucket"]
        center = np.asarray(mop["pose"][:3])
        vessel = np.asarray(bucket["pose"][:3])
        rot = quat_xyzw_to_matrix(mop["pose"][3:])
        bucket_rot = quat_xyzw_to_matrix(bucket["pose"][3:])
        relative = bucket_rot.T @ rot
        angle = np.arctan2(relative[1, 0], relative[0, 0])
        moving = self.previous_angle is not None
        turn = (angle - self.previous_angle + np.pi) % (2 * np.pi) - np.pi if moving else 0.
        self.previous_angle = angle
        depths = np.maximum(-np.asarray(list(mop["joints"].values())), 0)
        compression = float(np.mean(depths))
        if self.initial_compression is None:
            self.initial_compression = compression
        extra = max(0., compression - self.initial_compression - .006)
        self.highest_force = max(self.highest_force, C.BUNDLE_STIFFNESS * float(np.sum(depths)))
        upright = rot[2, 2] > .93
        bucket_upright = bucket_rot[2, 2] > .97
        in_cone = np.linalg.norm(center[:2] - vessel[:2]) < .048 and center[2] < vessel[2] + .390
        delta_g = 0.
        if in_cone and upright and bucket_upright:
            # New peak squeeze and further twist under load both reduce retained water.
            work = 6900. * max(0., extra - self.peak_squeeze)
            work += 135. * abs(turn) * min(1., extra / .012)
            self.peak_squeeze = max(self.peak_squeeze, extra)
            delta_g = min(C.WATER_START_G - self.caught - self.spill, work)
            self.caught += delta_g
        if bucket_rot[2, 2] < .5:
            self.spill += self.caught
            self.caught = 0.
        self.last_outflow_g = delta_g
        safe_draining = upright and bucket_upright and center[2] >= vessel[2] + .405 and np.linalg.norm(center[:2] - vessel[:2]) < .10
        self.hold_s = self.hold_s + .05 if safe_draining and self.caught >= C.WRING_G else 0.
        score = 0. if self.spill > C.WATER_ALLOW_G else (
            1. if self.hold_s >= .45 else .5 if self.caught >= C.WRING_G else .25 if self.caught >= 40. else 0.
        )
        return {
            "score": score, "success": score == 1.,
            "caught_g": self.caught, "remaining_g": C.WATER_START_G - self.caught - self.spill,
            "spill_g": self.spill, "outflow_g": delta_g, "peak_squeeze_m": self.peak_squeeze,
            "spring_load_sum_n": self.highest_force, "compression_m": compression,
            "shaft_z_m": float(center[2]), "shaft_yaw_deg": float(np.rad2deg(angle)),
            "tilt_deg": float(np.rad2deg(np.arccos(np.clip(rot[2, 2], -1., 1.)))),
            "head_at_bucket": bool(in_cone), "finish_clear": bool(safe_draining),
            "t_s": state["time_s"],
        }
