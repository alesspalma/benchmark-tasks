"""Score the visible mixing outcome and the released upright sample."""
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.upright_s = 0.
        self.lost = False

    def evaluate(self, state):
        tube = state["objects"]["tube"]
        position = np.asarray(tube["pose"][:3])
        rotation = quat_xyzw_to_matrix(tube["pose"][3:])
        material = state["material"]
        upright = rotation[2, 2] >= np.cos(np.deg2rad(12.))
        near_mixer = np.linalg.norm(position[:2] - C.MIXER[:2]) <= .23
        supported = abs(position[2] - C.TOP) <= .004
        still = np.linalg.norm(tube["velocity"][:3]) < .012
        end = position + rotation[:, 2] * C.TUBE_HEIGHT
        clearance = min(
            self._distance_to_segment(point, position, end)
            for hand in state["hands"].values() for point in hand["points"]
        )
        released = clearance >= .047
        in_place = upright and near_mixer and supported and still and released and not material["motor_on"]
        self.upright_s = self.upright_s + .05 if in_place else 0.
        self.lost |= bool(position[2] < C.TOP - .04)
        reduction = 1. - material["contrast"]
        if self.lost or reduction < .40:
            score = 0.
        elif reduction < .90:
            score = .25
        else:
            score = 1. if self.upright_s >= .999 else .5
        return {
            "score": score,
            "success": score == 1.,
            "contrast_reduction": reduction,
            "upright_released": bool(in_place),
            "tube_lost": self.lost,
            "motor_on": material["motor_on"],
            "coupled_time_s": material["contact_s"],
            "peak_coupling_orbit_m": material["max_tube_orbit_m"],
            "peak_press_depth_m": material["max_press_m"],
            "cup_joints": state["objects"]["mixer"]["joints"],
            "t_s": state["time_s"],
        }

    @staticmethod
    def _distance_to_segment(point, a, b):
        delta = b - a
        u = np.clip((np.asarray(point) - a) @ delta / (delta @ delta), 0., 1.)
        return np.linalg.norm(np.asarray(point) - a - u * delta)
