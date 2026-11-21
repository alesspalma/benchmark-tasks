"""Reduced tensile paper seam that loads the two native sleeve bodies."""

from math import cos, pi, sin

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def clip_length(value, maximum):
    return value * min(1.0, maximum / max(np.linalg.norm(value), 1e-12))


class PaperSeam:
    """Elastic fracture law at six sectors along the rolled paper rim."""

    def __init__(self):
        rim = np.array(
            [
                [C.PAPER_RADIUS * cos(angle), 0.0, C.PAPER_RADIUS * sin(angle)]
                for angle in np.linspace(0, 2 * pi, 6, endpoint=False)
            ]
        )
        self.anchors = np.stack((rim + [0, -C.HALF_Y, 0], rim + [0, C.HALF_Y, 0]))
        self.torn = False
        self.max_strain_m = 0.0
        self.tear_at_edge = False

    def wrenches(self, poses, velocities):
        if self.torn:
            return np.zeros((2, 3)), np.zeros((2, 3))
        rotations = [quat_xyzw_to_matrix(p[3:]) for p in poses]
        radii = np.stack([a @ r.T for a, r in zip(self.anchors, rotations)])
        points = poses[:, None, :3] + radii
        speed = velocities[:, None, :3] + np.cross(velocities[:, None, 3:], radii)
        gap = points[1] - points[0]
        strain = float(np.linalg.norm(gap, axis=1).max())
        self.max_strain_m = max(self.max_strain_m, strain)
        midpoint = points.mean(axis=(0, 1))
        at_edge = (
            abs(midpoint[0] - C.EDGE_X) < 0.020
            and abs(midpoint[1]) < 0.012
            and abs(midpoint[2] - C.EDGE_Z - C.PAPER_RADIUS) < 0.010
        )
        # The drawer lip concentrates deformation at the brittle crease.
        # Intact paper away from the lip sustains a larger extension.
        if strain >= (0.0036 if at_edge else 0.0085):
            self.tear_at_edge = bool(at_edge)
            self.torn = True
            return np.zeros((2, 3)), np.zeros((2, 3))
        tractions = np.stack(
            [
                clip_length(value, 3.8)
                for value in 700 * gap + 0.24 * (speed[1] - speed[0])
            ]
        )
        forces = np.stack((tractions, -tractions))
        return forces.sum(axis=1), np.cross(radii, forces).sum(axis=1)
