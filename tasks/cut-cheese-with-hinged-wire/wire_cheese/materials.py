"""Finite cheese cohesion and resistance from the native sweeping wire."""

from math import cos, tan

import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C

WIDTH_SECTIONS = np.linspace(-0.47, 0.47, 7) * C.STOCK_SIZE[1]
BLOCK_HEIGHT = C.STOCK_SIZE[2]


def capped(value, limit):
    norm = np.linalg.norm(value)
    return value * min(1.0, limit / max(norm, 1e-9))


class Cheese:
    def __init__(self):
        self.reset()

    def reset(self):
        self.depth = np.zeros(len(WIDTH_SECTIONS))
        self.work = np.zeros(len(WIDTH_SECTIONS))
        self.severed = False
        self.peak_force = 0.0

    def advance(self, slicer, poses, velocities, angle, rate):
        stock, portion = poses
        stock_axis = quat_xyzw_to_matrix(stock[3:])
        board_axis = quat_xyzw_to_matrix(slicer[3:])
        # Sample the material interface across its width. Each sector must
        # experience its own continuous top-to-bottom wire crossing.
        points = np.column_stack(
            (
                np.full_like(WIDTH_SECTIONS, C.STOCK_SIZE[0] / 2 + C.GAP / 2),
                WIDTH_SECTIONS,
                np.zeros_like(WIDTH_SECTIONS),
            )
        )
        plane = (stock[:3] + points @ stock_axis.T - slicer[:3]) @ board_axis - C.PIVOT
        upright = (board_axis.T @ stock_axis)[:, 2]
        radius = -(plane[:, 1] + C.WIRE_Z * np.sin(angle)) / cos(angle)
        wire_z = C.WIRE_Z / cos(angle) + plane[:, 1] * tan(angle)
        depth = plane[:, 2] + BLOCK_HEIGHT / 2 - wire_z
        geometry = (
            (abs(plane[:, 0]) < 0.0035)
            & (radius > 0.008)
            & (radius < C.WIRE_LENGTH - 0.002)
            & (upright[2] > 0.975)
        )
        closing = np.maximum(-plane[:, 1] * rate / cos(angle) ** 2, 0.0)
        engaged = geometry & (depth > 0.0) & (depth < BLOCK_HEIGHT + 0.003)
        reactions = 2.35 + 0.55 * np.sin(np.pi * np.clip(depth / BLOCK_HEIGHT, 0, 1))
        reactions *= engaged * np.tanh(closing / 0.004)
        hinge_torque = 0.0
        if not self.severed:
            self.peak_force = max(self.peak_force, float(np.sum(reactions)))
            hinge_torque = float(np.dot(reactions, np.maximum(radius, 0.0)))
            advance = np.clip(depth, 0, BLOCK_HEIGHT) - self.depth
            # A jump around the cut, or stock moved under an already-lowered
            # wire, cannot silently claim the skipped material.
            inc = np.where(
                (advance > 0.0) & (advance < 0.0035) & (reactions > 0.9), advance, 0.0
            )
            self.depth += inc
            self.work += inc * reactions
            self.severed = bool(
                np.all(self.depth >= BLOCK_HEIGHT - 0.00075)
                and np.all(self.work > 0.048)
            )

        forces = np.zeros((2, 3))
        moments = np.zeros_like(forces)
        if not self.severed:
            arm = C.BOND_LENGTH * stock_axis[:, 0]
            desired = stock[:3] + arm - portion[:3]
            rate_error = (
                velocities[0, :3] + np.cross(velocities[0, 3:], arm) - velocities[1, :3]
            )
            force = capped(510.0 * desired + 1.40 * rate_error, 5.5)
            relative = (
                Rotation.from_quat(stock[3:]) * Rotation.from_quat(portion[3:]).inv()
            )
            moment = capped(
                0.026 * relative.as_rotvec()
                + 0.00031 * (velocities[0, 3:] - velocities[1, 3:]),
                0.020,
            )
            forces[:] = (-force, force)
            moments[:] = (-moment - np.cross(arm, force), moment)
        return forces, moments, hinge_torque

    def metrics(self):
        return {
            "severed": self.severed,
            "wire_depth_mm": round(float(1000 * np.min(self.depth)), 3),
            "wire_work_j": round(float(np.sum(self.work)), 5),
            "peak_wire_force_n": round(self.peak_force, 3),
            "cut_sectors": int(np.count_nonzero(self.depth >= BLOCK_HEIGHT - 0.00075)),
        }
