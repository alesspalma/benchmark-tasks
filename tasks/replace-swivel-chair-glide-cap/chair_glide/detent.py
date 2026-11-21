"""Four elastic rim sectors connecting each cap to the native swivel."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C

SEAT_IN_SWIVEL = np.array([0.0, 0.0, 0.012 + C.CAP_HEIGHT])


def frame(swivel_pose, cap_pose):
    swivel_axes = quat_xyzw_to_matrix(swivel_pose[3:])
    cap_axes = quat_xyzw_to_matrix(cap_pose[3:])
    seat = swivel_pose[:3] + swivel_axes @ SEAT_IN_SWIVEL
    local_x = swivel_axes.T @ cap_axes[:, 0]
    azimuth = np.arctan2(local_x[1], local_x[0])
    c, sn = np.cos(azimuth), np.sin(azimuth)
    circular_frame = np.array([[c, -sn, 0], [sn, c, 0], [0, 0, 1]])
    glide_arms = (C.SNAP_ANCHORS @ circular_frame.T + SEAT_IN_SWIVEL) @ swivel_axes.T
    cap_arms = C.SNAP_ANCHORS @ cap_axes.T
    gap = cap_pose[:3] + cap_arms - (swivel_pose[:3] + glide_arms)
    return seat, swivel_axes, cap_axes, cap_arms, glide_arms, gap


class Rim:
    """Finite forces represent the elastic lips inside a molded round skirt."""

    def __init__(self, engaged):
        self.clips = np.full(4, engaged)
        self.approached = bool(engaged)
        self.release_height = 0.00365 if engaged else 0.0097
        self.capture_height = 0.00145 if engaged else 0.0088
        self.entry_force = 0.58 if engaged else 0.12

    def wrench(self, swivel_pose, swivel_velocity, cap_pose, cap_velocity):
        seat, axes, cap_axes, cap_arms, glide_arms, gap = frame(swivel_pose, cap_pose)
        centres = (cap_pose[:3] - seat) @ axes
        local_gap = gap @ axes
        axial = local_gap[:, 2]
        lateral = np.linalg.norm(local_gap[:, :2], axis=1)
        axis_dot = cap_axes[:, 2] @ axes[:, 2]
        if (
            centres[2] > 0.0042
            and np.linalg.norm(centres[:2]) < 0.009
            and axis_dot > 0.92
        ):
            self.approached = True
        if np.linalg.norm(centres) > 0.04:
            self.approached = False
        # A clipped sector must be lifted over its own lip. Tilting the cap in
        # a pry progressively unloads sectors rather than releasing a constraint
        # on a tool position or a gripper command.
        self.clips &= (
            (axial < self.release_height) & (lateral < 0.0060) & (axis_dot > 0.86)
        )
        if self.approached and axis_dot > 0.952:
            self.clips |= (
                (axial < self.capture_height) & (axial > -0.002) & (lateral < 0.0028)
            )

        cap_speed = cap_velocity[:3] + np.cross(cap_velocity[3:], cap_arms)
        glide_speed = swivel_velocity[:3] + np.cross(swivel_velocity[3:], glide_arms)
        relative_speed = (cap_speed - glide_speed) @ axes
        spring = -local_gap * [185.0, 185.0, 390.0] - relative_speed * [
            0.53,
            0.53,
            0.88,
        ]
        local_force = np.clip(spring, -1.55, 1.55) * self.clips[:, None]
        if self.approached and axis_dot > 0.91:
            ramp = (
                ~self.clips
                & (lateral < 0.0028)
                & (axial >= self.capture_height)
                & (axial < self.capture_height + 0.00245)
            )
            local_force[:, 2] += (
                self.entry_force
                * ramp
                * (self.capture_height + 0.00245 - axial)
                / 0.00245
            )
        world_force = local_force @ axes.T
        cap_torque = np.cross(cap_arms, world_force).sum(axis=0)
        glide_torque = -np.cross(glide_arms, world_force).sum(axis=0)
        return world_force.sum(axis=0), cap_torque, glide_torque
