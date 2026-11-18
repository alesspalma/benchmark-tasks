"""Ordinary two-height grasp, followed by downward pressure and shaft twist."""
import numpy as np

from yam_arena.urdf import axis_angle_matrix

from . import config as C


def waypoints(mode="demo"):
    result = {}
    for arm, height, yaw in [("left", .94, -55), ("right", .79, 10)]:
        pos = np.r_[C.BUCKET[:2], height]
        approach = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
        pitch = axis_angle_matrix(np.array([0., 1., 0.]), np.deg2rad(30.))
        vertical = np.array([0., 0., 1.])
        angle = axis_angle_matrix(vertical, np.deg2rad(yaw))
        rotation = angle @ pitch @ approach
        moves = []

        def go(position, orientation, grip, duration, label):
            moves.append(dict(position=position, rotation=orientation, grip=grip, duration=duration, label=label))

        pre = pos - .085 * rotation[:, 2]
        go(pre, rotation, 1., 2.75, "approach")
        go(pos, rotation, 1., 1.15, "shaft")
        go(pos, rotation, 0., .70, "grasp")
        if mode == "hold":
            go(pos, rotation, 0., 6., "hold")
            result[arm] = moves
            continue
        pressed = pos - np.array([0, 0, .128])
        end = axis_angle_matrix(vertical, np.deg2rad(70.)) @ rotation
        go(pos - vertical * .085, rotation, 0., 2.15, "press")
        go(pressed, end, 0., 3.70, "wring")
        go(pressed, end, 0., 1.60, "drain")
        go(pressed + vertical * .24, end, 0., 2.4, "lift to drain")
        go(pressed + vertical * .24, end, 0., 2., "finish")
        result[arm] = moves
    return result
