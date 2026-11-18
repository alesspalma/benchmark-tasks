"""Reach, load, then operate the ordinary crusher lever by its grip."""

import numpy as np

from . import config as C
from .motion import Program, rotation

CAN_ORIENTATION = rotation("Z", np.deg2rad(-38.)) @ rotation("Y", np.deg2rad(17.)) @ np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
BAR_ORIENTATION = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
CAN_HELD = .37
BAR_HELD = .09


def can_pose(bottom, grip=CAN_HELD):
    return np.asarray(bottom) + [0., 0., .108], CAN_ORIENTATION, grip


def lever_pose(degrees, grip=BAR_HELD):
    pitch = np.interp(degrees, [-176., -60., -22., 0.], [35., 35., 15., 5.])
    yaw = np.interp(degrees, [-175., -140., -100., -65., -25., 0.],
                    [42., 90., 90., 90., 60., 29.])
    orient = rotation("Z", np.deg2rad(yaw)) @ rotation("Y", np.deg2rad(pitch)) @ BAR_ORIENTATION
    centre = C.PIVOT + rotation("Y", np.deg2rad(degrees)) @ [0., -.025, C.LEVER_LENGTH]
    return centre, orient, grip


def away(pose, distance, height=0.):
    p, r, g = pose
    return p - distance * r[:, 2] + [0., 0., height], r, g


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    p.add(2., "approach the loose can", left=away(can_pose(C.CAN_START, 1.), .072, .022))
    p.add(.95, "align with the can shoulder", left=can_pose(C.CAN_START, 1.))
    p.add(.95, "grasp the can", left=can_pose(C.CAN_START))
    p.add(.85, "lift from the loading shelf", left=can_pose(C.CAN_START+[0., 0., .038]))
    p.add(1.7, "center the can in front of the open cradle", left=can_pose(C.LOAD+[-.110, 0., .004]))
    p.add(1.7, "place the can between the plates", left=can_pose(C.LOAD))
    p.add(.85, "release the supported can", left=can_pose(C.LOAD, 1.))
    p.add(1.1, "clear the pinch area", left=away(can_pose(C.LOAD, 1.), .10, .014))
    p.add(1.3, "park the loading hand", left=(p.pose["left"][0] + [0., .17, .13], CAN_ORIENTATION, 1.))
    if mode == "load-only":
        p.add(1.0, "rest with the lever open")
        return p.moves
    p.add(1.8, "approach the lever grip", right=away(lever_pose(0., 1.), .079))
    p.add(.8, "align around the lever grip", right=lever_pose(0., 1.))
    p.add(.9, "grasp the handle", right=lever_pose(0.))
    angles = np.linspace(0., -175., 20)
    for degrees in angles[1:]:
        p.add(.45, "pull the lever through the loaded stroke", right=lever_pose(degrees))
    p.add(.55, "finish the compaction", right=lever_pose(-176.))
    for degrees in angles[-2::-1]:
        p.add(.45, "raise the lever and release the flattened can", right=lever_pose(degrees))
    p.add(.70, "release the returned handle", right=lever_pose(0., 1.))
    p.add(.85, "withdraw from the cleared crusher", right=away(lever_pose(0., 1.), .068))
    return p.moves
