"""Reach, load, then operate the ordinary crusher lever by its grip."""

import numpy as np

from . import config as C
from .motion import Program, rotation

FRONT_ORIENTATION = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
CAN_YAW = rotation("Z", np.deg2rad(-38.))
CAN_ORIENTATION = CAN_YAW @ rotation("Y", np.deg2rad(17.)) @ FRONT_ORIENTATION
PUCK_ORIENTATION = CAN_YAW @ rotation("Y", np.deg2rad(36.)) @ FRONT_ORIENTATION
BAR_ORIENTATION = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
CAN_HELD = .37
BAR_HELD = .09
PULL_PHASE = "pull the lever through the loaded stroke"
FINISH_PHASE = "finish the compaction"


def can_pose(bottom, grip=CAN_HELD):
    return np.asarray(bottom) + [0., 0., .100], CAN_ORIENTATION, grip


def puck_pose(bottom, grip=CAN_HELD):
    return np.asarray(bottom) + [0., 0., .024], PUCK_ORIENTATION, grip


def lever_pose(degrees, grip=BAR_HELD, side="right"):
    pitch = np.interp(degrees, [-118., -60., -22., 0.], [35., 35., 15., 5.])
    yaw = np.interp(degrees, [-118., -65., -25., 0.], [90., 90., 60., 29.])
    orient = rotation("Z", np.deg2rad(yaw)) @ rotation("Y", np.deg2rad(pitch)) @ BAR_ORIENTATION
    centre = C.PIVOT + rotation("Y", np.deg2rad(degrees)) @ [0., -.049, C.LEVER_LENGTH]
    if side == "left":
        centre[1] *= -1.
        orient = np.diag([1., -1., 1.]) @ orient @ np.diag([-1., 1., 1.])
    return centre, orient, grip


def away(pose, distance, height=0.):
    p, r, g = pose
    return p - distance * r[:, 2] + [0., 0., height], r, g


def handle_targets(degrees, grip=BAR_HELD, retreat=0.):
    return {side: away(lever_pose(degrees, grip, side), retreat) for side in C.BASES}


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    p.add(2., "approach the loose can", left=away(can_pose(C.CAN_START, 1.), .072, .022))
    p.add(.95, "align with the can shoulder", left=can_pose(C.CAN_START, 1.))
    p.add(.95, "grasp the can", left=can_pose(C.CAN_START))
    p.add(.85, "lift from the loading shelf", left=can_pose(C.CAN_START+[0., 0., .038]))
    p.add(1.7, "center the can in front of the open cradle", left=can_pose(C.LOAD+[-.110, 0., .016]))
    p.add(1.9, "feed the can horizontally above the cradle", left=can_pose(C.LOAD+[0., 0., .016]))
    p.add(.75, "seat the can on the lower platen", left=can_pose(C.LOAD))
    p.add(.85, "release the supported can", left=can_pose(C.LOAD, 1.))
    p.add(1.1, "clear the pinch area", left=away(can_pose(C.LOAD, 1.), .10, .014))
    p.add(1.3, "park the loading hand", left=(p.pose["left"][0] + [0., .17, .13], CAN_ORIENTATION, 1.))
    if mode == "load-only":
        p.add(1.0, "rest with the lever open")
        return p.moves
    p.add(1.8, "approach the lever grip", **handle_targets(0., 1., .079))
    p.add(.8, "align around the lever grip", **handle_targets(0., 1.))
    p.add(.9, "grasp the handle", **handle_targets(0.))
    angles = np.linspace(0., np.rad2deg(C.CLOSED), 18)
    for degrees in angles[1:]:
        p.add(.45, PULL_PHASE, **handle_targets(degrees))
    p.add(2.25, FINISH_PHASE, **handle_targets(angles[-1] - 2.))
    p.add(.85, "release the handle for the sprung return", **handle_targets(angles[-1], 1.))
    p.add(1.1, "withdraw from the handle travel", **handle_targets(angles[-1], 1., .103))
    p.add(5., "let the return spring raise the ram")
    pickup = C.LOAD + [-.008, .009, -.014]
    p.add(1.75, "approach the crushed can", left=away(puck_pose(pickup, 1.), .108, .034))
    p.add(1.25, "align around the crushed can", left=puck_pose(pickup, 1.))
    p.add(1.10, "grasp the crushed can", left=puck_pose(pickup))
    p.add(.85, "lift clear of the cradle", left=puck_pose(pickup + [0., 0., .065]))
    p.add(1.2, "withdraw the crushed can", left=puck_pose(pickup + [-.094, 0., .065]))
    drop = C.BIN_CENTER + [-.018, 0., C.BIN_HEIGHT + .152]
    p.add(1.7, "carry the crushed can over the bin", left=puck_pose(drop))
    p.add(.90, "drop the crushed can into the bin", left=puck_pose(drop, 1.))
    p.add(.80, "clear the collection area", left=away(puck_pose(drop, 1.), .072, .075))
    return p.moves
