"""Timed Cartesian keyframes for two ordinary robot grasps."""
import numpy as np

from . import config as C
from .motion import Program, rotation

RIGHT_SIDE = rotation("Z", np.deg2rad(60.)) @ np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
LEFT_SIDE = np.array([[-1., 0., 0.], [0., 0., -1.], [0., -1., 0.]])
GLASS_GRIP = .44
TAP_GRIP = .18


def glass_pour(angle):
    orient = rotation("X", -angle)
    intercept = np.array([0., .010 * np.sin(angle) / np.sin(np.pi / 4.), C.HEIGHT])
    root = C.SPOUT - [0., 0., .030] - orient @ intercept
    return root + orient @ np.array([0., 0., C.GLASS_GRASP_HEIGHT]), orient @ RIGHT_SIDE, GLASS_GRIP


def tap_pose(angle, grip=TAP_GRIP):
    orient = rotation("Y", -angle)
    return C.PIVOT + orient @ [0., 0., C.TAP_GRASP_HEIGHT], orient @ LEFT_SIDE, grip


def waypoints(mode="demo"):
    if mode == "hold":
        return {}
    plan = Program()
    start = C.GLASS + [0., 0., C.GLASS_GRASP_HEIGHT]
    faucet, left_rotation, _ = tap_pose(0.)
    plan.add(2.4, "approach glass and faucet from opposite sides",
             right=(start + -.068 * RIGHT_SIDE[:, 2] + [0., 0., .035], RIGHT_SIDE, 1.),
             left=(faucet + [0., .095, .035], left_rotation, 1.))
    plan.add(1.8, "align around the glass and lever",
             right=(start, RIGHT_SIDE, 1.), left=(faucet, left_rotation, 1.))
    plan.add(1.4, "grasp glass and faucet",
             right=(start, RIGHT_SIDE, GLASS_GRIP), left=tap_pose(0.))
    plan.add(1.3, "lift glass clear of the mat",
             right=(start + [0., 0., .053], RIGHT_SIDE, GLASS_GRIP))
    angle = 0. if mode == "upright" else np.pi / 4.
    plan.add(2.3, "angle the glass beneath the faucet",
             right=glass_pour(angle))
    plan.add(.7, "fully open faucet", left=tap_pose(C.TAP_ANGLE+.015))
    plan.add(2.0, "guide the stream against the inner wall")
    for remaining in (.75, .5, .25, 0.):
        plan.add(.68, "level the glass as the pool rises", right=glass_pour(remaining * angle))
    plan.add(.95, "raise the foam head")
    plan.add(.65, "close faucet", left=tap_pose(-.015))
    plan.add(.6, "release closed lever", left=tap_pose(0., 1.))
    coaster = C.COASTER + [0., 0., C.GLASS_GRASP_HEIGHT]
    plan.add(1.6, "move the filled glass over the coaster",
             left=(faucet + [0., .09, .04], left_rotation, 1.),
             right=(coaster + [0., 0., .053], RIGHT_SIDE, GLASS_GRIP))
    plan.add(1.1, "seat glass upright on the coaster", right=(coaster, RIGHT_SIDE, GLASS_GRIP))
    plan.add(.6, "release supported glass", right=(coaster, RIGHT_SIDE, 1.))
    plan.add(1.1, "withdraw both arms", right=(coaster + -.068 * RIGHT_SIDE[:, 2] + [0., 0., .035], RIGHT_SIDE, 1.))
    plan.add(1.0, "leave finished drink standing")
    return plan.moves
