"""Cartesian instruction sequence for the coupon and handheld gauge."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def down(yaw, tilt):
    return rotation("Z", np.radians(yaw)) @ np.diag([1., -1., -1.]) @ rotation("X", tilt)


LEFT = down(-90., .42)
RIGHT = down(-90., .97)
OPEN = .96
LEFT_GRIP = .61
RIGHT_GRIP = .30
COUPON_PICK = C.START_COUPON + [0., 0., .003]
COUPON_SET = C.TEST_COUPON + [0., 0., .003]
GAUGE_PICK = C.TOOL + [0., 0., C.TOOL_GRIP_HEIGHT]
GAUGE_TEST = C.TEST_COUPON + [0., 0., C.COUPON_SIZE[2] / 2 + C.TOOL_GRIP_HEIGHT]
UP = np.array([0., 0., .052])


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    p.add(2., "approach coupon and durometer",
          left=(COUPON_PICK + UP, LEFT, OPEN),
          right=(GAUGE_PICK - RIGHT[:, 2] * .07, RIGHT, OPEN))
    p.add(1., "center the open jaws",
          left=(COUPON_PICK, LEFT, OPEN), right=(GAUGE_PICK, RIGHT, OPEN))
    p.add(.8, "grasp specimen and instrument",
          left=(COUPON_PICK, LEFT, LEFT_GRIP), right=(GAUGE_PICK, RIGHT, RIGHT_GRIP))
    p.add(1., "lift both objects clear",
          left=(COUPON_PICK + UP, LEFT, LEFT_GRIP),
          right=(GAUGE_PICK + UP, RIGHT, RIGHT_GRIP))
    p.add(.9, "move the coupon over the steel platen",
          left=(COUPON_SET + UP, LEFT, LEFT_GRIP))
    p.add(.9, "seat the coupon on the steel platen", left=(COUPON_SET, LEFT, LEFT_GRIP))
    p.add(.65, "release the coupon", left=(COUPON_SET, LEFT, OPEN))
    p.add(.82, "lift the left gripper clear of the specimen",
          left=(COUPON_SET + [0., .038, .14], LEFT, OPEN))
    p.add(.95, "park left arm clear of the work envelope",
          left=(np.asarray([.262, .232, .988]), LEFT, OPEN))
    sites = (C.SITES[0],) * 5 if mode == "repeat" else C.SITES
    for i, (x, y) in enumerate(sites):
        xy = np.asarray([x, y, 0.])
        test = GAUGE_TEST + xy
        hover = (test + [0., 0., .043], RIGHT, RIGHT_GRIP)
        press = (test + [0., 0., -.011], RIGHT, RIGHT_GRIP)
        p.add(1.15 if i == 0 else .92, "position the gauge above site", right=hover)
        p.add(.36, "position the gauge above site")
        p.add(1.15, "seat pressure foot on specimen", right=press)
        p.add(2.55, "hold for the Shore A reading", right=press)
        p.add(1.05, "lift the pressure foot", right=hover)
    return p.moves
