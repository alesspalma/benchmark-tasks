"""Reference motions through ordinary YAM joint and gripper commands."""

import math

import numpy as np

from . import config as c
from .builder import stock_rotation
from .motion import Program, rotation

APPROACH = np.array([[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]])
OPENING = 6.6
INSERT_PITCH = 3.2
RIGHT_YAW = 19.
LEFT_YAW = -21.


def waypoints(mode):
    if mode not in ("demo", "hold", "on-top"):
        raise ValueError(f"Unknown policy {mode!r}")
    p = Program()
    if mode == "hold":
        return p.moves

    lx = c.UPPER_LIP - .001
    ly = c.SLEEVE_Y + .082
    left_open = .60
    lz = c.UPPER_FACE + .023
    stock = stock_rotation()
    right_wrist = rotation("Z", math.radians(RIGHT_YAW)) @ APPROACH
    left_wrist = rotation("Y", math.radians(9.)) @ rotation("Z", math.radians(LEFT_YAW)) @ APPROACH
    stock_pose = stock @ right_wrist
    right_shift = -.044
    ry = c.SLEEVE_Y + right_shift
    thin_open = .19
    t_left = rotation("Y", math.radians(4.)) @ left_wrist
    t_right = rotation("Y", math.radians(INSERT_PITCH)) @ right_wrist

    def at(x, y, z, grip, orientation=APPROACH):
        return (x, y, z), orientation, grip

    def stock_at(x, z, grip):
        return np.asarray(c.STOCK_REAR) + stock @ np.array([x, right_shift, z]), stock_pose, grip

    def lip_open():
        pivot = np.array([c.FAR_EDGE, ly, c.UPPER_FACE])
        point = pivot + rotation("Y", math.radians(OPENING)) @ (
            np.array([lx, ly, lz]) - pivot
        )
        point[0] -= .003
        point[2] += .004
        return point, t_left, left_open

    p.add(3.1, "approach the two exposed edges",
          left=at(lx - .028, ly + .006, lz + .065, left_open, left_wrist),
          right=stock_at(-.025, .058, thin_open))
    p.add(1.35, "align the open jaws",
          left=at(lx - .018, ly + .006, lz, left_open, left_wrist),
          right=stock_at(-.015, .002, thin_open))
    p.add(1.4, "enter the edge overhang",
          left=at(lx, ly, lz, left_open, left_wrist),
          right=stock_at(.003, .0015, thin_open))
    p.add(1.0, "pinch the loose paper",
          right=stock_at(.003, .0015, 0.))
    p.add(.65, "clear the lower sleeve edge",
          left=at(lx - .002, ly, lz + .003, left_open, left_wrist))
    p.add(1.7, "raise the lip and lift the sheet",
          left=lip_open() if mode == "demo" else at(lx - .06, ly, lz + .095, 1.),
          right=stock_at(.003, .069, 0.))
    p.add(1.35, "carry the paper clear of the stand",
          right=at(.235, -.190, .977, 0., stock_pose))
    p.add(2.2, "turn the page toward the opening",
          right=at(.210, -.055, .954, 0., stock_pose))

    if mode == "on-top":
        p.add(3.6, "lay the page over the closed protector",
              right=at(c.MOUTH - .001, ry, c.LOWER_FACE + .078, 0.))
        p.add(.9, "get close to the closed protector",
              right=at(c.MOUTH - .001, ry, c.LOWER_FACE + .026, 0.))
        p.add(.85, "release on top of the clear film",
              right=at(c.MOUTH - .001, ry, c.LOWER_FACE + .026, thin_open))
    else:
        p.add(2.4, "offer the leading edge to the opening",
              right=at(c.MOUTH - .274, ry + .014, c.LOWER_FACE + .076, 0., stock_pose))
        p.add(2.3, "feed the leading third beneath the film",
              right=at(c.MOUTH - .179, ry + .010, c.LOWER_FACE + .027, 0.,
                       rotation("Y", math.radians(8.)) @ right_wrist))
        p.add(2.3, "feed the paper beneath the raised film",
              right=at(c.MOUTH - .075, ry + .007, c.LOWER_FACE + .021, 0.,
                       rotation("Y", math.radians(5.)) @ right_wrist))
        p.add(1.5, "seat the paper inside the bottom seam",
              right=at(c.MOUTH + .001, ry, c.LOWER_FACE + .016, 0., t_right))
        p.add(.8, "release the inserted sheet",
              right=at(c.MOUTH + .001, ry, c.LOWER_FACE + .016, thin_open, t_right))
    p.add(1.2, "withdraw the feeding hand",
          right=at(c.MOUTH - .085, ry, c.LOWER_FACE + .078, .8))
    p.add(.85, "release the protector",
          left=at(lx - .06, ly, lz + .022, .75, left_wrist) if mode == "demo" else None)
    p.add(1.2, "leave both hands clear",
          left=at(lx - .065, ly, lz + .105, .8),
          right=at(c.MOUTH - .091, ry - .07, c.LOWER_FACE + .115, .8))
    return p.moves
