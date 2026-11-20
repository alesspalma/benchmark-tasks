"""Reference scores a gravity-loaded lift, then rotates the actual keeper."""

import numpy as np

from . import config as C
from .motion import Program, rotation

FRONT = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
SUPPORT = rotation("Y", np.radians(12.)) @ FRONT


def tray_pose(deg, retraction=0.):
    r = rotation("Y", np.radians(deg))
    p = C.PIVOT + r @ [-.190, .128, -.020] - [retraction, 0., 0.]
    return p, SUPPORT, .07


def latch_pose(deg, retraction=0., grip=.09):
    rad = np.radians(deg)
    r = rotation("X", rad)
    p = C.LATCH_PIVOT + r @ [-.016, -.025, .019] - [retraction, 0., 0.]
    return p, FRONT, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    program = Program()
    program.add(1.6, "Approach the deployed table edge",
                left=tray_pose(0., .062))
    program.add(1.3, "Slide the closed fingertips under the table edge",
                left=tray_pose(0.))
    for a in np.linspace(0., C.STOW_ANGLE + 2., 15)[1:]:
        position, orientation, grip = tray_pose(a)
        program.add(.31, "Lift the meal table along its hinge arc",
                    left=(position, orientation, grip))
    wall = tray_pose(C.STOW_ANGLE + 2., -.003)
    if mode != "unlatched":
        program.add(1.8, "Hold the loaded stow position and approach the keeper",
                    left=wall, right=latch_pose(0., .080))
        program.add(.9, "Place fingertips at the top of the rotary keeper",
                    right=latch_pose(0., 0.))
        for a in np.linspace(0., 90., 19)[1:]:
            program.add(.19, "Swing the keeper down over the top edge",
                        right=latch_pose(a, 0.))
        program.add(1.0, "Withdraw the keeper hand",
                    right=latch_pose(90., .100))
    else:
        program.add(1., "Hold the table up while leaving the keeper open", left=wall)
    program.add(1.4, "Withdraw both hands and check hands-off retention",
                left=tray_pose(C.STOW_ANGLE, .117))
    return program.moves
