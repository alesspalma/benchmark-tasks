"""Grasp, pitch, load, draw and lift the real applicator."""

import numpy as np

from . import config as C
from .motion import Program, rotation

TOP = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
PITCH = rotation("Y", np.deg2rad(-41))
CLOSE = .24


def grasp_at(base, tool=np.eye(3), grip=CLOSE):
    return np.asarray(base) + tool @ C.TOOL_GRIP, tool @ TOP, grip


def at_nose(toe, pitch=PITCH, grip=CLOSE):
    return grasp_at(np.asarray(toe) - pitch @ C.TOOL_TOE, pitch, grip)


def waypoints(mode="demo"):
    p = Program()
    if mode == "hold":
        return p.moves
    clear = C.TOOL_START + [0., 0., .105]
    p.add(2., "approach dispenser", right=grasp_at(clear, grip=1.))
    p.add(1.25, "align around the housing", right=grasp_at(C.TOOL_START, grip=1.))
    p.add(.8, "grasp the casing", right=grasp_at(C.TOOL_START))
    p.add(1.2, "lift the dispenser", right=grasp_at(clear))
    start = C.BOARD_START + [-.0064, C.ROW_Y, C.PAPER_Z]
    end = C.BOARD_START + [.0064, C.ROW_Y, C.PAPER_Z]
    p.add(1.45, "pitch the nose toward the marked word", right=at_nose(start + [0, 0, .063]))
    press = .0014 if mode == "demo" else -.005
    p.add(1.65, "press the blade evenly", right=at_nose(start - [0, 0, press]))
    p.add(.40, "stabilize the loaded blade")
    p.add(2.35, "draw the coating across the entry", right=at_nose(end - [0, 0, press]))
    p.add(.85, "lift cleanly from the end of the line", right=at_nose(end + [0, 0, .05]))
    p.add(1.5, "return the tool to its parking place", right=grasp_at(clear))
    p.add(1.1, "set the dispenser down", right=grasp_at(C.TOOL_START))
    p.add(.7, "release the dispenser", right=grasp_at(C.TOOL_START, grip=1.))
    p.add(1.15, "withdraw", right=grasp_at(clear, grip=1.))
    p.add(.9, "settle")
    return p.moves
