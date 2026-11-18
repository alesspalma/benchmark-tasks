"""Fixed reference that grasps the handle and traces an axial plunge and lift."""

import numpy as np

from . import config as C
from .motion import Program, rotation

RELATIVE_HAND = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
HORIZONTAL_TOOL = rotation("Y", np.pi / 2)
LEFT_HAND = rotation("Z", np.deg2rad(-45.)) @ rotation("Y", np.deg2rad(15.)) @ RELATIVE_HAND
CORER_CLOSE = .09
UPRIGHT_TOOL = rotation("Z", np.deg2rad(32.))
APPLE_CLOSE = .69


def tool_pose(tip, r, grip=CORER_CLOSE):
    return np.asarray(tip) + r[:, 2] * C.GRIP, r @ RELATIVE_HAND, grip


def waypoints(mode="demo"):
    program = Program()
    if mode == "hold":
        return program.moves
    fruit = C.APPLE + [0., 0., .045]
    hold_apple = (fruit, LEFT_HAND, APPLE_CLOSE)
    start = C.TOOL_START
    lifted = C.TOOL_START + [0, 0, .094]
    approach = (fruit - .069 * LEFT_HAND[:, 2], LEFT_HAND, 1.)
    program.add(2.1, "approach the corer",
                right=tool_pose(start + [0., 0., .085], HORIZONTAL_TOOL, 1.))
    program.add(1.25, "align the right gripper",
                right=tool_pose(start, HORIZONTAL_TOOL, 1.))
    program.add(1., "grasp the handle", right=tool_pose(start, HORIZONTAL_TOOL))
    hover = (approach[0] + [0., 0., .105], LEFT_HAND, 1.)
    program.add(1.45, "lift the tool", right=tool_pose(lifted, HORIZONTAL_TOOL),
                left=hover)
    # Keep the measured grip through the broad wrist rotation. A slower arc
    # keeps every joint below the action-rate limit while the tip clears the board.
    program.add(5., "bring the tool upright in free space",
                right=tool_pose([.38, -.20, .91], UPRIGHT_TOOL))
    program.add(1.5, "settle the tool and approach the apple", left=approach)
    program.add(1.5, "align the tool above the fruit",
                right=tool_pose(C.APPLE + [0, 0, .106], UPRIGHT_TOOL),
                left=(fruit, LEFT_HAND, 1.))
    program.add(1., "brace the apple under the raised tool", left=hold_apple)
    program.add(1.45, "seat the rim around the stem",
                right=tool_pose(C.APPLE + [0, 0, .0685], UPRIGHT_TOOL))
    program.add(.65, "settle the rim above the stem")
    if mode == "shallow":
        program.add(2.1, "press halfway down",
                    right=tool_pose(C.APPLE + [0, 0, .035], UPRIGHT_TOOL @ rotation("Z", .12)))
    else:
        program.add(2.45, "press and twist through the first half",
                    right=tool_pose(C.APPLE + [0, 0, .035], UPRIGHT_TOOL @ rotation("Z", .13)))
        program.add(2.45, "finish the cut through the fruit",
                    right=tool_pose(C.APPLE + [0, 0, .0015], UPRIGHT_TOOL @ rotation("Z", -.09)))
        program.add(1.1, "finish the cut with a small turn",
                    right=tool_pose(C.APPLE + [0, 0, -.0035], UPRIGHT_TOOL))
    program.add(1.5, "keep the core inside the tube")
    program.add(3.2, "withdraw the corer and core",
                right=tool_pose(C.APPLE + [0, 0, .140], UPRIGHT_TOOL))
    program.add(1., "withdraw fully before releasing the apple")
    program.add(.85, "release the apple on the board", left=(fruit, LEFT_HAND, 1.))
    program.add(1.25, "clear the apple while holding the core", left=approach)
    program.add(.9, "hold the extracted core clear")
    return program.moves
