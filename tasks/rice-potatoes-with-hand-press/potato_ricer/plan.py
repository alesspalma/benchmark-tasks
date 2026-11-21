"""Cartesian reference keeps to ordinary parallel-gripper actions."""

import numpy as np

from . import config as C
from .motion import Program, rotation
from .scoring import transform


def hand(approach, pinch):
    axis = np.asarray(approach, dtype=float)
    axis /= np.linalg.norm(axis)
    closing = np.asarray(pinch, dtype=float) - np.dot(axis, pinch) * axis
    closing /= np.linalg.norm(closing)
    return np.column_stack((closing, np.cross(axis, closing), axis))


LOAD = hand([0.55, -0.12, -0.83], [0.0, -1.0, 0.0])
STEADY = hand([1.0, -0.36, -0.19], [0.0, -1.0, 0.0])
PRESS = hand([0.68, 0.45, -0.65], [0.0, -1.0, 0.0])


def live_squeeze(state):
    tool = state["objects"]["ricer"]
    center, axes = transform(tool["pose"])
    measured = tool["joints"]["HandleHinge"]
    aim = max(-0.030, measured - 0.135)
    point = center + axes @ (C.HINGE + rotation("Y", aim) @ C.PRESS_GRIP)
    orient = axes @ PRESS
    return point + axes @ np.array([0.012, -0.001, -0.010]), orient


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    stock = C.PORTION + [0.0, 0.0, 0.0235]
    basket = C.RICER + [-0.006, 0.001, 0.063]
    lower = C.RICER + C.FIXED_GRIP + [0, 0, 0.005]

    def lever(angle, opening, offset=(0.0, 0.0, 0.0)):
        # Surveyed pose of the unloaded press on its bowl rests.
        rest_position = C.RICER + np.array([0.002, 0.0, 0.007])
        rest_rotation = rotation("Y", np.deg2rad(-1.8)) @ rotation("X", np.deg2rad(1.1))
        point = rest_position + rest_rotation @ (
            C.HINGE + rotation("Y", angle) @ C.PRESS_GRIP
        )
        grip_alignment = PRESS
        return point + [0.003, 0.0, 0.006] + offset, grip_alignment, opening

    p.add(2.0, "Approach the cooked potato", left=(stock + [0, 0, 0.10], LOAD, 0.94))
    p.add(1.0, "Lower around the potato portion", left=(stock, LOAD, 0.93))
    p.add(0.65, "Grip the loose cooked potato", left=(stock, LOAD, 0.30))
    p.add(
        0.95,
        "Lift the potato from the side plate",
        left=(stock + [0, 0, 0.19], LOAD, 0.30),
    )
    p.add(
        1.45,
        "Bring the potato over the open ricer basket",
        left=(basket + [-0.042, -0.006, 0.036], LOAD, 0.30),
    )
    p.add(2.1, "Lower the potato into the basket", left=(basket, LOAD, 0.30))
    p.add(0.65, "Settle over the center of the basket", left=(basket, LOAD, 0.30))
    p.add(
        0.75,
        "Open the jaws and let the portion settle in the basket",
        left=(basket, LOAD, 0.97),
    )
    p.add(
        0.70, "Lift clear of the basket", left=(basket + [0, 0.014, 0.030], LOAD, 0.96)
    )
    p.add(
        2.30,
        "Approach the two separate handles",
        left=(lower + [-0.055, 0.103, 0.082], STEADY, 0.92),
        right=lever(C.OPEN, 0.08, (-0.058, -0.083, 0.041)),
    )
    p.add(
        2.35,
        "Line up with the rubber handle grips",
        left=(lower, STEADY, 0.91),
        right=lever(C.OPEN, 0.08, (0.000, 0.0, 0.022)),
    )
    p.add(0.8, "Align both wrists over the two handles")
    p.add(
        0.95,
        "Hold the lower handle and place the right gripper on the upper handle",
        left=(lower, STEADY, 0.12),
        right=lever(C.OPEN, 0.08, (0.000, 0.0, 0.018)),
    )
    if mode == "no_press":
        p.add(
            0.75,
            "Release the handles without pressing",
            left=(lower, STEADY, 0.94),
            right=lever(C.OPEN, 0.08, (0.0, 0.0, 0.035)),
        )
        p.add(
            1.0,
            "Withdraw from the loaded ricer",
            left=(lower + [0.015, 0.158, 0.084], STEADY, 0.94),
            right=lever(C.OPEN, 0.08, (-0.04, -0.12, 0.05)),
        )
        return p.moves
    for degrees in (69, 55, 42, 29, 20, 12, 6, 0):
        offset = (0.0, 0.0, -0.006) if degrees <= 5 else (0.0, 0.0, 0.0)
        p.add(
            0.30,
            "Squeeze the hinged press down through the cooked potato",
            right=lever(np.deg2rad(degrees), 0.08, offset),
        )
    p.add(0.95, "Finish pressing over the mixing bowl")
    p.add(
        1.20,
        "Apply the last portion of the compression stroke",
        right=lever(0.0, 0.08, (-0.039, -0.022, -0.027)),
    )
    p.add(0.55, "Hold the ricer closed while the strands fall")
    p.add(
        0.75,
        "Release the handles",
        left=(lower, STEADY, 0.95),
        right=lever(0.0, 0.08, (-0.030, -0.023, 0.014)),
    )
    p.add(
        1.40,
        "Move both hands clear of the ricer",
        left=(lower + [0.015, 0.158, 0.084], STEADY, 0.95),
        right=lever(0.0, 0.96, (-0.03, -0.14, 0.040)),
    )
    p.add(0.75, "Leave the riced potato in the bowl")
    return p.moves
