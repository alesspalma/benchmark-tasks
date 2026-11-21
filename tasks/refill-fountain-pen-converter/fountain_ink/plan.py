"""Ordinary bimanual Cartesian waypoints for the empty pen on its rest."""

import numpy as np

from . import config as C
from .motion import Program


def wrist(tilt):
    angle = np.deg2rad(tilt)
    front = np.array([np.sin(angle), 0, -np.cos(angle)])
    close = np.array([0.0, -1.0, 0.0])
    return np.column_stack((close, np.cross(front, close), front))


def waypoints(mode):
    program = Program()
    if mode == "hold":
        return program.moves
    hold = wrist(73)
    pump = wrist(9)
    start = np.array([*C.REST[:2], C.REST_NIB_Z + C.GRIP_Z - 0.0025])
    nib_height = C.FILL_NIB_Z + (0.034 if mode == "dry" else 0.0)
    fill = np.array([*C.BOTTLE[:2], nib_height + C.GRIP_Z])
    head = np.array([*C.BOTTLE[:2], nib_height + C.BUTTON_Z + 0.004])
    left_hover = start + [-0.075, 0, 0.025]
    right_hover = head + [-0.025, -0.043, 0.068]

    def l(point, grip=0.03):
        return point, hold, grip

    def r(point):
        return point, pump, 0.0

    program.add(2.2, "Approach the pen on the rest", left=l(left_hover, 0.66))
    program.add(0.95, "Align the jaws with the pen grip", left=l(start, 0.59))
    program.add(1.00, "Grip the pen section without touching the nib", left=l(start))
    program.add(
        1.25, "Lift the pen clear of the rest", left=l(start + [0.006, 0, 0.062])
    )
    program.add(
        1.65, "Bring the pen above the open ink bottle", left=l(fill + [0, 0, 0.075])
    )
    program.add(
        1.33, "Hold the pen at filling depth", left=l(fill), right=r(right_hover)
    )
    program.add(
        1.1,
        "Position the other gripper above the pump knob",
        right=r(head + [0, 0, 0.021]),
    )
    for _ in range(3 if mode == "short" else 6):
        program.add(
            0.65,
            "Compress the converter pump inside the bottle",
            right=r(head + [0, 0, -0.002]),
        )
        program.add(
            0.65,
            "Let the pump spring return and draw ink",
            right=r(head + [0, 0, 0.021]),
        )
    program.add(0.85, "Move the pumping hand clear", right=r(right_hover))
    program.add(
        1.2, "Lift the filled pen from the bottle", left=l(fill + [0, 0, 0.076])
    )
    program.add(
        1.57, "Move the filled pen over its rest", left=l(start + [0, 0, 0.064])
    )
    program.add(
        1.25,
        "Seat the pen on the protective shoulder rest",
        left=l(start + [0, 0, -0.001]),
    )
    program.add(0.6, "Let go of the pen", left=l(start + [0, 0, -0.001], 0.75))
    program.add(
        1.25,
        "Withdraw both hands",
        left=l(left_hover + [0, 0.03, 0.035], 0.88),
        right=r(right_hover + [-0.08, -0.04, 0.015]),
    )
    return program.moves
