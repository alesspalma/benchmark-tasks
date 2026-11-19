"""Bimanual contact reference and native negative controls."""

import numpy as np

from . import config as C
from .motion import Program, rotation

LEFT_DIRECTION = np.array([0.64, -0.73, -0.24])
LEFT_DIRECTION /= np.linalg.norm(LEFT_DIRECTION)
LEFT_CLOSING = np.array([LEFT_DIRECTION[1], -LEFT_DIRECTION[0], 0.0])
LEFT_CLOSING /= np.linalg.norm(LEFT_CLOSING)
LEFT = np.column_stack(
    (LEFT_CLOSING, np.cross(LEFT_DIRECTION, LEFT_CLOSING), LEFT_DIRECTION)
)
RIGHT = rotation("X", np.pi + 0.65) @ rotation("Y", 0.25)
LEFT_GRIP = 0.19
LIFT_CLEARANCE = np.array([0.027, -0.026, 0.078])


def base_at(xy):
    return np.r_[xy, C.TRAY_TOP]


def barrel(base, grip=LEFT_GRIP, offset=0.0):
    point = np.asarray(base) + [0.0, 0.0, C.BARREL_GRIP_Z]
    return point - offset * LEFT[:, 2], LEFT, grip


def lever(base, degrees=0.0, raised=0.0):
    tilt = rotation("X", np.deg2rad(degrees))
    point = (
        np.asarray(base) + C.PIVOT + tilt @ C.LEVER_VECTOR + [0.0, 0.0, 0.006 + raised]
    )
    return point - 0.0125 * RIGHT[:, 2], RIGHT, 0.02


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    targets = C.SPOTS if mode != "same-spot" else (C.SPOTS[0],) * 3
    first = base_at(targets[0])
    p.add(
        2.1,
        "approach the primed press",
        left=barrel(first, grip=0.88, offset=0.108),
        right=(np.array([0.315, -0.151, 1.095]), RIGHT, 0.02),
    )
    p.add(1.2, "straddle the barrel", left=barrel(first, grip=0.88))
    p.add(0.8, "grasp the barrel", left=barrel(first))
    p.add(
        1.3,
        "bring the right arm above the held lever",
        right=lever(first, raised=0.048),
    )
    for i, xy in enumerate(targets):
        base = base_at(xy)
        if i:
            previous = base_at(targets[i - 1])
            p.add(
                1.15,
                "lift the die clear of the dose",
                left=barrel(previous + LIFT_CLEARANCE),
                right=lever(previous + [0.0, 0.0, 0.067], raised=0.027),
            )
            p.add(0.45, "hold the separated press above the sheet")
            p.add(
                1.25,
                "move to a clear spot on the sheet",
                left=barrel(base + [0.0, 0.0, 0.067]),
                right=lever(base + [0.0, 0.0, 0.067], raised=0.027),
            )
        p.add(
            0.90,
            "seat the ring on the sheet",
            left=barrel(base - [0.0, 0.0, 0.013]),
            right=lever(base, raised=0.029),
        )
        if i:
            p.add(
                0.36,
                "let the relocated ring settle flat",
                left=barrel(base - [0.0, 0.0, 0.002], grip=0.76),
            )
            p.add(
                0.52,
                "brace the press against the sheet",
                left=barrel(base - [0.0, 0.0, 0.007]),
            )
        p.add(0.7, "touch the lever pad", right=lever(base, raised=0.002))
        p.add(
            1.25,
            "press the lever under dough load",
            right=lever(base, degrees=42.0, raised=-0.018),
        )
        p.add(0.3, "complete the shot")
        p.add(0.65, "release the lever", right=lever(base, raised=0.033))
    end = base_at(targets[-1])
    park = np.array([0.415, 0.238, C.TABLE])
    p.add(1.15, "lift the press vertically", left=barrel(end + LIFT_CLEARANCE))
    p.add(0.45, "hold the separated press above the sheet")
    p.add(2.0, "move the press beside the sheet", left=barrel(park + [0.0, 0.0, 0.054]))
    p.add(0.9, "set the press upright", left=barrel(park))
    p.add(0.6, "open the barrel gripper", left=barrel(park, grip=0.87))
    p.add(
        0.7,
        "withdraw from the parked press",
        left=barrel(park, grip=0.87, offset=0.095),
    )
    return p.moves
