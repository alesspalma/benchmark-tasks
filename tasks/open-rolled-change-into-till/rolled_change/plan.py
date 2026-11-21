"""Cartesian policy for opening against the steel drawer ridge."""

from math import cos, radians, sin

import numpy as np

from . import config as C
from .motion import Program

SIGNS = {"left": 1, "right": -1}


def wrist(forward, close):
    axis = np.asarray(forward, dtype=float)
    axis /= np.linalg.norm(axis)
    jaws = np.asarray(close, dtype=float)
    jaws -= np.dot(jaws, axis) * axis
    jaws /= np.linalg.norm(jaws)
    return np.column_stack((jaws, np.cross(axis, jaws), axis))


def turn_x(angle):
    c, s = cos(radians(angle)), sin(radians(angle))
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


NEUTRAL = {
    name: wrist([0.10, -0.76 * sign, -0.642], [sign, 0, 0])
    for name, sign in SIGNS.items()
}
GRIP_POINT = {
    name: np.array([0.0, 0.017 * sign, 0.0025]) for name, sign in SIGNS.items()
}


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves

    def approach(name, center, grip, reach_y=0, height=0):
        return (
            center + GRIP_POINT[name] + [0, reach_y * SIGNS[name], height],
            NEUTRAL[name],
            grip,
        )

    def hold_half(x, y, z, bend=0, grip=0.07):
        return (
            [x, y, z],
            turn_x(bend),
            grip,
        )

    initial = {
        name: np.array([C.ROLL_X, sign * C.HALF_Y, C.ROLL_Z])
        for name, sign in SIGNS.items()
    }
    p.add(
        1.45,
        "Reach to opposite ends of the wrapped roll",
        **{
            name: approach(name, center, 0.5, 0.075, 0.087)
            for name, center in initial.items()
        },
    )
    p.add(
        1.25,
        "Place the fingertips just outside the crimped ends",
        **{name: approach(name, center, 0.48) for name, center in initial.items()},
    )
    p.add(
        1.40,
        "Grip both ends of the paper wrapper",
        **{name: approach(name, center, 0.055) for name, center in initial.items()},
    )
    p.add(
        1.25,
        "Lift the intact roll",
        **{
            name: hold_half(C.ROLL_X, sign * C.HALF_Y, 0.890)
            for name, sign in SIGNS.items()
        },
    )
    p.add(
        1.5,
        "Set the middle of the roll over the steel drawer edge",
        **{
            name: hold_half(
                C.EDGE_X, sign * C.HALF_Y, C.EDGE_Z + C.PAPER_RADIUS + 0.014
            )
            for name, sign in SIGNS.items()
        },
    )
    p.add(
        1.1,
        "Lower the intact roll against the drawer edge",
        **{
            name: hold_half(
                C.EDGE_X, sign * C.HALF_Y, C.EDGE_Z + C.PAPER_RADIUS - 0.0015
            )
            for name, sign in SIGNS.items()
        },
    )
    for angle in (6, 18, 28):
        p.add(
            0.62,
            "Bend the roll at the drawer edge until the kraft paper opens",
            **{
                name: hold_half(
                    C.EDGE_X,
                    sign * (C.HALF_Y * cos(radians(angle)) + 0.002 * angle / 28),
                    C.EDGE_Z + C.PAPER_RADIUS - C.HALF_Y * sin(radians(angle)) - 0.001,
                    -sign * angle,
                )
                for name, sign in SIGNS.items()
            },
        )
    p.add(
        1.35,
        "Raise both opened halves straight off the steel edge",
        **{
            name: hold_half(C.EDGE_X, sign * 0.035, 0.902, -sign * 22)
            for name, sign in SIGNS.items()
        },
    )
    p.add(
        1.20,
        "Separate the raised halves with their crimped ends down",
        **{
            name: hold_half(C.EDGE_X, sign * 0.076, 0.895, -sign * 24)
            for name, sign in SIGNS.items()
        },
    )
    if mode == "opened":
        p.add(
            1.25,
            "Set the opened roll on the bench without filling the till",
            **{
                name: hold_half(0.345, sign * 0.108, 0.798, -sign * 10)
                for name, sign in SIGNS.items()
            },
        )
        p.add(
            0.7,
            "Release both pieces",
            **{
                name: hold_half(0.345, sign * 0.108, 0.798, -sign * 10, 0.55)
                for name, sign in SIGNS.items()
            },
        )
        return p.moves

    for name, sign in SIGNS.items():
        pour_x = 0.437 if name == "left" else 0.421
        pour_pitch = 39 if name == "left" else 50
        p.add(
            1.35,
            "Carry the first opening above the 5 cent compartment"
            if name == "left"
            else "Carry the second opening above the same compartment",
            **{name: hold_half(0.439, sign * 0.054, 0.900, -sign * 24)},
        )
        p.add(
            3.35,
            "Tilt the half-roll and let the nickels descend into the compartment",
            **{name: hold_half(pour_x, sign * 0.039, 0.887, sign * pour_pitch)},
        )
        p.add(
            1.53,
            "Hold the opening over the drawer until the last coin falls",
            **{name: hold_half(pour_x, sign * 0.039, 0.887, sign * pour_pitch)},
        )
        p.add(
            1.80,
            "Turn the empty paper back to horizontal",
            **{name: hold_half(0.448, sign * 0.107, 0.898, 0)},
        )
        p.add(
            1.45,
            "Carry the empty paper to the separate scrap tray",
            **{name: hold_half(0.255, sign * 0.096, 0.881, 0)},
        )
        p.add(
            0.95,
            "Lower the empty wrapper into the scrap tray",
            **{name: hold_half(0.255, sign * 0.096, 0.787, 0)},
        )
        p.add(
            0.6,
            "Release the paper inside the scrap tray",
            **{name: hold_half(0.255, sign * 0.096, 0.787, 0, 0.55)},
        )
        p.add(
            0.85,
            "Withdraw the gripper before inspecting the filled drawer",
            **{name: hold_half(0.254, sign * 0.153, 0.878, 0, 0.66)},
        )
    p.add(0.85, "Let the nickels settle")
    return p.moves
