"""Reach the two different control surfaces and follow their revolute arcs."""

from math import cos, radians, sin

import numpy as np

from . import config as C
from .motion import Program


def frame(forward):
    forward = np.asarray(forward, dtype=float)
    forward /= np.linalg.norm(forward)
    closing = np.asarray([1.0, 0.0, 0.0])
    closing -= np.dot(closing, forward) * forward
    closing /= np.linalg.norm(closing)
    return np.column_stack((closing, np.cross(forward, closing), forward))


def rotate_x(angle):
    c, s = cos(angle), sin(angle)
    return np.asarray([[1, 0, 0], [0, c, -s], [0, s, c]])


def rotate_y(angle):
    c, s = cos(angle), sin(angle)
    return np.asarray([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def waypoints(mode):
    program = Program()
    if mode == "hold":
        return program.moves
    ring_basis = frame([0.38, -0.34, -0.86])
    lock_basis = frame([0.15, 0.95, -0.25])

    def ring(angle, grip, offset=(0, 0, 0)):
        r = rotate_x(angle)
        return (
            np.asarray(C.FOCUS_ORIGIN) + r @ C.FOCUS_GRIP + offset,
            rotate_x(0.24 * angle) @ ring_basis,
            grip,
        )

    def screw(angle, grip, offset=(0, 0, 0)):
        r = rotate_y(angle)
        return (
            np.asarray(C.LOCK_ORIGIN) + (0, -0.007, 0) + offset,
            r @ lock_basis,
            grip,
        )

    program.add(
        1.9, "Approach the focus-ring lever", left=ring(0, 0.46, (-0.018, 0.042, 0.084))
    )
    program.add(1.8, "Reach down to the focus lever", left=ring(0, 0.41))
    program.add(0.8, "Grasp the knurled ring lever", left=ring(0, 0.07))
    target = radians(7 if mode == "wrong_focus" else 37)
    for fraction in (0.30, 0.60, 1.0):
        program.add(
            0.65, "Fine-focus the line chart", left=ring(fraction * target, 0.07)
        )
    program.add(0.8, "Hold the sharp image", left=ring(target, 0.07))

    if mode == "no_lock":
        program.add(
            7.0, "Keep focus without engaging the screw", left=ring(target, 0.07)
        )
    else:
        program.add(
            1.55,
            "Reach the focus-lock thumbscrew",
            right=screw(0, 0.44, (-0.014, -0.066, 0.084)),
        )
        program.add(
            1.55, "Line up the jaws with the locking screw", right=screw(0, 0.43)
        )
        program.add(0.8, "Grip the thumbscrew head", right=screw(0, 0.055))
        for degrees in (19, 38, 59, 75):
            program.add(
                0.53,
                "Tighten the screw while the focus is held",
                right=screw(radians(degrees), 0.055),
            )
        program.add(0.55, "Open the locking hand", right=screw(radians(75), 0.48))
        program.add(
            0.95,
            "Withdraw from the locking screw",
            right=screw(0, 0.53, (-0.025, -0.082, 0.060)),
        )
    program.add(0.65, "Release the focus ring", left=ring(target, 0.53))
    program.add(
        1.10,
        "Withdraw to verify sharpness and retention",
        left=ring(target, 0.55, (-0.035, 0.108, 0.094)),
    )
    program.add(0.80, "Keep both hands clear")
    return program.moves
