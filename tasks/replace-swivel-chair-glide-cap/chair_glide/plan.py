"""Cartesian service sequence for the fixed padded chair bench."""

from math import cos, radians, sin

import numpy as np

from . import config as C
from .motion import Program


def wrist(forward, close):
    axis = np.asarray(forward, dtype=float)
    axis /= np.linalg.norm(axis)
    jaws = np.asarray(close, dtype=float)
    jaws -= np.dot(jaws, axis) * axis
    jaws /= np.linalg.norm(jaws)
    return np.column_stack((jaws, np.cross(axis, jaws), axis))


def turn_x(degrees):
    angle = radians(degrees)
    return np.array(
        [[1, 0, 0], [0, cos(angle), -sin(angle)], [0, sin(angle), cos(angle)]]
    )


def waypoints(mode):
    seq = Program()
    if mode == "hold":
        return seq.moves
    left_axes = wrist([0.30, -0.86, -0.40], [-1, 0, 0])
    right_axes = wrist([0.50, 0.56, -0.66], [1, 0, 0])
    cap_axes = wrist([0.44, 0.62, -0.65], [0.62, -0.44, 0])

    def l(delta, grip):
        return C.FOOT + delta, left_axes, grip

    def r(position, grip):
        return position, right_axes, grip

    t = C.TOOL_HOME + [0, -0.006, -0.001]
    wrist_at_notch = C.FOOT + [-0.019, -0.077, 0.013]
    fulcrum = C.FOOT + [0, -0.0195, 0.0045]

    def pry(degrees):
        rot = turn_x(degrees)
        return (
            fulcrum + rot @ (wrist_at_notch - fulcrum),
            rot @ right_axes,
            0.09,
        )

    support = l([-0.002, 0.002, -0.010], 0.12)
    leave_left = l([-0.037, 0.138, 0.018], 0.62)
    side = np.array([0.0, -0.027, 0.010])
    seq.add(
        1.5,
        "Reach toward the glide and the flat-blade handle",
        left=l([-0.032, 0.129, 0.025], 0.56),
        right=r(t + [-0.025, -0.038, 0.079], 0.60),
    )
    seq.add(
        1.5,
        "Reach the glide from the side and lower onto the handle",
        left=l([-0.007, 0.037, -0.011], 0.56),
        right=r(t, 0.55),
    )
    seq.add(
        0.75,
        "Grasp the glide body and the flat-blade handle",
        left=support,
        right=r(t, 0.08),
    )
    seq.add(
        1.55,
        "Lift the screwdriver clear of the padded bench",
        right=r(t + [0.005, -0.008, 0.155], 0.08),
    )
    seq.add(
        1.6,
        "Position the driver outside the service notch",
        right=r(wrist_at_notch + side, 0.08),
    )
    seq.add(1.20, "Seat the blade in the notch beneath the worn cap", right=pry(0))
    seq.add(1.08, "Pry upward against the glide flange", right=pry(13))
    seq.add(1.45, "Continue the leverage until the worn cap releases", right=pry(22))
    seq.add(
        0.85,
        "Withdraw the pry tool and release the swivel",
        left=l([-0.010, 0.061, 0.016], 0.48),
        right=r(wrist_at_notch + [0.0, -0.066, 0.020], 0.08),
    )
    seq.add(
        1.5,
        "Return the screwdriver to the bench",
        left=leave_left,
        right=r(t + [0.0, -0.012, 0.055], 0.08),
    )
    seq.add(1.05, "Set the screwdriver down", right=r(t + [0.0, -0.012, 0.001], 0.08))
    seq.add(
        0.65,
        "Open the jaws around the parked tool",
        right=r(t + [0.0, -0.012, 0.001], 0.55),
    )
    if mode == "remove-only":
        seq.add(
            1.2,
            "Clear both hands for the removal inspection",
            left=leave_left,
            right=r(t + [0, -0.032, 0.079], 0.52),
        )
        return seq.moves

    cap = C.FRESH_HOME + [-0.005, -0.004, 0.001]

    def fingertips(position, grip):
        return position, cap_axes, grip

    seq.add(
        1.6,
        "Reach over the new felt cap on its rest",
        right=fingertips(cap + [-0.013, -0.031, 0.061], 0.51),
    )
    seq.add(1.0, "Lower to the sleeve of the new cap", right=fingertips(cap, 0.68))
    seq.add(0.75, "Grip the felt cap by the nylon skirt", right=fingertips(cap, 0.115))
    seq.add(
        1.45,
        "Carry the new cap flat with its opening downward",
        right=fingertips(cap + [0.0, 0.0, 0.15], 0.09),
    )
    seq.add(
        1.65,
        "Approach the exposed swivel from the bench side",
        right=fingertips(C.FOOT + [-0.012, -0.059, 0.038], 0.09),
    )
    offset = np.array([0.0, -0.013 if mode == "loose-cap" else -0.007, 0.017])
    seq.add(
        2.0,
        "Align the new cap around the swivel flange",
        right=fingertips(C.FOOT + offset, 0.09),
    )
    seq.add(
        2.33, "Seat the cap with a light press", right=fingertips(C.FOOT + offset, 0.11)
    )
    seq.add(
        0.80,
        "Release the aligned cap for seating",
        right=fingertips(C.FOOT + offset, 0.61),
    )
    seq.add(
        0.85,
        "Raise the fingers before moving them sideways",
        right=fingertips(C.FOOT + offset + [-0.008, -0.014, 0.033], 0.62),
    )

    m = np.array([C.MALLET_HOME[0], C.MALLET_HOME[1], C.TABLE + 0.029])
    seq.add(
        1.45, "Reach over the rubber mallet", right=r(m + [-0.017, 0.0, 0.081], 0.58)
    )
    seq.add(1.0, "Lower to the handle of the rubber mallet", right=r(m, 0.58))
    seq.add(0.65, "Grip the mallet handle", right=r(m, 0.08))
    seq.add(1.30, "Lift the mallet", right=r(m + [-0.03, 0.004, 0.16], 0.08))
    seq.add(
        1.15,
        "Raise the mallet alongside the chair foot",
        right=r(C.FOOT + [-0.022, -0.139, 0.048], 0.08),
    )
    seq.add(
        1.45,
        "Bring the rubber face over the new cap",
        right=r(C.FOOT + [0.0, -0.063, 0.063], 0.08),
    )
    seq.add(
        1.25,
        "Press the rubber face against the cap until the rim clicks",
        right=r(C.FOOT + [0.0, -0.063, 0.035], 0.08),
    )
    seq.add(
        0.55,
        "Lift the mallet from the retained cap",
        right=r(C.FOOT + [0.0, -0.077, 0.059], 0.08),
    )
    seq.add(
        1.1,
        "Release the swivel and carry the mallet down",
        left=leave_left,
        right=r(m + [-0.020, 0.0, 0.081], 0.08),
    )
    seq.add(0.95, "Set the mallet down on the padded bench", right=r(m, 0.08))
    seq.add(0.7, "Release the mallet handle", right=r(m, 0.61))
    seq.add(
        1.15,
        "Clear the chair foot for final inspection",
        right=r(m + [-0.025, -0.026, 0.105], 0.63),
    )
    return seq.moves
