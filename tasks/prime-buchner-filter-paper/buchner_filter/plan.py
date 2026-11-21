"""Bimanual Cartesian reference for the native filter station."""

from math import cos, radians, sin

import numpy as np

from . import config as C
from .motion import Program


def rotate_y(angle):
    c, s = cos(angle), sin(angle)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def hand(approach, pinch):
    axis = np.asarray(approach, dtype=float)
    axis /= np.linalg.norm(axis)
    closing = np.asarray(pinch, dtype=float) - (axis @ pinch) * axis
    closing /= np.linalg.norm(closing)
    return np.column_stack((closing, np.cross(axis, closing), axis))


PAPER_HAND = hand([0.56, -0.10, -0.82], [0.0, -1.0, 0.0])
LEFT_CUP = hand([0.43, -0.13, -0.90], [1.0, 0.0, 0.0])
RIGHT_CUP = hand([0.43, 0.13, -0.90], [-1.0, 0.0, 0.0])


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    if mode != "no_paper":
        pick = C.PAPER + [0.0, 0.0, 0.001]
        lay = np.r_[C.FUNNEL + [-0.001, 0.0], C.RIM + 0.013]
        p.add(
            1.8,
            "Approach the clean paper disk",
            left=(pick + [0, 0, 0.088], PAPER_HAND, 0.94),
        )
        p.add(1.10, "Lower around the loose paper", left=(pick, PAPER_HAND, 0.94))
        p.add(
            0.55, "Grasp the edges of the filter paper", left=(pick, PAPER_HAND, 0.68)
        )
        p.add(
            1.55,
            "Lift the paper off its clean support",
            left=(pick + [0, 0, 0.237], PAPER_HAND, 0.68),
        )
        p.add(
            2.15,
            "Center the paper over the funnel plate",
            left=(lay + [0, 0, 0.066], PAPER_HAND, 0.68),
        )
        p.add(
            1.10,
            "Lower the horizontal disk above the porcelain wall",
            left=(lay, PAPER_HAND, 0.68),
        )
        p.add(0.45, "Steady the paper at the mouth", left=(lay, PAPER_HAND, 0.68))
        p.add(
            0.70,
            "Release the paper onto the perforated plate",
            left=(lay, PAPER_HAND, 0.98),
        )
        p.add(
            0.95,
            "Clear the paper and funnel",
            left=(lay + [0, 0.14, 0.065], PAPER_HAND, 0.98),
        )
    if mode == "paper_only":
        return p.moves

    def pour(vessel, name, wrist):
        pick = vessel.position + vessel.grip + [0.0, 0.0, 0.002]

        def at_lip(degrees):
            angle = radians(degrees)
            turn = rotate_y(-angle)
            # Raise the bottom of an upright jug over the high porcelain rim.
            # Once tilted, the lip can move closer to the receiving funnel.
            pos = np.r_[C.FUNNEL + [0.014, 0.0], C.RIM + 0.055 + 0.049 * cos(angle)]
            return pos + turn @ (vessel.grip - vessel.outlet), turn @ wrist, 0.09

        p.add(
            1.40,
            f"Approach the {vessel.name} beaker",
            **{name: (pick + [0, 0, 0.095], wrist, 0.84)},
        )
        p.add(1.05, "Lower around the beaker handle", **{name: (pick, wrist, 0.84)})
        p.add(0.65, "Grip the native beaker handle", **{name: (pick, wrist, 0.09)})
        p.add(
            1.70,
            "Lift the beaker upright",
            **{name: (pick + [0, 0, 0.236], wrist, 0.09)},
        )
        p.add(
            0.80,
            "Steady the lifted beaker",
            **{name: (pick + [0, 0, 0.236], wrist, 0.09)},
        )
        p.add(2.70, "Bring the beaker spout over the funnel", **{name: at_lip(0)})
        p.add(0.90, "Steady the spout over the funnel", **{name: at_lip(0)})
        p.add(1.05, "Align the tilted spout before liquid flows", **{name: at_lip(32)})
        for angle in (44, 57, 67, 76):
            p.add(0.75, "Pour slowly into the funnel", **{name: at_lip(angle)})
        p.add(1.65, "Let the measured liquid drain", **{name: at_lip(76)})
        for angle in (56, 30, 0):
            p.add(0.65, "Restore the beaker upright", **{name: at_lip(angle)})
        p.add(
            1.55, "Let the upright beaker settle before transport", **{name: at_lip(0)}
        )
        p.add(
            1.45,
            "Return the upright beaker to its place",
            **{name: (pick + [0, 0, 0.15], wrist, 0.09)},
        )
        p.add(
            1.35,
            "Lower the beaker to the bench",
            **{name: (pick + [0, 0, -0.004], wrist, 0.09)},
        )
        p.add(
            0.68,
            "Release the beaker on the bench",
            **{name: (pick + [0, 0, -0.004], wrist, 0.84)},
        )
        p.add(
            0.85,
            "Withdraw the hand from the bench",
            **{name: (pick + [-0.055, 0, 0.10], wrist, 0.84)},
        )

    if mode != "dry_paper":
        pour(C.RINSE, "left", LEFT_CUP)
    pour(C.SUSPENSION, "right", RIGHT_CUP)
    p.add(1.1, "Leave the drained funnel and the collected filtrate clear")
    return p.moves
