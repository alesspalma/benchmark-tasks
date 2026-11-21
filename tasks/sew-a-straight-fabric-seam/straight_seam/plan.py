"""Bimanual reference waypoints for aligning, sewing and lifting the seam."""

import numpy as np

from . import config as C
from .motion import Program


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves

    top, bottom = C.UPPER + C.UPPER_CLIP, C.LOWER + C.LOWER_CLIP
    top_goal = C.UPPER_WORK + C.UPPER_CLIP + [0, 0.007, -0.005]
    bottom_goal = C.LOWER_WORK + C.LOWER_CLIP + [0.012, -0.004, -0.004]
    if mode == "misaligned":
        top_goal += [0.0, 0.026, 0.0]
    standby_left = np.array([0.242, 0.173, C.BED + 0.124])
    standby_right = np.array([0.211, -0.205, C.BED + 0.117])
    lever_high = C.LEVER + [0, 0, C.LEVER_TRAVEL - 0.0010]
    lever_low = C.LEVER + [0, 0, 0.001]
    begin_lever_high = lever_high + [0, 0, 0.048]
    begin_lever_low = lever_low + [0, 0, 0.062]
    button_near = C.BUTTON + [0, 0, 0.064]
    button_press = C.BUTTON + [0, 0, -0.001]
    hover = [0, 0, 0.080]

    p.add(
        1.55,
        "Approach both loose swatches",
        left=(top + hover, C.LEFT_GRIP, 1),
        right=(bottom + hover, C.RIGHT_GRIP, 1),
    )
    p.add(
        0.85,
        "Place the jaws over the two sewing clips",
        left=(top, C.LEFT_GRIP, 0.62),
        right=(bottom, C.RIGHT_GRIP, 0.62),
    )
    p.add(
        0.55,
        "Grasp the separate swatches",
        left=(top, C.LEFT_GRIP, 0.15),
        right=(bottom, C.RIGHT_GRIP, 0.15),
    )
    p.add(
        1.03,
        "Lift fabric above the machine bed",
        left=(top + [0, 0, 0.025], C.LEFT_GRIP, 0.15),
        right=(bottom + [0, 0, 0.021], C.RIGHT_GRIP, 0.15),
    )
    p.add(
        1.26,
        "Position the teal underlay under the raised foot",
        right=(bottom_goal + [-0.011, 0, 0.011], C.RIGHT_GRIP, 0.15),
    )
    p.add(
        1.30,
        "Lower teal swatch to the needle plate",
        right=(bottom_goal, C.RIGHT_GRIP, 0.15),
    )
    p.add(0.60, "Release the teal outer edge", right=(bottom_goal, C.RIGHT_GRIP, 0.75))
    p.add(
        0.67,
        "Clear the teal clip before moving across the machine",
        right=(bottom_goal + [-0.009, 0, 0.051], C.RIGHT_GRIP, 0.90),
    )
    p.add(
        1.10,
        "Align the orange edge at the seam guide",
        left=(top_goal + [-0.010, 0, 0.011], C.LEFT_GRIP, 0.15),
        right=(standby_right, C.RIGHT_GRIP, 0.76),
    )
    p.add(1.36, "Lower the orange lap over teal", left=(top_goal, C.LEFT_GRIP, 0.15))
    p.add(0.64, "Release the aligned orange edge", left=(top_goal, C.LEFT_GRIP, 0.72))
    p.add(
        0.96,
        "Clear the sewing area for the lifter",
        left=(standby_left, C.LEFT_GRIP, 0.72),
        right=(standby_right + [0.000, -0.009, 0.018], C.RIGHT_GRIP, 0.74),
    )
    p.add(
        0.65,
        "Approach the raised lifter from the front",
        right=(begin_lever_high, C.RIGHT_GRIP, 0.74),
    )
    if mode != "foot_up":
        p.add(
            0.92,
            "Reach to the raised presser-foot lifter",
            right=(lever_high, C.RIGHT_GRIP, 0.74),
        )
        p.add(0.42, "Grip the lifter", right=(lever_high, C.RIGHT_GRIP, 0.14))
        p.add(
            0.82,
            "Lower the presser foot over both layers",
            right=(lever_low, C.RIGHT_GRIP, 0.14),
        )
        p.add(
            0.37,
            "Release the lifter at its lower stop",
            right=(lever_low, C.RIGHT_GRIP, 0.78),
        )
        p.add(
            0.65,
            "Back away from the presser foot",
            right=(begin_lever_low, C.RIGHT_GRIP, 0.78),
        )
    p.add(
        0.90, "Approach the green start button", right=(button_near, C.RIGHT_GRIP, 0.14)
    )
    p.add(
        0.73,
        "Push start and let the feed dogs take the cloth",
        right=(button_press, C.RIGHT_GRIP, 0.12),
    )
    p.add(0.73, "Release the start button", right=(button_near, C.RIGHT_GRIP, 0.12))
    p.add(3.62, "Sew straight through the overlap")
    p.add(0.73, "Press stop", right=(button_press, C.RIGHT_GRIP, 0.12))
    p.add(
        0.72, "Clear the start and stop switch", right=(button_near, C.RIGHT_GRIP, 0.72)
    )
    if mode != "foot_up":
        p.add(
            0.98,
            "Approach beneath the lifter",
            right=(begin_lever_low, C.RIGHT_GRIP, 0.72),
        )
        p.add(
            0.82,
            "Reach to the lowered presser-foot lifter",
            right=(lever_low, C.RIGHT_GRIP, 0.72),
        )
        p.add(0.38, "Take the lifter again", right=(lever_low, C.RIGHT_GRIP, 0.14))
        p.add(0.79, "Raise the presser foot", right=(lever_high, C.RIGHT_GRIP, 0.14))
        p.add(0.38, "Release the raised lifter", right=(lever_high, C.RIGHT_GRIP, 0.75))
        p.add(
            0.81, "Withdraw the control hand", right=(standby_right, C.RIGHT_GRIP, 0.79)
        )
    proof = top_goal + [0.090, 0, 0]
    p.add(
        1.30,
        "Approach the orange proof clip",
        left=(proof + [0, 0.037, 0.082], C.PROOF_HAND, 0.72),
    )
    p.add(1.22, "Lower jaws onto the orange clip", left=(proof, C.PROOF_HAND, 0.72))
    p.add(0.57, "Grip only the orange swatch", left=(proof, C.PROOF_HAND, 0.15))
    pull = proof + [-0.110, 0.028, 0.039]
    raised = proof + [-0.115, 0.039, 0.143]
    p.add(1.36, "Pull the seam clear of the machine", left=(pull, C.PROOF_HAND, 0.12))
    p.add(1.35, "Lift the seam by the orange swatch", left=(raised, C.PROOF_HAND, 0.08))
    p.add(1.12, "Show the teal swatch hanging unsupported")
    return p.moves
