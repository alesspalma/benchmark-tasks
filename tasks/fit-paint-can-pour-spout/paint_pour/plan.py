"""Ordinary gripper poses for supporting, pressing and pouring."""

import numpy as np

from . import config as C
from .motion import Program


def rot_y(degrees):
    a = np.deg2rad(degrees)
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    bail = C.HANDLE + [0.0, 0.0, 0.012]
    right = C.CAN + bail
    left = C.LID + C.LID_GRIP
    left_hi = left + [0.0, 0.0, 0.073]
    seat = C.CAN + C.LID_SEAT + C.LID_GRIP
    past_spout = seat + [-0.013, 0.0, 0.065]
    far = seat + [-0.060, 0.235, 0.028]
    brace = C.SIDE_GRIP @ np.diag([-1.0, -1.0, 1.0])
    lid_hand = rot_y(-24) @ brace
    clear_hand = rot_y(-46) @ brace
    bail_hand = C.DOWN
    gr = 0.10
    gl = 0.14
    p.add(
        2.1,
        "Approach the wire bail and separate spout",
        left=(left_hi, lid_hand, 1),
        right=(right + [0, 0, 0.055], bail_hand, 0.72),
    )
    p.add(
        1.25,
        "Lower the left and right hands to their grips",
        left=(left, lid_hand, 1),
        right=(right, bail_hand, 0.72),
    )
    p.add(
        0.8,
        "Grip the can bail and the separate pour lid",
        left=(left, lid_hand, gl if mode != "open_pour" else 1),
        right=(right, bail_hand, gr),
    )
    p.add(
        1.5,
        "Lift the free pouring lid",
        left=(left + [0.0, 0.0, 0.196], lid_hand, gl if mode != "open_pour" else 1),
    )
    if mode != "open_pour":
        p.add(
            2.0,
            "Center the pour lid over the can rim",
            left=(seat + [0.0, 0.0, 0.018], lid_hand, gl),
        )
        p.add(
            1.6,
            "Seat the flexible lid while holding the can",
            left=(seat + [0.0, 0.0, -0.012], lid_hand, gl),
        )
        p.add(0.55, "Press the skirt past the rolled bead")
        p.add(
            0.9,
            "Open the lid hand once the seal is seated",
            left=(seat, lid_hand, 1),
        )
        p.add(
            1.35,
            "Lift the open lid hand clear",
            left=(past_spout, clear_hand, 1),
        )
        p.add(
            0.85,
            "Withdraw toward the front away from the lid",
            left=(past_spout + [-0.10, 0.0, 0.0], clear_hand, 1),
        )
        p.add(
            1.1,
            "Move the lid hand clear of the pouring area",
            left=(far, clear_hand, 1),
        )
    if mode == "seal_only":
        p.add(0.7, "Release the supported can", right=(right, bail_hand, 1))
        p.add(1.1, "Clear the can", right=(right + [0, 0, 0.055], bail_hand, 1))
        return p.moves
    center = C.CAN + [0.0, 0.0, 0.096]
    lower_bail = C.AUX_HANDLE + [0.0, 0.0, 0.012]
    support = C.CAN + lower_bail
    support_hand = bail_hand @ np.diag([-1.0, -1.0, 1.0])
    return_offset = np.array([0.032, 0.0, 0.0])
    p.add(
        1.1,
        "Move the lid hand above the auxiliary can grip",
        left=(support + [0.0, 0.0, 0.062], support_hand, 0.75),
    )
    p.add(1.0, "Approach the auxiliary grip", left=(support, support_hand, 0.75))
    p.add(
        0.60,
        "Take the second can grip for the pour",
        left=(support, support_hand, 0.11),
    )

    def pouring(angle, offset=0.0):
        r = rot_y(angle)
        position = center + [
            -0.081 * r[0, 2] + offset,
            0.041 * r[0, 2],
            -0.062 * r[0, 2],
        ]
        return {
            "left": (position + r @ lower_bail, r @ support_hand, 0.11),
            "right": (position + r @ bail, r @ bail_hand, gr),
        }

    p.add(1.8, "Lift the can with both hands supporting it", **pouring(0))
    p.add(1.7, "Tilt the spout toward the roller tray", **pouring(-50))
    p.add(1.2, "Begin a measured pour", **pouring(-80))
    p.add(3.6, "Pour through the seated lid", **pouring(-88))
    p.add(1.55, "Pour through the seated lid", **pouring(-88))
    p.add(2.1, "Return the can toward upright to stop the flow", **pouring(-37))
    p.add(1.4, "Hold the can vertically", **pouring(0, return_offset[0]))
    p.add(
        1.55,
        "Set the can upright on its mat",
        left=(support + return_offset, support_hand, 0.11),
        right=(right + return_offset, bail_hand, gr),
    )
    p.add(
        0.65,
        "Release the can with the lid still secured",
        left=(support + return_offset, support_hand, 1),
        right=(right + return_offset, bail_hand, 1),
    )
    p.add(
        1.05,
        "Withdraw from the can and tray",
        left=(support + return_offset + [0, 0, 0.055], support_hand, 1),
        right=(right + return_offset + [0, 0, 0.055], bail_hand, 1),
    )
    p.add(0.65, "Leave the filled tray steady")
    return p.moves
