"""Dual-arm nozzle release, exchange and pull test."""

import numpy as np

from . import config as C
from .motion import Program

ALIGN_PLUG = "Align the plug in front of the socket"
SEAT_PLUG = "Seat the replacement plug against the valve spring"
LOCK = "Release the sleeve onto the retaining groove"
LIFT_LEFT = "Lift the left hand clear of the sleeve"
PULL_TEST = "Pull on the replacement to check the mechanical lock"
RELEASE = "Release the locked replacement"


def orientation(tilt):
    """Jaws span X; wrists approach obliquely from opposite ends."""
    tilt = np.deg2rad(tilt)
    closing = np.array([1.0, 0.0, 0.0])
    approach = np.array([0.0, np.sin(tilt), -np.cos(tilt)])
    return np.column_stack([closing, np.cross(approach, closing), approach])


def waypoints(mode):
    program = Program()
    if mode == "hold":
        return program.moves
    left_rotation = orientation(-22)
    right_rotation = orientation(18)

    def hold(point, grip=1.0):
        return (point, left_rotation, grip)

    def carry(point, grip=1.0):
        return (point, right_rotation, grip)

    left_latch = C.SOCKET + [0.0, -0.001, 0.003]
    right_grasp = C.GRAY_READY + [0.0, -0.010, 0.004]
    left_slide = left_latch + [0.0, 0.025, 0.0]
    right_pull = right_grasp - [0.0, 0.042, 0.0]
    blue_grasp = C.BLUE_READY + [0.0, -0.007, 0.002]
    blue_ready = right_grasp - [0.0, 0.039, 0.0]
    park = C.PARK + [0.0, -0.007, 0.003]

    program.add(
        2.7,
        "Approach the sleeve and old spray head",
        left=hold(left_latch + [0.0, 0.019, 0.084]),
        right=carry(right_grasp + [0.0, -0.027, 0.081]),
    )
    program.add(
        1.35,
        "Lower the jaws around both grips",
        left=hold(left_latch, 0.77),
        right=carry(right_grasp, 0.78),
    )
    program.add(
        0.78,
        "Grip the spring sleeve and old spray head",
        left=hold(left_latch, 0.26),
        right=carry(right_grasp, 0.23),
    )
    program.add(0.90, "Retract the orange sleeve", left=hold(left_slide, 0.26))
    program.add(
        1.08,
        "Pull the old head free and let the poppet shut",
        right=carry(right_pull, 0.23),
    )
    program.add(
        0.80,
        "Lift the old head clear of the tray",
        right=carry(right_pull + [0.0, 0.0, 0.042], 0.23),
    )
    program.add(
        1.40,
        "Carry the old head to the parts tray",
        right=carry(park + [0.0, 0.0, 0.042], 0.23),
        left=hold(left_slide + [0.0, 0.0, 0.002], 0.86),
    )
    program.add(
        0.92,
        "Set the old head on its tray",
        right=carry(park, 0.23),
        left=hold(left_latch + [0.0, 0.017, 0.067], 0.88),
    )
    program.add(0.55, "Let go of the parked old head", right=carry(park, 0.85))
    program.add(
        0.95, "Clear the old head", right=carry(park + [0.0, -0.019, 0.083], 0.93)
    )
    if mode == "disconnect_only":
        return program.moves

    program.add(
        2.40,
        "Reach over the replacement fan nozzle",
        right=carry(blue_grasp + [0.0, -0.012, 0.093]),
        left=hold(left_latch, 0.84),
    )
    program.add(
        1.55,
        "Lower around the replacement and grip the sleeve",
        right=carry(blue_grasp, 0.95),
        left=hold(left_latch, 0.26),
    )
    program.add(0.57, "Grip the replacement", right=carry(blue_grasp, 0.22))
    program.add(
        0.90,
        "Raise the replacement and retract the sleeve again",
        right=carry(blue_grasp + [0.0, 0.0, 0.047], 0.22),
        left=hold(left_slide, 0.26),
    )
    program.add(
        1.50,
        "Carry the replacement in line with the hose fitting",
        right=carry(blue_ready + [0.0, 0.0, 0.047], 0.22),
    )
    program.add(1.29, ALIGN_PLUG, right=carry(blue_ready, 0.22))
    program.add(1.47, SEAT_PLUG, right=carry(right_grasp + [0.0, 0.020, -0.001], 0.22))
    program.add(0.95, LOCK, left=hold(left_slide, 0.86))
    program.add(0.77, LIFT_LEFT, left=hold(left_latch + [0.0, 0.027, 0.073], 0.93))
    program.add(0.70, PULL_TEST, right=carry(right_grasp + [0.0, 0.004, -0.001], 0.22))
    program.add(0.60, RELEASE, right=carry(right_grasp + [0.0, 0.004, -0.001], 0.89))
    program.add(
        1.05,
        "Move both hands clear while the fan nozzle runs",
        right=carry(right_grasp + [0.0, -0.041, 0.090], 0.96),
    )
    return program.moves
