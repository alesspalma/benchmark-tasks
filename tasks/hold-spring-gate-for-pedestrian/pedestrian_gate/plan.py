"""Taught ordinary gripper actions for the fixed gate layout."""

import numpy as np
from yam_arena import ik

from . import config as C
from .motion import Program, rotation

# Jaw closure is horizontal, with a slight approach from the robot side.
FRONT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
RIGHT = rotation("Y", np.radians(30.0)) @ FRONT
LEFT = rotation("Y", np.radians(38.0)) @ FRONT


def gate_grip(angle, dz=0.0, grip=0.06):
    theta = np.radians(angle)
    center = C.HINGE + [
        C.HOLD_RADIUS * np.sin(theta) - C.HANDLE_OFFSET * np.cos(theta),
        -C.HOLD_RADIUS * np.cos(theta) - C.HANDLE_OFFSET * np.sin(theta),
        C.RAIL_HEIGHT + dz,
    ]
    return center, LEFT, grip


def latch_grip(z=0.0, dx=0.0, grip=0.08):
    return C.BOLT_GRIP + [dx, 0.0, z], RIGHT, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    program = Program()
    program.add(
        1.6,
        "Approach the latch and pull handle",
        left=(gate_grip(0.0)[0] + [-0.090, 0.0, 0.037], LEFT, 0.72),
        right=latch_grip(dx=-0.085, grip=0.56),
    )
    program.add(
        1.3,
        "Put fingers around the handle and lift knob",
        left=gate_grip(0.0, 0.0, 0.72),
        right=latch_grip(grip=0.56),
    )
    program.add(
        0.7, "Grasp both gate controls", left=gate_grip(0.0), right=latch_grip()
    )
    if mode != "bypass_latch":
        program.add(
            1.07,
            "Raise the latch bolt out of its keeper",
            right=latch_grip(z=C.LIFT + 0.009),
        )
    for angle in np.linspace(0.0, C.OPEN_TARGET, 12)[1:]:
        program.add(
            0.31, "Open the gate against the closing spring", left=gate_grip(angle)
        )
    right_park = (*ik.tcp_pose(np.asarray(C.STOWED["right"]), C.BASES["right"]), 0.17)
    if mode == "premature":
        program.add(
            0.42,
            "Let go before the pedestrian clears",
            left=gate_grip(C.OPEN_TARGET, 0.0, 1.0),
            right=latch_grip(z=0.046, grip=0.85),
        )
        program.add(
            0.88,
            "Retract without carrying the spring load",
            left=((0.565, 0.403, 1.039), LEFT, 0.93),
            right=right_park,
        )
        program.add(
            1.2,
            "Remain clear of the uncontrolled return",
            left=((0.390, 0.402, 1.110), LEFT, 0.90),
        )
        return program.moves
    program.add(0.45, "Release the open latch", right=latch_grip(z=0.046, grip=0.85))
    program.add(0.85, "Park the latch hand beside the path", right=right_park)
    program.add(4.02, "Hold the gate while the person passes")
    program.add(
        1.10,
        "Regrasp the latch while the gate is held clear",
        right=latch_grip(grip=0.56),
    )
    program.add(0.55, "Grip the lift knob", right=latch_grip())
    program.add(
        0.8,
        "Raise the latch to accept the returning keeper",
        right=latch_grip(z=C.LIFT + 0.009),
    )
    for angle in np.linspace(C.OPEN_TARGET, 0.0, 13)[1:]:
        program.add(
            0.25, "Guide the gate back to the closed stop", left=gate_grip(angle)
        )
    program.add(0.70, "Settle against the closed stop", left=gate_grip(-3.0))
    program.add(0.68, "Let the latch drop into the keeper", right=latch_grip(grip=0.72))
    program.add(0.75, "Let go of the latched gate", left=gate_grip(0.0, 0.008, 0.79))
    program.add(
        1.3,
        "Clear both gate controls",
        left=(gate_grip(0.0)[0] + [-0.105, 0.0, 0.125], LEFT, 0.79),
        right=latch_grip(dx=-0.12, z=0.055, grip=0.78),
    )
    return program.moves
