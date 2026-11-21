"""Taught native arm actions for pickup, blending, switch release and set-down."""

import numpy as np

from . import config as C
from .motion import Program, rotation

FACE = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
TOOL = rotation("Y", np.radians(10.0)) @ FACE
JUG = rotation("Y", np.radians(16.0)) @ FACE
LIGHT_GRIP = 0.478
POWER_GRIP = 0.338


def stick(origin, grip=LIGHT_GRIP, x=0.0):
    return np.asarray(origin) + C.GRIP_CENTER + [x, 0.0, 0.0], TOOL, grip


def pot(gap, grip):
    return C.JUG_RESET + C.HANDLE + [-gap, 0.0, 0.0], JUG, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    below_rim = np.r_[C.JUG_RESET[:2] + [-0.003, -0.008], C.BLEND_Z]
    over_jug = below_rim + [0.0, 0.0, 0.162]
    lift_rest = C.REST + [0.0, 0.0, 0.168]
    p.add(
        2.55,
        "Approach the loose jug and the blender",
        left=pot(0.101, 0.72),
        right=stick(C.REST, 0.74, -0.103),
    )
    p.add(
        1.6,
        "Line up on both molded grips",
        left=pot(0.0, 0.72),
        right=stick(C.REST, 0.74),
    )
    p.add(
        1.1,
        "Take a light grip without pressing Power",
        left=pot(0.0, 0.125),
        right=stick(C.REST),
    )
    p.add(1.55, "Lift the guarded blade clear of its rest", right=stick(lift_rest))
    p.add(1.9, "Bring the unpowered blender over the jug", right=stick(over_jug))
    if mode == "premature":
        p.add(
            0.7,
            "Press Power while the guard is exposed",
            right=stick(over_jug, POWER_GRIP),
        )
        p.add(0.45, "Leave the motor on above the liquid")
    p.add(
        1.7,
        "Lower the bell under the surface",
        right=stick(below_rim, POWER_GRIP if mode == "premature" else LIGHT_GRIP),
    )
    p.add(0.75, "Hold the jug upright and let the guard settle")
    p.add(
        0.75,
        "Squeeze the momentary button with the guard submerged",
        right=stick(below_rim, POWER_GRIP),
    )
    for theta in np.linspace(0, 2 * np.pi, 17)[1:]:
        offset = [0.022 * np.cos(theta), 0.020 * np.sin(theta), 0.009]
        p.add(
            0.39,
            "Circulate the blade through the soup",
            right=stick(below_rim + offset, POWER_GRIP),
        )
    p.add(0.5, "Finish mixing in the middle", right=stick(below_rim, POWER_GRIP))
    if mode == "withdrawal":
        p.add(
            1.6,
            "Raise the running guard out of the jug",
            right=stick(over_jug, POWER_GRIP),
        )
        p.add(0.69, "Release Power after the guard is exposed", right=stick(over_jug))
    else:
        p.add(
            0.69,
            "Release Power while keeping the bell submerged",
            right=stick(below_rim),
        )
        p.add(0.65, "Wait for the blade to stop before withdrawing")
        p.add(
            1.6, "Lift the switched-off guard clear of the jug", right=stick(over_jug)
        )
    p.add(1.8, "Carry the blender above the drip rest", right=stick(lift_rest))
    set_down = C.REST + [0.0, 0.004, -0.005]
    p.add(
        1.5,
        "Set the blender upright on its rest",
        right=stick(set_down),
    )
    p.add(
        0.90, "Let go of both objects", left=pot(0.0, 0.82), right=stick(set_down, 0.96)
    )
    p.add(
        1.25,
        "Clear the jug and the parked blender",
        left=pot(0.133, 0.82),
        right=stick(set_down, 0.96, -0.122),
    )
    p.add(0.55, "Leave both objects hands clear")
    return p.moves
