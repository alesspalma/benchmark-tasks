"""Taught ordinary arm and gripper movements in the fixed starting layout."""

import numpy as np

from . import config as C
from .motion import Program


def approach(tilt, close_y=True):
    a = np.deg2rad(tilt)
    forward = np.array([np.sin(a), 0, -np.cos(a)])
    close = np.array([0, -1, 0]) if close_y else np.array([-np.cos(a), 0, -np.sin(a)])
    return np.column_stack((close, np.cross(forward, close), forward))


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves

    lhs = approach(20)
    facing = np.array([0.22, 0.22, -0.95])
    facing /= np.linalg.norm(facing)
    closing = np.cross(facing, [1.0, 0.0, 0.0])
    closing /= np.linalg.norm(closing)
    rhs = np.column_stack((closing, np.cross(facing, closing), facing))
    finger = approach(31, close_y=False)
    press_y = 0.061
    tail_y = C.ANCHOR_Y - C.FIRST_SPAN - C.SECOND_SPAN + C.TAIL_LOCAL_Y

    def cable(x, y, z, grip=0.05):
        return (x, y, z), lhs, grip

    def gate(i, angle, dx=0, dz=0, grip=0.06):
        theta = np.deg2rad(angle)
        return (
            (
                C.HINGE_X + 0.073 * np.cos(theta) + dx,
                C.GATE_Y[i] - 0.006,
                C.BAR_Z + 0.073 * np.sin(theta) + dz,
            ),
            rhs,
            grip,
        )

    def tail(z, grip, x=C.CHAIN_X, y=tail_y):
        return (x, y, z), finger, grip

    route = mode != "miss_cables"
    p.add(
        1.8,
        "Approach over the left cable-handling bay",
        left=cable(0.352, 0.116, 1.024, 0.20) if route else None,
    )
    p.add(
        1.28,
        "Position above the raised cable pair",
        left=cable(C.CHAIN_X, press_y, 0.952) if route else None,
    )
    p.add(
        1.20,
        "Lay the paired cable below the crossbars",
        left=cable(C.CHAIN_X, press_y, 0.836) if route else None,
    )
    p.add(
        0.85,
        "Hold the cable in the channel",
        left=cable(C.CHAIN_X, press_y, 0.821) if route else None,
    )
    p.add(
        1.30,
        "Raise the right wrist above the service bay",
        right=((0.318, -0.180, 1.038), rhs, 0.08),
    )
    p.add(1.25, "Approach the first crossbar", right=gate(1, C.BAR_OPEN, dz=0.055))
    stations = (1,) if mode == "open_gate" else (1, 0)
    for i in stations:
        dx = -0.006 if i == 0 else 0.0
        p.add(
            0.85,
            "Bring the pads above the open crossbar",
            right=gate(i, C.BAR_OPEN, dz=0.022),
        )
        p.add(
            0.75, "Contact the crossbar's free end", right=gate(i, 80, dx=dx, dz=0.001)
        )
        for angle in (66, 48, 30, 14, 1):
            p.add(
                0.48,
                "Press the crossbar to its receiver",
                right=gate(i, angle, dx=dx, dz=0.001),
            )
        p.add(0.42, "Seat the snap end", right=gate(i, 0, dz=0.001))
        p.add(
            0.64,
            "Lift clear of the seated crossbar",
            right=gate(i, 0, dx=-0.056, dz=0.078),
        )
        if i == 1 and mode != "open_gate":
            p.add(
                0.91,
                "Pass above the second crossbar",
                right=gate(0, C.BAR_OPEN, dz=0.050),
            )
    p.add(0.90, "Release the cable-laying hand", left=cable(0.342, 0.075, 0.957, 0.80))
    p.add(1.0, "Move to the exposed cable tie", right=tail(0.917, 0.75))
    p.add(0.80, "Lower around the paired lead tail", right=tail(0.838, 0.75))
    p.add(0.60, "Grip the service tie", right=tail(0.838, 0.04))
    p.add(1.0, "Lift the tail against the closed crossbars", right=tail(0.869, 0.04))
    p.add(0.55, "Hold the upward proof pull")
    p.add(0.65, "Open the fingers", right=tail(0.869, 0.79))
    p.add(
        1.10,
        "Retreat clear of the cable carrier",
        right=tail(0.974, 0.87, x=0.355, y=-0.236),
    )
    p.add(0.72, "Leave both crossbars seated")
    return p.moves
