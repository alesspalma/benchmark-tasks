"""Repeatable reference actions for the two-handed inflation sequence."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def pinch(axis, approach):
    z = np.asarray(approach) / np.linalg.norm(approach)
    x = np.asarray(axis) - np.dot(axis, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


NEEDLE_ROT = pinch([1., 0., 0.], [.40, -.40, -1.])
VALVE_ROT = rotation("X", .088) @ NEEDLE_ROT
BARREL_ROT = pinch([-1., 0., 0.], [.50, -.30, -.40])
ROD_ROT = pinch([0., -1., 0.], [.80, 0., -.40])
BALL_ROT = pinch([1., 0., 0.], [.50, 1., -.65])


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    dock = C.NEEDLE + [0, 0, .008]
    clear_dock = dock + [0, 0, .145]
    port = C.PORT + [0., -.016, -.0055]
    axis = np.array([0., -.088, .996])
    port_top = port + axis * .070
    coupler = port + axis * .0138
    steady = C.BALL + [-.042, -.116, .004]
    steady_clear = steady + [-.025, -.080, .044]
    barrel = C.PUMP + C.PUMP_ROT @ C.BARREL_GRIP + [0, 0, .007]
    barrel_approach = barrel + [-.060, .006, .063]
    rod_lower = C.PUMP + C.PUMP_AXIS * C.PISTON_HOME + [0, 0, .006]
    rod_upper = rod_lower + C.PUMP_AXIS * (C.STROKE+.012)
    rod_clear = rod_lower + [-.035, -.055, .105]
    if mode == "demo":
        p.add(2.2, "Find the ball valve and the needle", right=(steady_clear, BALL_ROT, .34), left=(clear_dock, NEEDLE_ROT, .68))
        p.add(1.4, "Steady the football and approach the needle", right=(steady, BALL_ROT, .12), left=(dock, NEEDLE_ROT, .57))
        p.add(.5, "Pinch the hose chuck", left=(dock, NEEDLE_ROT, .08))
        p.add(1., "Lift the needle straight off its rack", left=(clear_dock, NEEDLE_ROT, .08))
        p.add(1.7, "Hold the needle square above the valve", left=(port_top, VALVE_ROT, .08))
        p.add(2.2, "Center the needle over the valve opening", left=(coupler, VALVE_ROT, .08))
        p.add(2.9, "Seat the needle in the rubber valve", left=(coupler, VALVE_ROT, .08))
        p.add(.48, "Release the inserted chuck", left=(coupler, VALVE_ROT, .60))
        p.add(1.2, "Clear the seated needle", left=(port_top, VALVE_ROT, .60), right=(steady_clear, BALL_ROT, .48))
    p.add(1.3, "Clear the football", left=(barrel_approach+[.025,0,.100], BARREL_ROT, .68),
          right=(rod_clear, ROD_ROT, .70))
    p.add(1.2, "Approach the barrel and piston", left=(barrel_approach, BARREL_ROT, .66))
    p.add(1.25, "Line up both pump grips", left=(barrel, BARREL_ROT, .62), right=(rod_lower, ROD_ROT, .50))
    p.add(.65, "Hold the barrel and piston handle", left=(barrel, BARREL_ROT, .10), right=(rod_lower, ROD_ROT, .10))
    for _ in range(6):
        p.add(.9, "Draw the piston up against the barrel grip", right=(rod_upper, ROD_ROT, .10))
        p.add(.9, "Push the piston down", right=(rod_lower - C.PUMP_AXIS * .010, ROD_ROT, .10))
    p.add(.6, "Release the pump", left=(barrel, BARREL_ROT, .61), right=(rod_lower, ROD_ROT, .66))
    p.add(1.4, "Withdraw from the pump", left=(barrel_approach, BARREL_ROT, .61), right=(rod_clear, ROD_ROT, .66))
    if mode == "demo":
        p.add(2., "Return to the needle and ball", right=(steady_clear, BALL_ROT, .48), left=(port_top, VALVE_ROT, .60))
        p.add(1.45, "Grip near the valve again", left=(coupler, VALVE_ROT, .55))
        p.add(1.65, "Center the open jaws on the chuck", left=(coupler, VALVE_ROT, .55))
        p.add(.54, "Close on the hose chuck", left=(coupler, VALVE_ROT, .08))
        p.add(1.45, "Withdraw the needle straight", left=(port_top, VALVE_ROT, .08))
        p.add(1.7, "Return the needle to its rack", right=(steady_clear, BALL_ROT, .48), left=(clear_dock, NEEDLE_ROT, .08))
        p.add(1.2, "Lower the needle into the rack", left=(dock, NEEDLE_ROT, .08))
        p.add(.54, "Release the needle", left=(dock, NEEDLE_ROT, .65))
        p.add(1.3, "Leave the ball and pump clear", left=(clear_dock, NEEDLE_ROT, .65))
    return p.moves
