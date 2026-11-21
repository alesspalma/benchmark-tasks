"""Taught Cartesian reference waypoints for mounting and loading the basket."""

import numpy as np

from . import config as C
from .motion import Program, rotation

DOWN = np.diag([1.0, -1.0, -1.0])
CUP_ROT = rotation("Y", -0.77) @ DOWN @ rotation("Z", np.pi / 2)
BOTTLE_ROT = rotation("Y", -0.67) @ DOWN @ rotation("Z", np.pi / 2)
CUP_GRIP = 0.14
BOTTLE_GRIP = 0.12


def tab(center, side, angle, lift=0.0, grip=CUP_GRIP):
    pivot = np.asarray(center) + C.LEVER_PIVOT + [0.0, side * C.CUP_Y, 0.0]
    end = rotation("Y", np.deg2rad(angle)) @ [0.0, 0.0, -0.034]
    return pivot + end + [-0.001, 0.0, 0.0025 + lift], CUP_ROT, grip


def bottle(origin, lift=0.0, grip=BOTTLE_GRIP):
    return np.asarray(origin) + [-0.003, 0.0, 0.078 + lift], BOTTLE_ROT, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    start = C.BASKET_START.copy()
    target = C.BASKET_MOUNT.copy()
    if mode == "grout":
        target[1] += 0.074
    stage = target + [-0.043, 0.0, 0.0]
    lifted = start + [0.0, 0.0, target[2] - start[2]]
    catch = C.BOTTLE_START.copy()
    placed = target + [0.005, -0.067, 0.012]
    a = C.LEVER_ANGLE_OPEN
    pressed = target + [0.010, 0.0, 0.0]
    p.add(
        1.8,
        "Approach both open cup levers",
        left=tab(start, 1, a, 0.063, 0.56),
        right=tab(start, -1, a, 0.063, 0.56),
    )
    p.add(
        1.18,
        "Straddle the two mounting levers",
        left=tab(start, 1, a, 0.0, 0.57),
        right=tab(start, -1, a, 0.0, 0.57),
    )
    p.add(
        0.68,
        "Grip both mounting levers",
        left=tab(start, 1, a),
        right=tab(start, -1, a),
    )
    p.add(
        1.20, "Raise the loose basket", left=tab(lifted, 1, a), right=tab(lifted, -1, a)
    )
    p.add(
        1.4,
        "Bring both cups square to the wall",
        left=tab(stage, 1, a),
        right=tab(stage, -1, a),
    )
    p.add(
        1.3,
        "Compress both cup lips on the tile",
        left=tab(pressed, 1, a),
        right=tab(pressed, -1, a),
    )
    p.add(
        0.8,
        "Start rotating the two levers while holding the basket square",
        left=tab(pressed, 1, 37.0),
        right=tab(pressed, -1, a if mode == "one-cup" else 37.0),
    )
    p.add(
        0.82,
        "Squeeze the two locking levers down",
        left=tab(pressed + [0.002, 0.0, 0.0], 1, 15.0),
        right=tab(pressed + [0.002, 0.0, 0.0], -1, a if mode == "one-cup" else 15.0),
    )
    p.add(
        0.68,
        "Finish both cam strokes",
        left=tab(pressed + [0.002, 0.0, 0.0], 1, 3.0),
        right=tab(pressed + [0.002, 0.0, 0.0], -1, a if mode == "one-cup" else 3.0),
    )
    close_right = a if mode == "one-cup" else 3.0
    p.add(
        0.73,
        "Maintain pressure as the closed seals settle",
        left=tab(pressed + [-0.003, 0.0, 0.003], 1, 3.0),
        right=tab(pressed + [-0.003, 0.0, 0.003], -1, close_right),
    )
    p.add(
        0.89,
        "Let the cup seals take the weight",
        left=tab(pressed + [-0.003, 0.0, 0.003], 1, 3.0, 0.0, 0.65),
        right=tab(pressed + [-0.003, 0.0, 0.003], -1, close_right, 0.0, 0.65),
    )
    p.add(
        0.88,
        "Withdraw clear of both locks",
        left=tab(target + [-0.039, 0.0, 0.0], 1, 3.0, 0.058, 0.53),
        right=tab(target + [-0.039, 0.0, 0.0], -1, close_right, 0.058, 0.53),
    )
    if mode == "grout":
        p.add(1.0, "Withdraw from cups that cross grout")
        return p.moves
    p.add(
        1.45,
        "Approach the shampoo bottle",
        left=tab(target + [-0.164, 0.100, 0.0], 1, 3.0, 0.065, 0.36),
        right=bottle(catch, 0.047, 0.61),
    )
    p.add(0.75, "Straddle the bottle shoulders", right=bottle(catch, 0.0, 0.62))
    p.add(0.72, "Grasp the bottle shoulders", right=bottle(catch))
    p.add(0.95, "Lift the bottle from the counter", right=bottle(catch, 0.165))
    p.add(1.33, "Carry the bottle over the basket", right=bottle(placed, 0.093))
    p.add(1.13, "Seat the bottle upright on the draining base", right=bottle(placed))
    p.add(0.55, "Release the bottle", right=bottle(placed, 0.0, 0.67))
    p.add(
        0.81,
        "Withdraw from the seated bottle",
        right=bottle(placed + [-0.029, 0.0, 0.0], 0.060, 0.61),
    )
    p.add(
        0.95,
        "Clear the basket and let the load settle",
        right=bottle(placed + [-0.115, -0.118, 0.0], 0.076, 0.56),
    )
    return p.moves
