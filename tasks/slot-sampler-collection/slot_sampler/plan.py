"""Two hands operate a passive physical sleeve, capture and discharge material."""

import math

import numpy as np

from . import config as C
from .motion import SIDE_X, SIDE_Y, Program, rotation


def waypoints(mode="demo"):
    if mode == "hold":
        return {}
    p = Program()
    offset = np.array([0.0, 0.060, 0.255])
    grasp = C.PROBE + offset
    p.add(3.0, "approach outer sleeve", left=(grasp + [-0.08, 0, 0], SIDE_X, 1.0))
    p.add(2.0, "reach outer sleeve", left=(grasp, SIDE_X, 1.0))
    p.add(2.0, "support outer sleeve", left=(grasp, SIDE_X, 0.06))
    p.add(3.0, "lift probe clear", left=(grasp + [0, 0, 0.17], SIDE_X, 0.06))
    origin = C.TUB + [0, 0, 0.014]
    target = origin + offset
    p.add(4.0, "closed probe over source", left=(target + [0, 0, 0.17], SIDE_X, 0.06))
    p.add(4.0, "insert closed sampling chamber", left=(target, SIDE_X, 0.06))
    if mode == "insert":
        return p.moves
    lever = lambda angle: origin + rotation("Z", angle) @ np.array([0.0, -0.068, 0.312])
    p.add(3.0, "approach sleeve lever", right=(lever(0) + [0, -0.08, 0], SIDE_Y, 1.0))
    p.add(2.0, "reach sleeve lever", right=(lever(0), SIDE_Y, 1.0))
    p.add(2.0, "grasp sleeve lever", right=(lever(0), SIDE_Y, 0.14))
    for a in np.linspace(0, -math.pi / 2, 13)[1:]:
        p.add(
            0.40,
            "open buried sampling window",
            right=(lever(a), rotation("Z", a) @ SIDE_Y, 0.14),
        )
    p.add(3.0, "admit granules through window")
    for delta in (
        [0, 0, 0.010],
        [0, 0, 0],
        [0, 0, 0.010],
        [0, 0, 0],
        [0.003, 0, 0],
        [-0.003, 0, 0],
        [0, 0.003, 0],
        [0, -0.003, 0],
        [0, 0, 0],
    ):
        p.add(
            0.55,
            "gently agitate open probe",
            left=(target + delta, SIDE_X, 0.06),
            right=(
                lever(-math.pi / 2) + delta,
                rotation("Z", -math.pi / 2) @ SIDE_Y,
                0.14,
            ),
        )
    if mode == "open":
        return p.moves
    for a in np.linspace(-math.pi / 2, 0.0, 13)[1:]:
        p.add(
            0.40,
            "close filled window",
            right=(lever(a), rotation("Z", a) @ SIDE_Y, 0.14),
        )
    p.add(2.0, "release sleeve lever", right=(lever(0), SIDE_Y, 1.0))
    p.add(3.0, "clear sleeve lever", right=(lever(0) + [0, -0.06, 0], SIDE_Y, 1.0))
    p.add(3.0, "withdraw retained sample", left=(target + [0, 0, 0.17], SIDE_X, 0.06))
    for delta in (
        [0.006, 0, 0],
        [-0.006, 0, 0],
        [0, 0.006, 0],
        [0, -0.006, 0],
        [0, 0, 0],
    ):
        p.add(
            0.8,
            "clear loose exterior grains over source",
            left=(target + [0, 0, 0.17] + delta, SIDE_X, 0.06),
        )
    p.add(2.0, "let loose exterior grains fall back")
    if mode == "retain":
        return p.moves
    outlet = np.array([C.JAR[0], C.JAR[1], C.TOP + 0.40])
    proximal = np.array([0.0, 0.0, 0.310])
    pivot = lambda angle: outlet + rotation("X", angle) @ (offset - proximal)
    over = pivot(0.0)
    p.add(4.0, "sample outlet over jar", left=(over, SIDE_X, 0.06))
    for a in np.linspace(0, -math.pi, 19)[1:]:
        p.add(
            0.35,
            "invert about discharge opening",
            left=(pivot(a), rotation("X", a) @ SIDE_X, 0.06),
        )
    low = pivot(-math.pi) + [0, 0, -0.315]
    inverse = rotation("X", -math.pi) @ SIDE_X
    p.add(3.0, "lower discharge opening", left=(low, inverse, 0.06))
    p.add(4.0, "discharge actual sample")
    for a in (-math.pi + 0.12, -math.pi - 0.12, -math.pi):
        p.add(
            1.0,
            "release remaining granules",
            left=(low, rotation("X", a) @ SIDE_X, 0.06),
        )
    if mode == "deliver":
        return p.moves
    p.add(3.0, "raise empty sampler", left=(pivot(-math.pi), inverse, 0.06))
    for a in np.linspace(-math.pi, 0, 19)[1:]:
        p.add(
            0.30,
            "restore probe upright",
            left=(pivot(a), rotation("X", a) @ SIDE_X, 0.06),
        )
    p.add(4.0, "return probe to stand", left=(grasp + [0, 0, 0.17], SIDE_X, 0.06))
    p.add(3.0, "support probe in stand", left=(grasp, SIDE_X, 0.06))
    p.add(2.0, "release probe", left=(grasp, SIDE_X, 1.0))
    p.add(2.0, "clear parked probe", left=(grasp + [-0.10, 0, 0.03], SIDE_X, 1.0))
    p.add(2.0, "settle sample")
    return p.moves
