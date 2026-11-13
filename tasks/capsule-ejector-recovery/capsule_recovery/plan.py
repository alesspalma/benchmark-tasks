"""A taught fixed-fixture program; contact physics and ordinary joint feedback."""

import math

import numpy as np

from . import config as C
from .motion import SIDE_X, Program, rotation


def waypoints(mode="demo"):
    if mode == "hold":
        return {}
    p = Program()
    head_r = SIDE_X
    closed = np.array([0.38, -0.12, C.TOP + 0.240])
    handle = lambda a: C.HINGE + rotation("Y", a) @ (closed - C.HINGE)
    h = handle(0)
    p.add(3.0, "approach head", right=(h + [-0.08, 0, 0], head_r, 1.0))
    p.add(2.0, "reach head handle", right=(h, head_r, 1.0))
    p.add(2.0, "grasp head handle", right=(h, head_r, 0.06))
    for a in np.linspace(0, math.radians(82), 21)[1:]:
        p.add(0.40, "open ejecting head", right=(handle(a), head_r, 0.06))
    p.add(
        2.0,
        "release head beyond its balance point",
        right=(handle(math.radians(82)), head_r, 1.0),
    )
    p.add(
        2.0,
        "clear open head",
        right=(handle(math.radians(82)) + [-0.08, -0.05, 0], head_r, 1.0),
    )
    p.add(1.0, "let head rest on open stop")
    if mode == "open":
        return p.moves
    grips = {
        "left": np.array([0.0, 0.100, 0.090]),
        "right": np.array([0.0, -0.100, 0.090]),
    }

    def tray(duration, label, origin, angle=0.0, opening=0.0):
        R = rotation("X", angle)
        return p.add(
            duration,
            label,
            **{
                n: (origin + R @ offset, R @ SIDE_X, opening)
                for n, offset in grips.items()
            },
        )

    tray(4.0, "approach both collector grips", C.BIN + [-0.08, 0, 0], opening=1.0)
    tray(2.0, "reach collector side grips", C.BIN, opening=1.0)
    tray(3.0, "grasp collector with both hands", C.BIN)
    tray(1.5, "lift collector off foot", C.BIN + [0, 0, 0.009])
    midway = C.BIN + [-0.105, 0, 0.009]
    tray(2.0, "draw collector clear of carrier", midway)
    tray(1.0, "raise clear collector", midway + [0, 0, 0.011])
    front = C.BIN + [-0.145, 0, 0.020]
    tray(1.5, "withdraw level collector", front)
    elevated = np.array([front[0], 0.0, C.TOP + 0.260])
    tray(3.0, "lift supported collector", elevated)
    carry = np.array([C.WASTE[0], C.WASTE[1] - 0.120, C.TOP + 0.260])
    tray(3.0, "collector over recycling tub", carry)
    for a in np.linspace(0, math.radians(-140), 16)[1:]:
        tray(0.35, "tip collector into recycling tub", carry, a)
    p.add(3.0, "let all spent capsules fall")
    for a in (math.radians(-135), math.radians(-145), math.radians(-140)):
        tray(0.7, "clear collector", carry, a)
    for a in np.linspace(math.radians(-140), 0, 13)[1:]:
        tray(0.25, "restore collector level", carry, a)
    tray(3.0, "return collector to front", elevated)
    tray(3.0, "lower collector", front)
    tray(3.0, "fit collector on guides", C.BIN + [0, 0, 0.020])
    tray(1.5, "support collector on foot", C.BIN)
    tray(2.0, "release collector", C.BIN, opening=1.0)
    tray(2.0, "clear collector grips", C.BIN + [-0.10, 0, 0.04], opening=1.0)
    if mode == "empty":
        return p.moves
    pick = C.FRESH + [0, 0, 0.019]
    capsule_r = rotation("X", -math.pi / 2) @ SIDE_X
    p.add(
        3.0, "approach fresh capsule", left=(pick + [-0.07, 0, 0.025], capsule_r, 1.0)
    )
    p.add(2.0, "reach fresh capsule", left=(pick, capsule_r, 1.0))
    p.add(2.0, "grasp fresh capsule", left=(pick, capsule_r, 0.15))
    high = np.array([C.FRESH[0], C.FRESH[1], C.TOP + 0.24])
    p.add(2.0, "lift fresh capsule", left=(high, capsule_r, 0.15))
    target = C.SLOT + [0, 0, 0.019]
    p.add(
        3.0,
        "fresh capsule over carrier",
        left=(target + [-0.03, 0, 0.05], capsule_r, 0.15),
    )
    p.add(3.0, "seat fresh capsule", left=(target, capsule_r, 0.15))
    p.add(2.0, "release fresh capsule", left=(target, capsule_r, 1.0))
    p.add(
        2.0,
        "clear loading carrier",
        left=(target + [-0.10, 0.09, 0.07], capsule_r, 1.0),
    )
    if mode == "reload":
        return p.moves
    opened = math.radians(85)
    p.add(
        3.0, "approach open head", right=(handle(opened) + [-0.07, 0, 0], head_r, 1.0)
    )
    p.add(2.0, "reach open head", right=(handle(opened), head_r, 1.0))
    p.add(2.0, "grasp open head", right=(handle(opened), head_r, 0.06))
    for a in np.linspace(opened, 0, 21)[1:]:
        p.add(0.35, "close head on fresh capsule", right=(handle(a), head_r, 0.06))
    p.add(2.0, "release head", right=(handle(0), head_r, 1.0))
    p.add(2.0, "clear head", right=(handle(0) + [-0.1, -0.06, 0.05], head_r, 1.0))
    p.add(2.0, "settle final assembly")
    return p.moves
