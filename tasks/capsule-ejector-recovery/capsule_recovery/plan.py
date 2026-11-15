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
    closed = C.HEAD_HANDLE
    handle = lambda a: C.HINGE + rotation("Y", a) @ (closed - C.HINGE)
    h = handle(0)
    p.add(3.0, "approach head", right=(h + [-0.08, 0, 0], head_r, 1.0))
    p.add(2.0, "reach head handle", right=(h, head_r, 1.0))
    p.add(2.0, "grasp head handle", right=(h, head_r, 0.06))
    for a in np.linspace(0, math.radians(82), 21)[1:]:
        p.add(0.30, "open ejecting head", right=(handle(a), head_r, 0.06))
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

    # The left arm keeps one front-bar grasp throughout the collection cycle.
    # The collector slides straight out before any lift or lateral transfer.
    def tray(duration, label, origin, angle=0.0, opening=0.0):
        R = rotation("X", angle)
        return p.add(
            duration,
            label,
            left=(origin + R @ C.BIN_GRIP, R @ SIDE_X, opening),
        )

    grip = C.BIN + C.BIN_GRIP
    p.add(
        3.0,
        "collector approach above front",
        left=(grip + [-0.075, 0, 0.08], SIDE_X, 1.0),
    )
    p.add(2.0, "lower in front of collector", left=(grip + [-0.075, 0, 0], SIDE_X, 1.0))
    tray(2.0, "reach collector front bar", C.BIN, opening=1.0)
    tray(2.0, "grasp collector front bar", C.BIN)
    slide = C.BIN + [0, 0, 0.002]
    tray(1.0, "unweight collector on guides", slide)
    front = slide + [-0.165, 0, 0]
    tray(3.0, "withdraw collector straight from machine", front)
    elevated = np.array([front[0], 0.0, C.TOP + 0.240])
    tray(3.0, "lift clear collector", elevated)
    carry = np.array([C.WASTE[0], C.WASTE[1] - 0.100, C.TOP + 0.240])
    alongside = np.array([front[0], carry[1], elevated[2]])
    tray(3.0, "move collector clear of housing side", alongside)
    tray(1.5, "collector over recycling tub", carry)
    for a in np.linspace(0, math.radians(-130), 21)[1:]:
        tray(0.3, "tip collector into recycling tub", carry, a)
    p.add(3.0, "let spent capsules fall")
    for a in (math.radians(-125), math.radians(-132), math.radians(-130)):
        tray(0.6, "clear collector", carry, a)
    for a in np.linspace(math.radians(-130), 0, 16)[1:]:
        tray(0.25, "restore collector level", carry, a)
    tray(1.5, "withdraw collector clear of housing side", alongside)
    tray(3.0, "return collector to clear front", elevated)
    tray(3.0, "lower collector above guide lips", front + [0, 0, 0.038])
    # Allow for the pad's contact centre, a few mm beyond its nominal TCP.
    fitted = C.BIN + [-0.003, 0, 0]
    tray(3.0, "slide collector back into machine", fitted + [0, 0, 0.040])
    tray(2.0, "support collector on foot", fitted)
    tray(2.0, "release fitted collector", fitted, opening=1.0)
    tray(
        2.0,
        "withdraw fingers straight from front bar",
        C.BIN + [-0.080, 0, 0],
        opening=1.0,
    )
    tray(
        2.0, "lift clear of refitted collector", C.BIN + [-0.080, 0, 0.140], opening=1.0
    )
    if mode == "empty":
        return p.moves
    pick = C.FRESH + [0, 0, 0.019]
    capsule_r = rotation("X", -math.pi / 2) @ SIDE_X
    p.add(
        2.0,
        "clear machine toward fresh pedestal",
        left=(np.array([0.32, C.FRESH[1], C.TOP + 0.27]), capsule_r, 1.0),
    )
    p.add(
        3.0, "approach fresh capsule", left=(pick + [-0.07, 0, 0.025], capsule_r, 1.0)
    )
    p.add(2.0, "reach fresh capsule", left=(pick, capsule_r, 1.0))
    p.add(2.0, "grasp fresh capsule", left=(pick, capsule_r, 0.15))
    high = np.array([C.FRESH[0], C.FRESH[1], C.TOP + 0.37])
    p.add(2.0, "lift fresh capsule", left=(high, capsule_r, 0.15))
    # Align above the carrier before lowering. The measured nominal grasp is
    # under the flange; allow a few millimetres of release clearance above it.
    target = C.SLOT + [0.003, 0, 0.027]
    p.add(
        3.0,
        "fresh capsule over carrier",
        left=(np.array([target[0], 0.0, high[2]]), capsule_r, 0.15),
    )
    p.add(
        2.0,
        "lower above open carrier",
        left=(target + [0, 0, 0.045], capsule_r, 0.15),
    )
    p.add(3.0, "seat fresh capsule", left=(target, capsule_r, 0.15))
    p.add(2.0, "release fresh capsule", left=(target, capsule_r, 1.0))
    p.add(
        2.0,
        "clear loading carrier",
        left=(target + [0, 0, 0.060], capsule_r, 1.0),
    )
    p.add(
        2.0,
        "raise left arm clear of machine",
        left=(np.array([0.32, 0.0, high[2]]), capsule_r, 1.0),
    )
    p.add(
        2.0,
        "park left arm on its side",
        left=(np.array([0.30, 0.32, high[2]]), capsule_r, 1.0),
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
    p.add(1.0, "settle final assembly")
    return p.moves
