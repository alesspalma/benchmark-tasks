"""Extend notched rod, retract into sleeve, tilt closed tool, expose and pour."""

import numpy as np

from . import config as C
from .motion import SIDE_X, SIDE_Y, Program, rotation


def waypoints(mode="demo"):
    if mode == "hold":
        return {}
    p = Program()
    barrel = rotation("Z", np.pi) @ SIDE_Y
    closed = 0.06
    park = np.array([0.32, -0.30, 1.08])
    grasp = C.PROBE + C.SUPPORT_GRASP
    p.add(3, "approach support handle", left=(grasp + [0, 0.08, 0.03], barrel, 1))
    p.add(2, "reach support handle", left=(grasp, barrel, 1))
    p.add(2, "grasp support handle", left=(grasp, barrel, closed))
    p.add(3, "lift enclosed sampler", left=(grasp + [0, 0, 0.19], barrel, closed))
    source = C.TUB + [0, 0.012, C.SAMPLING_BASE_HEIGHT]
    p.add(
        4,
        "sampler above source",
        left=(source + C.SUPPORT_GRASP + [0, 0, 0.14], barrel, closed),
    )
    p.add(
        4,
        "position closed tip at source",
        left=(source + C.SUPPORT_GRASP, barrel, closed),
    )
    p.add(1, "settle sleeve above bed")
    if mode == "insert":
        return p.moves
    h = source + C.GATE_GRASP
    p.add(3, "approach blue rod control", right=(h + [0, -0.12, 0.06], SIDE_X, closed))
    p.add(2, "reach blue rod control", right=(h, SIDE_X, closed))
    p.add(1, "settle control reach")
    p.add(2, "close right jaws above pushbutton", right=(h, SIDE_X, closed))
    p.add(
        4,
        "extend notched rod into granules",
        right=(h + [0, 0, -C.GATE_TRAVEL - 0.006], SIDE_X, closed),
    )
    p.add(4, "granules flow into exposed notch")
    for dz in (0.008, 0, 0.008, 0):
        p.add(
            1,
            "gently settle granules into pocket",
            left=(source + C.SUPPORT_GRASP + [0, 0, dz], barrel, closed),
            right=(h + [0, 0, -C.GATE_TRAVEL - 0.006 + dz], SIDE_X, closed),
        )
    delta = np.array([0, -0.024, 0])
    p.add(
        4,
        "short fill sweep with exposed notch",
        left=(source + C.SUPPORT_GRASP + delta, barrel, closed),
        right=(h + [0, 0, -C.GATE_TRAVEL - 0.006] + delta, SIDE_X, closed),
    )
    source += delta
    h = source + C.GATE_GRASP
    if mode == "open":
        return p.moves
    p.add(4, "retract collecting rod into sleeve", right=(h, SIDE_X, closed))
    p.add(2, "settle enclosed sample")
    p.add(2, "release pushbutton", right=(h + [0, 0, 0.05], SIDE_X, closed))
    p.add(2, "clear blue control", right=(h + [0, -0.12, 0.05], SIDE_X, 1))
    p.add(2, "park right arm before withdrawal", right=(park, SIDE_X, 1))
    p.add(
        4,
        "withdraw enclosed sample over source",
        left=(source + C.SUPPORT_GRASP + [0, 0, 0.14], barrel, closed),
    )
    p.add(2, "shed exterior grains over source")
    if mode == "retain":
        return p.moves
    a = np.deg2rad(C.DISCHARGE_TILT_DEG)
    tilt = rotation("X", a)
    ori = SIDE_X  # A closed finger presses the inclined button; no wrist twist.
    left_ori = tilt @ barrel
    origin = C.JAR + [0, 0, C.DISCHARGE_HEIGHT]
    p.add(
        4,
        "closed sampler above jar",
        left=(origin + C.SUPPORT_GRASP + [0, 0, 0.10], barrel, closed),
    )
    p.add(
        4,
        "tilt enclosed notch toward jar",
        left=(origin + tilt @ C.SUPPORT_GRASP + [0, 0, 0.08], left_ori, closed),
    )
    p.add(
        3,
        "lower enclosed tip over jar",
        left=(origin + tilt @ C.SUPPORT_GRASP, left_ori, closed),
    )
    p.add(1, "settle closed tip over jar")
    h = origin + tilt @ C.GATE_GRASP
    p.add(3, "approach rod control over jar", right=(h + [0, -0.12, 0.05], ori, closed))
    p.add(2, "reach rod control over jar", right=(h, ori, closed))
    p.add(1, "settle pouring reach")
    p.add(2, "close right jaws above pouring button", right=(h, ori, closed))
    p.add(
        4,
        "expose collecting notch inside jar",
        right=(h - tilt[:, 2] * (C.GATE_TRAVEL + 0.006), ori, closed),
    )
    p.add(6, "pour sample from exposed notch by gravity")
    p.add(4, "retract empty pocket into sleeve", right=(h, ori, closed))
    p.add(2, "settle enclosed empty rod")
    p.add(2, "release pouring button", right=(h + tilt[:, 2] * 0.05, ori, closed))
    p.add(2, "withdraw right fingers", right=(h + [0, -0.12, 0.05], ori, 1))
    p.add(3, "park right arm clear of sampler", right=(park, SIDE_X, 1))
    if mode == "deliver":
        return p.moves
    p.add(
        3,
        "lift enclosed tip clear of jar",
        left=(origin + tilt @ C.SUPPORT_GRASP + [0, 0, 0.08], left_ori, closed),
    )
    p.add(4, "return upright above cradle", left=(grasp + [0, 0, 0.19], barrel, closed))
    p.add(3, "lower sampler into cradle", left=(grasp, barrel, closed))
    p.add(2, "release supported sampler", left=(grasp, barrel, 1))
    p.add(2, "settle with left fingers open")
    p.add(2, "clear left fingers", left=(grasp + [0, 0.08, 0], barrel, 1))
    p.add(3, "park left arm", left=(np.array([0.27, 0.35, 1.05]), SIDE_X, 1))
    p.add(2, "settle jar sample")
    return p.moves
