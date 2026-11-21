"""Visible compressed potato and gravity-deposited riced strands."""

from math import cos, pi, sin

import numpy as np

from . import config as C
from .scoring import transform


def potato_skin(height):
    return [
        (r * cos(t), r * sin(t), z)
        for z, base_radius in (
            (0.003, 0.021),
            (height * 0.37, 0.022),
            (height * 0.77, 0.021),
            (height, 0.019),
        )
        for i in range(24)
        for t in (2 * pi * i / 24,)
        for r in (base_radius * (1 + 0.025 * sin(5 * t) + 0.035 * cos(3 * t)),)
    ]


def create_deposits(stage):
    from pxr import UsdGeom

    result = []
    for name, count in (("RicedPotato", 45), ("FallingPotato", len(C.DIE_HOLES))):
        curves = UsdGeom.BasisCurves.Define(stage, f"/World/{name}")
        curves.CreateTypeAttr("linear")
        curves.CreateCurveVertexCountsAttr([18] * count)
        curves.CreateWidthsAttr([0.00325])
        curves.SetWidthsInterpolation("constant")
        curves.CreateDisplayColorAttr([(0.91, 0.71, 0.34)])
        curves.CreateDisplayOpacityAttr([1.0])
        curves.MakeInvisible()
        result.append(curves)
    return result


def update_deposits(curves, scorer, ricer_pose):
    from pxr import Gf

    piled, falling = curves
    if scorer.delivered < 0.05:
        piled.MakeInvisible()
    else:
        piled.MakeVisible()
        results = []
        amount = min(scorer.delivered / 37.0, 1.0)
        for row in range(45):
            radial = 0.048 * ((row + 0.6) / 45) ** 0.63
            theta = row * pi * (3 - np.sqrt(5))
            origin = C.BOWL[:2] + radial * np.array([cos(theta), sin(theta)])
            layer = 0.003 + 0.0038 * ((row * 29) % 7)
            for q in np.linspace(0, amount, 18):
                tip = np.array([cos(theta + 7.2 * q), sin(theta + 7.2 * q)])
                xy = origin + (0.005 + 0.014 * q) * tip
                z = C.BOWL_FLOOR + 0.003 + amount * layer + 0.003 * sin(13 * q)
                results.append(Gf.Vec3f(float(xy[0]), float(xy[1]), float(z)))
        piled.GetPointsAttr().Set(results)
    flying = scorer.pending
    if not flying:
        falling.MakeInvisible()
    else:
        falling.MakeVisible()
        body, axes = transform(ricer_pose)
        age = min(max(p.age for p in flying), 0.12)
        depth = min(0.050, 0.002 + 0.90 * age)
        lengths = []
        for i, (x, y) in enumerate(C.DIE_HOLES):
            top = body + axes @ np.array([x, y, C.BASKET_FLOOR - 0.001])
            for q in np.linspace(0, 1, 18):
                pos = top + [
                    0.0019 * q * sin(8 * q + i),
                    0.0009 * q * cos(7 * q + i),
                    -q * depth,
                ]
                lengths.append(Gf.Vec3f(*map(float, pos)))
        falling.GetPointsAttr().Set(lengths)
