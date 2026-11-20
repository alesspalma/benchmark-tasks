"""Build the commercial two-line attachment and the instrumented inert reset."""
from math import cos, pi, sin

import numpy as np

from . import config as C
from .motion import rotation
from .usd_scene import Writer

ORANGE = (0.83, 0.21, 0.055)
BLACK = (0.115, 0.121, 0.126)
RUBBER = (0.19, 0.20, 0.21)
ALUMINUM = (0.60, 0.63, 0.65)
BRASS = (0.62, 0.46, 0.19)
NYLON = (0.91, 0.45, 0.04)


def bench(w):
    s = w.stage("bench")
    w.box(s, "/Asset/Table", (1.80, 1.25, 0.045),
          (0.625, 0, C.TABLE - 0.0225), (0.67, 0.62, 0.54))
    for i, x in enumerate((-0.18, 1.43)):
        for j, y in enumerate((-0.55, 0.55)):
            height = C.TABLE - 0.045
            w.box(s, f"/Asset/Leg{i}{j}", (0.055, 0.055, height),
                  (x, y, height / 2), (0.21, 0.23, 0.25))
    w.box(s, "/Asset/Floor", (8., 8., 0.02), (0, 0, -0.01), (0.34, 0.38, 0.39))
    for i, side in enumerate(("left", "right")):
        y = C.line_center(side)[1]
        for j, spacing in enumerate((-0.060, 0.060)):
            mid = np.array([C.STOCK_X, y + spacing, C.STOCK_Z])
            height = mid[2] - C.TABLE - C.LINE_DIAMETER / 2
            key = f"/Asset/LineSupport{i}{j}"
            w.box(s, key + "Block", (0.037, 0.017, height),
                  [mid[0], mid[1], C.TABLE + height / 2], RUBBER)
            for offset, mark in ((-0.0041, "A"), (0.0041, "B")):
                w.box(s, key + mark, (0.004, 0.014, 0.006), mid + [offset, 0, 0.001], BLACK)
    return s


def body(w):
    s = w.stage("attachment")
    p = w.body(s, "/Asset/Head", 4.4, fixed=True, articulation=True)
    center = C.HEAD
    base_color = BLACK
    w.box(s, p + "/PowerheadSupport", (0.14, 0.18, 0.021),
          center + [1.067, 0, C.TABLE - center[2] + 0.0105], RUBBER)
    w.cylinder(s, p + "/LowerFlange", 0.054, 0.019, center + [0, 0, -0.019], base_color)
    w.cylinder(s, p + "/BearingBoss", 0.027, 0.022, center + [0, 0, -0.040], base_color)
    w.cylinder(s, p + "/CenterCover", 0.018, 0.007, center + [0, 0, -0.010], RUBBER)

    for j in range(28):
        a = 2 * pi * j / 28
        rad = [cos(a), sin(a), 0]
        x, y, _ = np.asarray(rad) * 0.045
        side = 1 if y > 0 else -1
        if abs(y) > 0.033 and abs(x - side * C.THREAD_X) < 0.015:
            continue
        w.box(s, f"{p}/Shell{j}", (0.017, 0.011, 0.019),
              center + [x, y, 0.0005], base_color, rotation("Z", a))
    for j in range(19):
        a = np.radians(-70 + 8 * j)
        w.box(s, f"{p}/DebrisGuard{j}", (0.029, 0.023, 0.007),
              center + [0.085 * cos(a), 0.085 * sin(a), -0.045],
              ORANGE, rotation("Z", a))
    w.rounded_box(s, p + "/GuardHub", (0.116, 0.145, 0.014),
                  center + [0.066, 0, -0.052], ORANGE, 0.018, 0.002)
    w.box(s, p + "/KickStand", (0.095, 0.112, 0.018),
          center + [-0.005, 0, C.TABLE - center[2] + 0.009], RUBBER)

    for j, (x, z) in enumerate(((0.10, -0.062), (0.24, -0.050),
                                (0.45, -0.038), (0.63, -0.038),
                                (0.81, -0.038), (0.955, -0.031))):
        w.box(s, f"{p}/Shaft{j}", (0.185, 0.024, 0.024),
              center + [x, 0, z], ALUMINUM, rotation("Y", -0.09 if j < 3 else 0.))
    w.rounded_box(s, p + "/Powerhead", (0.225, 0.158, 0.112),
                  center + [1.062, 0, -0.010], ORANGE, 0.029, 0.005)
    w.rounded_box(s, p + "/StarterShell", (0.140, 0.108, 0.016),
                  center + [1.090, 0, 0.048], BLACK, 0.018, 0.003)
    w.rounded_box(s, p + "/ThrottleGrip", (0.165, 0.037, 0.041),
                  center + [0.850, 0, -0.031], RUBBER, 0.012, 0.003)
    for j, (y, z, tilt) in enumerate(((-0.051, 0.006, 0.6), (0.051, 0.006, -0.6))):
        w.box(s, f"{p}/LoopUpright{j}", (0.029, 0.019, 0.105),
              center + [0.350, y, z], BLACK, rotation("X", tilt))
    w.box(s, p + "/LoopHandle", (0.030, 0.157, 0.021),
          center + [0.350, 0, 0.047], BLACK)

    for i, side in enumerate(("left", "right")):
        q = C.sign(side)
        c = center + [q * C.THREAD_X, q * 0.038, 0]
        depth, throat = .028, .0039
        w.box(s, f"{p}/LowerGuide{i}", (0.016, depth, 0.0045),
              c + [0, 0, -(throat + .0045) / 2], BRASS)
        for k, direction in enumerate((-1, 1)):
            w.box(s, f"{p}/EyeletSide{i}{k}", (.0065, depth, .013),
                  c + [direction * (throat + .0065) / 2, 0, 0], BRASS)
        w.box(s, f"{p}/EyeletRoof{i}", (.011, .009, .006),
              c + [0, q * .0095, (throat + .006) / 2], BRASS)
        w.box(s, f"{p}/GuideTongue{i}", (.013, .010, .0048),
              center + [q * C.THREAD_X, q * .021, -.00435], BRASS)

        # A spring steel sprag sits outward of its bite point. Inward
        # sliding raises it, while an outward pull wedges it into the line.
        pivot = center + [q * C.THREAD_X, q * .036, .0062]
        tooth = w.body(s, f"/Asset/Grip{i}", .008, position=pivot, ccd=True)
        w.box(s, tooth + "/SpringGrip", (.0038, .0105, .0015),
              [0, -q * .0044, -.00255], ALUMINUM, rotation("X", q * .52))
        w.cylinder(s, tooth + "/Pin", .0018, .008, (0, 0, 0), BLACK, "X",
                   collide=False)
        j = w.Phy.RevoluteJoint.Define(s, f"/Asset/GripJoint{i}")
        j.CreateBody0Rel().SetTargets([p])
        j.CreateBody1Rel().SetTargets([tooth])
        j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, pivot)))
        j.CreateAxisAttr("X")
        lo, hi = ((-34., 5.) if q == 1 else (-5., 34.))
        j.CreateLowerLimitAttr(lo)
        j.CreateUpperLimitAttr(hi)
        d = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular")
        d.CreateTypeAttr("force")
        d.CreateTargetPositionAttr(q * 3.0)
        d.CreateStiffnessAttr(.017)
        d.CreateDampingAttr(.00035)
        d.CreateMaxForceAttr(.014)
    return s


def stock(w, name):
    s = w.stage(name)
    root = w.body(s, "/Asset/Nylon", .006, ccd=True)
    w.cylinder(s, root + "/Strip", C.LINE_DIAMETER / 2, C.LINE_LENGTH, (0, 0, 0), NYLON, "Y")
    # Cut tip contrasting band shows that the ends reach the recess.
    for i, q in enumerate((-1, 1)):
        w.cylinder(s, root + f"/TipBand{i}", C.LINE_DIAMETER * .502, .0035,
                   (0, q * (C.LINE_LENGTH / 2 - .00175), 0), (0.94, 0.75, .16), "Y",
                   collide=False)
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [
        {"name": "table", "type": "base", "file": w.save(bench(w))},
        {"name": "attachment", "type": "articulation", "file": w.save(body(w))},
    ]
    for side in ("left", "right"):
        key = side + "_line"
        assets.append({"name": key, "type": "rigid", "file": w.save(stock(w, key)),
                       "position": C.line_center(side).tolist()})
    return {"assets": assets, "settle_actions": 14}
