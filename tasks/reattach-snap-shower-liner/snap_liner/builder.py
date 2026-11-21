"""Compound collision geometry for the shower rail, ring and flexible hem."""

from itertools import pairwise
from math import cos, pi, sin

import numpy as np

from . import config as C
from .usd_scene import Writer

TILE = (0.78, 0.86, 0.86)
GROUT = (0.53, 0.67, 0.69)
RAIL = (0.54, 0.59, 0.65)
PANEL = (0.28, 0.61, 0.66)
HEM = (0.19, 0.47, 0.52)
WHITE = (0.93, 0.93, 0.87)
HOOK = (0.96, 0.58, 0.13)
CLASP = (0.82, 0.39, 0.08)
CRADLE = (0.35, 0.42, 0.43)


def segment(w, stage, path, start, end, radius, color, collide=True):
    start, end = np.asarray(start), np.asarray(end)
    delta = end - start
    cylinder = w.Geom.Cylinder.Define(stage, path)
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(np.linalg.norm(delta) + (0.001 if collide else 0.002))
    cylinder.AddTranslateOp().Set(w.Gf.Vec3d(*map(float, (start + end) / 2)))
    rotation = w.Gf.Rotation(w.Gf.Vec3d(0, 0, 1), w.Gf.Vec3d(*map(float, delta)))
    cylinder.AddOrientOp(precision=w.Geom.XformOp.PrecisionDouble).Set(
        rotation.GetQuat()
    )
    w._finish(stage, cylinder, color, collide, contact=0.00028)


def arc(w, stage, path, start, end, center, color, collide=True, scale=(1, 1, 1)):
    angles = np.linspace(start, end, int((end - start) / 13) + 2)
    for i, (a, b) in enumerate(pairwise(angles)):
        segment(
            w,
            stage,
            f"{path}{i}",
            center + C.ring_point(a) * scale,
            center + C.ring_point(b) * scale,
            C.WIRE_RADIUS,
            color,
            collide,
        )


def eyelet(w, stage, path, center, collide):
    radius = (C.EYELET_RADIUS + C.EYELET_OUTER) / 2
    angles = np.linspace(0.0, 2 * pi, 29)
    points = [center + [0.0, radius * cos(t), radius * sin(t)] for t in angles]
    for i, (start, end) in enumerate(pairwise(points)):
        segment(w, stage, f"{path}{i}", start, end, 0.0031, RAIL, collide)


def sheet(w, stage, path, points, faces, color):
    mesh = w.Geom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([w.Gf.Vec3f(*map(float, point)) for point in points])
    mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
    mesh.CreateFaceVertexIndicesAttr([v for face in faces for v in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    w._finish(stage, mesh, color, False)


def shower(w):
    s = w.stage("shower")
    w.box(s, "/Asset/Floor", (4.0, 4.0, 0.024), (0.45, 0.0, -0.812), (0.71, 0.75, 0.74))
    w.box(s, "/Asset/Back", (0.024, 1.75, 2.62), (0.635, 0.33, 0.51), GROUT)
    for row in range(17):
        for col in range(9):
            x = 0.622
            y = -0.445 + col * 0.198
            z = -0.724 + row * 0.151
            w.box(
                s,
                f"/Asset/Tile{row}_{col}",
                (0.006, 0.196, 0.149),
                (x, y, z),
                TILE,
                False,
            )
    w.box(
        s, "/Asset/Counter", (0.73, 0.94, 0.046), (0.028, 0.0, C.TABLE - 0.023), WHITE
    )
    for side, y in (("Left", 0.323), ("Right", -0.323)):
        w.box(
            s,
            f"/Asset/ServiceStand{side}",
            (0.32, 0.152, 1.50),
            (0.016, y, -0.047),
            CRADLE,
            False,
        )
    for name, y in (("Near", -0.416), ("Far", 1.125)):
        w.box(
            s,
            f"/Asset/Flange{name}",
            (0.124, 0.044, 0.124),
            (C.ROD[0], y, C.ROD[2]),
            WHITE,
            False,
        )
    w.cylinder(
        s, "/Asset/Rod", C.ROD_RADIUS, 1.54, (C.ROD[0], 0.353, C.ROD[2]), RAIL, "Y"
    )
    w.rounded_box(
        s,
        "/Asset/Rest",
        (0.184, 0.158, 0.009),
        (C.RING_START[0] + 0.033, C.RING_START[1], C.TABLE + 0.0045),
        CRADLE,
        0.013,
        0.002,
    )
    # Static rippled lower body and the already supported portion of the liner.
    ys = np.linspace(-0.046, 1.12, 72)
    for body, top, bottom in (("LowerLiner", 0.72, -0.712),):
        points = [
            [0.403 + 0.012 * sin(i * 0.39), y, z]
            for i, y in enumerate(ys)
            for z in (top, bottom)
        ]
        faces = [[i, i + 2, i + 3, i + 1] for i in range(0, len(points) - 2, 2)]
        sheet(w, s, f"/Asset/{body}", points, faces, PANEL)
    ys = np.linspace(0.050, 1.12, 70)
    points = [
        [0.402 + 0.010 * sin(i * 0.39), y, z]
        for i, y in enumerate(ys)
        for z in (1.015, 0.515)
    ]
    faces = [[i, i + 2, i + 3, i + 1] for i in range(0, len(points) - 2, 2)]
    sheet(w, s, "/Asset/FastenedLiner", points, faces, PANEL)
    w.box(
        s, "/Asset/FastenedHem", (0.005, 1.07, 0.033), (0.404, 0.584, 0.998), HEM, False
    )
    for i, y in enumerate(np.linspace(0.138, 1.085, 8)):
        center = np.array([C.ROD[0], y, C.ROD[2] - 0.017])
        arc(
            w,
            s,
            f"/Asset/Original{i}_",
            0.0,
            360.0,
            center,
            WHITE,
            False,
            scale=(0.78, 1, 0.69),
        )
        eyelet(
            w,
            s,
            f"/Asset/FastenedEyelet{i}_",
            np.array([C.ROD[0] + 0.002, y, 0.992]),
            False,
        )
    w.rounded_box(
        s,
        "/Asset/BasinRim",
        (0.168, 1.46, 0.068),
        (0.465, 0.35, -0.515),
        WHITE,
        0.025,
        0.008,
        False,
    )
    w.cylinder(
        s, "/Asset/ShowerControl", 0.051, 0.023, (0.607, -0.167, 1.23), RAIL, "X", False
    )
    return s


def hinge(w, s, name, parent, link, point, axis, limits):
    joint = w.Phy.RevoluteJoint.Define(s, f"/Asset/{name}")
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([link])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, point)))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(limits[0])
    joint.CreateUpperLimitAttr(limits[1])
    joint.CreateCollisionEnabledAttr(False)
    d = w.Phy.DriveAPI.Apply(joint.GetPrim(), "angular")
    d.CreateTypeAttr("force")
    d.CreateDampingAttr(0.004 if axis == "Y" else 0.0005)
    if axis == "X":
        d.CreateStiffnessAttr(0.004)
        d.CreateTargetPositionAttr(float(np.rad2deg(C.LOOSE)))
        d.CreateMaxForceAttr(0.25)


def snap_ring(w):
    s = w.stage("snap_ring")
    ring = w.body(s, "/Asset/Ring", 0.046, articulation=True, ccd=True)
    gate = w.body(s, "/Asset/Clasp", 0.019, C.GATE_PIVOT, ccd=True)
    arc(w, s, ring + "/Spine", C.PIVOT_ANGLE, 270.0, np.zeros(3), HOOK)
    arc(w, s, gate + "/Gate", 270.4, 360.0 + C.PIVOT_ANGLE - 0.5, -C.GATE_PIVOT, CLASP)
    w.rounded_box(
        s, ring + "/ThumbRib", (0.014, 0.017, 0.031), C.RING_GRIP, HOOK, 0.004, 0.0016
    )
    w.cylinder(s, ring + "/HingeRivet", 0.007, 0.013, C.GATE_PIVOT, RAIL, "Y")
    w.rounded_box(
        s, gate + "/ThumbClasp", (0.013, 0.016, 0.021), C.GATE_TAB, CLASP, 0.005, 0.0015
    )
    hinge(w, s, "Snap", ring, gate, C.GATE_PIVOT, "Y", (-105.0, 1.3))
    return s


def liner_corner(w):
    s = w.stage("liner_corner")
    anchor = w.body(s, "/Asset/Neighbor", 0.55, articulation=True)
    w.Phy.MassAPI.Get(s, anchor).CreateDiagonalInertiaAttr(w.Gf.Vec3f(0.01, 0.01, 0.01))
    joint = w.Phy.FixedJoint.Define(s, "/Asset/NeighborSupport")
    joint.CreateBody1Rel().SetTargets([anchor])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, C.CORNER_PIVOT)))
    part = w.body(s, "/Asset/Corner", 0.075)
    w.Phy.MassAPI.Get(s, part).CreateCenterOfMassAttr(w.Gf.Vec3f(0, -0.075, -0.135))
    w.Phy.MassAPI.Get(s, part).CreateDiagonalInertiaAttr(
        w.Gf.Vec3f(0.00036, 0.00026, 0.00016)
    )
    eyelet(w, s, part + "/Eyelet", C.EYELET, True)
    w.rounded_box(
        s,
        part + "/ReinforcedHem",
        (0.007, 0.124, 0.028),
        (0.0, -0.083, 0.004),
        HEM,
        0.002,
        0.001,
    )
    w.rounded_box(
        s,
        part + "/FoldBelowEyelet",
        (0.004, 0.155, 0.013),
        (0.0, -0.096, -0.020),
        PANEL,
        0.0016,
        0.0008,
    )
    w.rounded_box(
        s,
        part + "/EndHem",
        (0.004, 0.009, 0.043),
        (0.0, -0.187, -0.002),
        HEM,
        0.0015,
        0.0007,
    )
    sheet(
        w,
        s,
        part + "/MoveableLiner",
        [
            (0.0, -0.193, -0.023),
            (0.0, -0.008, -0.014),
            (0.008, -0.006, -0.455),
            (0.018, -0.182, -0.428),
            (0.012, -0.063, -0.249),
        ],
        ((0, 1, 4), (1, 2, 4), (2, 3, 4), (3, 0, 4)),
        PANEL,
    )
    hinge(w, s, "Fold", anchor, part, np.zeros(3), "X", (-16.0, 47.0))
    return s


def create_assets(directory):
    w = Writer(directory)
    return {
        "settle_actions": 25,
        "assets": [
            {"name": "shower", "type": "base", "file": w.save(shower(w))},
            {
                "name": "ring",
                "type": "articulation",
                "position": C.RING_START.tolist(),
                "quaternion": C.RING_FLAT_QUAT,
                "file": w.save(snap_ring(w)),
            },
            {"name": "liner", "type": "articulation", "file": w.save(liner_corner(w))},
        ],
    }
