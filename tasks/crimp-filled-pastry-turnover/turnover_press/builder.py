"""Dimensioned native press and its prefilled, two-lobe wrapper."""

from math import cos, pi, sin

import numpy as np

from . import config as C
from .usd_scene import Writer

BLUE = (0.10, 0.43, 0.57)
EDGE = (0.13, 0.52, 0.66)
DARK = (0.068, 0.34, 0.47)
DOUGH = (0.95, 0.73, 0.42)
CREASE = (0.88, 0.60, 0.32)
FILL = (0.53, 0.24, 0.105)


def convex(w, stage, path, points, faces, color, collide=True):
    mesh = w.Geom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([w.Gf.Vec3f(*point) for point in points])
    mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
    mesh.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    if collide:
        w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    w._finish(stage, mesh, color, collide)
    return mesh


def ring_vertex(radius, step, sign, height):
    angle = pi * (step / 26)
    return (radius * cos(angle), sign * radius * sin(angle), height)


def disk(w, stage, path, radius, sign, height, depth, thickness, color, collide=True):
    """Half-disk with a shallow bowl and a narrow perimeter."""
    points = []
    for r, z in ((0.002, height - depth), (0.54 * radius, height - 0.8 * depth),
                 (0.83 * radius, height - 0.25 * depth), (radius, height)):
        points.extend(ring_vertex(r, k, sign, z) for k in range(27))
    faces = []
    for part in range(3):
        offset = 27 * part
        faces.extend((offset + k, offset + k + 1, offset + k + 28, offset + k + 27)
                     for k in range(26))
    points.extend(ring_vertex(radius, k, sign, height - thickness) for k in range(27))
    faces.extend((81 + k, 82 + k, 109 + k, 108 + k) for k in range(26))
    faces.append(tuple(range(108, 135)))
    faces.append((0, 27, 54, 81, 108, 134, 107, 80, 53, 26))
    if sign > 0:
        faces = [tuple(reversed(face)) for face in faces]
    return convex(w, stage, path, points, faces, color, collide)


def bearing(w, stage, parent, child, name, minimum, maximum):
    joint = w.Phy.RevoluteJoint.Define(stage, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(0, 0, 0))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    joint.CreateAxisAttr("X")
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateLowerLimitAttr(minimum)
    joint.CreateUpperLimitAttr(maximum)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.0003)


def bench(w):
    stage = w.stage("bench")
    w.box(stage, "/Asset/Worktop", (0.99, 1.23, 0.043), (0.342, 0, C.TABLE - 0.0215), (0.61, 0.55, 0.45))
    w.box(stage, "/Asset/Floor", (4, 4, 0.020), (0, 0, -0.010), (0.32, 0.35, 0.34))
    w.rounded_box(stage, "/Asset/CuttingBoard", (0.215, 0.116, 0.032), (0.371, 0.0, C.TABLE + 0.016), (0.44, 0.38, 0.28), 0.013, 0.002)
    for i, dx in enumerate((-0.067, 0.067)):
        w.rounded_box(stage, f"/Asset/AlignmentPad{i}", (0.014, 0.028, 0.009),
                      C.PRESS + [dx, 0.023, -0.012], DARK, 0.003, 0.001, collide=False)
    return stage


def mold_leaf(w, stage, link, side, tab):
    # The central recess supports the dough. The perimeter uses native teeth.
    disk(w, stage, link + "/Shell", C.RADIUS + 0.003, side, -0.0102, 0.0014, 0.0062, BLUE)
    w.rounded_box(stage, link + "/FingerTab", (0.037, 0.046, 0.013), tab, BLUE, 0.006, 0.002)
    w.rounded_box(stage, link + "/GripPad", (0.027, 0.022, 0.0016), tab + [0, side * 0.004, 0.0066], DARK, 0.004, 0.00045, collide=False)
    w.box(stage, link + "/TabRoot", (0.041, 0.046, 0.005), (tab[0] * 0.62, side * 0.041, -0.0137), BLUE)
    for i, x in enumerate((-0.026, 0.026)):
        w.rounded_box(stage, f"{link}/Foot{i}", (0.014, 0.014, 0.003), (x, side * 0.026, -0.0152), DARK, 0.002, 0.0005)
    for k in range(25):
        a = (k + 0.5) * pi / 25
        radial = np.array([cos(a), side * sin(a), 0])
        tangent = np.array([-side * sin(a), cos(a), 0])
        rotation = np.column_stack((radial, tangent, [0, 0, 1]))
        ctr = 0.0485 * radial + [0, 0, -0.0045]
        w.box(stage, f"{link}/Rim{k}", (0.0082, 0.0062, 0.0055), ctr, EDGE, rotation)
        w.box(stage, f"{link}/Tooth{k}", (0.0084, 0.00265, 0.0013), ctr + [0, 0, 0.00315], BLUE, rotation)


def press(w):
    stage = w.stage("press")
    base = w.body(stage, "/Asset/LowerPress", 0.19, articulation=True, ccd=True)
    lid = w.body(stage, "/Asset/UpperPress", 0.067, ccd=True)
    w.Px.PhysxArticulationAPI.Get(stage, base).CreateEnabledSelfCollisionsAttr(False)
    mold_leaf(w, stage, base, 1, C.BASE_TAB)
    mold_leaf(w, stage, lid, -1, C.LID_TAB)
    w.cylinder(stage, lid + "/MoldedStem", 0.009, 0.016, tuple(C.LID_GRIP + [0., 0., 0.008]), EDGE)
    knob = w.Geom.Sphere.Define(stage, lid + "/RoundedHandle")
    knob.CreateRadiusAttr(0.013)
    knob.AddTranslateOp().Set(w.Gf.Vec3d(*C.LID_GRIP))
    w._finish(stage, knob, EDGE, True)
    for i, x in enumerate((-0.057, 0.057)):
        w.cylinder(stage, f"{base}/HingeBearing{i}", 0.0048, 0.009, (x, 0, 0), DARK, axis="X")
    for i, x in enumerate((-0.051, 0.051)):
        w.cylinder(stage, f"{lid}/HingePin{i}", 0.0032, 0.011, (x, 0, 0), EDGE, axis="X")
    bearing(w, stage, base, lid, "Fold", -180, 0)
    return stage


def filling(w, stage, link):
    for j in range(5):
        x = (j - 2) * 0.0083
        width = 0.022 - abs(x) * 0.31
        w.rounded_box(stage, f"{link}/FillingMound{j}", (0.017, width, 0.010 - abs(x) * 0.12), (x, 0.023, -0.001), FILL, 0.006, 0.002, collide=False)


def wrapper(w):
    stage = w.stage("pastry")
    lower = w.body(stage, "/Asset/LowerDough", 0.018, articulation=True, ccd=True)
    upper = w.body(stage, "/Asset/UpperDough", 0.011, ccd=True)
    w.Px.PhysxArticulationAPI.Get(stage, lower).CreateEnabledSelfCollisionsAttr(False)
    for link, sign in ((lower, 1), (upper, -1)):
        disk(w, stage, link + "/DoughSkin", C.RADIUS - 0.0005, sign, C.WRAPPER_RIM, 0.008, 0.0033, DOUGH, collide=False)
        # The thin shell hull supplies rigid-body support and native grasp.
        shape = disk(w, stage, link + "/DoughContact", 0.049, sign, C.WRAPPER_RIM - 0.0015, 0.00725, 0.0035, DOUGH)
        shape.MakeInvisible()
        for k in range(C.SECTOR_COUNT):
            a = pi * (k + 0.5) / C.SECTOR_COUNT
            radial = np.array([cos(a), sign * sin(a), 0])
            tangent = np.array([-sign * sin(a), cos(a), 0])
            rotation = np.column_stack((radial, tangent, [0, 0, 1]))
            w.box(stage, f"{link}/Impression{k}", (0.0064, 0.0013, 0.0006), C.SEAM_RADIUS * radial + [0, 0, C.WRAPPER_RIM + 0.00025], CREASE, rotation, collide=False)
    filling(w, stage, lower)
    bearing(w, stage, lower, upper, "DoughFold", -176.5, 10)
    return stage


def create_assets(directory):
    w = Writer(directory)
    return {"assets": [
        {"name": "bench", "type": "base", "file": w.save(bench(w))},
        {"name": "press", "type": "articulation", "file": w.save(press(w)), "position": C.PRESS, "joints": {"Fold": -C.READY_FOLD}},
        {"name": "pastry", "type": "articulation", "file": w.save(wrapper(w)), "position": C.PRESS + [0, 0, C.PASTRY_Z], "joints": {"DoughFold": -C.READY_DOUGH}},
    ], "settle_actions": 28}
