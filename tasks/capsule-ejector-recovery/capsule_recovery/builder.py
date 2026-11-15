"""Passive tilting head and fork, removable collector, native rigid capsules."""

import math

import numpy as np
from yam_arena.urdf import axis_angle_matrix

from . import config as C
from .usd_scene import Writer


def capsule(w, s, path, color):
    # Original convex tapered cup, OD 33 mm, 27 mm high; 37 mm flange.
    from pxr import Gf, UsdGeom, UsdPhysics

    n = 32
    points = []
    for z, r in ((0.0, 0.012), (0.026, 0.0165)):
        points.extend(
            [
                (r * math.cos(k * math.tau / n), r * math.sin(k * math.tau / n), z)
                for k in range(n)
            ]
        )
    indices = []
    counts = []
    for k in range(n):
        indices.extend([k, (k + 1) % n, (k + 1) % n + n, k + n])
        counts.append(4)
    indices.extend(list(range(n - 1, -1, -1)))
    counts.append(n)
    indices.extend(list(range(n, 2 * n)))
    counts.append(n)
    m = UsdGeom.Mesh.Define(s, path + "/Cup")
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    m.CreateFaceVertexCountsAttr(counts)
    m.CreateFaceVertexIndicesAttr(indices)
    m.CreateSubdivisionSchemeAttr("none")
    w._finish(s, m, color, True)
    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    w.cylinder(s, path + "/Flange", 0.0185, 0.0018, (0, 0, 0.0269), color)
    w.cylinder(
        s,
        path + "/Foil",
        0.016,
        0.0003,
        (0, 0, 0.0280),
        (0.34, 0.26, 0.16) if color[0] < 0.6 else (0.88, 0.77, 0.42),
        collide=False,
    )


def collector_guide(w, s, path, side, color):
    """Passive lead-in chamfer: wide at the lip, 6 mm side clearance below."""
    from pxr import Gf, UsdGeom, UsdPhysics

    section = [
        (0.064, 0.009),
        (0.090, 0.009),
        (0.090, 0.035),
        (0.084, 0.035),
        (0.064, 0.015),
    ]
    vertices = [(x, side * y, C.TOP + z) for x in (0.3825, 0.5475) for y, z in section]
    faces = [list(range(4, -1, -1)), list(range(5, 10))]
    faces += [[i, (i + 1) % 5, (i + 1) % 5 + 5, i + 5] for i in range(5)]
    if side < 0:
        faces = [face[::-1] for face in faces]
    mesh = UsdGeom.Mesh.Define(s, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in vertices])
    mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
    mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    w._finish(s, mesh, color, True)
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
        "convexHull"
    )


def create_assets(directory):
    from pxr import Gf, UsdGeom

    w = Writer(directory)
    assets = []
    silver = (0.53, 0.56, 0.58)
    dark = (0.10, 0.12, 0.14)
    case = (0.72, 0.74, 0.72)
    s = w.stage("machine")
    base = w.body(s, "/Asset/Base", 2.8, fixed=True, articulation=True)
    # Cutaway shell and a genuine open ejection path below the loading carrier.
    w.box(
        s,
        base + "/Rear",
        (0.025, 0.19, 0.25 + C.HEAD_RISE),
        (0.625, 0, C.TOP + (0.25 + C.HEAD_RISE) / 2),
        case,
    )
    for y in (-0.096, 0.096):
        w.box(
            s,
            base + f"/SideUpper{y}",
            (0.22, 0.008, 0.060),
            (0.510, y, C.TOP + 0.215 + C.HEAD_RISE),
            case,
        )
        w.box(
            s,
            base + f"/SideRear{y}",
            (0.025, 0.008, 0.175 + C.HEAD_RISE),
            (0.6025, y, C.TOP + 0.0975 + C.HEAD_RISE / 2),
            case,
        )
    w.box(s, base + "/Foot", (0.28, 0.17, 0.008), (0.50, 0, C.TOP + 0.004), dark)
    # Side-supported annular lower carrier; its opening remains physical.
    for n in range(32):
        a = (n + 0.5) * math.tau / 32
        r = 0.0184
        if abs(r * math.sin(a)) > 0.010:
            continue  # Clear slots for the lifting fork.
        rot = axis_angle_matrix(np.array([0, 0, 1]), a)
        w.box(
            s,
            base + f"/Seat{n}",
            (0.0032, 0.0040, 0.005),
            (
                C.SLOT[0] + r * math.cos(a),
                r * math.sin(a),
                C.TOP + 0.1925 + C.HEAD_RISE,
            ),
            silver,
            rotation=rot,
        )
    for y in (-0.085, 0.085):
        w.box(
            s,
            base + f"/Bridge{y}",
            (0.11, 0.008, 0.008),
            (0.445, y, C.TOP + 0.188 + C.HEAD_RISE),
            dark,
        )
    head = w.body(s, "/Asset/Head", 0.22)
    w.box(
        s,
        head + "/Cover",
        (0.22, 0.17, 0.018),
        (0.48, 0, C.TOP + 0.225 + C.HEAD_RISE),
        (0.21, 0.25, 0.27),
    )
    # Rim-support fork is open toward the rear. Gravity discharges the old
    # capsule down the chute when the head is raised far enough.
    for y in (-0.0195, 0.0195):
        w.box(
            s,
            head + f"/Fork{y}",
            (0.075, 0.005, 0.003),
            (0.445, y, C.TOP + 0.192 + C.HEAD_RISE),
            silver,
        )
    w.box(
        s,
        head + "/ForkFront",
        (0.006, 0.041, 0.006),
        (0.405, 0, C.TOP + 0.194 + C.HEAD_RISE),
        silver,
    )
    # Ordinary bar handle; the right end is clear of the machine's sidewall.
    w.cylinder(
        s,
        head + "/Handle",
        0.009,
        0.24,
        (C.HEAD_HANDLE[0], 0, C.HEAD_HANDLE[2]),
        dark,
        axis="Y",
    )
    for y in (-0.07, 0.07):
        w.box(
            s,
            head + f"/HandleLeg{y}",
            (0.012, 0.012, 0.035),
            (0.38, y, C.TOP + 0.222 + C.HEAD_RISE),
            dark,
        )
    w.hinge(
        s,
        "/Asset/HeadHinge",
        base,
        head,
        "Y",
        C.HINGE,
        C.HINGE,
        (0, math.radians(85)),
        friction=0.30,
        damping=0.008,
    )
    assets.append(
        {
            "name": "machine",
            "type": "articulation",
            "file": w.save(s),
            "initial_joints": {"HeadHinge": C.OPEN_ANGLE},
        }
    )
    s = w.stage("bin")
    body = w.body(s, "/Asset/Body", 0.12)
    w.tray(s, body, C.BIN_INNER, C.BIN_H, 0.0, (0.30, 0.34, 0.36), wall=0.003)
    # A front pull bar with an unobstructed 26 mm vertical pinch. Side stems
    # leave the finger pads clear; no lip blocks their approach or release.
    w.box(s, body + "/Handle", (0.040, 0.100, 0.026), C.BIN_GRIP, dark)
    for side in (-1, 1):
        w.box(
            s,
            body + f"/HandleStem{side + 1}",
            (0.040, 0.012, 0.012),
            (-0.098, side * 0.044, C.BIN_GRIP[2]),
            dark,
        )
    assets.append(
        {"name": "bin", "type": "rigid", "file": w.save(s), "position": C.BIN.tolist()}
    )
    s = w.stage("fresh")
    body = w.body(s, "/Asset/Body", 0.008)
    capsule(w, s, body, (0.76, 0.58, 0.13))
    assets.append(
        {
            "name": "fresh",
            "type": "rigid",
            "file": w.save(s),
            "position": C.FRESH.tolist(),
        }
    )
    used_positions = []
    for k in range(C.USED_COUNT):
        name = f"used{k}"
        s = w.stage(name)
        body = w.body(s, "/Asset/Body", 0.012)
        capsule(w, s, body, (0.42, 0.44, 0.45))
        if k < C.USED_COUNT - 1:
            pos = C.BIN + np.array([0, 0, 0.003 + 0.0285 * k])
            q = (0, 0, 0, 1)
        else:
            r = axis_angle_matrix(np.array([0, 1, 0]), C.OPEN_ANGLE)
            pos = C.HINGE + r @ (C.SLOT + np.array([0, 0, 0.002]) - C.HINGE)
            q = (0, math.sin(C.OPEN_ANGLE / 2), 0, math.cos(C.OPEN_ANGLE / 2))
        used_positions.append(pos.tolist())
        assets.append(
            {
                "name": name,
                "type": "rigid",
                "file": w.save(s),
                "position": pos.tolist(),
                "quaternion": q,
            }
        )
    s = w.stage("fixtures")
    p = "/Asset/Recycling"
    x = UsdGeom.Xform.Define(s, p)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, C.WASTE)))
    w.tray(s, p, C.WASTE_INNER, C.WASTE_H, 0.002, (0.18, 0.44, 0.28), wall=0.004)
    # Bin side rails and rear stop establish a reproducible removable fit.
    for side in (-1, 1):
        collector_guide(w, s, "/Asset/Guide" + str(side + 1), side, dark)
    w.box(s, "/Asset/BinStop", (0.006, 0.13, 0.015), (0.557, 0, C.TOP + 0.015), dark)
    # Stand for the single unused capsule, preventing it rolling before pickup.
    w.cylinder(
        s,
        "/Asset/FreshFoot",
        0.025,
        0.004,
        (C.FRESH[0], C.FRESH[1], C.TOP + 0.002),
        dark,
    )
    w.cylinder(
        s,
        "/Asset/FreshStand",
        0.009,
        0.066,
        (C.FRESH[0], C.FRESH[1], C.TOP + 0.037),
        dark,
    )
    assets.append({"name": "fixtures", "type": "base", "file": w.save(s)})
    w.inventory()
    return {"assets": assets, "settle_actions": 30, "used_initial": used_positions}
