"""Native compliant wash bottle and borosilicate laboratory glassware."""

import math

import numpy as np

from . import config as C
from .motion import rotation
from .usd_scene import Writer


def surface(w, stage, name, color, opacity=1.0):
    from pxr import Sdf

    material = w.Shade.Material.Define(stage, "/Asset/" + name)
    shader = w.Shade.Shader.Define(stage, "/Asset/" + name + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        w.Gf.Vec3f(*color)
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.24)
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def mesh(w, stage, path, material, points=(), faces=()):
    geo = w.Geom.Mesh.Define(stage, path)
    geo.CreateSubdivisionSchemeAttr("none")
    geo.CreateDoubleSidedAttr(True)
    geo.CreatePointsAttr([w.Gf.Vec3f(*map(float, p)) for p in points])
    geo.CreateFaceVertexCountsAttr([len(f) for f in faces])
    geo.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    w.Shade.MaterialBindingAPI.Apply(geo.GetPrim()).Bind(material)
    return geo


def profile(w, stage, path, material, levels, origin=(0.0, 0.0, 0.0), n=64):
    points = [
        np.asarray(origin)
        + (radius * math.cos(i * math.tau / n), radius * math.sin(i * math.tau / n), z)
        for radius, z in levels
        for i in range(n)
    ]
    faces = [
        (h * n + i, h * n + (i + 1) % n, (h + 1) * n + (i + 1) % n, (h + 1) * n + i)
        for h in range(len(levels) - 1)
        for i in range(n)
    ]
    return mesh(w, stage, path, material, points, faces)


def bar(w, stage, path, a, b, radius, color):
    a, b = np.asarray(a), np.asarray(b)
    axis = (b - a) / np.linalg.norm(b - a)
    cross = np.cross([1.0, 0.0, 0.0], axis)
    cross /= np.linalg.norm(cross)
    r = np.column_stack((np.cross(cross, axis), cross, axis))
    # Box colliders lie inside the circular tube outline.
    w.box(
        stage,
        path + "Collision",
        (1.4 * radius, 1.4 * radius, np.linalg.norm(b - a)),
        (a + b) / 2,
        color,
        rotation=r,
    )
    w.Geom.Imageable.Get(stage, path + "Collision").MakeInvisible()
    n = 16
    points = [
        p
        + radius
        * (np.cos(i * math.tau / n) * r[:, 0] + np.sin(i * math.tau / n) * r[:, 1])
        for p in (a, b)
        for i in range(n)
    ]
    faces = [(i, (i + 1) % n, n + (i + 1) % n, n + i) for i in range(n)]
    return points, faces


def make_beaker(w):
    stage = w.stage("beaker")
    body = w.body(stage, "/Asset/Body", 0.043, ccd=True)
    glass = surface(w, stage, "Glass", (0.78, 0.90, 0.95), 0.115)
    w.cylinder(
        stage, body + "/Foot", 0.0205, C.FLOOR, (0, 0, C.FLOOR / 2), (0.81, 0.91, 0.93)
    )
    w.Shade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(body + "/Foot")).Bind(glass)
    w.ring(
        stage,
        body + "/Contact",
        0.020,
        0.002,
        C.HEIGHT - C.FLOOR,
        (C.HEIGHT + C.FLOOR) / 2,
        (0.8, 0.9, 0.95),
    )
    w.Geom.Imageable.Get(stage, body + "/Contact").MakeInvisible()
    n = 64
    zlevels = [C.FLOOR, C.HEIGHT - 0.012, C.HEIGHT, C.HEIGHT]
    points = []
    for j, z in enumerate(zlevels):
        for i in range(n):
            a = i * math.tau / n
            r = 0.021 if j < 3 else C.RADIUS
            if j >= 2:
                r += 0.003 * math.exp(-(((a - 1.5 * math.pi) / 0.16) ** 2))
            points.append((r * math.cos(a), r * math.sin(a), z))
    faces = [
        (j * n + i, j * n + (i + 1) % n, (j + 1) * n + (i + 1) % n, (j + 1) * n + i)
        for j in range(3)
        for i in range(n)
    ]
    mesh(w, stage, body + "/GlassWall", glass, points, faces)
    for strip, center in enumerate(C.FILM_HEIGHTS):
        for sector in range(8):
            angles = [(sector + k / 8) * math.tau / 8 for k in range(9)]
            points = [
                (
                    C.RADIUS * 0.96 * math.cos(a),
                    C.RADIUS * 0.96 * math.sin(a),
                    center + z + 0.0007 * math.sin(31 * a),
                )
                for z in (-0.0034, 0.0034)
                for a in angles
            ]
            film = surface(
                w, stage, f"Residue{strip}_{sector}", tuple(C.FILM_COLORS[strip]), 0.86
            )
            mesh(
                w,
                stage,
                body + f"/Residue{strip}_{sector}",
                film,
                points,
                [(i, i + 1, i + 10, i + 9) for i in range(8)],
            )
    mesh(w, stage, body + "/Pool", surface(w, stage, "Pool", (0.14, 0.55, 0.92), 0.76))
    return stage


def make_bottle(w):
    stage = w.stage("bottle")
    root = w.body(stage, "/Asset/Base", 0.088, articulation=True, ccd=True)
    w.Px.PhysxArticulationAPI.Get(stage, root).CreateEnabledSelfCollisionsAttr(False)
    poly = surface(w, stage, "LDPE", (0.78, 0.86, 0.87), 0.72)
    cap = (0.10, 0.28, 0.52)
    soft = (0.80, 0.87, 0.87)
    r = C.BOTTLE_RADIUS
    w.cylinder(stage, root + "/Heel", r, 0.016, (0, 0, 0.008), soft)
    w.cylinder(stage, root + "/Shoulder", r, 0.010, (0, 0, 0.075), soft)
    w.cylinder(stage, root + "/Neck", 0.012, 0.020, (0, 0, 0.085), soft)
    w.cylinder(stage, root + "/Cap", 0.013, 0.011, (0, 0, 0.096), cap)
    for name in ("Heel", "Shoulder"):
        w.Geom.Imageable.Get(stage, root + "/" + name).MakeInvisible()
    profile(
        w,
        stage,
        root + "/BottleSkin",
        poly,
        [(r, h) for h in (0.002, 0.019, 0.033, 0.048, 0.063, 0.075)]
        + [(r - 0.004, 0.082), (0.012, 0.091)],
    )
    # Fixed front/back strips preserve the real cylindrical profile along the
    # direction perpendicular to the squeeze. The lateral wall sections move.
    heading = math.radians(-10)
    n = np.array([math.cos(heading), math.sin(heading), 0.0])
    t = np.array([-n[1], n[0], 0.0])
    frame = np.column_stack((n, t, [0.0, 0.0, 1.0]))
    for sign in (-1, 1):
        path = root + ("/SpineNear" if sign > 0 else "/SpineFar")
        w.box(
            stage,
            path,
            (0.012, 0.008, 0.054),
            sign * 0.0195 * t + [0, 0, 0.043],
            soft,
            rotation=frame,
        )
        w.Geom.Imageable.Get(stage, path).MakeInvisible()
    for name, a in (
        ("Lower", np.array([0, 0, 0.100])),
        ("Upper", np.array([0, 0.009, 0.117])),
        ("Elbow", np.array([0, 0.026, 0.119])),
    ):
        b = {"Lower": [0, 0.009, 0.117], "Upper": [0, 0.026, 0.119], "Elbow": C.NOZZLE}[
            name
        ]
        points, faces = bar(w, stage, root + "/" + name, a, b, 0.0026, soft)
        mesh(w, stage, root + "/" + name + "Skin", poly, points, faces)
    for name, sign in (("Near", 1), ("Far", -1)):
        p = w.body(stage, "/Asset/" + name, 0.012)
        # Each spring carries the real projected area touched by one finger.
        for k in range(-5, 6):
            ang = heading + (math.pi if sign < 0 else 0.0) + k * math.pi / 24
            local = np.array([math.cos(ang), math.sin(ang), 0.0])
            rot = rotation("Z", ang)
            path = p + f"/Wall{k + 5}"
            w.box(
                stage,
                path,
                (0.004, 0.0064, 0.053),
                (r - 0.002) * local + [0, 0, 0.043],
                soft,
                rotation=rot,
            )
            w.Geom.Imageable.Get(stage, path).MakeInvisible()
        joint = w.Phy.PrismaticJoint.Define(stage, "/Asset/Squeeze" + name)
        joint.CreateBody0Rel().SetTargets([root])
        joint.CreateBody1Rel().SetTargets([p])
        joint.CreateAxisAttr("X")
        q = w.Gf.Quatf(math.cos(heading / 2), 0.0, 0.0, math.sin(heading / 2))
        joint.CreateLocalRot0Attr(q)
        joint.CreateLocalRot1Attr(q)
        joint.CreateLowerLimitAttr(-C.PANEL_TRAVEL if sign > 0 else 0.0)
        joint.CreateUpperLimitAttr(0.0 if sign > 0 else C.PANEL_TRAVEL)
        joint.CreateCollisionEnabledAttr(False)
        spring = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
        spring.CreateTypeAttr("force")
        spring.CreateStiffnessAttr(C.PANEL_STIFFNESS)
        spring.CreateDampingAttr(4.0)
        spring.CreateMaxForceAttr(25.0)
        spring.CreateTargetPositionAttr(0.0)
    friction = w.Phy.MaterialAPI.Get(stage, "/Asset/Physics")
    friction.CreateStaticFrictionAttr(0.87)
    friction.CreateDynamicFrictionAttr(0.72)
    return stage


def make_receiver(w):
    stage = w.stage("receiver")
    glass = surface(w, stage, "Glass", (0.78, 0.90, 0.95), 0.14)
    funnel = surface(w, stage, "Funnel", (0.82, 0.91, 0.92), 0.45)
    profile(
        w,
        stage,
        "/Asset/Flask",
        glass,
        [
            (0, 0.002),
            (0.025, 0.002),
            (0.029, 0.009),
            (0.031, 0.026),
            (0.026, 0.048),
            (0.009, 0.069),
            (0.0078, 0.142),
            (0.0095, 0.146),
            (0.0070, 0.146),
        ],
        C.FLASK,
    )
    profile(
        w,
        stage,
        "/Asset/Funnel",
        funnel,
        [
            (0.0035, -0.082),
            (0.0035, -0.034),
            (0.027, 0),
            (0.028, 0),
            (0.0045, -0.035),
            (0.0035, -0.035),
        ],
        C.FUNNEL,
    )
    w.box(
        stage,
        "/Asset/Calibration",
        (0.0038, 0.0006, 0.0010),
        C.FLASK + [-0.008, 0, 0.076],
        (0.07, 0.18, 0.33),
        collide=False,
    )
    w.cylinder(
        stage,
        "/Asset/FlaskBase",
        0.015,
        0.003,
        C.FLASK + [0, 0, 0.002],
        (0.67, 0.80, 0.84),
    )
    # The native scene includes the bottle neck and funnel rim as obstacles.
    rim = w.Geom.Xform.Define(stage, "/Asset/FunnelGuard")
    rim.AddTranslateOp().Set(w.Gf.Vec3d(*C.FUNNEL[:2], 0.0))
    w.ring(
        stage,
        "/Asset/FunnelGuard",
        0.0277,
        0.0016,
        0.007,
        C.FUNNEL[2] - 0.003,
        (0.80, 0.87, 0.90),
        n=28,
    )
    w.Geom.Imageable.Get(stage, "/Asset/FunnelGuard").MakeInvisible()
    mesh(
        w,
        stage,
        "/Asset/Collected",
        surface(w, stage, "CollectedMaterial", (0.10, 0.38, 0.95), 0.94),
    )
    mesh(
        w,
        stage,
        "/Asset/Spray",
        surface(w, stage, "SprayMaterial", (0.58, 0.84, 0.96), 0.68),
    )
    mesh(
        w,
        stage,
        "/Asset/Pour",
        surface(w, stage, "PourMaterial", (0.23, 0.55, 0.90), 0.72),
    )
    return stage


def create_assets(directory):
    w = Writer(directory)
    assets = [
        {
            "name": "beaker",
            "type": "rigid",
            "file": w.save(make_beaker(w)),
            "position": C.BEAKER.tolist(),
        },
        {
            "name": "bottle",
            "type": "articulation",
            "file": w.save(make_bottle(w)),
            "position": C.BOTTLE.tolist(),
        },
        {"name": "receiver", "type": "base", "file": w.save(make_receiver(w))},
    ]
    w.inventory()
    return {"assets": assets, "settle_actions": 24}
