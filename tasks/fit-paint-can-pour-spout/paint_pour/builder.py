"""Original rigid scene, including native cylindrical rims and handle contacts."""

import math

import numpy as np

from . import config as C
from .usd_scene import Writer

STEEL = (0.48, 0.52, 0.56)
CHIME = (0.70, 0.72, 0.73)
TRAY = (0.29, 0.34, 0.41)
RUBBER = (0.19, 0.22, 0.24)


def annular(
    w,
    stage,
    path,
    inner,
    outer,
    low,
    high,
    color,
    count=32,
    collide=True,
    low_inner=None,
):
    """Build thin annular wedges without filling a concavity with a collider."""
    for index in range(count):
        a = index * math.tau / count
        b = a + math.tau / count + 0.001
        ring = [
            (r * math.cos(t), r * math.sin(t))
            for r, t in ((inner, a), (outer, a), (outer, b), (inner, b))
        ]
        mesh = w.Geom.Mesh.Define(stage, f"{path}{index}")
        mouth = inner if low_inner is None else low_inner
        lower = [
            (r * math.cos(t), r * math.sin(t))
            for r, t in ((mouth, a), (outer, a), (outer, b), (mouth, b))
        ]
        points = [
            w.Gf.Vec3f(x, y, z)
            for z, outline in ((low, lower), (high, ring))
            for x, y in outline
        ]
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([4] * 6)
        mesh.CreateFaceVertexIndicesAttr(
            [
                i
                for face in (
                    (3, 2, 1, 0),
                    (4, 5, 6, 7),
                    (0, 1, 5, 4),
                    (1, 2, 6, 5),
                    (2, 3, 7, 6),
                    (3, 0, 4, 7),
                )
                for i in face
            ]
        )
        mesh.CreateSubdivisionSchemeAttr("none")
        if collide:
            w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
                "convexHull"
            )
        w._finish(stage, mesh, color, collide, contact=0.00035)


def can(w):
    s = w.stage("can")
    body = w.body(s, "/Asset", 0.64, ccd=True)
    mass = w.Phy.MassAPI.Get(s, body)
    mass.CreateCenterOfMassAttr(w.Gf.Vec3f(0.0, 0.0, 0.062))
    mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(0.0032, 0.0032, 0.0021))
    material = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    material.GetStaticFrictionAttr().Set(1.05)
    material.GetDynamicFrictionAttr().Set(0.83)
    w.cylinder(s, "/Asset/Base", C.OUTER_RADIUS, C.FLOOR, (0, 0, C.FLOOR / 2), STEEL)
    annular(
        w,
        s,
        "/Asset/Body",
        C.INNER_RADIUS,
        C.OUTER_RADIUS,
        C.FLOOR,
        C.HEIGHT - 0.004,
        (0.65, 0.70, 0.73),
    )
    annular(w, s, "/Asset/Rim", 0.074, 0.0835, C.HEIGHT - 0.004, C.HEIGHT, CHIME)
    annular(w, s, "/Asset/Chime", 0.0799, 0.085, 0, 0.006, STEEL, collide=False)
    annular(
        w,
        s,
        "/Asset/Label",
        0.0820,
        0.08235,
        0.027,
        0.149,
        (0.17, 0.43, 0.53),
        collide=False,
    )
    for z in (0.035, 0.123):
        annular(
            w,
            s,
            "/Asset/Stripe" + str(int(1000 * z)),
            0.08235,
            0.08248,
            z,
            z + 0.006,
            (0.91, 0.91, 0.81),
            collide=False,
        )
    # Swung-out wire bail rests against its molded stop. A centered protective
    # grip sleeve provides an ordinary hand contact outside the can diameter.
    for sign in (-1, 1):
        w.cylinder(
            s,
            f"/Asset/BailEar{sign}",
            0.011,
            0.008,
            (sign * 0.082, 0, 0.147),
            STEEL,
            "X",
        )
        for k in range(10):
            phi = (k + 0.5) * np.pi / 20
            p = np.array(
                [
                    sign * 0.087 * np.cos(phi),
                    -0.149 * np.sin(phi),
                    0.147 - 0.057 * np.sin(phi),
                ]
            )
            v = np.array(
                [
                    -sign * 0.087 * np.sin(phi),
                    -0.149 * np.cos(phi),
                    -0.057 * np.cos(phi),
                ]
            )
            long = v / np.linalg.norm(v)
            normal = np.cross([0.0, 0.0, 1.0], long)
            normal /= np.linalg.norm(normal)
            rotation = np.column_stack([long, normal, np.cross(long, normal)])
            w.box(
                s,
                f"/Asset/Wire{sign}_{k}",
                (np.linalg.norm(v) * np.pi / 20 + 0.001, 0.004, 0.004),
                p,
                CHIME,
                rotation=rotation,
            )
    w.rounded_box(
        s, "/Asset/GripSleeve", (0.044, 0.025, 0.035), C.HANDLE, RUBBER, 0.008, 0.003
    )
    # A clip-on auxiliary handgrip lets the operator support the lower
    # part of the tin while the wire bail takes most of the lifting load.
    for i, x in enumerate((-0.013, 0.013)):
        w.box(
            s, f"/Asset/BracingArm{i}", (0.008, 0.047, 0.014), (x, 0.104, 0.077), STEEL
        )
        w.rounded_box(
            s,
            f"/Asset/BracingPad{i}",
            (0.026, 0.014, 0.038),
            (x, 0.082, 0.077),
            RUBBER,
            0.005,
            0.002,
        )
    w.rounded_box(
        s,
        "/Asset/BracingGrip",
        (0.045, 0.043, 0.037),
        C.AUX_HANDLE,
        RUBBER,
        0.007,
        0.002,
    )
    return s


def lid(w):
    s = w.stage("lid")
    body = w.body(s, "/Asset", 0.11, ccd=True)
    mass = w.Phy.MassAPI.Get(s, body)
    mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(0.00024, 0.00024, 0.00044))
    w.cylinder(s, "/Asset/LidPlate", 0.087, 0.0052, (0, 0, 0), C.ORANGE)
    annular(
        w,
        s,
        "/Asset/Skirt",
        0.0860,
        0.0905,
        -0.0080,
        0.0015,
        (0.72, 0.177, 0.055),
        low_inner=0.0892,
    )
    annular(
        w,
        s,
        "/Asset/RimGasket",
        0.073,
        0.0834,
        -0.0032,
        -0.0014,
        (0.33, 0.09, 0.065),
        collide=False,
    )
    w.cylinder(s, "/Asset/SpoutFoot", 0.023, 0.009, (-0.058, 0, 0.007), C.ORANGE)
    for index, sign in enumerate((-1, 1)):
        w.rounded_box(
            s,
            f"/Asset/SpoutWall{index}",
            (0.059, 0.006, 0.030),
            (-0.070, sign * 0.013, 0.023),
            C.ORANGE,
            0.0025,
            0.001,
        )
    w.rounded_box(
        s,
        "/Asset/SpoutBridge",
        (0.043, 0.031, 0.008),
        (-0.059, 0, 0.028),
        C.ORANGE,
        0.003,
        0.0014,
    )
    w.rounded_box(
        s,
        "/Asset/Nose",
        (0.042, 0.031, 0.006),
        (-0.082, 0, 0.016),
        C.ORANGE,
        0.0025,
        0.0014,
    )
    w.cylinder(s, "/Asset/Vent", 0.008, 0.008, (0.058, 0, 0.007), C.ORANGE)
    w.cylinder(
        s,
        "/Asset/VentOpening",
        0.0045,
        0.0005,
        (0.058, 0, 0.0113),
        RUBBER,
        collide=False,
    )
    for i, angle in enumerate(C.CLIP_ANGLES):
        w.cylinder(
            s,
            f"/Asset/Tab{i}",
            0.007,
            0.009,
            (0.084 * np.cos(angle), 0.084 * np.sin(angle), 0.003),
            C.ORANGE,
            collide=False,
        )
    return s


def visual_mesh(w, stage, name, color):
    mesh = w.Geom.Mesh.Define(stage, "/Asset/" + name)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreatePointsAttr([])
    mesh.CreateFaceVertexCountsAttr([])
    mesh.CreateFaceVertexIndicesAttr([])
    w._finish(stage, mesh, color, False)


def bench(w):
    s = w.stage("bench")
    xc, yc = np.mean(C.WELL_X), np.mean(C.WELL_Y)
    sx, sy = np.ptp(C.WELL_X), np.ptp(C.WELL_Y)
    w.rounded_box(
        s,
        "/Asset/BlackTray",
        (sx + 0.015, sy + 0.015, 0.007),
        (xc, yc, C.TABLE + 0.0035),
        TRAY,
        0.010,
        0.0015,
    )
    for sign in (-1, 1):
        w.box(
            s,
            f"/Asset/TrayEnd{sign}",
            (0.006, sy + 0.014, 0.041),
            (xc + sign * (sx / 2 + 0.0025), yc, C.TABLE + 0.0205),
            TRAY,
        )
        w.box(
            s,
            f"/Asset/TraySide{sign}",
            (sx, 0.006, 0.041),
            (xc, yc + sign * (sy / 2 + 0.0025), C.TABLE + 0.0205),
            TRAY,
        )
    # A commercially familiar roller tray combines a level well and a ribbed ramp.
    for i in range(8):
        z = C.TABLE + 0.010 + i * 0.0038
        y = C.WELL_Y[1] + 0.010 + 0.012 * i
        w.box(
            s,
            f"/Asset/RollerRamp{i}",
            (sx, 0.014, 0.008),
            (xc, y, z),
            (0.34, 0.37, 0.41),
        )
    for sign in (-1, 1):
        w.box(
            s,
            f"/Asset/FillLine{sign}",
            (sx - 0.004, 0.0016, 0.0014),
            (xc, yc + sign * (sy / 2 - 0.00075), C.WELL_FLOOR + 0.0044),
            (0.08, 0.56, 0.90),
            collide=False,
        )
    w.rounded_box(
        s,
        "/Asset/CanPad",
        (0.210, 0.205, 0.006),
        (C.CAN[0] + 0.022, C.CAN[1], C.TABLE + 0.003),
        RUBBER,
        0.008,
        0.001,
    )
    w.cylinder(
        s,
        "/Asset/LidRest",
        0.066,
        0.0124,
        (*C.LID[:2], C.TABLE + 0.0062),
        (0.36, 0.37, 0.38),
    )
    visual_mesh(w, s, "InsidePaint", C.PAINT)
    visual_mesh(w, s, "TrayPaint", C.PAINT)
    visual_mesh(w, s, "Stream", C.PAINT)
    visual_mesh(w, s, "Spill", C.PAINT)
    return s


def create_assets(directory):
    writer = Writer(directory)
    assets = [
        {
            "name": "can",
            "type": "rigid",
            "file": writer.save(can(writer)),
            "position": C.CAN.tolist(),
        },
        {
            "name": "lid",
            "type": "rigid",
            "file": writer.save(lid(writer)),
            "position": C.LID.tolist(),
        },
        {
            "name": "bench",
            "type": "base",
            "file": writer.save(bench(writer)),
            "position": [0.0, 0.0, 0.0],
        },
    ]
    writer.inventory()
    return {"assets": assets, "settle_actions": 32}
