"""Original writing desk geometry, with a separate native piston button."""

import math

import numpy as np

from . import config as C
from .usd_scene import Writer

GRAY = (0.31, 0.37, 0.40)
GLASS = (0.31, 0.46, 0.54)
INK = (0.045, 0.15, 0.55)
METAL = (0.67, 0.71, 0.72)
TEAL = (0.13, 0.44, 0.42)
GOLD = (0.86, 0.66, 0.26)


def ring(w, s, path, outer, inner, height, z, color, collide=True):
    """Build a hollow round wall from narrow collision panels."""
    count = 16
    width = 2 * outer * math.tan(math.pi / count)
    for i in range(count):
        a = math.tau * i / count
        x, y = math.cos(a), math.sin(a)
        r = (outer + inner) / 2
        w.box(
            s,
            f"{path}/Panel{i}",
            (outer - inner, width, height),
            (r * x, r * y, z),
            color,
            rotation=np.array([[x, -y, 0.0], [y, x, 0.0], [0.0, 0.0, 1.0]]),
            collide=collide,
        )


def desk(w):
    s = w.stage("desk")
    w.box(
        s,
        "/Asset/Bench",
        (0.94, 1.16, 0.042),
        (0.36, 0, C.TABLE_Z - 0.021),
        (0.50, 0.42, 0.33),
    )
    w.box(s, "/Asset/Floor", (4, 4, 0.020), (0, 0, -0.010), (0.36, 0.38, 0.35))
    w.rounded_box(
        s,
        "/Asset/Mat",
        (0.27, 0.35, 0.004),
        (0.368, 0.026, C.TABLE_Z + 0.002),
        (0.30, 0.34, 0.31),
        0.015,
        0.001,
    )
    # A small upright ceramic pen rest supports the shoulder without loading the nib.
    stand = w.Geom.Xform.Define(s, "/Asset/Rest")
    stand.AddTranslateOp().Set(w.Gf.Vec3d(*map(float, C.REST)))
    w.rounded_box(
        s,
        "/Asset/Rest/Foot",
        (0.057, 0.062, 0.012),
        (0, 0, 0.010),
        (0.70, 0.72, 0.67),
        0.010,
        0.002,
    )
    w.cylinder(s, "/Asset/Rest/Post", 0.015, 0.032, (0, 0, 0.034), (0.70, 0.72, 0.67))
    ring(w, s, "/Asset/Rest/Cup", 0.012, 0.0078, 0.037, 0.0685, (0.70, 0.72, 0.67))
    # The detached cap and barrel make the filling state unambiguous.
    w.cylinder(
        s, "/Asset/Cap", 0.008, 0.057, (0.471, 0.115, C.TABLE_Z + 0.014), TEAL, axis="Y"
    )
    w.box(
        s,
        "/Asset/CapClip",
        (0.004, 0.036, 0.003),
        (0.471, 0.119, C.TABLE_Z + 0.025),
        GOLD,
    )
    w.cylinder(
        s,
        "/Asset/Barrel",
        0.007,
        0.093,
        (0.492, 0.064, C.TABLE_Z + 0.012),
        TEAL,
        axis="Y",
    )
    w.cylinder(
        s,
        "/Asset/CapBand",
        0.0082,
        0.003,
        (0.471, 0.091, C.TABLE_Z + 0.014),
        GOLD,
        axis="Y",
    )
    return s


def bottle(w):
    s = w.stage("bottle")
    # The mouth, shoulder and socket floor constrain the real pen body.
    root = w.body(s, "/Asset/Bottle", 0.12, ccd=True)
    w.box(s, root + "/Base", (0.064, 0.064, 0.008), (0, 0, 0.008), GLASS)
    ring(w, s, root + "/Vessel", 0.031, 0.026, 0.044, 0.029, GLASS)
    ring(w, s, root + "/Well", 0.026, C.WELL_R, 0.039, 0.0315, GLASS)
    ring(w, s, root + "/Mouth", 0.029, C.MOUTH_R, 0.012, 0.054, METAL)
    ring(w, s, root + "/FillingInsert", C.MOUTH_R, C.INSERT_R, 0.012, 0.054, GLASS)
    w.rounded_box(
        s,
        root + "/Label",
        (0.002, 0.036, 0.023),
        (-0.0315, 0, 0.031),
        (0.83, 0.84, 0.75),
        0.001,
        0.00035,
        False,
    )
    w.cylinder(
        s,
        root + "/Ink",
        C.WELL_R - 0.0004,
        0.0015,
        (0, 0, C.INK_Z - C.TABLE_Z),
        INK,
        collide=False,
    )
    return s


def pen(w):
    s = w.stage("pen")
    root = w.body(s, "/Asset/Section", 0.026, articulation=True, ccd=True)
    # Native thin nib and feed. The small collar above it rests on the ceramic cup.
    w.rounded_box(
        s,
        root + "/Nib",
        (0.006, 0.0032, 0.020),
        (0, -0.001, 0.011),
        GOLD,
        0.0014,
        0.0004,
    )
    w.box(
        s,
        root + "/Slit",
        (0.0005, 0.0003, 0.013),
        (0, -0.00265, 0.011),
        GRAY,
        collide=False,
    )
    w.cylinder(s, root + "/Feed", 0.0034, 0.017, (0, 0.0009, 0.017), GRAY)
    w.rounded_box(
        s, root + "/Neck", (0.0132, 0.0126, 0.031), (0, 0, 0.038), TEAL, 0.0031, 0.0004
    )
    w.cylinder(s, root + "/Shoulder", 0.0088, 0.0028, (0, 0, 0.0295), GOLD)
    w.cylinder(s, root + "/Coupling", 0.0045, 0.010, (0, 0, 0.055), METAL)
    # The side window exposes the reservoir and retains its physical scale.
    for i in range(4):
        a = (i + 0.5) * math.pi / 2
        w.cylinder(
            s,
            root + f"/WindowRim{i}",
            0.00042,
            0.035,
            (0.0040 * math.cos(a), 0.0040 * math.sin(a), 0.074),
            (0.71, 0.84, 0.83),
            collide=False,
        )
    w.cylinder(
        s,
        root + "/InkColumn",
        0.00312,
        1.0,
        (0, 0, C.RESERVOIR_BOTTOM),
        INK,
        collide=False,
    )
    ink = w.Geom.Cylinder.Get(s, root + "/InkColumn")
    ink.AddScaleOp().Set(w.Gf.Vec3d(1, 1, 0.00005))
    w.Geom.Imageable(ink.GetPrim()).MakeInvisible()
    w.cylinder(
        s,
        root + "/WindowCollider",
        0.0045,
        0.034,
        (0, 0, 0.075),
        (0.43, 0.62, 0.65),
        collide=False,
    )
    shell = w.Geom.Cylinder.Get(s, root + "/WindowCollider")
    w.Geom.Imageable(shell.GetPrim()).MakeInvisible()
    w.Phy.CollisionAPI.Apply(shell.GetPrim())
    w.Px.PhysxCollisionAPI.Apply(shell.GetPrim()).CreateContactOffsetAttr(0.00025)
    w.cylinder(s, root + "/Head", 0.0045, 0.016, (0, 0, 0.101), METAL)
    knob = w.body(s, "/Asset/Pump", 0.0032, (0, 0, C.BUTTON_Z), ccd=True)
    w.cylinder(s, knob + "/Pad", 0.0040, 0.004, (0, 0, 0.002), GRAY)
    w.cylinder(
        s,
        knob + "/Witness",
        0.0027,
        0.0070,
        (0, 0, -0.002),
        (0.74, 0.26, 0.15),
        collide=False,
    )
    j = w.Phy.PrismaticJoint.Define(s, "/Asset/Stroke")
    j.CreateBody0Rel().SetTargets([root])
    j.CreateBody1Rel().SetTargets([knob])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(0, 0, C.BUTTON_Z))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    j.CreateAxisAttr("Z")
    j.CreateLowerLimitAttr(-C.STROKE)
    j.CreateUpperLimitAttr(0.0)
    j.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(j.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(190.0)
    drive.CreateDampingAttr(1.2)
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateMaxForceAttr(5.0)
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = []
    for name, shape, kind, xyz in (
        ("desk", desk, "base", (0.0, 0.0, 0.0)),
        ("bottle", bottle, "rigid", C.BOTTLE),
        ("pen", pen, "articulation", (*C.REST[:2], C.REST_NIB_Z)),
    ):
        assets.append(
            {"name": name, "type": kind, "file": str(w.save(shape(w))), "position": xyz}
        )
    return {"assets": assets, "settle_actions": 25}


def show_ink(stage, prefix, volume_ml):
    from pxr import Gf, UsdGeom

    ink = UsdGeom.Cylinder.Get(stage, prefix + "/pen/Section/InkColumn")
    if not ink:
        return
    height = max(0.00005, C.RESERVOIR_HEIGHT * volume_ml / C.CAPACITY_ML)
    translation, scale = ink.GetOrderedXformOps()
    translation.Set(Gf.Vec3d(0, 0, C.RESERVOIR_BOTTOM + height / 2))
    scale.Set(Gf.Vec3d(1, 1, height))
    UsdGeom.Imageable(
        ink.GetPrim()
    ).MakeVisible() if volume_ml > 0.001 else UsdGeom.Imageable(
        ink.GetPrim()
    ).MakeInvisible()
    dot = UsdGeom.Cylinder.Get(stage, prefix + "/bottle/Bottle/Ink")
    if dot:
        z = C.INK_Z - C.TABLE_Z - volume_ml * 1e-6 / (C.WELL_R**2 * math.pi)
        dot.GetOrderedXformOps()[0].Set(Gf.Vec3d(0, 0, z))
