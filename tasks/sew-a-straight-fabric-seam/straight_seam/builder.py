"""Domestic machine geometry, actuators and two separate fabric strips."""

import numpy as np

from . import config as C
from .usd_scene import Writer

BODY = (0.87, 0.85, 0.78)
PANEL = (0.16, 0.29, 0.34)
STEEL = (0.58, 0.61, 0.64)
DARK = (0.13, 0.17, 0.19)
ORANGE = (0.81, 0.32, 0.13)
TEAL = (0.09, 0.44, 0.52)
YELLOW = (0.94, 0.72, 0.20)


def bench(w):
    s = w.stage("bench")
    w.box(
        s,
        "/Asset/Table",
        (0.94, 1.22, 0.040),
        (0.398, 0, C.TABLE - 0.020),
        (0.56, 0.44, 0.35),
    )
    w.box(s, "/Asset/Floor", (4, 4, 0.020), (0.4, 0, -0.010), (0.32, 0.34, 0.36))
    shift = C.MACHINE_X_SHIFT
    w.rounded_box(
        s,
        "/Asset/MachineBed",
        (0.445, 0.435, 0.040),
        (0.403 + shift, -0.018, C.BED - 0.020),
        BODY,
        0.021,
        0.002,
    )
    w.rounded_box(
        s,
        "/Asset/Column",
        (0.146, 0.107, 0.263),
        (0.500 + shift, -0.175, C.BED + 0.1315),
        BODY,
        0.021,
        0.006,
    )
    w.rounded_box(
        s,
        "/Asset/UpperArm",
        (0.142, 0.286, 0.068),
        (0.501 + shift, -0.087, C.BED + 0.220),
        BODY,
        0.023,
        0.006,
    )
    w.rounded_box(
        s,
        "/Asset/NeedleHead",
        (0.124, 0.094, 0.125),
        (0.484 + shift, 0.018, C.BED + 0.175),
        BODY,
        0.014,
        0.004,
    )
    w.rounded_box(
        s,
        "/Asset/Controls",
        (0.012, 0.142, 0.063),
        (0.376 + shift, -0.121, C.BED + 0.188),
        PANEL,
        0.004,
        0.001,
        collide=False,
    )
    w.rounded_box(
        s,
        "/Asset/Display",
        (0.003, 0.061, 0.025),
        (0.3685 + shift, -0.143, C.BED + 0.198),
        DARK,
        0.0014,
        0.0004,
        collide=False,
    )
    w.box(
        s,
        "/Asset/StraightStitchSymbol",
        (0.002, 0.024, 0.0025),
        (0.3665 + shift, -0.143, C.BED + 0.198),
        (0.64, 0.91, 0.86),
        collide=False,
    )
    for i in range(3):
        w.cylinder(
            s,
            f"/Asset/Select{i}",
            0.0048,
            0.002,
            (0.366 + shift, -0.106 + i * 0.013, C.BED + 0.195),
            STEEL,
            axis="X",
            collide=False,
        )
    w.cylinder(
        s,
        "/Asset/Handwheel",
        0.031,
        0.013,
        (0.484 + shift, -0.232, C.BED + 0.205),
        PANEL,
        axis="Y",
    )
    w.cylinder(
        s,
        "/Asset/SpoolPin",
        0.003,
        0.035,
        (0.480 + shift, -0.126, C.BED + 0.267),
        DARK,
        collide=False,
    )
    w.cylinder(
        s,
        "/Asset/ThreadSpool",
        0.013,
        0.027,
        (0.480 + shift, -0.126, C.BED + 0.268),
        (0.94, 0.89, 0.64),
        collide=False,
    )
    w.box(
        s,
        "/Asset/ThroatPlate",
        (0.105, 0.073, 0.00065),
        (0.441 + shift, 0, C.BED + 0.0003),
        STEEL,
        collide=False,
    )
    for i in range(7):
        w.box(
            s,
            f"/Asset/FeedDog{i}",
            (0.003, 0.008, 0.0007),
            (0.417 + i * 0.006 + shift, 0.011, C.BED + 0.0007),
            DARK,
            collide=False,
        )
    for i, y in enumerate((-0.020, -0.013, -0.0065)):
        w.box(
            s,
            f"/Asset/ThroatMark{i}",
            (0.073, 0.00115, 0.00050),
            (0.423 + shift, y, C.BED + 0.0007),
            DARK if i < 2 else (0.06, 0.57, 0.67),
            collide=False,
        )
    bx, by, bz = C.BUTTON
    w.rounded_box(
        s,
        "/Asset/SwitchPedestal",
        (0.069, 0.056, 0.103),
        (bx + 0.020, by, C.BED + 0.049),
        BODY,
        0.009,
        0.001,
    )
    w.cylinder(
        s,
        "/Asset/SwitchBezel",
        0.015,
        0.003,
        (bx, by, bz - 0.004),
        PANEL,
        collide=False,
    )
    return s


def linear(w, s, name, root, link, origin, axis, limits, stiffness, damping, force):
    joint = w.Phy.PrismaticJoint.Define(s, f"/Asset/{name}Joint")
    joint.CreateBody0Rel().SetTargets([root])
    joint.CreateBody1Rel().SetTargets([link])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, origin)))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(limits[0])
    joint.CreateUpperLimitAttr(limits[1])
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateMaxForceAttr(force)
    return name + "Joint"


def machine(w):
    s = w.stage("machine")
    root = w.body(s, "/Asset/Base", 3.0, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(s, root).CreateEnabledSelfCollisionsAttr(False)
    foot = w.body(s, "/Asset/Foot", 0.055, position=C.FOOT_BASE)
    for i, y in enumerate((-0.0075, 0.0075)):
        w.rounded_box(
            s,
            f"{foot}/Toe{i}",
            (0.033, 0.009, 0.006),
            (-0.003, y, 0.003),
            STEEL,
            0.003,
            0.001,
        )
    w.box(s, f"{foot}/Bridge", (0.011, 0.024, 0.005), (0.010, 0, 0.006), STEEL)
    w.cylinder(s, f"{foot}/Bar", 0.0038, 0.116, (0.013, 0, 0.061), STEEL, collide=False)
    needle = w.body(s, "/Asset/Needle", 0.008, position=C.NEEDLE_BASE)
    w.cylinder(
        s, f"{needle}/Shaft", 0.00105, 0.038, (0, 0, 0.019), STEEL, collide=False
    )
    w.cylinder(s, f"{needle}/Clamp", 0.0050, 0.010, (0, 0, 0.041), DARK, collide=False)
    w.cylinder(
        s, f"{needle}/NeedleBar", 0.0041, 0.088, (0, 0, 0.087), STEEL, collide=False
    )
    lever = w.body(s, "/Asset/Lever", 0.030, position=C.LEVER)
    w.rounded_box(
        s, f"{lever}/Grip", (0.017, 0.026, 0.016), (0, 0, 0), PANEL, 0.006, 0.002
    )
    w.box(
        s, f"{lever}/Stem", (0.083, 0.006, 0.007), (0.0415, 0, 0), DARK, collide=False
    )
    button = w.body(s, "/Asset/Button", 0.009, position=C.BUTTON)
    w.cylinder(
        s, f"{button}/Cap", 0.010, 0.012, (0, 0, 0), (0.20, 0.68, 0.43), axis="Z"
    )
    joints = {
        linear(
            w, s, "Foot", root, foot, C.FOOT_BASE, "Z", (0, C.FOOT_TRAVEL), 500, 11, 5
        ): C.FOOT_TRAVEL,
        linear(
            w,
            s,
            "Needle",
            root,
            needle,
            C.NEEDLE_BASE,
            "Z",
            (0, C.NEEDLE_TRAVEL),
            600,
            4,
            4,
        ): C.NEEDLE_TRAVEL,
        linear(
            w, s, "Lever", root, lever, C.LEVER, "Z", (0, C.LEVER_TRAVEL), 105, 2.7, 4
        ): C.LEVER_TRAVEL,
        linear(
            w, s, "Button", root, button, C.BUTTON, "Z", (-0.010, 0), 390, 2.5, 5
        ): 0.0,
    }
    return s, joints


def swatch(w, name, color, clip):
    s = w.stage(name)
    body = w.body(s, "/Asset", 0.022, ccd=True)
    surface = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    surface.GetStaticFrictionAttr().Set(0.45)
    surface.GetDynamicFrictionAttr().Set(0.31)
    w.box(s, f"{body}/WovenCanvas", (C.LENGTH, C.WIDTH, C.THICK), (0, 0, 0), color)
    mass = w.Phy.MassAPI.Get(s, body)
    mass.CreateCenterOfMassAttr(w.Gf.Vec3f(0, 0, 0.002))
    mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(0.000020, 0.000040, 0.000058))
    for i, x in enumerate(np.linspace(-0.068, 0.068, 16)):
        w.box(
            s,
            f"{body}/Warp{i}",
            (0.0005, C.WIDTH, 0.00020),
            (x, 0, 0.00082),
            tuple(v * 0.81 for v in color),
            collide=False,
        )
    for i, y in enumerate(np.linspace(-C.WIDTH / 2 + 0.003, C.WIDTH / 2 - 0.003, 19)):
        w.box(
            s,
            f"{body}/Weft{i}",
            (C.LENGTH, 0.0005, 0.00021),
            (0, y, 0.00082),
            tuple(min(v * 1.18, 1.0) for v in color),
            collide=False,
        )
    w.box(s, f"{body}/ClipFoot", (0.016, 0.025, 0.003), clip - [0, 0, 0.007], YELLOW)
    w.box(
        s,
        f"{body}/Clip",
        (0.016, 0.025, 0.013),
        clip,
        YELLOW,
        rotation=C.FRONT_APPROACH,
    )
    w.box(
        s,
        f"{body}/ClipSpring",
        (0.015, 0.003, 0.011),
        clip + [0.0, -0.005, 0.002],
        (0.42, 0.36, 0.19),
        collide=False,
    )
    return s


def create_assets(directory):
    w = Writer(directory)
    m, joints = machine(w)
    return {
        "settle_actions": 15,
        "assets": [
            {"name": "bench", "type": "base", "file": w.save(bench(w))},
            {
                "name": "machine",
                "type": "articulation",
                "file": w.save(m),
                "joints": joints,
            },
            {
                "name": "upper",
                "type": "rigid",
                "file": w.save(swatch(w, "upper", ORANGE, C.UPPER_CLIP)),
                "position": C.UPPER,
            },
            {
                "name": "lower",
                "type": "rigid",
                "file": w.save(swatch(w, "lower", TEAL, C.LOWER_CLIP)),
                "position": C.LOWER,
            },
        ],
    }
