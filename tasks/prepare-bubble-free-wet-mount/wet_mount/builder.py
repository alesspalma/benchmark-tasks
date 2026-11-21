"""Native contact assets for the slide, loose glass slip and spring-actuated dropper."""

from math import cos, pi, sin

import numpy as np

from . import config as C
from .motion import rotation
from .usd_scene import Writer

DARK = (0.10, 0.16, 0.20)
BLUE = (0.15, 0.41, 0.56)
GOLD = (0.96, 0.62, 0.19)
GLASS = (0.67, 0.87, 0.91)
WATER = (0.21, 0.54, 0.76)


def bench(w):
    s = w.stage("bench")
    w.box(
        s,
        "/Asset/Bench",
        (0.96, 1.16, 0.044),
        (0.35, 0, C.TABLE - 0.022),
        (0.61, 0.67, 0.65),
    )
    w.box(s, "/Asset/Floor", (6, 6, 0.02), (0, 0, -0.01), (0.35, 0.39, 0.4))
    w.box(
        s,
        "/Asset/Backboard",
        (0.024, 1.03, 0.42),
        (0.740, 0, C.TABLE + 0.21),
        (0.67, 0.73, 0.76),
    )
    x, y, _ = C.SLIDE
    h = C.STAGE_TOP - C.TABLE
    depression = 0.018
    w.rounded_box(
        s,
        "/Asset/InspectionPad",
        (0.104, 0.061, h - depression),
        (x, y, C.TABLE + (h - depression) / 2),
        DARK,
        0.007,
        0.003,
    )
    for i, sx in enumerate((-0.032, 0.032)):
        w.box(
            s,
            f"/Asset/SlideRest{i}",
            (0.010, 0.037, depression),
            (x + sx, y, C.STAGE_TOP - depression / 2),
            BLUE,
        )
        w.box(
            s,
            f"/Asset/SlideEndStop{i}",
            (0.004, 0.030, 0.004),
            (x + np.sign(sx) * 0.0415, y, C.STAGE_TOP - 0.0003),
            BLUE,
        )
        for k, sy in enumerate((-0.0158, 0.0158)):
            w.box(
                s,
                f"/Asset/SlideSideStop{i}_{k}",
                (0.010, 0.005, 0.003),
                (x + sx, y + sy, C.STAGE_TOP + 0.0003),
                BLUE,
            )
        for n in range(7):
            w.box(
                s,
                f"/Asset/Scale{i}_{n}",
                (0.003, 0.0005, 0.00015),
                (x + sx, y + (n - 3) * 0.0048, C.STAGE_TOP + 0.0001),
                DARK,
                collide=False,
            )
    x, y, _ = C.SLIP_REST
    h = C.SLIP_REST_TOP - C.TABLE
    w.rounded_box(
        s,
        "/Asset/SlipHolderBase",
        (0.061, 0.056, 0.012),
        (x, y, C.TABLE + 0.006),
        DARK,
        0.006,
        0.002,
    )
    for i, dx in enumerate((-0.006, 0.006)):
        w.box(
            s,
            f"/Asset/SlipHolder{i}",
            (0.004, 0.008, h),
            (x + dx, y, C.TABLE + h / 2),
            BLUE,
        )
    x, y, _ = C.DROPPER
    w.box(s, "/Asset/DropperFoot", (0.047, 0.046, 0.010), (x, y, C.TABLE + 0.005), BLUE)
    for i in range(8):
        angle = i * pi / 4
        radius = 0.01115
        w.box(
            s,
            f"/Asset/DropperSocket{i}",
            (0.0041, 0.009, 0.0605),
            (x + radius * cos(angle), y + radius * sin(angle), C.TABLE + 0.03025),
            BLUE,
            rotation("Z", angle),
        )
    return s


def translucent(w, s, prim_path, name, opacity, color):
    """Use an ordinary preview material so the wet area shows through the glass."""
    material = w.Shade.Material.Define(s, "/Asset/" + name)
    shader = w.Shade.Shader.Define(s, "/Asset/" + name + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    from pxr import Sdf

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        w.Gf.Vec3f(*color)
    )
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.29)
    shader.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.46)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    w.Shade.MaterialBindingAPI.Apply(s.GetPrimAtPath(prim_path)).Bind(material)


def slide(w):
    s = w.stage("slide")
    p = w.body(s, "/Asset", 0.0047, ccd=True)
    w.box(s, p + "/Glass", C.SLIDE_SIZE, (0, 0, 0), GLASS)
    translucent(w, s, p + "/Glass", "SlideMaterial", 0.37, GLASS)
    top = C.SLIDE_SIZE[2] / 2
    w.box(
        s,
        p + "/FrostedGrip",
        (0.012, 0.025, 0.0004),
        (-0.031, 0, top),
        (0.86, 0.92, 0.89),
        collide=False,
    )
    w.box(
        s,
        p + "/ReadingArea",
        (0.025, 0.023, 0.00020),
        (0, 0, -0.00046),
        (0.83, 0.87, 0.86),
        collide=False,
    )
    # The lowercase e is on the top surface of an inert transparent target.
    bitmap = ("00000", "01110", "10001", "11111", "10000", "01110")
    for j, row in enumerate(bitmap):
        for i, val in enumerate(row):
            if val == "1":
                w.box(
                    s,
                    f"{p}/Ink{j}_{i}",
                    (0.00115, 0.00115, 0.00008),
                    ((i - 2) * 0.00112, (2.5 - j) * 0.00112, top + 0.00005),
                    (0.05, 0.08, 0.10),
                    collide=False,
                )
    f = w.Geom.Sphere.Define(s, p + "/WaterDrop")
    f.CreateRadiusAttr(1.0)
    f.AddTranslateOp().Set(w.Gf.Vec3d(*C.DOSE_OFFSET))
    f.AddScaleOp().Set(w.Gf.Vec3f(C.DROP_RADIUS, C.DROP_RADIUS, C.DROP_HEIGHT * 0.5))
    w._finish(s, f, WATER, False)
    f.MakeInvisible()
    wet = w.Geom.Cube.Define(s, p + "/WaterFilm")
    wet.CreateSizeAttr(1.0)
    wet.AddTranslateOp().Set(w.Gf.Vec3d(0, 0, top + 0.00006))
    wet.AddScaleOp().Set(w.Gf.Vec3f(0.022, 0.022, 0.00010))
    w._finish(s, wet, WATER, False)
    translucent(w, s, str(wet.GetPath()), "WaterMaterial", 0.44, WATER)
    wet.MakeInvisible()
    gap = w.Geom.Sphere.Define(s, p + "/TrappedAir")
    gap.CreateRadiusAttr(1.0)
    gap.AddTranslateOp().Set(w.Gf.Vec3d(0.003, 0.002, top + 0.000225))
    gap.AddScaleOp().Set(w.Gf.Vec3f(0.004, 0.004, 0.00010))
    w._finish(s, gap, (0.88, 0.96, 0.97), False)
    gap.MakeInvisible()
    return s


def coverslip(w):
    s = w.stage("coverslip")
    p = w.body(s, "/Asset", C.SLIP_MASS, ccd=True)
    width, thick = C.SLIP_SIZE, C.SLIP_THICK
    w.box(s, p + "/Glass", (width, width, thick), (0, 0, 0), GLASS)
    translucent(w, s, p + "/Glass", "SlipMaterial", 0.11, GLASS)
    for i, dy in enumerate((-1, 1)):
        edge = dy * (width / 2 - 0.00006)
        w.box(
            s,
            f"{p}/LongEdge{i}",
            (width, 0.00012, thick),
            (0, edge, 0),
            GLASS,
            collide=False,
        )
        w.box(
            s,
            f"{p}/ShortEdge{i}",
            (0.00012, width, thick),
            (edge, 0, 0),
            GLASS,
            collide=False,
        )
    return s


def dropper(w):
    s = w.stage("dropper")
    p = w.body(s, "/Asset/Tube", 0.014, articulation=True, ccd=True)
    w.cylinder(s, p + "/Barrel", 0.0068, 0.051, (0.0, 0.0, -0.041), (0.79, 0.84, 0.82))
    w.cylinder(s, p + "/Nozzle", 0.0023, 0.027, (0.0, 0.0, -0.067), (0.79, 0.84, 0.82))
    w.cylinder(s, p + "/Neck", 0.0095, 0.006, (0.0, 0.0, -0.0375), (0.79, 0.84, 0.82))
    w.rounded_box(
        s,
        p + "/BulbFixed",
        (0.012, 0.026, 0.028),
        (-0.0085, 0.0, 0.0),
        GOLD,
        0.0055,
        0.001,
    )
    center = (0.0085, 0.0, 0.0)
    pad = w.body(s, "/Asset/Bulb", 0.003, position=center, ccd=True)
    w.rounded_box(
        s,
        pad + "/BulbSqueeze",
        (0.012, 0.026, 0.028),
        (0.0, 0.0, 0.0),
        GOLD,
        0.0055,
        0.001,
    )
    joint = w.Phy.PrismaticJoint.Define(s, "/Asset/BulbTravel")
    joint.CreateBody0Rel().SetTargets([p])
    joint.CreateBody1Rel().SetTargets([pad])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*center))
    joint.CreateAxisAttr("X")
    joint.CreateLowerLimitAttr(-C.BULB_TRAVEL)
    joint.CreateUpperLimitAttr(0.0)
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateStiffnessAttr(600.0)
    drive.CreateDampingAttr(2.1)
    drive.CreateMaxForceAttr(12.0)
    drive.CreateTargetPositionAttr(0.0006)
    return s


def create_assets(directory):
    w = Writer(directory)
    a = C.DROPPER_YAW / 2
    stages = [
        ("bench", "base", bench(w), (0, 0, 0), (0, 0, 0, 1)),
        ("slide", "rigid", slide(w), C.SLIDE, (0, 0, 0, 1)),
        ("coverslip", "rigid", coverslip(w), C.SLIP_REST, (0, 0, 0, 1)),
        ("dropper", "articulation", dropper(w), C.DROPPER, (0, 0, sin(a), cos(a))),
    ]
    w.inventory()
    return {
        "assets": [
            {
                "name": name,
                "type": kind,
                "file": w.save(stage),
                "position": pos,
                "quaternion": quat,
            }
            for name, kind, stage, pos, quat in stages
        ],
        "settle_actions": 12,
    }


def update_water(stage, root, water, metrics):
    from pxr import Gf, UsdGeom

    film = metrics["wetted_fraction"]
    top = C.SLIDE_SIZE[2] / 2
    x, y, _ = np.asarray(metrics["slip_relative_mm"]) / 1000
    handles = [
        UsdGeom.Xformable(stage.GetPrimAtPath(root + "/" + k))
        for k in ("WaterDrop", "WaterFilm", "TrappedAir")
    ]
    bead, patch, gap = handles
    if water.volume_ml and film < 0.2:
        UsdGeom.Imageable(bead.GetPrim()).MakeVisible()
        bead.GetOrderedXformOps()[0].Set(
            Gf.Vec3d(*water.center, top + C.DROP_HEIGHT * 0.40)
        )
    else:
        UsdGeom.Imageable(bead.GetPrim()).MakeInvisible()
    if film > 0.03:
        UsdGeom.Imageable(patch.GetPrim()).MakeVisible()
        translation, scale = patch.GetOrderedXformOps()
        side = C.SLIP_SIZE * np.sqrt(film + metrics["air_fraction"])
        translation.Set(Gf.Vec3d(x, y, top + 0.00006))
        scale.Set(Gf.Vec3f(side, side, 0.00010))
    else:
        UsdGeom.Imageable(patch.GetPrim()).MakeInvisible()
    bubble = metrics["largest_bubble_mm"] / 2000
    if film > 0.12 and bubble > 0.0005:
        UsdGeom.Imageable(gap.GetPrim()).MakeVisible()
        gap.GetOrderedXformOps()[1].Set(Gf.Vec3f(bubble, bubble, 0.0001))
    else:
        UsdGeom.Imageable(gap.GetPrim()).MakeInvisible()
