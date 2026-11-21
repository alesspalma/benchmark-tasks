"""Camera, two physical lens-control joints, resolution chart and monitor."""

from math import cos, pi, sin
from pathlib import Path

import numpy as np

from . import config as C
from . import optics
from .usd_scene import Writer

GRAPHITE = (0.10, 0.13, 0.16)
RUBBER = (0.17, 0.19, 0.22)
STEEL = (0.49, 0.54, 0.55)
ALUMINUM = (0.72, 0.76, 0.78)
GOLD = (0.86, 0.54, 0.14)
TEAL = (0.12, 0.50, 0.54)
BLACK = (0.018, 0.020, 0.026)
WHITE = (0.86, 0.88, 0.87)


def joint(w, s, name, parent, child, anchor, axis, limits, rotational):
    parent_path, parent_origin = parent
    child_path, child_origin = child
    api = w.Phy.RevoluteJoint if rotational else w.Phy.PrismaticJoint
    j = api.Define(s, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([parent_path])
    j.CreateBody1Rel().SetTargets([child_path])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, np.subtract(anchor, parent_origin))))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(*map(float, np.subtract(anchor, child_origin))))
    j.CreateAxisAttr(axis)
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    j.CreateCollisionEnabledAttr(False)


def bench(w):
    s = w.stage("bench")
    w.box(
        s,
        "/Asset/Bench",
        (1.20, 1.18, 0.044),
        (0.47, 0, C.TABLE_Z - 0.022),
        (0.47, 0.50, 0.47),
    )
    w.box(s, "/Asset/Floor", (4, 4, 0.020), (0, 0, -0.010), (0.31, 0.35, 0.34))
    return s


def camera(w):
    s = w.stage("camera")
    base = w.body(s, "/Asset/Housing", 1.3, fixed=True, articulation=True)
    x, y, z = C.FOCUS_ORIGIN
    w.rounded_box(
        s,
        base + "/StandBase",
        (0.145, 0.154, 0.010),
        (x - 0.042, y, C.TABLE_Z + 0.006),
        STEEL,
        0.009,
        0.001,
    )
    w.box(
        s,
        base + "/StandPost",
        (0.024, 0.030, 0.100),
        (x - 0.063, y, C.TABLE_Z + 0.055),
        ALUMINUM,
    )
    w.rounded_box(
        s,
        base + "/MachineVisionBody",
        (0.058, 0.049, 0.047),
        (x - 0.054, y, z),
        TEAL,
        0.004,
        0.001,
    )
    w.cylinder(
        s, base + "/MountCollar", 0.021, 0.012, (x - 0.021, y, z), BLACK, axis="X"
    )
    w.cylinder(
        s,
        base + "/IrisCollar",
        0.020,
        0.012,
        (x + 0.022, y, z),
        GRAPHITE,
        axis="X",
        collide=False,
    )
    w.cylinder(
        s,
        base + "/LensHood",
        0.023,
        0.025,
        (x + 0.036, y, z),
        BLACK,
        axis="X",
        collide=False,
    )
    w.cylinder(
        s,
        base + "/CoatedFrontOptic",
        0.015,
        0.001,
        (x + 0.049, y, z),
        (0.10, 0.27, 0.24),
        axis="X",
        collide=False,
    )
    w.cylinder(
        s,
        base + "/OpticReflection",
        0.0028,
        0.001,
        (x + 0.050, y - 0.007, z + 0.006),
        TEAL,
        axis="X",
        collide=False,
    )
    w.box(
        s,
        base + "/RadialLockBoss",
        (0.018, 0.020, 0.018),
        (C.LOCK_ORIGIN[0], -0.028, C.LOCK_ORIGIN[2]),
        GRAPHITE,
    )
    for row in range(5):
        w.box(
            s,
            base + f"/CoolingFin{row}",
            (0.004, 0.044, 0.003),
            (x - 0.072 + row * 0.008, y, z + 0.025),
            STEEL,
            collide=False,
        )
    for offset in (-0.036, 0.036):
        w.cylinder(
            s,
            base + f"/FixtureScrew{offset}",
            0.005,
            0.0025,
            (x - 0.037, offset, C.TABLE_Z + 0.012),
            GRAPHITE,
            collide=False,
        )

    focus_pos = C.FOCUS_ORIGIN
    focus = w.body(s, "/Asset/FocusRing", 0.055, focus_pos)
    w.cylinder(s, focus + "/Ring", C.FOCUS_RADIUS, 0.022, (0, 0, 0), RUBBER, axis="X")
    for i in range(36):
        angle = 2 * pi * i / 36
        w.cylinder(
            s,
            focus + f"/Knurl{i}",
            0.0011,
            0.021,
            (0, 0.0227 * cos(angle), 0.0227 * sin(angle)),
            STEEL,
            axis="X",
            collide=False,
        )
    gx, gy, gz = C.FOCUS_GRIP
    w.cylinder(
        s,
        focus + "/FocusLeverStem",
        0.0050,
        gy - 0.0160,
        (gx, (gy + 0.016) / 2 - 0.002, gz),
        STEEL,
        axis="Y",
    )
    w.rounded_box(
        s,
        focus + "/FocusLeverTip",
        (0.017, 0.020, 0.016),
        C.FOCUS_GRIP,
        GOLD,
        0.003,
        0.001,
    )
    w.box(
        s,
        focus + "/ScaleIndex",
        (0.008, 0.003, 0.0015),
        (0.0, 0.0, C.FOCUS_RADIUS + 0.0002),
        WHITE,
        collide=False,
    )
    joint(
        w,
        s,
        "FocusHelix",
        (base, (0, 0, 0)),
        (focus, focus_pos),
        focus_pos,
        "X",
        (-16, 73),
        True,
    )

    lock_pos = C.LOCK_ORIGIN
    slide = w.body(s, "/Asset/LockThread", 0.020, lock_pos)
    w.cylinder(
        s,
        slide + "/StainlessShank",
        0.0032,
        0.012,
        (0, 0.005, 0),
        ALUMINUM,
        axis="Y",
        collide=False,
    )
    joint(
        w,
        s,
        "LockAdvance",
        (base, (0, 0, 0)),
        (slide, lock_pos),
        lock_pos,
        "Y",
        (-0.0003, 0.0007),
        False,
    )
    thumb = w.body(s, "/Asset/Thumbscrew", 0.028, lock_pos)
    w.cylinder(
        s,
        thumb + "/ThumbHead",
        C.LOCK_RADIUS,
        0.012,
        (0, -0.004, 0),
        ALUMINUM,
        axis="Y",
    )
    for i in range(6):
        angle = 2 * pi * i / 6
        w.cylinder(
            s,
            thumb + f"/ThumbLobe{i}",
            0.003,
            0.012,
            (0.0071 * cos(angle), -0.004, 0.0071 * sin(angle)),
            ALUMINUM,
            axis="Y",
        )
    w.box(
        s,
        thumb + "/ThumbIndex",
        (0.013, 0.001, 0.0026),
        (0, -0.0103, 0),
        GOLD,
        collide=False,
    )
    joint(
        w,
        s,
        "LockScrew",
        (slide, lock_pos),
        (thumb, lock_pos),
        lock_pos,
        "Y",
        (-9, 96),
        True,
    )
    for moving in (focus, slide, thumb):
        w.Px.PhysxRigidBodyAPI.Get(s, moving).CreateDisableGravityAttr(True)
    return s


def chart(w):
    s = w.stage("chart")
    x = C.FOCUS_ORIGIN[0] + C.TARGET_DISTANCE
    z = C.FOCUS_ORIGIN[2]
    w.box(
        s,
        "/Asset/TargetBase",
        (0.105, 0.196, 0.009),
        (x, 0, C.TABLE_Z + 0.0048),
        ALUMINUM,
    )
    w.box(
        s,
        "/Asset/TargetUpright",
        (0.016, 0.018, 0.090),
        (x + 0.014, 0, C.TABLE_Z + 0.050),
        STEEL,
    )
    w.rounded_box(
        s,
        "/Asset/BacklightHousing",
        (0.024, 0.141, 0.103),
        (x + 0.014, 0, z),
        GRAPHITE,
        0.005,
        0.001,
    )
    w.box(s, "/Asset/ChartBacking", (0.002, 0.122, 0.082), (x + 0.0005, 0, z), WHITE)
    value = C.CHART_SPAN / optics.IMAGE_SHAPE[1]
    for i, run in enumerate((39, 53, 67, 81, 95, 109, 123, 137, 151, 165, 179, 193)):
        w.box(
            s,
            f"/Asset/TargetStripe{i}",
            (0.0004, 7 * value, 56 * value),
            (x - 0.0012, (112 - run) * value, z - 4 * value),
            BLACK,
            collide=False,
        )
    for dy, dz in ((-0.049, -0.030), (0.049, -0.030), (-0.049, 0.030), (0.049, 0.030)):
        w.box(
            s,
            f"/Asset/Registration_{dy}_{dz}",
            (0.0005, 0.007, 0.007),
            (x - 0.0012, dy, z + dz),
            BLACK,
            collide=False,
        )
    return s


def monitor(w):
    from pxr import Sdf

    s = w.stage("monitor")
    x, y, z = C.DISPLAY_POSITION
    w.box(s, "/Asset/Foot", (0.108, 0.245, 0.012), (x, y, C.TABLE_Z + 0.006), GRAPHITE)
    w.box(
        s,
        "/Asset/MonitorStand",
        (0.027, 0.062, 0.152),
        (x + 0.014, y, C.TABLE_Z + 0.079),
        STEEL,
    )
    w.rounded_box(
        s,
        "/Asset/Shell",
        (0.035, 0.326, 0.191),
        (x + 0.012, y, z),
        GRAPHITE,
        0.010,
        0.002,
    )
    w.box(
        s,
        "/Asset/PowerLED",
        (0.002, 0.003, 0.003),
        (x - 0.007, y - 0.147, z - 0.085),
        TEAL,
        collide=False,
    )
    vertices = [
        (x - 0.007, y + 0.145, z - 0.073),
        (x - 0.007, y - 0.145, z - 0.073),
        (x - 0.007, y - 0.145, z + 0.073),
        (x - 0.007, y + 0.145, z + 0.073),
    ]
    mesh = w.Geom.Mesh.Define(s, "/Asset/Screen")
    mesh.CreatePointsAttr([w.Gf.Vec3f(*p) for p in vertices])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr("none")
    st = w.Geom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, "vertex"
    )
    st.Set([(0, 0), (1, 0), (1, 1), (0, 1)])
    material = w.Shade.Material.Define(s, "/Asset/MonitorMaterial")
    surface = w.Shade.Shader.Define(s, "/Asset/MonitorMaterial/Surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.65)
    texture = w.Shade.Shader.Define(s, "/Asset/MonitorMaterial/Preview")
    texture.CreateIdAttr("UsdUVTexture")
    level = optics.blur_level(0.0)
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(f"./preview-{level:02}.png")
    )
    texture.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    coordinates = w.Shade.Shader.Define(s, "/Asset/MonitorMaterial/UV")
    coordinates.CreateIdAttr("UsdPrimvarReader_float2")
    coordinates.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    coordinates.CreateOutput("result", Sdf.ValueTypeNames.Float2)
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        coordinates.ConnectableAPI(), "result"
    )
    texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(), "rgb"
    )
    surface.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        texture.ConnectableAPI(), "rgb"
    )
    material.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    w.Shade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    return s


def show_display(stage, directory, angle):
    from pxr import Sdf

    level = optics.blur_level(angle)
    preview = stage.GetPrimAtPath("/World/envs/env_0/monitor/MonitorMaterial/Preview")
    path = str(Path(directory) / f"preview-{level:02}.png")
    preview.GetAttribute("inputs:file").Set(Sdf.AssetPath(path))
    return level


def create_assets(directory):
    w = Writer(directory)
    optics.write_display_images(directory)
    outputs = []
    for name, spec, kind in (
        ("bench", bench, "base"),
        ("camera", camera, "articulation"),
        ("chart", chart, "base"),
        ("monitor", monitor, "base"),
    ):
        outputs.append({"name": name, "type": kind, "file": w.save(spec(w))})
    w.inventory()
    return {"assets": outputs, "settle_actions": 40}
