"""Compound rigid fixtures and native switch and blade articulation."""

from math import cos, pi, sin, sqrt

import numpy as np

from . import config as C
from .motion import rotation
from .scoring import BANKS, tracer_color
from .usd_scene import Writer

SILVER = (0.58, 0.64, 0.66)
STEEL = (0.33, 0.38, 0.41)
TEAL = (0.072, 0.33, 0.37)
RUBBER = (0.072, 0.12, 0.13)
YELLOW = (0.95, 0.57, 0.14)
PORCELAIN = (0.86, 0.87, 0.82)
SOUP = (0.87, 0.40, 0.12)


def bench(w):
    s = w.stage("bench")
    w.rounded_box(
        s,
        "/Asset/Counter",
        (0.96, 1.20, 0.036),
        (0.40, 0.0, C.TABLE_Z - 0.018),
        (0.52, 0.44, 0.35),
        0.020,
        0.002,
    )
    w.box(s, "/Asset/Floor", (4.0, 4.0, 0.020), (0.40, 0.0, -0.010), (0.36, 0.41, 0.42))
    w.box(
        s,
        "/Asset/BackWall",
        (0.022, 1.6, 1.4),
        (0.914, 0.0, 0.70),
        (0.71, 0.76, 0.72),
        collide=False,
    )
    for i, y in enumerate(np.linspace(-0.76, 0.76, 10)):
        w.box(
            s,
            f"/Asset/UprightGrout{i}",
            (0.002, 0.003, 0.43),
            (0.902, float(y), 0.959),
            (0.53, 0.60, 0.56),
            collide=False,
        )
    for i in range(4):
        w.box(
            s,
            f"/Asset/LevelGrout{i}",
            (0.002, 1.6, 0.003),
            (0.902, 0.0, 0.79 + 0.12 * i),
            (0.53, 0.60, 0.56),
            collide=False,
        )

    x, y, z = C.REST
    w.cylinder(s, "/Asset/DripRest", 0.055, 0.009, (x, y, z - 0.0045), PORCELAIN)
    for i in range(14):
        phi = i * 2 * pi / 14
        w.box(
            s,
            f"/Asset/SaucerRim{i}",
            (0.011, 0.023, 0.012),
            (x + 0.048 * cos(phi), y + 0.048 * sin(phi), z + 0.004),
            PORCELAIN,
            rotation("Z", phi),
        )
    # A shallow tapered pocket withstands normal set-down and releases by lifting.
    for i in range(6):
        phi = i * 2 * pi / 6
        w.box(
            s,
            f"/Asset/RestPad{i}",
            (0.008, 0.019, 0.007),
            (x + 0.040 * cos(phi), y + 0.040 * sin(phi), z + 0.0035),
            (0.35, 0.48, 0.48),
            rotation("Z", phi),
        )
    return s


def transparent(w, s, path, color, opacity):
    from pxr import Sdf

    mat = w.Shade.Material.Define(s, "/Asset/JugPlastic")
    surf = w.Shade.Shader.Define(s, "/Asset/JugPlastic/Surface")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(*color))
    surf.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.22)
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    w.Shade.MaterialBindingAPI.Apply(s.GetPrimAtPath(path)).Bind(mat)


def hollow_ring(w, s, path, r0, r1, z0, z1, count, color, *, translucent=False):
    for i in range(count):
        a, b = 2 * pi * i / count, 2 * pi * (i + 1) / count
        points = [
            (r * cos(phi), r * sin(phi), z)
            for z in (z0, z1)
            for r, phi in ((r0, a), (r1, a), (r1, b), (r0, b))
        ]
        name = path + str(i)
        mesh = w.Geom.Mesh.Define(s, name)
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([4] * 6)
        mesh.CreateFaceVertexIndicesAttr(
            [3, 2, 1, 0, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7]
        )
        mesh.CreateSubdivisionSchemeAttr("none")
        w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
            "convexHull"
        )
        w._finish(s, mesh, color, True)
        if translucent:
            transparent(w, s, name, color, 0.24)


def surface(w, s, path):
    """Assign each finite-volume cell a native material instance."""
    from pxr import Sdf

    for ring in range(C.RINGS):
        inner = (C.JUG_RADIUS - 0.002) * sqrt(ring / C.RINGS)
        outer = (C.JUG_RADIUS - 0.002) * sqrt((ring + 1) / C.RINGS)
        for sector in range(C.SECTORS):
            index = ring * C.SECTORS + sector
            phi = np.linspace(
                2 * pi * sector / C.SECTORS, 2 * pi * (sector + 1) / C.SECTORS, 5
            )
            cell = [(outer * cos(q), outer * sin(q), C.SURFACE_Z) for q in phi]
            cell += (
                [(inner * cos(q), inner * sin(q), C.SURFACE_Z) for q in phi[::-1]]
                if ring
                else [(0.0, 0.0, C.SURFACE_Z)]
            )
            mesh = w.Geom.Mesh.Define(s, f"{path}/Cell{index}")
            mesh.CreatePointsAttr(cell)
            mesh.CreateFaceVertexCountsAttr([len(cell)])
            mesh.CreateFaceVertexIndicesAttr(list(range(len(cell))))
            mesh.CreateSubdivisionSchemeAttr("none")
            mesh.CreateDoubleSidedAttr(True)
            material = w.Shade.Material.Define(s, f"{path}/Cell{index}/Material")
            shader = w.Shade.Shader.Define(s, f"{path}/Cell{index}/Material/Color")
            shader.CreateIdAttr("UsdPreviewSurface")
            shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                w.Gf.Vec3f(*tracer_color(BANKS[index]))
            )
            shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.43)
            material.CreateSurfaceOutput().ConnectToSource(
                shader.ConnectableAPI(), "surface"
            )
            w.Shade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)


def thermometer_display(w, s, path):
    from PIL import Image, ImageDraw, ImageFont
    from pxr import Sdf

    image = Image.new("RGB", (160, 64), (34, 43, 41))
    draw = ImageDraw.Draw(image)
    draw.text(
        (80, 32),
        "72°C",
        anchor="mm",
        font=ImageFont.load_default(46),
        fill=(253, 223, 122),
    )
    picture = w.directory / "temperature.png"
    image.save(picture)
    mesh = w.Geom.Mesh.Define(s, path)
    y, z = -0.028, 0.180
    mesh.CreatePointsAttr(
        [
            (0.015, y - 0.028, z - 0.015),
            (0.015, y + 0.028, z - 0.015),
            (0.015, y + 0.028, z + 0.015),
            (0.015, y - 0.028, z + 0.015),
        ]
    )
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr("none")
    coords = w.Geom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, "vertex"
    )
    coords.Set([(1, 0), (0, 0), (0, 1), (1, 1)])
    mat_path = "/Asset/ThermometerFace"
    mat = w.Shade.Material.Define(s, mat_path)
    uv = w.Shade.Shader.Define(s, mat_path + "/UV")
    uv.CreateIdAttr("UsdPrimvarReader_float2")
    uv.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = w.Shade.Shader.Define(s, mat_path + "/Texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(picture))
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(
        uv.ConnectableAPI(), "result"
    )
    tex.CreateOutput("rgb", Sdf.ValueTypeNames.Color3f)
    shader = w.Shade.Shader.Define(s, mat_path + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.ConnectableAPI(), "rgb"
    )
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    w.Shade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)


def pitcher(w):
    s = w.stage("jug")
    body = w.body(s, "/Asset/Jug", 1.94, ccd=True)
    w.cylinder(s, body + "/Base", 0.074, 0.008, (0.0, 0.0, 0.004), (0.30, 0.50, 0.50))
    hollow_ring(
        w,
        s,
        body + "/Walls",
        C.JUG_RADIUS,
        C.JUG_RADIUS + 0.004,
        0.007,
        C.JUG_HEIGHT,
        24,
        (0.57, 0.80, 0.79),
        translucent=True,
    )
    hollow_ring(
        w,
        s,
        body + "/RolledRim",
        C.JUG_RADIUS,
        C.JUG_RADIUS + 0.006,
        C.JUG_HEIGHT - 0.006,
        C.JUG_HEIGHT + 0.002,
        24,
        (0.54, 0.72, 0.70),
    )
    # The loose jug has an ordinary molded U-handle with a rubber overmold.
    for tag, z in (("Upper", 0.129), ("Lower", 0.055)):
        w.rounded_box(
            s,
            body + "/" + tag + "Handle",
            (0.027, 0.067, 0.019),
            (-0.012, 0.100, z),
            TEAL,
            0.006,
            0.002,
        )
    w.rounded_box(
        s,
        body + "/HandleGrip",
        (0.027, 0.021, 0.091),
        (-0.012, 0.122, 0.092),
        RUBBER,
        0.007,
        0.002,
    )
    for i in range(5):
        w.box(
            s,
            body + f"/Tick{i}",
            (0.002, 0.017 - 0.0018 * (i % 2), 0.0025),
            (-0.078, -0.017, 0.048 + 0.017 * i),
            (0.24, 0.44, 0.42),
            collide=False,
        )
    w.cylinder(
        s,
        body + "/SoupVolume",
        C.JUG_RADIUS - 0.002,
        0.054,
        (0.0, 0.0, C.SURFACE_Z - 0.029),
        SOUP,
        collide=False,
    )
    surface(w, s, body + "/SoupSurface")
    w.rounded_box(
        s,
        body + "/Thermometer",
        (0.014, 0.062, 0.039),
        (0.024, -0.028, 0.180),
        RUBBER,
        0.004,
        0.001,
        collide=False,
    )
    w.cylinder(
        s,
        body + "/TemperatureProbe",
        0.0018,
        0.094,
        (0.031, -0.038, 0.125),
        SILVER,
        collide=False,
    )
    w.box(
        s,
        body + "/ProbeClip",
        (0.035, 0.031, 0.017),
        (0.046, -0.046, 0.147),
        TEAL,
        collide=False,
    )
    thermometer_display(w, s, body + "/TemperatureDigits")
    return s


def blender(w):
    s = w.stage("blender")
    tool = w.body(s, "/Asset/Tool", 0.36, articulation=True, ccd=True)
    w.Px.PhysxArticulationAPI.Apply(
        s.GetPrimAtPath(tool)
    ).CreateEnabledSelfCollisionsAttr(False)
    w.cylinder(s, tool + "/BlendShaft", 0.0095, 0.128, (0.0, 0.0, 0.103), SILVER)
    w.cylinder(s, tool + "/Coupling", 0.021, 0.026, (0.0, 0.0, 0.171), STEEL)
    w.rounded_box(
        s,
        tool + "/Handle",
        (0.045, 0.040, 0.131),
        (0.0, 0.0, 0.234),
        TEAL,
        0.014,
        0.004,
    )
    w.rounded_box(
        s,
        tool + "/RubberGrip",
        (0.040, 0.005, 0.084),
        (-0.003, -0.0205, 0.233),
        RUBBER,
        0.002,
        0.001,
    )
    w.rounded_box(
        s, tool + "/Top", (0.050, 0.044, 0.026), (0.0, 0.0, 0.302), TEAL, 0.014, 0.003
    )
    w.box(
        s,
        tool + "/RunLamp",
        (0.014, 0.018, 0.002),
        (-0.007, 0.0, 0.316),
        (0.25, 0.31, 0.22),
        collide=False,
    )
    # This light is visible below the gripping hand while the motor runs.
    w.rounded_box(
        s,
        tool + "/PowerLightBezel",
        (0.011, 0.027, 0.013),
        (-0.017, 0.0, 0.171),
        RUBBER,
        0.003,
        0.001,
        collide=False,
    )
    w.box(
        s,
        tool + "/PowerLightOn",
        (0.003, 0.022, 0.007),
        (-0.023, 0.0, 0.172),
        (0.34, 0.97, 0.61),
        collide=False,
    )
    w.Geom.Imageable(s.GetPrimAtPath(tool + "/PowerLightOn")).MakeInvisible()
    # Circumferential slots leave a recognisable bell guard around the blade.
    w.cylinder(
        s, tool + "/BellRoof", 0.025, 0.008, (0.0, 0.0, C.GUARD_TOP - 0.004), SILVER
    )
    hollow_ring(
        w, s, tool + "/BellCrown", 0.022, 0.027, 0.024, C.GUARD_TOP - 0.002, 14, SILVER
    )
    hollow_ring(w, s, tool + "/BellFoot", 0.024, C.GUARD_RADIUS, 0.0, 0.007, 14, SILVER)
    for i in range(8):
        phi = 2 * pi * i / 8
        w.box(
            s,
            tool + f"/GuardWeb{i}",
            (0.0045, 0.009, 0.020),
            (0.026 * cos(phi), 0.026 * sin(phi), 0.016),
            SILVER,
            rotation("Z", phi),
        )

    button = w.body(s, "/Asset/PowerButton", 0.009, C.BUTTON)
    w.rounded_box(
        s,
        button + "/SoftPad",
        (0.032, 0.008, 0.031),
        (0.0, 0.0, 0.0),
        YELLOW,
        0.003,
        0.001,
    )
    slider = w.Phy.PrismaticJoint.Define(s, "/Asset/PowerTravel")
    slider.CreateBody0Rel().SetTargets([tool])
    slider.CreateBody1Rel().SetTargets([button])
    slider.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, C.BUTTON)))
    slider.CreateAxisAttr("Y")
    slider.CreateLowerLimitAttr(-C.BUTTON_TRAVEL)
    slider.CreateUpperLimitAttr(0.0)
    slider.CreateCollisionEnabledAttr(False)
    spring = w.Phy.DriveAPI.Apply(slider.GetPrim(), "linear")
    spring.CreateTypeAttr("force")
    spring.CreateTargetPositionAttr(0.0015)
    spring.CreateStiffnessAttr(1700.0)
    spring.CreateDampingAttr(4.2)
    spring.CreateMaxForceAttr(18.0)

    axis = (0.0, 0.0, 0.019)
    rotor = w.body(s, "/Asset/Blade", 0.006, axis)
    w.cylinder(
        s, rotor + "/BladeHub", 0.004, 0.007, (0.0, 0.0, 0.0), STEEL, collide=False
    )
    w.rounded_box(
        s,
        rotor + "/BladeArms",
        (0.042, 0.008, 0.002),
        (0.0, 0.0, -0.002),
        STEEL,
        0.0015,
        0.0004,
        collide=False,
    )
    spindle = w.Phy.RevoluteJoint.Define(s, "/Asset/BladeSpin")
    spindle.CreateBody0Rel().SetTargets([tool])
    spindle.CreateBody1Rel().SetTargets([rotor])
    spindle.CreateLocalPos0Attr(w.Gf.Vec3f(*axis))
    spindle.CreateAxisAttr("Z")
    spindle.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(spindle.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateDampingAttr(0.008)
    drive.CreateMaxForceAttr(0.05)

    # The gripper's pads and the molded rubber tolerate a light holding preload.
    for path in (tool + "/RubberGrip", button + "/SoftPad"):
        mat = w.Shade.Material.Define(s, "/Asset/RubberContact")
        rubber = w.Phy.MaterialAPI.Apply(mat.GetPrim())
        rubber.CreateStaticFrictionAttr(1.3)
        rubber.CreateDynamicFrictionAttr(1.1)
        w.Px.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr("max")
        w.Shade.MaterialBindingAPI.Apply(s.GetPrimAtPath(path)).Bind(
            mat, materialPurpose="physics"
        )
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = []
    for name, kind, scene, pos in (
        ("bench", "base", bench(w), (0, 0, 0)),
        ("jug", "rigid", pitcher(w), C.JUG_RESET),
        ("blender", "articulation", blender(w), C.REST),
    ):
        item = {
            "name": name,
            "type": kind,
            "file": w.save(scene),
            "position": list(pos),
        }
        if kind == "articulation":
            item["joints"] = {"PowerTravel": 0.0, "BladeSpin": 0.0}
        assets.append(item)
    w.inventory()
    return {"assets": assets, "settle_actions": 14}
