"""Full-size procedural labware using a free PhysX articulation."""

import math

from . import config as C
from .motion import rotation
from .usd_scene import Writer

TEAL = (0.09, 0.47, 0.57)
PALE = (0.82, 0.88, 0.92)
INK = (0.045, 0.15, 0.17)
CLEAR = (0.75, 0.87, 0.90)
TINT = (0.72, 0.16, 0.24)
MOLD = (0.92, 0.55, 0.08)
GRAY = (0.28, 0.36, 0.38)


def glass_material(w, stage, tint, opacity):
    from pxr import Sdf

    path = "/Asset/GlassMaterial"
    material = w.Shade.Material.Define(stage, path)
    mdl = w.Shade.Shader.Define(stage, path + "/OmniGlass")
    mdl.SetSourceAsset("OmniGlass.mdl", "mdl")
    mdl.SetSourceAssetSubIdentifier("OmniGlass", "mdl")
    mdl.CreateInput("glass_color", Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(*tint))
    mdl.CreateInput("glass_ior", Sdf.ValueTypeNames.Float).Set(1.10)
    mdl.CreateInput("thin_walled", Sdf.ValueTypeNames.Bool).Set(True)
    mdl.CreateOutput("out", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput("mdl").ConnectToSource(mdl.ConnectableAPI(), "out")
    preview = w.Shade.Shader.Define(stage, path + "/Preview")
    preview.CreateIdAttr("UsdPreviewSurface")
    preview.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        w.Gf.Vec3f(*tint)
    )
    preview.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.10)
    preview.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    preview.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(preview.ConnectableAPI(), "surface")
    return material


def bind_visual(w, stage, path, material):
    w.Shade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(material)


def open_wall(w, stage, path, center, radius, low, high, material):
    """A thin cylindrical sleeve with an open mouth at either end."""
    sectors = 48
    points = [
        w.Gf.Vec3f(
            center[0] + radius * math.cos(math.tau * i / sectors),
            center[1] + radius * math.sin(math.tau * i / sectors),
            z,
        )
        for z in (low, high)
        for i in range(sectors)
    ]
    faces = [
        (i, (i + 1) % sectors, (i + 1) % sectors + sectors, i + sectors)
        for i in range(sectors)
    ]
    mesh = w.Geom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4] * sectors)
    mesh.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    bind_visual(w, stage, path, material)


def syringe(w):
    stage = w.stage("syringe")
    barrel = w.body(stage, "/Asset/Barrel", 0.023, articulation=True, ccd=True)
    plunger = w.body(stage, "/Asset/Plunger", 0.006, ccd=True)
    w.Px.PhysxArticulationAPI.Get(stage, barrel).CreateEnabledSelfCollisionsAttr(False)
    for path, center, inertia in (
        (barrel, (0.0, 0.0, 0.054), (0.000024, 0.000024, 0.0000015)),
        (plunger, (0.0, 0.0, 0.117), (0.000005, 0.000005, 0.0000005)),
    ):
        mass = w.Phy.MassAPI.Get(stage, path)
        mass.CreateCenterOfMassAttr(w.Gf.Vec3f(*center))
        mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(*inertia))
    # The outlet is open geometrically. Fluid accounting starts at its tip.
    w.ring(
        stage,
        barrel + "/Outlet",
        C.OUTLET_INNER_R + 0.00055,
        0.0011,
        0.008,
        0.004,
        TEAL,
        n=8,
    )
    w.cylinder(stage, barrel + "/DownstreamHousing", 0.008, 0.007, (0, 0, 0.0115), TEAL)
    w.cylinder(
        stage, barrel + "/MembraneHousing", C.FILTER_R, 0.0055, (0, 0, 0.0165), TEAL
    )
    w.cylinder(
        stage, barrel + "/FilterRim", C.FILTER_R * 0.985, 0.002, (0, 0, 0.0198), PALE
    )
    w.cylinder(stage, barrel + "/LuerLock", 0.0065, 0.009, (0, 0, 0.025), PALE)
    for z in (0.024, 0.027):
        w.cylinder(
            stage,
            barrel + f"/LockLip{round(z * 1000)}",
            0.0068,
            0.00045,
            (0, 0, z),
            GRAY,
            collide=False,
        )

    lo = C.BARREL_BOTTOM
    hi = C.BARREL_FLANGE
    shell = w.cylinder(
        stage, barrel + "/Tube", C.BARREL_R, hi - lo, (0, 0, (lo + hi) / 2), CLEAR
    )
    shell.MakeInvisible()  # The native convex collision surface remains active.
    glass = glass_material(w, stage, (0.98, 0.99, 0.99), 0.18)
    open_wall(w, stage, barrel + "/ClearWall", (0, 0), C.BARREL_R, lo, hi, glass)
    w.cylinder(stage, barrel + "/Shoulder", C.BARREL_R, 0.0026, (0, 0, lo), PALE)
    w.cylinder(stage, barrel + "/FingerFlange", 0.0185, 0.0025, (0, 0, hi), PALE)
    for idx, size in enumerate(((0.012, 0.0019, 0.085), (0.0019, 0.012, 0.085))):
        w.box(
            stage,
            plunger + f"/Stem{idx}",
            size,
            (0, 0, C.PLUNGER_HEAD - 0.044),
            PALE,
            collide=False,
        )
    w.cylinder(
        stage,
        plunger + "/PistonSeal",
        C.BORE_R,
        0.0042,
        (0, 0, lo + C.MAX_TRAVEL + 0.0023),
        INK,
        collide=False,
    )
    w.cylinder(
        stage, plunger + "/ThumbPad", 0.0125, 0.004, (0, 0, C.PLUNGER_HEAD), MOLD
    )
    for ml in range(1, 10):
        z = lo + ml / (C.PISTON_AREA * 1e6)
        w.box(
            stage,
            barrel + f"/Graduation{ml}",
            (0.0003, 0.008 if ml % 2 == 0 else 0.004, 0.00055),
            (C.BARREL_R, 0.002, z),
            INK,
            collide=False,
        )
    water = w.cylinder(
        stage,
        barrel + "/Suspension",
        C.BORE_R * 0.99,
        C.MAX_TRAVEL,
        (0, 0, lo + C.MAX_TRAVEL / 2),
        TINT,
        collide=False,
    )
    water.CreateDisplayOpacityAttr([0.81])

    joint = w.Phy.PrismaticJoint.Define(stage, "/Asset/Depress")
    joint.CreateBody0Rel().SetTargets([barrel])
    joint.CreateBody1Rel().SetTargets([plunger])
    joint.CreateAxisAttr("Z")
    joint.CreateLowerLimitAttr(-float(C.MAX_TRAVEL))
    joint.CreateUpperLimitAttr(0.0)
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(C.FLOW_DAMPING)
    drive.CreateTargetVelocityAttr(0.0)
    drive.CreateMaxForceAttr(22.0)
    return stage


def tray(w):
    stage = w.stage("tray")
    sx, sy = C.TRAY_SIZE
    w.box(stage, "/Asset/TrayFloor", (sx, sy, 0.004), (0, 0, 0.003), (0.44, 0.52, 0.55))
    for sign in (-1, 1):
        w.box(
            stage,
            f"/Asset/TrayEdgeX{sign > 0}",
            (0.004, sy, 0.010),
            (sign * (sx / 2 - 0.002), 0, 0.008),
            GRAY,
        )
        w.box(
            stage,
            f"/Asset/TrayEdgeY{sign > 0}",
            (sx, 0.004, 0.010),
            (0, sign * (sy / 2 - 0.002), 0.008),
            GRAY,
        )

    x, y, z = C.START - C.TRAY
    w.box(stage, "/Asset/StandFoot", (0.069, 0.050, 0.004), (x, y, 0.007), GRAY)
    # Shelf cradles the membrane rim only in the initial position. The open
    # centre clears the outlet. Low locating pins release on a vertical lift.
    shelf = z + 0.01375
    for k in range(3):
        angle = math.tau * k / 3 + math.pi / 6
        dx, dy = math.cos(angle), math.sin(angle)
        w.cylinder(
            stage,
            f"/Asset/ShelfPost{k}",
            0.0027,
            shelf - 0.006,
            (x + dx * 0.0109, y + dy * 0.0109, (shelf + 0.006) / 2),
            GRAY,
        )
        w.cylinder(
            stage,
            f"/Asset/GuidePin{k}",
            0.0032,
            0.009,
            (x + dx * 0.01665, y + dy * 0.01665, shelf + 0.003),
            GRAY,
        )
    # Low well constrains the receiving vial below its mouth, as on a normal
    # vial block. It cannot brace the syringe or its thumb pad.
    x, y, z = C.VIAL - C.TRAY
    w.cylinder(stage, "/Asset/VialPuck", 0.023, 0.006, (x, y, z - 0.0005), GRAY)
    w.cylinder(
        stage,
        "/Asset/VialBase",
        C.VIAL_INNER_R + 0.0015,
        0.003,
        (x, y, z + 0.0015),
        CLEAR,
    )
    glass = glass_material(w, stage, (0.91, 0.96, 0.98), 0.23)
    open_wall(
        w,
        stage,
        "/Asset/VialWall",
        (x, y),
        C.VIAL_INNER_R + 0.0014,
        z + C.VIAL_FLOOR,
        z + C.VIAL_LIP,
        glass,
    )
    open_wall(
        w,
        stage,
        "/Asset/VialLip",
        (x, y),
        C.VIAL_INNER_R + 0.0016,
        z + C.VIAL_LIP - 0.0011,
        z + C.VIAL_LIP,
        glass,
    )
    # Hidden native collision facets keep the opening physically clear.
    for i in range(16):
        a = math.tau * i / 16
        geom = w.cylinder(
            stage,
            f"/Asset/VialRim{i}",
            0.0015,
            0.042,
            (
                x + C.VIAL_INNER_R * math.cos(a),
                y + C.VIAL_INNER_R * math.sin(a),
                z + 0.025,
            ),
            CLEAR,
        )
        geom.MakeInvisible()
    for ml in (2, 4, 6):
        mark = z + C.VIAL_FLOOR + ml / (math.pi * C.VIAL_INNER_R**2 * 1e6)
        for quadrant in range(4):
            angle = math.tau * quadrant / 4
            r = C.VIAL_INNER_R + 0.0015
            w.box(
                stage,
                f"/Asset/Level{ml}_{quadrant}",
                (0.00045, 0.0085, 0.0010),
                (x + r * math.cos(angle), y + r * math.sin(angle), mark),
                MOLD if ml == 4 else GRAY,
                rotation=rotation("Z", angle),
                collide=False,
            )
    water = w.cylinder(
        stage,
        "/Asset/Filtrate",
        C.VIAL_INNER_R * 0.99,
        0.0001,
        (x, y, z + C.VIAL_FLOOR),
        (0.42, 0.74, 0.86),
        collide=False,
    )
    water.CreateDisplayOpacityAttr([0.73])
    water.MakeInvisible()
    return stage


def create_assets(directory):
    writer = Writer(directory)
    return {
        "assets": [
            {
                "name": "syringe",
                "type": "articulation",
                "file": writer.save(syringe(writer)),
                "position": C.START,
            },
            {
                "name": "tray",
                "type": "base",
                "file": writer.save(tray(writer)),
                "position": C.TRAY,
            },
        ],
        "settle_actions": 22,
    }
