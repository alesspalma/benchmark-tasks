"""Native counter, kitchen utensils, ricer hinge, ram and yielding potato."""

from math import cos, pi, sin

from . import config as C
from .motion import rotation
from .render import potato_skin
from .usd_scene import Writer

STEEL = (0.53, 0.62, 0.66)
EDGE = (0.70, 0.74, 0.75)
DARK = (0.090, 0.12, 0.13)
RUBBER = (0.105, 0.32, 0.34)
POTATO = (0.91, 0.67, 0.25)
COOKED = (0.83, 0.56, 0.20)
CERAMIC = (0.69, 0.34, 0.23)
GLASS = (0.53, 0.75, 0.70)


def profile_segment(w, s, path, sections, i, n, color, collide=True):
    """A hollow profile assembled from native convex wedges."""
    a = (2 * pi / n) * i
    b = (2 * pi / n) * (i + 1)
    points = [
        (radius * cos(theta), radius * sin(theta), height)
        for inner, outer, height in sections
        for radius, theta in ((inner, a), (outer, a), (outer, b), (inner, b))
    ]
    faces = [[3, 2, 1, 0], [len(points) - 4 + j for j in range(4)]]
    for base in range(0, len(points) - 4, 4):
        for side in range(4):
            nxt = (side + 1) % 4
            faces.append([base + side, base + nxt, base + nxt + 4, base + side + 4])
    obj = w.Geom.Mesh.Define(s, path)
    obj.CreatePointsAttr(points)
    obj.CreateFaceVertexCountsAttr([len(face) for face in faces])
    obj.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
    obj.CreateSubdivisionSchemeAttr("none")
    if collide:
        w.Phy.MeshCollisionAPI.Apply(obj.GetPrim()).CreateApproximationAttr(
            "convexHull"
        )
    return w._finish(s, obj, color, collide)


def transparent(w, s, path, color, opacity):
    from pxr import Sdf

    mat = w.Shade.Material.Define(s, "/Asset/BowlGlass")
    shade = w.Shade.Shader.Define(s, "/Asset/BowlGlass/Surface")
    shade.CreateIdAttr("UsdPreviewSurface")
    shade.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        w.Gf.Vec3f(*color)
    )
    shade.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    shade.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.24)
    mat.CreateSurfaceOutput().ConnectToSource(shade.ConnectableAPI(), "surface")
    for geom in path:
        w.Shade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(mat)


def bench(w):
    s = w.stage("bench")
    w.rounded_box(
        s,
        "/Asset/Counter",
        (1.03, 1.23, 0.040),
        (0.38, 0.0, C.TABLE_Z - 0.020),
        (0.53, 0.45, 0.37),
        0.018,
        0.003,
    )
    w.box(s, "/Asset/Floor", (3.5, 4.0, 0.025), (0.40, 0.0, -0.0125), DARK)
    w.box(
        s,
        "/Asset/KitchenWall",
        (0.025, 1.43, 0.65),
        (0.865, 0.0, 0.925),
        (0.74, 0.77, 0.69),
        collide=False,
    )
    for i in range(10):
        w.box(
            s,
            f"/Asset/TileV{i}",
            (0.002, 0.003, 0.55),
            (0.851, -0.636 + i * 0.141, 1.00),
            (0.49, 0.56, 0.55),
            collide=False,
        )
    for i in range(4):
        w.box(
            s,
            f"/Asset/TileH{i}",
            (0.002, 1.43, 0.003),
            (0.851, 0.0, 0.76 + i * 0.14),
            (0.49, 0.56, 0.55),
            collide=False,
        )
    w.cylinder(
        s, "/Asset/SidePlate", 0.055, 0.007, C.PORTION - [0.0, 0.0, 0.0035], CERAMIC
    )
    for i in range(20):
        t = 2 * pi * i / 20
        w.box(
            s,
            f"/Asset/SidePlateLip{i}",
            (0.008, 0.0175, 0.007),
            C.PORTION + [0.053 * cos(t), 0.053 * sin(t), -0.001],
            CERAMIC,
            rotation("Z", t),
        )
    w.cylinder(
        s,
        "/Asset/BowlMat",
        0.079,
        0.004,
        C.BOWL + [0.0, 0.0, 0.002],
        DARK,
        collide=False,
    )
    return s


def bowl(w):
    s = w.stage("bowl")
    h = C.BOWL_TOP - C.TABLE_Z
    w.cylinder(s, "/Asset/Foot", 0.066, 0.008, (0.0, 0.0, 0.004), GLASS)
    surfaces = []
    layers = ((0.061, 0.065, 0.007), (0.077, 0.081, 0.031), (0.107, 0.111, h - 0.006))
    for i in range(24):
        for level in range(len(layers) - 1):
            surfaces.append(
                profile_segment(
                    w,
                    s,
                    f"/Asset/Wall{i}Band{level}",
                    layers[level : level + 2],
                    i,
                    24,
                    GLASS,
                )
            )
        surfaces.append(
            profile_segment(
                w,
                s,
                f"/Asset/Rim{i}",
                ((0.106, 0.115, h - 0.006), (0.106, 0.115, h)),
                i,
                24,
                GLASS,
            )
        )
    transparent(w, s, surfaces, GLASS, 0.38)
    for arm, x in enumerate((-0.111, 0.111)):
        for side, y in enumerate((-0.025, 0.025)):
            w.rounded_box(
                s,
                f"/Asset/RimSaddle{arm}Side{side}",
                (0.033, 0.009, 0.025),
                (x, y, h + 0.008),
                DARK,
                0.002,
                0.001,
            )
    return s


def hinge(w, s, path, parent, child, anchor, lower, upper):
    joint = w.Phy.RevoluteJoint.Define(s, path)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, anchor)))
    joint.CreateAxisAttr("Y")
    joint.CreateLowerLimitAttr(lower)
    joint.CreateUpperLimitAttr(upper)
    joint.CreateCollisionEnabledAttr(False)
    return joint


def drive(w, joint, kind, stiffness, damping, force, target):
    motor = w.Phy.DriveAPI.Apply(joint.GetPrim(), kind)
    motor.CreateTypeAttr("force")
    motor.CreateTargetPositionAttr(target)
    motor.CreateStiffnessAttr(stiffness)
    motor.CreateDampingAttr(damping)
    motor.CreateMaxForceAttr(force)


def press(w):
    s = w.stage("ricer")
    root = w.body(s, "/Asset/Body", 0.34, articulation=True, ccd=True)
    w.Px.PhysxArticulationAPI.Apply(
        s.GetPrimAtPath(root)
    ).CreateEnabledSelfCollisionsAttr(False)
    for i in range(20):
        profile_segment(
            w,
            s,
            f"{root}/BasketWall{i}",
            ((C.BASKET_INNER, 0.041, C.BASKET_FLOOR), (C.BASKET_INNER, 0.041, 0.002)),
            i,
            20,
            STEEL,
        )
        profile_segment(
            w,
            s,
            f"{root}/LippedRim{i}",
            ((0.0375, 0.044, -0.001), (0.0375, 0.044, 0.003)),
            i,
            20,
            EDGE,
        )
    w.cylinder(
        s,
        root + "/PerforatedPlate",
        0.039,
        0.0036,
        (0.0, 0.0, C.BASKET_FLOOR),
        STEEL,
    )
    for i, (x, y) in enumerate(C.DIE_HOLES):
        for tag, z in (
            ("Inside", C.BASKET_FLOOR + 0.00185),
            ("Outside", C.BASKET_FLOOR - 0.00185),
        ):
            w.cylinder(
                s,
                f"{root}/{tag}Perforation{i}",
                0.0017,
                0.0001,
                (x, y, z),
                DARK,
                collide=False,
            )
    w.rounded_box(
        s,
        root + "/LowerLever",
        (0.178, 0.030, 0.015),
        (-0.124, 0, -0.001),
        STEEL,
        0.008,
        0.002,
    )
    lower_grip = w.rounded_box(
        s,
        root + "/LowerGrip",
        (0.100, 0.033, 0.030),
        (-0.173, 0, 0.002),
        RUBBER,
        0.009,
        0.003,
    )
    w.rounded_box(
        s,
        root + "/PivotEar",
        (0.048, 0.034, 0.022),
        (0.051, 0, 0.019),
        STEEL,
        0.008,
        0.002,
    )
    w.rounded_box(
        s,
        root + "/FarBowlHook",
        (0.111, 0.037, 0.018),
        (0.101, 0, 0.0),
        DARK,
        0.009,
        0.003,
    )
    w.rounded_box(
        s,
        root + "/NearPad",
        (0.029, 0.043, 0.015),
        (-0.110, 0, -0.0015),
        DARK,
        0.006,
        0.002,
    )
    for i, local_x in enumerate((-0.099, 0.099)):
        w.rounded_box(
            s,
            f"{root}/LippedBowlFoot{i}",
            (0.014, 0.020, 0.006),
            (local_x, 0, -0.022),
            DARK,
            0.002,
            0.001,
        )
    for i, local_x in enumerate((-0.123, -0.100, 0.100, 0.123)):
        w.rounded_box(
            s,
            f"{root}/RimCatch{i}",
            (0.006, 0.025, 0.012),
            (local_x, 0, -0.012),
            DARK,
            0.002,
            0.001,
        )
    w.cylinder(s, root + "/PivotPin", 0.009, 0.044, C.HINGE, EDGE, "Y")
    lever = w.body(s, "/Asset/Lever", 0.13, C.HINGE)
    w.rounded_box(
        s,
        lever + "/StainlessArm",
        (0.261, 0.031, 0.013),
        (-0.123, 0, 0.0),
        STEEL,
        0.007,
        0.002,
    )
    upper_grip = w.rounded_box(
        s,
        lever + "/UpperGrip",
        (0.122, 0.034, 0.027),
        (-0.182, 0, 0.006),
        RUBBER,
        0.008,
        0.003,
    )
    grip_material = w.Shade.Material.Define(s, "/Asset/RubberPhysics")
    grip_properties = w.Phy.MaterialAPI.Apply(grip_material.GetPrim())
    grip_properties.CreateStaticFrictionAttr(1.40)
    grip_properties.CreateDynamicFrictionAttr(1.15)
    grip_properties.CreateRestitutionAttr(0.0)
    w.Px.PhysxMaterialAPI.Apply(grip_material.GetPrim()).CreateFrictionCombineModeAttr(
        "max"
    )
    for grip in (lower_grip, upper_grip):
        w.Shade.MaterialBindingAPI.Apply(grip.GetPrim()).Bind(
            grip_material, materialPurpose="physics"
        )
    lever_joint = hinge(w, s, "/Asset/HandleHinge", root, lever, C.HINGE, 0.0, 72.0)
    drive(w, lever_joint, "angular", 0.055, 0.016, 0.30, 81.0)
    w.Px.PhysxJointAPI.Apply(lever_joint.GetPrim()).CreateJointFrictionAttr(0.06)

    ram = w.body(s, "/Asset/Ram", 0.072, C.HINGE + C.PLATE_PIN)
    w.cylinder(s, ram + "/RamPin", 0.006, 0.035, (0, 0, 0), EDGE, "Y")
    w.cylinder(s, ram + "/Stem", 0.006, C.PLATE_DROP, (0, 0, -C.PLATE_DROP / 2), STEEL)
    w.cylinder(s, ram + "/Plate", 0.031, 0.005, (0, 0, -C.PLATE_DROP), EDGE)
    ram_joint = hinge(w, s, "/Asset/RamSwivel", lever, ram, C.PLATE_PIN, -115.0, 10.0)
    drive(w, ram_joint, "angular", 0.35, 0.035, 1.8, -76.0)
    return s


def potato(w):
    s = w.stage("potato")
    root = w.body(s, "/Asset/LowerPotato", 0.034, articulation=True, ccd=True)
    w.Px.PhysxArticulationAPI.Apply(
        s.GetPrimAtPath(root)
    ).CreateEnabledSelfCollisionsAttr(False)
    w.cylinder(s, root + "/Base", C.POTATO_RADIUS, 0.008, (0, 0, 0.004), COOKED)
    top = w.body(s, "/Asset/TopPotato", 0.010, (0, 0, C.POTATO_HEIGHT))
    w.cylinder(s, top + "/Cap", C.POTATO_RADIUS * 0.88, 0.004, (0, 0, -0.002), POTATO)
    joint = w.Phy.PrismaticJoint.Define(s, "/Asset/YieldTravel")
    joint.CreateBody0Rel().SetTargets([root])
    joint.CreateBody1Rel().SetTargets([top])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(0, 0, C.POTATO_HEIGHT))
    joint.CreateAxisAttr("Z")
    joint.CreateLowerLimitAttr(-C.FOOD_FULL_TRAVEL)
    joint.CreateUpperLimitAttr(0.0)
    joint.CreateCollisionEnabledAttr(False)
    drive(w, joint, "linear", 350.0, 6.0, 18.0, 0.003)
    skin = w.Geom.Mesh.Define(s, root + "/Skin")
    skin.CreatePointsAttr(potato_skin(C.POTATO_HEIGHT))
    faces = [list(reversed(range(24)))]
    for ring in range(3):
        for i in range(24):
            nxt = (i + 1) % 24
            faces.append(
                [
                    ring * 24 + i,
                    ring * 24 + nxt,
                    (ring + 1) * 24 + nxt,
                    (ring + 1) * 24 + i,
                ]
            )
    faces.append(list(range(72, 96)))
    skin.CreateFaceVertexCountsAttr([len(face) for face in faces])
    skin.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
    skin.CreateSubdivisionSchemeAttr("none")
    w._finish(s, skin, POTATO, False)
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = []
    for name, kind, scene, pos, joints in (
        ("bench", "base", bench(w), (0, 0, 0), {}),
        ("bowl", "base", bowl(w), C.BOWL, {}),
        (
            "ricer",
            "articulation",
            press(w),
            C.RICER,
            {"HandleHinge": C.OPEN, "RamSwivel": -C.OPEN},
        ),
        ("potato", "articulation", potato(w), C.PORTION, {"YieldTravel": 0.0}),
    ):
        item = {
            "name": name,
            "type": kind,
            "file": w.save(scene),
            "position": list(pos),
        }
        if joints:
            item["joints"] = joints
        assets.append(item)
    w.inventory()
    return {"assets": assets, "settle_actions": 24}
