"""Compound rigid geometry for the dispenser and a two-link paper web."""

from math import cos, pi, sin

from yam_arena.ik import matrix_to_quat_xyzw

from . import config as C
from .motion import rotation
from .usd_scene import Writer

PAPER = (0.96, 0.95, 0.89)
CARD = (0.58, 0.44, 0.30)
WALL = (0.65, 0.76, 0.75)
FLOOR = (0.42, 0.46, 0.47)
HOUSING = (0.09, 0.18, 0.21)
SUPPORT = (0.22, 0.30, 0.32)
COVER = (0.19, 0.37, 0.39)
RUBBER = (0.11, 0.12, 0.11)
ORANGE = (0.83, 0.34, 0.10)
METAL = (0.54, 0.58, 0.60)


def solid_mesh(w, stage, path, vertices, faces, color):
    mesh = w.Geom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([w.Gf.Vec3f(*map(float, v)) for v in vertices])
    mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
    mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    w._finish(stage, mesh, color, True, contact=0.00035)


def annulus(w, stage, path, profile, color, segments=24):
    for i in range(segments):
        a, b = [2 * pi * j / segments for j in (i, i + 1)]
        vertices = [
            (r * cos(angle), y, r * sin(angle)) for angle in (a, b) for r, y in profile
        ]
        faces = (
            [3, 2, 1, 0],
            [4, 5, 6, 7],
            [0, 1, 5, 4],
            [1, 2, 6, 5],
            [2, 3, 7, 6],
            [3, 0, 4, 7],
        )
        solid_mesh(w, stage, f"{path}{i:02d}", vertices, faces, color)


def joint(
    w, stage, name, parent, child, at, axis, limits, stiffness, damping, target, force
):
    rotational = name in ("Bend", "Hinge")
    kind = w.Phy.RevoluteJoint if rotational else w.Phy.PrismaticJoint
    j = kind.Define(stage, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([parent])
    j.CreateBody1Rel().SetTargets([child])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, at)))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    j.CreateAxisAttr(axis)
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    j.CreateCollisionEnabledAttr(False)
    d = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular" if rotational else "linear")
    d.CreateTypeAttr("force")
    d.CreateTargetPositionAttr(target)
    d.CreateStiffnessAttr(stiffness)
    d.CreateDampingAttr(damping)
    d.CreateMaxForceAttr(force)


def mass(w, stage, path, inertia):
    w.Phy.MassAPI.Get(stage, path).CreateDiagonalInertiaAttr(w.Gf.Vec3f(*inertia))


def gravity_off(w, stage, path):
    w.Px.PhysxRigidBodyAPI.Get(stage, path).CreateDisableGravityAttr(True)


def roll_rails(w, stage, path, x, y, z, length):
    """The smooth low saddle keeps the roll axis stable as it slides."""
    material = w.Shade.Material.Define(stage, "/Asset/CradlePlastic")
    contact = w.Phy.MaterialAPI.Apply(material.GetPrim())
    contact.CreateStaticFrictionAttr(0.19)
    contact.CreateDynamicFrictionAttr(0.15)
    w.Px.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("min")
    for i, sign in enumerate((-1, 1)):
        shoulder = f"{path}/Shoulder{i}"
        axial = f"{path}/AxialGuide{i}"
        w.rounded_box(
            stage,
            shoulder,
            (0.068, length, 0.014),
            (x + sign * 0.059, y, z - 0.089),
            SUPPORT,
            0.005,
            0.001,
        )
        w.rounded_box(
            stage,
            axial,
            (0.012, length, 0.020),
            (x + sign * 0.086, y, z - 0.048),
            SUPPORT,
            0.004,
            0.001,
        )
        for rail in (shoulder, axial):
            w.Shade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(rail)).Bind(
                material, materialPurpose="physics"
            )


def world(w):
    stage = w.stage("station")
    w.box(stage, "/Asset/Floor", (4.0, 4.0, 0.02), (0.45, 0.0, -0.01), FLOOR)
    w.box(stage, "/Asset/Wall", (0.044, 1.44, 1.63), (0.707, 0.0, 0.815), WALL)
    for j in range(6):
        w.box(
            stage,
            f"/Asset/HorizontalGrout{j}",
            (0.0007, 1.44, 0.003),
            (0.6847, 0.0, 0.382 + j * 0.205),
            (0.51, 0.62, 0.60),
            collide=False,
        )
    for j in range(7):
        w.box(
            stage,
            f"/Asset/VerticalGrout{j}",
            (0.0007, 0.003, 1.44),
            (0.6847, -0.615 + j * 0.205, 0.825),
            (0.51, 0.62, 0.60),
            collide=False,
        )
    w.box(
        stage,
        "/Asset/Worktop",
        (0.425, 0.962, 0.033),
        (-0.1915, 0.0, C.TABLE_Z - 0.0165),
        (0.65, 0.52, 0.37),
    )
    for i, y in enumerate((-0.327, 0.327)):
        w.box(
            stage,
            f"/Asset/ArmShelf{i}",
            (0.151, 0.252, 0.033),
            (0.0395, y, C.TABLE_Z - 0.0165),
            (0.65, 0.52, 0.37),
        )
    w.box(
        stage,
        "/Asset/StockCartTop",
        (0.166, 0.213, 0.029),
        (0.555, 0.236, 0.844),
        SUPPORT,
    )
    w.box(
        stage,
        "/Asset/StockCartBase",
        (0.242, 0.266, 0.022),
        (0.555, 0.236, 0.12),
        HOUSING,
    )
    for i, dy in enumerate((-0.079, 0.079)):
        for j, dx in enumerate((-0.067, 0.067)):
            w.box(
                stage,
                f"/Asset/CartLeg{i}{j}",
                (0.025, 0.025, 0.760),
                (0.555 + dx, 0.236 + dy, 0.453),
                METAL,
            )
            w.cylinder(
                stage,
                f"/Asset/Wheel{i}{j}",
                0.053,
                0.023,
                (0.555 + dx, 0.236 + dy, 0.053),
                RUBBER,
                "Y",
            )
    roll_rails(w, stage, "/Asset/CartCradle", *C.STOCK, 0.212)
    return stage


def cabinet(w):
    stage = w.stage("cabinet")
    x, y, z = C.SEAT
    w.rounded_box(
        stage,
        "/Asset/BackPanel",
        (0.018, 0.316, 0.353),
        (0.676, y, 0.939),
        HOUSING,
        0.003,
        0.0015,
    )
    w.box(stage, "/Asset/UpperBack", (0.116, 0.292, 0.049), (0.628, y, 1.111), HOUSING)
    w.box(stage, "/Asset/LowerMount", (0.185, 0.316, 0.039), (0.586, y, 0.787), HOUSING)
    for i, dy in enumerate((-0.156, 0.156)):
        w.rounded_box(
            stage,
            f"/Asset/LowLip{i}",
            (0.042, 0.020, 0.024),
            (0.478, y + dy, 0.753),
            HOUSING,
            0.006,
            0.001,
        )
    w.rounded_box(
        stage,
        "/Asset/RearSlotEdge",
        (0.042, 0.310, 0.016),
        (0.522, y, 0.762),
        HOUSING,
        0.006,
        0.001,
    )
    for i, sign in enumerate((-1, 1)):
        # Open upper sides give maintenance access to the roll and feed path.
        sy = y + sign * 0.134
        w.rounded_box(
            stage,
            f"/Asset/UpperSide{i}",
            (0.029, 0.028, 0.210),
            (0.671, sy, 0.970),
            HOUSING,
            0.006,
            0.001,
        )
        w.box(
            stage,
            f"/Asset/LowerCheek{i}",
            (0.176, 0.035, 0.060),
            (0.585, sy, 0.801),
            HOUSING,
        )
        w.cylinder(
            stage,
            f"/Asset/HingeCap{i}",
            0.014,
            0.013,
            (*C.COVER_HINGE[[0]], sy, 0.766),
            SUPPORT,
            "Y",
            collide=False,
        )
    roll_rails(w, stage, "/Asset/DispenserCradle", x, y, z, 0.310)
    for i, rx in enumerate((C.NIP_X - 0.020, C.NIP_X + 0.020)):
        # The compliant nip is wider than the collidable rigid envelope of its rubber drums.
        w.cylinder(
            stage,
            f"/Asset/Drum{i}",
            0.014,
            0.212,
            (rx, y, C.NIP_Z - 0.012),
            RUBBER,
            "Y",
        )
        marks = f"/Asset/RollerMarks{i}"
        xform = w.Geom.Xform.Define(stage, marks)
        xform.AddTranslateOp().Set(w.Gf.Vec3d(rx, y, C.NIP_Z - 0.012))
        xform.AddRotateYOp().Set(0.0)
        for j in range(6):
            angle = j * pi / 3
            loc = (0.0145 * cos(angle), 0.0, 0.0145 * sin(angle))
            w.box(
                stage,
                f"{marks}/Ridge{j}",
                (0.0018, 0.193, 0.002),
                loc,
                SUPPORT,
                rotation("Y", -angle),
                collide=False,
            )
        for j in range(7):
            w.cylinder(
                stage,
                f"/Asset/DrumRib{i}{j}",
                0.0146,
                0.011,
                (rx, y + 0.029 * (j - 3), C.NIP_Z - 0.012),
                SUPPORT,
                "Y",
                collide=False,
            )
        for sign in (-1, 1):
            w.cylinder(
                stage,
                f"/Asset/Axle{i}_{sign}",
                0.006,
                0.014,
                (rx, y + sign * 0.111, C.NIP_Z - 0.012),
                METAL,
                "Y",
                collide=False,
            )
    return stage


def mechanism(w):
    stage = w.stage("mechanism")
    root = w.body(stage, "/Asset/Mount", 0.5, fixed=True, articulation=True)
    mass(w, stage, root, (0.01, 0.01, 0.01))
    w.Px.PhysxArticulationAPI.Get(stage, root).CreateEnabledSelfCollisionsAttr(False)
    paddle = w.body(stage, "/Asset/Paddle", 0.080, C.PADDLE)
    mass(w, stage, paddle, (0.0002, 0.00008, 0.0002))
    w.rounded_box(
        stage, paddle + "/Pad", (0.027, 0.285, 0.057), (0, 0, 0), ORANGE, 0.010, 0.0015
    )
    for i, dy in enumerate((-0.130, 0.130)):
        w.rounded_box(
            stage,
            f"{paddle}/PressShoulder{i}",
            (0.027, 0.016, 0.055),
            (0.0, dy, 0.034),
            ORANGE,
            0.006,
            0.001,
        )
    for i in range(-2, 3):
        w.box(
            stage,
            f"{paddle}/GripBand{i}",
            (0.002, 0.008, 0.022),
            (-0.014, i * 0.041, 0),
            (0.64, 0.24, 0.08),
            collide=False,
        )
    for i in (-1, 1):
        w.box(
            stage,
            f"{paddle}/Guide{i}",
            (0.053, 0.014, 0.013),
            (0.031, i * 0.060, 0.001),
            SUPPORT,
            collide=False,
        )
    joint(
        w,
        stage,
        "Stroke",
        root,
        paddle,
        C.PADDLE,
        "X",
        (0, C.PADDLE_TRAVEL),
        52,
        3,
        0,
        18,
    )
    cover = w.body(stage, "/Asset/Cover", 0.55, C.COVER_HINGE)
    mass(w, stage, cover, (0.020, 0.019, 0.004))
    gravity_off(w, stage, cover)
    h = C.COVER_HEIGHT
    w.rounded_box(
        stage,
        cover + "/Face",
        (0.018, 0.311, h - 0.048),
        (-0.006, 0, (h + 0.048) / 2),
        COVER,
        0.006,
        0.002,
    )
    for i in (-1, 1):
        w.rounded_box(
            stage,
            f"{cover}/Return{i + 1}",
            (0.042, 0.013, h - 0.020),
            (0.019, i * 0.146, h / 2),
            HOUSING,
            0.004,
            0.001,
            collide=False,
        )
    w.rounded_box(
        stage,
        cover + "/Crown",
        (0.187, 0.311, 0.018),
        (0.081, 0, h - 0.006),
        COVER,
        0.008,
        0.0015,
        collide=False,
    )
    w.rounded_box(
        stage,
        cover + "/LatchHousing",
        (0.029, 0.056, 0.013),
        (0.172, 0, h - 0.017),
        HOUSING,
        0.003,
        0.0007,
        collide=False,
    )
    w.rounded_box(
        stage,
        cover + "/SightWindow",
        (0.005, 0.117, 0.074),
        (-0.016, 0, 0.218),
        (0.31, 0.52, 0.51),
        0.002,
        0.0006,
        collide=False,
    )
    w.rounded_box(
        stage,
        cover + "/Handle",
        (0.026, 0.178, 0.027),
        (-0.052, 0, h - 0.064),
        HOUSING,
        0.008,
        0.0015,
    )
    for i, dy in enumerate((-0.078, 0.078)):
        w.box(
            stage,
            f"{cover}/HandleMount{i}",
            (0.040, 0.012, 0.019),
            (-0.028, dy, h - 0.064),
            HOUSING,
        )
    joint(
        w,
        stage,
        "Hinge",
        root,
        cover,
        C.COVER_HINGE,
        "Y",
        (-76.5, 0.1),
        7.0,
        1.0,
        -75.63,
        1.1,
    )
    return stage


def towel(w):
    stage = w.stage("roll")
    roll = w.body(stage, "/Asset/Roll", 0.51, articulation=True, ccd=True)
    mass(w, stage, roll, (0.0032, 0.0023, 0.0032))
    w.Px.PhysxArticulationAPI.Get(stage, roll).CreateEnabledSelfCollisionsAttr(False)
    half = C.ROLL_WIDTH / 2
    from pxr import UsdPhysics

    material = UsdPhysics.MaterialAPI.Get(stage, "/Asset/Physics")
    material.CreateStaticFrictionAttr(0.86)
    material.CreateDynamicFrictionAttr(0.72)
    annulus(
        w,
        stage,
        roll + "/Hardwound",
        ((0.028, -half), (C.ROLL_RADIUS, -half), (C.ROLL_RADIUS, half), (0.028, half)),
        PAPER,
    )
    annulus(
        w,
        stage,
        roll + "/Core",
        (
            (C.CORE_RADIUS, -half - 0.0003),
            (0.029, -half - 0.0003),
            (0.029, half + 0.0003),
            (C.CORE_RADIUS, half + 0.0003),
        ),
        CARD,
        20,
    )
    for sign in (-1, 1):
        for radius in (0.042, 0.054, 0.067, 0.080, 0.090):
            for k in range(36):
                a, b = (2 * pi * j / 36 for j in (k, k + 1))
                mid = (a + b) / 2
                w.box(
                    stage,
                    f"{roll}/PaperEdge{sign}_{round(radius * 1000)}_{k}",
                    (radius * (b - a) * 1.03, 0.00065, 0.00011),
                    (radius * cos(mid), sign * (half + 0.00038), radius * sin(mid)),
                    (0.81, 0.80, 0.74),
                    rotation("Y", pi / 2 - mid),
                    collide=False,
                )
    start = (C.PAPER_X, 0, C.PAPER_HINGE_Z)
    carriage = w.body(stage, "/Asset/PayOut", 0.032, start)
    mass(w, stage, carriage, (0.0001, 0.00001, 0.0001))
    gravity_off(w, stage, carriage)
    w.box(
        stage,
        carriage + "/Mount",
        (0.002, 0.195, 0.008),
        (0, 0, 0),
        PAPER,
        collide=False,
    )
    w.Geom.Imageable(stage.GetPrimAtPath(carriage + "/Mount")).MakeInvisible()
    joint(
        w,
        stage,
        "Travel",
        roll,
        carriage,
        start,
        "Z",
        (-0.196, 0.001),
        430,
        12,
        0,
        3.5,
    )
    lead = w.body(stage, "/Asset/FreeEdge", 0.014, start, ccd=True)
    mass(w, stage, lead, (0.00006, 0.000035, 0.00006))
    gravity_off(w, stage, lead)
    w.box(
        stage,
        lead + "/PaperContact",
        (0.0018, 0.181, C.PAPER_LENGTH),
        (0, 0, -C.PAPER_LENGTH / 2),
        PAPER,
    )
    w.Geom.Imageable(stage.GetPrimAtPath(lead + "/PaperContact")).MakeInvisible()
    joint(
        w,
        stage,
        "Bend",
        carriage,
        lead,
        (0, 0, 0),
        "Y",
        (-99.0, 43.0),
        0.0025,
        0.003,
        0,
        0.040,
    )
    web = w.Geom.Mesh.Define(stage, "/Asset/UpperWeb")
    web.CreatePointsAttr([w.Gf.Vec3f(0, 0, 0)] * 10)
    web.CreateFaceVertexCountsAttr([4] * 4)
    web.CreateFaceVertexIndicesAttr([i + j for i in (0, 2, 4, 6) for j in (0, 1, 3, 2)])
    web.CreateExtentAttr(
        [w.Gf.Vec3f(-0.35, -0.65, -0.38), w.Gf.Vec3f(0.26, 0.24, 0.26)]
    )
    web.CreateDoubleSidedAttr(True)
    web.CreateSubdivisionSchemeAttr("none")
    w._finish(stage, web, PAPER, False)
    return stage


def create_assets(directory):
    w = Writer(directory)
    assets = [
        {"name": "station", "type": "base", "file": w.save(world(w))},
        {"name": "dispenser", "type": "base", "file": w.save(cabinet(w))},
        {
            "name": "mechanism",
            "type": "articulation",
            "file": w.save(mechanism(w)),
            "joints": {"Hinge": C.COVER_OPEN, "Stroke": 0.0},
        },
        {
            "name": "roll",
            "type": "articulation",
            "file": w.save(towel(w)),
            "position": C.STOCK,
            "quaternion": matrix_to_quat_xyzw(rotation("Z", C.STOCK_YAW)),
            "joints": {"Bend": 0.0, "Travel": 0.0},
        },
    ]
    w.inventory()
    return {"assets": assets, "settle_actions": 20}
