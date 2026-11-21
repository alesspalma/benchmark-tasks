"""Native task geometry for a four-leg chair and replaceable wear cap."""

from math import cos, pi, sin
from pathlib import Path

import numpy as np

from . import config as C
from .usd_scene import Writer

STEEL = (0.54, 0.56, 0.55)
CHAIR = (0.13, 0.34, 0.42)
NYLON = (0.105, 0.12, 0.14)
WORN = (0.47, 0.35, 0.29)
FRESH = (0.89, 0.70, 0.42)
BLADE = (0.48, 0.53, 0.56)
TOOL = (0.82, 0.26, 0.10)
RUBBER = (0.12, 0.15, 0.16)


def z_turn(angle):
    return np.array(
        [[cos(angle), -sin(angle), 0], [sin(angle), cos(angle), 0], [0, 0, 1]]
    )


def beam(w, s, path, start, end, width, color):
    start = np.asarray(start)
    end = np.asarray(end)
    centre = (start + end) / 2
    axis = end - start
    length = np.linalg.norm(axis)
    axis = axis / length
    cross = np.cross(axis, [0, 1, 0])
    cross /= np.linalg.norm(cross)
    w.cylinder(
        s,
        path,
        width / 2,
        length,
        centre,
        color,
        rotation=np.column_stack((cross, np.cross(axis, cross), axis)),
    )


def chamfer(w, s, path, angle):
    rim = w.Geom.Mesh.Define(s, path)
    vertices = []
    for radius, z in ((0.0131, 0.0140), (0.0093, 0.0180)):
        for radial, theta in (
            (0.007, angle - 0.262),
            (radius, angle - 0.262),
            (radius, angle + 0.262),
            (0.007, angle + 0.262),
        ):
            vertices.append(w.Gf.Vec3f(radial * cos(theta), radial * sin(theta), z))
    faces = [
        (3, 2, 1, 0),
        (4, 5, 6, 7),
        (0, 1, 5, 4),
        (1, 2, 6, 5),
        (2, 3, 7, 6),
        (3, 0, 4, 7),
    ]
    rim.CreatePointsAttr(vertices)
    rim.CreateFaceVertexCountsAttr([4] * len(faces))
    rim.CreateFaceVertexIndicesAttr([vertex for face in faces for vertex in face])
    rim.CreateSubdivisionSchemeAttr("none")
    w.Phy.MeshCollisionAPI.Apply(rim.GetPrim()).CreateApproximationAttr("convexHull")
    w._finish(s, rim, NYLON, True)


def room(w):
    s = w.stage("bench")
    w.box(
        s,
        "/Asset/Worktop",
        (0.99, 1.24, 0.047),
        (0.18, 0, C.TABLE - 0.0235),
        (0.64, 0.59, 0.52),
    )
    w.box(s, "/Asset/Floor", (4, 4, 0.032), (0, 0, -0.016), (0.40, 0.43, 0.41))
    for x in (-0.218, 0.636):
        w.box(
            s,
            f"/Asset/Frame{x}",
            (0.052, 1.12, 0.060),
            (x, 0, C.TABLE - 0.085),
            (0.30, 0.33, 0.31),
            collide=False,
        )
        for y in (-0.55, 0.55):
            w.box(
                s,
                f"/Asset/BenchLeg{x}_{y}",
                (0.054, 0.054, 0.71),
                (x, y, 0.355),
                (0.27, 0.30, 0.28),
                collide=False,
            )
    w.rounded_box(
        s,
        "/Asset/SeatPad",
        (0.424, 0.42, 0.006),
        (0.509, 0.137, C.TABLE + 0.003),
        (0.29, 0.33, 0.37),
        0.035,
        0.001,
    )
    w.cylinder(
        s,
        "/Asset/CapRest",
        0.010,
        0.0214,
        (C.FRESH_HOME[0], C.FRESH_HOME[1], C.TABLE + 0.0107),
        (0.35, 0.42, 0.46),
    )
    w.rounded_box(
        s,
        "/Asset/MalletHandleRest",
        (0.018, 0.038, 0.017),
        (C.MALLET_HOME[0], C.MALLET_HOME[1] - 0.010, C.TABLE + 0.0085),
        (0.35, 0.42, 0.46),
        0.007,
        0.001,
    )
    return s


def chair_frame(w):
    s = w.stage("chair_frame")
    geom = "/Asset/ChairLocal"
    offset = w.Geom.Xform.Define(s, geom)
    offset.AddTranslateOp().Set(w.Gf.Vec3d(*C.FOOT))
    w.rounded_box(
        s,
        geom + "/MoldedSeat",
        (0.403, 0.394, 0.020),
        (0.175, 0.184, C.TABLE + 0.017 - C.FOOT[2]),
        CHAIR,
        0.042,
        0.006,
    )
    w.rounded_box(
        s,
        geom + "/SeatFlange",
        (0.359, 0.341, 0.012),
        (0.175, 0.184, C.TABLE + 0.029 - C.FOOT[2]),
        CHAIR,
        0.024,
        0.002,
    )
    for i, (dx, dy) in enumerate(((0, 0), (0, 0.36), (0.383, 0.36), (0.383, 0))):
        seat = [
            0.175 + (dx - 0.175) * 0.78,
            0.184 + (dy - 0.184) * 0.83,
            C.TABLE + 0.037 - C.FOOT[2],
        ]
        end = [dx, dy, -0.021]
        elbow = [dx, dy, -0.056]
        beam(w, s, geom + f"/Leg{i}", seat, elbow, 0.0222, STEEL)
        beam(w, s, geom + f"/LegEnd{i}", elbow, end, 0.0222, STEEL)
        w.cylinder(
            s,
            geom + f"/Socket{i}",
            0.0107,
            0.010,
            (dx, dy, -0.020),
            NYLON,
            collide=i != 0,
        )
        if i:
            w.cylinder(
                s, geom + f"/GoodGlide{i}", 0.0128, 0.016, (dx, dy, -0.007), NYLON
            )
            w.cylinder(s, geom + f"/GoodFelt{i}", 0.0148, 0.009, (dx, dy, 0.006), WORN)
    for i, y in enumerate((0.054, 0.313)):
        beam(
            w,
            s,
            geom + f"/BackUpright{i}",
            (0.33, y, -0.324),
            (0.442, y, -0.513),
            0.019,
            STEEL,
        )
        beam(
            w,
            s,
            geom + f"/BackReturn{i}",
            (0.442, y, -0.513),
            (0.446, y, -0.625),
            0.019,
            STEEL,
        )
    w.rounded_box(
        s,
        geom + "/Spine",
        (0.020, 0.336, 0.214),
        (0.446, 0.184, -0.568),
        CHAIR,
        0.008,
        0.004,
    )
    for i, dy in enumerate((-0.064, 0.064)):
        w.rounded_box(
            s,
            geom + f"/BackRib{i}",
            (0.008, 0.012, 0.168),
            (0.457, 0.184 + dy, -0.570),
            (0.17, 0.40, 0.49),
            0.003,
            0.001,
        )
    return s


def chair(w):
    s = w.stage("chair")
    b = w.body(s, "/Asset/ChairFrame", 1.0, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(s, b).CreateEnabledSelfCollisionsAttr(False)
    mass = w.Phy.MassAPI.Get(s, b)
    mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(0.01, 0.01, 0.01))
    pivot = C.FOOT + [0, 0, -0.012]
    link = w.body(s, "/Asset/Swivel", 0.018, pivot)
    w.cylinder(s, link + "/PivotBall", 0.0082, 0.014, (0, 0, 0.002), NYLON)
    w.cylinder(s, link + "/FlangeBody", 0.0128, 0.010, (0, 0, 0.010), NYLON)
    w.cylinder(s, link + "/TopBoss", 0.0083, 0.004, (0, 0, 0.016), NYLON)
    for i in range(12):
        t = 2 * pi * i / 12
        if i == 9:
            continue
        chamfer(w, s, link + f"/Rim{i}", t)
    joint = w.Phy.RevoluteJoint.Define(s, "/Asset/GlideSwivel")
    joint.CreateBody0Rel().SetTargets([b])
    joint.CreateBody1Rel().SetTargets([link])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*pivot))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    joint.CreateAxisAttr("X")
    joint.CreateLowerLimitAttr(-C.SWIVEL_LIMIT)
    joint.CreateUpperLimitAttr(C.SWIVEL_LIMIT)
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateDampingAttr(0.00035)
    drive.CreateStiffnessAttr(0.0005)
    return s


def cap(w, name, color, worn=False):
    s = w.stage(name)
    b = w.body(s, "/Asset/Cap", 0.008, ccd=True)
    w.cylinder(s, b + "/CapFloor", C.CAP_RADIUS, 0.004, (0, 0, 0), NYLON)
    for i in range(12):
        if i == 9:
            continue
        t = 2 * pi * i / 12
        radius = 0.0145
        w.box(
            s,
            b + f"/Skirt{i}",
            (0.0023, 0.0075, 0.006),
            (radius * cos(t), radius * sin(t), -0.0042),
            NYLON,
            z_turn(t),
        )
    if not worn:
        for i in (0, 3, 6, 9):
            t = 2 * pi * i / 12
            w.box(
                s,
                b + f"/FlexibleLip{i}",
                (0.0023, 0.0045, 0.0074),
                (0.0137 * cos(t), 0.0137 * sin(t), -0.0091),
                NYLON,
                z_turn(t),
                collide=False,
            )
    w.cylinder(
        s,
        b + "/Felt",
        C.CAP_RADIUS - 0.0011,
        0.0025 if worn else 0.004,
        (0, 0, 0.00325 if worn else 0.004),
        color,
    )
    if worn:
        for i, (x, y, length) in enumerate(
            ((-0.004, 0.004, 0.010), (0.004, -0.005, 0.008), (0.002, 0.008, 0.004))
        ):
            w.box(
                s,
                b + f"/Worn{i}",
                (length, 0.0008, 0.00017),
                (x, y, 0.00458),
                (0.32, 0.29, 0.26),
                collide=False,
            )
    return s


def screwdriver(w):
    s = w.stage("pry")
    b = w.body(s, "/Asset/Tool", 0.064, ccd=True)
    w.rounded_box(
        s,
        b + "/OrangeGrip",
        (0.020, 0.075, 0.018),
        (0, -0.005, 0.004),
        TOOL,
        0.007,
        0.004,
    )
    w.box(s, b + "/DriverSteel", (0.005, 0.021, 0.005), (0, 0.040, 0.000), BLADE)
    w.box(s, b + "/Taper", (0.0065, 0.012, 0.0020), (0, 0.050, -0.0012), BLADE)
    w.box(s, b + "/PryTip", (0.0065, 0.013, 0.0015), (0, 0.060, -0.0020), BLADE)
    return s


def mallet(w):
    s = w.stage("mallet")
    b = w.body(s, "/Asset/Mallet", 0.093, ccd=True)
    w.rounded_box(
        s,
        b + "/Grip",
        (0.017, 0.084, 0.017),
        (0, 0, 0),
        (0.29, 0.54, 0.68),
        0.006,
        0.003,
    )
    w.cylinder(s, b + "/Head", 0.015, 0.049, (0, 0.061, 0), RUBBER, axis="Z")
    w.cylinder(s, b + "/Face", 0.0154, 0.004, (0, 0.061, -0.024), (0.20, 0.23, 0.22))
    return s


def create_assets(directory):
    w = Writer(Path(directory))
    setups = [
        ("bench", "base", room(w), (0, 0, 0)),
        ("chair_frame", "base", chair_frame(w), (0, 0, 0)),
        ("chair", "articulation", chair(w), (0, 0, 0)),
        (
            "old_cap",
            "rigid",
            cap(w, "old_cap", WORN, True),
            C.FOOT + [0, 0, C.CAP_HEIGHT],
        ),
        ("new_cap", "rigid", cap(w, "new_cap", FRESH), C.FRESH_HOME),
        ("pry", "rigid", screwdriver(w), C.TOOL_HOME),
        ("mallet", "rigid", mallet(w), C.MALLET_HOME + [0, 0, 0.012]),
    ]
    files = [
        {"name": name, "type": kind, "file": w.save(stage), "position": position}
        for name, kind, stage, position in setups
    ]
    w.inventory()
    return {"assets": files, "settle_actions": 10}
