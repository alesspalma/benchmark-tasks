"""Build the cobot gripper inspection fixture and the native valve controls."""

from math import cos, pi, sin

import numpy as np

from . import config as C
from .usd_scene import Writer

DARK = (0.10, 0.14, 0.17)
METAL = (0.59, 0.65, 0.69)
BLUE = (0.12, 0.48, 0.64)
RIG = (0.20, 0.28, 0.33)
BELLOWS = (0.19, 0.23, 0.28)
LIP = (0.18, 0.21, 0.23)
ORANGE = (0.84, 0.36, 0.08)
TEAL = (0.16, 0.40, 0.38)
WHITE = (0.90, 0.91, 0.88)


def slider(w, s, name, parent, child, origin, axis, limits, stiffness, damping, force):
    j = w.Phy.PrismaticJoint.Define(s, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([parent])
    j.CreateBody1Rel().SetTargets([child])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, origin)))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    j.CreateAxisAttr(axis)
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    j.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(j.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateMaxForceAttr(force)


def bellows_tilt(w, s, name, parent, child, axis):
    joint = w.Phy.RevoluteJoint.Define(s, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(-8.0)
    joint.CreateUpperLimitAttr(8.0)
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(0.0)
    drive.CreateStiffnessAttr(0.60 * pi / 180)
    drive.CreateDampingAttr(0.055 * pi / 180)
    drive.CreateMaxForceAttr(0.55)


def bench(w):
    s = w.stage("bench")
    w.box(
        s,
        "/Asset/Worktop",
        (1.18, 1.16, 0.044),
        (0.44, 0, C.TABLE - 0.022),
        (0.65, 0.64, 0.60),
    )
    w.box(s, "/Asset/Floor", (4, 4, 0.02), (0, 0, -0.01), (0.32, 0.37, 0.39))
    for i, x in enumerate((-0.033, 0.033)):
        w.box(
            s,
            f"/Asset/PaddedPlateRest{i}",
            (0.022, 0.103, 0.016),
            (C.PICK[0] + x, C.PICK[1], C.TABLE + 0.008),
            TEAL,
        )
    return s


def label(w, s, path, letters, position, scale, color):
    glyphs = {
        "0": ("111", "101", "101", "101", "111"),
        "2": ("111", "001", "111", "100", "111"),
        "4": ("101", "101", "111", "001", "001"),
        "6": ("111", "100", "111", "101", "111"),
        "8": ("111", "101", "111", "101", "111"),
        "O": ("111", "101", "101", "101", "111"),
        "N": ("101", "111", "111", "111", "101"),
        "F": ("111", "100", "110", "100", "100"),
        "V": ("101", "101", "101", "101", "010"),
        "E": ("111", "100", "110", "100", "111"),
        "T": ("111", "010", "010", "010", "010"),
        "K": ("101", "110", "100", "110", "101"),
        "P": ("110", "101", "110", "100", "100"),
        "A": ("010", "101", "111", "101", "101"),
    }
    for index, letter in enumerate(letters):
        for row, line in enumerate(glyphs[letter]):
            for col, value in enumerate(line):
                if value == "1":
                    y = -(4 * index + col - (4 * len(letters) - 2) / 2) * scale
                    z = (2 - row) * scale
                    w.box(
                        s,
                        f"{path}_{index}_{row}_{col}",
                        (0.0005, scale, scale),
                        np.asarray(position) + [0, y, z],
                        color,
                        collide=False,
                    )


def gauge(w, s, base):
    x, y, z = C.GAUGE
    w.cylinder(s, base + "/GaugeBody", 0.052, 0.020, (x, y, z), DARK, axis="X")
    w.cylinder(
        s,
        base + "/GaugeFace",
        0.047,
        0.001,
        (x - 0.0105, y, z),
        WHITE,
        axis="X",
        collide=False,
    )
    # Gauge scale progresses from lower left to lower right via the top.
    for i in range(17):
        ang = (315 + 270 * i / 16) * pi / 180
        radius = 0.041
        yc, zc = y + radius * cos(ang), z + radius * sin(ang)
        rot = np.column_stack(
            ([1, 0, 0], [0, cos(ang), sin(ang)], [0, -sin(ang), cos(ang)])
        )
        color = TEAL if i >= 8 else DARK
        w.box(
            s,
            f"{base}/GaugeTick{i}",
            (0.0005, 0.0075 if i % 4 == 0 else 0.0045, 0.0014),
            (x - 0.0113, yc, zc),
            color,
            rot,
            collide=False,
        )
        if i % 4 == 0:
            label(
                w,
                s,
                f"{base}/GaugeNumber{i}",
                str(5 * i),
                (x - 0.0115, y + 0.030 * cos(ang), z + 0.030 * sin(ang)),
                0.0016,
                DARK,
            )
    label(w, s, base + "/GaugeUnits", "KPA", (x - 0.0115, y, z - 0.017), 0.0015, DARK)
    pointer = w.Geom.Xform.Define(s, base + "/GaugeNeedle")
    pointer.AddTranslateOp().Set(w.Gf.Vec3d(x - 0.012, y, z))
    pointer.AddRotateXOp().Set(315.0)
    w.box(
        s,
        base + "/GaugeNeedle/Pointer",
        (0.001, 0.033, 0.0023),
        (0, 0.0165, 0),
        ORANGE,
        collide=False,
    )
    w.cylinder(
        s,
        base + "/GaugePin",
        0.0030,
        0.003,
        (x - 0.013, y, z),
        DARK,
        axis="X",
        collide=False,
    )


def station(w):
    s = w.stage("station")
    base = w.body(s, "/Asset/Chassis", 8.4, fixed=True, articulation=True)
    cx, cy, _ = C.CUP_CENTER
    w.rounded_box(
        s,
        base + "/FixtureFoot",
        (0.258, 0.390, 0.025),
        (0.525, -0.068, C.TABLE + 0.0125),
        RIG,
        0.009,
        0.003,
    )
    w.rounded_box(
        s,
        base + "/SupportColumn",
        (0.069, 0.071, 0.301),
        (0.535, cy, 0.9095),
        RIG,
        0.009,
        0.002,
    )
    w.rounded_box(
        s,
        base + "/HeadMount",
        (0.160, 0.084, 0.039),
        (0.472, cy, 1.048),
        RIG,
        0.008,
        0.002,
    )
    w.rounded_box(
        s,
        base + "/CobotToolBody",
        (0.078, 0.070, 0.045),
        (cx, cy, 1.022),
        METAL,
        0.010,
        0.003,
    )
    w.cylinder(s, base + "/ToolFlange", 0.030, 0.010, (cx, cy, 1.047), DARK)
    w.cylinder(s, base + "/CupStem", 0.011, 0.078, (cx, cy, 0.962), METAL)
    for i in range(3):
        w.rounded_box(
            s,
            f"{base}/ToolCoolingVent{i}",
            (0.002, 0.006, 0.020),
            (cx - 0.037, cy + 0.012 * (i - 1), 1.025),
            DARK,
            0.0018,
            0.0006,
            collide=False,
        )
    w.rounded_box(
        s,
        base + "/ControlManifold",
        (0.087, 0.177, 0.056),
        (0.422, -0.232, 0.818),
        RIG,
        0.007,
        0.002,
    )
    w.cylinder(
        s,
        base + "/ExhaustMuffler",
        0.010,
        0.035,
        (0.462, -0.270, 0.834),
        METAL,
        axis="X",
    )
    gauge(w, s, base)
    # Routed tubing and fittings give the tester a visibly continuous circuit.
    w.cylinder(
        s, base + "/HoseRun", 0.005, 0.240, (0.485, -0.036, 0.895), BLUE, axis="Y"
    )
    for i, y in enumerate((-0.156, 0.084)):
        w.cylinder(s, f"{base}/HoseUpright{i}", 0.005, 0.126, (0.485, y, 0.949), BLUE)
        w.cylinder(s, f"{base}/TubeFitting{i}", 0.007, 0.009, (0.485, y, 0.891), METAL)
    w.cylinder(s, base + "/HeadLine", 0.005, 0.066, (0.454, cy, 1.017), BLUE, axis="X")
    w.box(
        s,
        base + "/SupplyPositionStrip",
        (0.059, 0.030, 0.004),
        (C.SPOOL[0] + 0.013, C.SPOOL[1], 0.850),
        METAL,
    )
    w.box(
        s,
        base + "/ClosedWitness",
        (0.008, 0.010, 0.002),
        (C.SPOOL[0] - 0.015, C.SPOOL[1], 0.853),
        ORANGE,
        collide=False,
    )
    w.box(
        s,
        base + "/OpenWitness",
        (0.008, 0.010, 0.002),
        (C.SPOOL[0] + 0.044, C.SPOOL[1], 0.853),
        TEAL,
        collide=False,
    )
    label(w, s, base + "/ValveClosed", "OFF", (0.376, C.SPOOL[1], 0.830), 0.0022, WHITE)
    label(w, s, base + "/ValveVent", "VENT", (0.376, C.VENT[1], 0.830), 0.0022, WHITE)
    spool = w.body(s, "/Asset/SupplySpool", 0.073, C.SPOOL)
    w.cylinder(
        s, spool + "/SpoolShaft", 0.005, 0.037, (0.012, 0, -0.014), METAL, axis="X"
    )
    w.rounded_box(
        s, spool + "/SpoolGrip", (0.018, 0.025, 0.027), (0, 0, 0), BLUE, 0.004, 0.0014
    )
    vent = w.body(s, "/Asset/VentButton", 0.045, C.VENT)
    w.cylinder(s, vent + "/VentStem", 0.006, 0.021, (0, 0, -0.007), METAL)
    w.cylinder(s, vent + "/VentCap", 0.018, 0.012, (0, 0, 0.005), ORANGE)
    carrier = w.body(s, "/Asset/BellowsCarrier", 0.015, C.CUP_CENTER)
    gimbal = w.body(s, "/Asset/BellowsGimbal", 0.015, C.CUP_CENTER)
    for body in (carrier, gimbal):
        w.Phy.MassAPI(s.GetPrimAtPath(body)).CreateDiagonalInertiaAttr(w.Gf.Vec3f(1e-6, 1e-6, 1e-6))
    cup = w.body(s, "/Asset/Bellows", 0.035, C.CUP_CENTER, ccd=True)
    # The annular face, carried by the compliant native axial joint, has twelve
    # pressure-sector contact pads under the visual rolled lip.
    for i in range(12):
        a = 2 * pi * i / 12
        rot = np.column_stack(([cos(a), sin(a), 0], [-sin(a), cos(a), 0], [0, 0, 1]))
        w.box(
            s,
            f"{cup}/ContactPad{i}",
            (0.0057, 0.0125, 0.0021),
            (C.LIP_R * cos(a), C.LIP_R * sin(a), 0.00105),
            LIP,
            rot,
        )
    for i, (radius, z) in enumerate(
        (
            (0.026, 0.0035),
            (0.0234, 0.0063),
            (0.0255, 0.009),
            (0.0216, 0.012),
            (0.0234, 0.0147),
            (0.0188, 0.018),
            (0.0204, 0.0208),
        )
    ):
        w.cylinder(
            s,
            f"{cup}/BellowsRib{i}",
            radius,
            0.0030,
            (0, 0, z),
            BELLOWS if i % 2 else LIP,
            collide=False,
        )
    slider(
        w, s, "SupplyTravel", base, spool, C.SPOOL, "X", (0, C.SPOOL_TRAVEL), 0, 3, 7
    )
    slider(w, s, "VentTravel", base, vent, C.VENT, "Z", (-C.VENT_TRAVEL, 0), 240, 5, 8)
    slider(
        w,
        s,
        "BellowsTravel",
        base,
        carrier,
        C.CUP_CENTER,
        "Z",
        (0, C.BELLOWS_STROKE),
        C.STIFFNESS,
        16,
        130,
    )
    bellows_tilt(w, s, "LipRoll", carrier, gimbal, "X")
    bellows_tilt(w, s, "LipPitch", gimbal, cup, "Y")
    return s


def plate(w):
    s = w.stage("test_plate")
    body = w.body(s, "/Asset/TestPlate", C.PLATE_MASS, ccd=True)
    contact = w.Px.PhysxContactReportAPI.Apply(s.GetPrimAtPath(body))
    contact.CreateThresholdAttr(0.0)
    w.rounded_box(
        s, body + "/MachinedCoupon", tuple(C.PLATE_SIZE), (0, 0, 0), METAL, 0.005, 0.001
    )
    w.rounded_box(
        s,
        body + "/HandleShank",
        (0.040, 0.025, 0.009),
        (-0.060, 0, 0.0075),
        METAL,
        0.002,
        0.001,
    )
    w.rounded_box(
        s,
        body + "/HandlingTab",
        (0.020, 0.0135, 0.024),
        tuple(C.HANDLE_LOCAL),
        BLUE,
        0.0020,
        0.001,
    )
    for i, x in enumerate((-0.029, 0.029)):
        w.box(
            s,
            f"{body}/AlignmentMark{i}",
            (0.00125, 0.013, 0.0006),
            (x, 0, 0.0036),
            RIG,
            collide=False,
        )
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [
        {"name": "bench", "type": "base", "file": w.save(bench(w))},
        {
            "name": "fixture",
            "type": "articulation",
            "file": w.save(station(w)),
            "joints": {"SupplyTravel": 0.0, "VentTravel": 0.0, "BellowsTravel": 0.0},
        },
        {
            "name": "plate",
            "type": "rigid",
            "file": w.save(plate(w)),
            "position": C.PICK,
        },
    ]
    w.inventory()
    return {"assets": assets, "settle_actions": 18}
