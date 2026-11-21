"""Build a dimensioned FTIR instrument and its native pressure controls."""

from math import cos, pi, sin

import numpy as np

from . import config as C
from .usd_scene import Writer

WHITE = (0.89, 0.91, 0.91)
DARK = (0.11, 0.16, 0.18)
BLACK = (0.045, 0.068, 0.076)
GREY = (0.32, 0.39, 0.42)
STEEL = (0.57, 0.65, 0.68)
TEAL = (0.08, 0.38, 0.45)
PE = (0.33, 0.76, 0.82)
YELLOW = (0.99, 0.65, 0.12)
GREEN = (0.23, 0.76, 0.29)


def joint(w, s, name, parent, child, origin, limits, rotational):
    kind = w.Phy.RevoluteJoint if rotational else w.Phy.PrismaticJoint
    j = kind.Define(s, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([parent])
    j.CreateBody1Rel().SetTargets([child])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, origin)))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    j.CreateAxisAttr("Z")
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    j.CreateCollisionEnabledAttr(not rotational)
    drive = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular" if rotational else "linear")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(0.0)
    if rotational:
        # USD angular drive coefficients use degrees. This low limit represents
        # the load-dependent static thread holding force in the solver.
        drive.CreateStiffnessAttr(0.10)
        drive.CreateDampingAttr(0.00005)
        drive.CreateMaxForceAttr(0.31)
    else:
        drive.CreateStiffnessAttr(C.CUP_STIFFNESS * C.NATIVE_LOAD_SCALE)
        drive.CreateDampingAttr(38.0)
        drive.CreateMaxForceAttr(C.NOMINAL_LOAD * C.NATIVE_LOAD_SCALE)


def bench(w):
    s = w.stage("bench")
    w.box(
        s,
        "/Asset/Worktop",
        (1.18, 1.16, 0.044),
        (0.44, 0, C.TABLE_Z - 0.022),
        (0.62, 0.62, 0.57),
    )
    w.box(s, "/Asset/Floor", (4, 4, 0.02), (0, 0, -0.01), (0.31, 0.38, 0.38))
    tray = C.PICK + [0.0, -0.012, -0.010 - 0.5 * C.COUPON_SIZE[2]]
    w.rounded_box(
        s, "/Asset/CleanSampleTray", (0.031, 0.067, 0.020), tray, TEAL, 0.007, 0.001
    )
    w.box(
        s,
        "/Asset/SampleTraySupport",
        (0.043, 0.064, tray[2] - 0.010 - C.TABLE_Z),
        (tray[0], tray[1], 0.5 * (tray[2] - 0.010 + C.TABLE_Z)),
        DARK,
    )
    return s


def instrument(w):
    s = w.stage("instrument")
    base = w.body(s, "/Asset/Chassis", 15.0, fixed=True, articulation=True)
    cx, cy, cz = C.CRYSTAL
    # The working bay slopes down from the rear optical compartment.
    w.rounded_box(
        s,
        base + "/DieCastShell",
        (0.346, 0.324, 0.074),
        (0.534, 0, C.TABLE_Z + 0.039),
        WHITE,
        0.018,
        0.004,
    )
    w.rounded_box(
        s,
        base + "/RearOpticalCompartment",
        (0.215, 0.320, 0.115),
        (0.597, 0, 0.868),
        WHITE,
        0.018,
        0.006,
    )
    w.rounded_box(
        s,
        base + "/TopCover",
        (0.208, 0.309, 0.024),
        (0.600, 0, 0.934),
        GREY,
        0.016,
        0.004,
    )
    w.rounded_box(
        s,
        base + "/SamplingBay",
        (0.169, 0.240, 0.010),
        (0.426, 0, 0.825),
        DARK,
        0.013,
        0.001,
    )
    w.cylinder(
        s,
        base + "/CrystalSteelPlate",
        C.PLATE_RADIUS,
        0.017,
        (cx, cy, cz - 0.0085),
        STEEL,
    )
    w.cylinder(
        s,
        base + "/DiamondWindow",
        C.DIAMOND_R,
        0.00016,
        (cx, cy, cz + 0.00008),
        (0.96, 0.93, 0.72),
        collide=False,
    )
    for sign in (-1, 1):
        for a, b in ((0.034, 0), (0, 0.034)):
            w.box(
                s,
                base + f"/CentreWitness{sign}_{int(a > 0)}",
                (0.002, 0.002, 0.0002),
                (cx + sign * a, cy + sign * b, 0.8301),
                YELLOW,
                collide=False,
            )
    for i in range(8):
        w.box(
            s,
            base + f"/Vent{i}",
            (0.004, 0.080, 0.0005),
            (0.592 + i * 0.011, -0.089, 0.9463),
            DARK,
            collide=False,
        )
    for y in (-0.136, 0.136):
        for x in (0.386, 0.669):
            w.cylinder(
                s,
                base + f"/Foot{int(x > 0.4)}{int(y > 0)}",
                0.013,
                0.003,
                (x, y, C.TABLE_Z + 0.0015),
                BLACK,
            )
    tower_x = cx + 0.080
    w.cylinder(
        s, base + "/PressureTowerPost", 0.012, 0.114, (tower_x, cy, 0.891), STEEL
    )
    w.cylinder(
        s, base + "/PressureTowerBoss", 0.023, 0.016, (tower_x, cy, 0.837), BLACK
    )
    w.rounded_box(
        s,
        base + "/PressureTowerArm",
        (0.114, 0.033, 0.019),
        ((cx + tower_x) / 2, cy, 0.948),
        DARK,
        0.010,
        0.003,
    )
    w.cylinder(s, base + "/DriveBearing", 0.016, 0.017, (cx, cy, 0.962), GREY)
    w.cylinder(
        s, base + "/AnvilGuide", 0.010, 0.018, (cx, cy, 0.927), STEEL, collide=False
    )
    # The pressure witness is a calibrated visual aid on the simulated front panel.
    w.cylinder(
        s, base + "/DialBezel", 0.018, 0.006, C.DIAL, BLACK, axis="X", collide=False
    )
    w.cylinder(
        s,
        base + "/DialFace",
        0.015,
        0.001,
        (C.DIAL[0] - 0.0034, *C.DIAL[1:]),
        WHITE,
        axis="X",
        collide=False,
    )
    for i in range(7):
        angle = -1.16 + i * 0.39
        tick = np.asarray(C.DIAL) + [-0.0041, 0.0124 * sin(angle), 0.0124 * cos(angle)]
        w.cylinder(
            s,
            base + f"/LoadTick{i}",
            0.0011,
            0.001,
            tick,
            GREEN if i >= 5 else BLACK,
            axis="X",
            collide=False,
        )
    w.cylinder(
        s,
        base + "/NeedleHub",
        0.002,
        0.001,
        (C.DIAL[0] - 0.0044, *C.DIAL[1:]),
        BLACK,
        axis="X",
        collide=False,
    )
    needle = w.Geom.Xform.Define(s, base + "/PressureNeedle")
    needle.AddTranslateOp().Set(w.Gf.Vec3d(C.DIAL[0] - 0.0049, *C.DIAL[1:]))
    needle.AddRotateXOp().Set(66.5)
    w.box(
        s,
        base + "/PressureNeedle/Needle",
        (0.001, 0.0013, 0.010),
        (0, 0, 0.005),
        (0.91, 0.13, 0.08),
        collide=False,
    )

    knob = w.body(s, "/Asset/Knob", 0.072, position=C.KNOB)
    w.cylinder(s, knob + "/KnurledDrum", C.KNOB_R, 0.024, (0, 0, 0), BLACK)
    w.cylinder(s, knob + "/Crown", 0.025, 0.002, (0, 0, 0.013), GREY, collide=False)
    for i in range(28):
        theta = 2 * pi * i / 28
        w.cylinder(
            s,
            knob + f"/Knurl{i}",
            0.0011,
            0.021,
            (C.KNOB_R * cos(theta), C.KNOB_R * sin(theta), 0),
            GREY,
            collide=False,
        )
    w.box(
        s,
        knob + "/IndexLine",
        (0.011, 0.0022, 0.0007),
        (0.016, 0.0, 0.0142),
        YELLOW,
        collide=False,
    )
    joint(w, s, "PressureKnob", base, knob, C.KNOB, (-155.0, 4.0), True)

    tip_origin = np.array([cx, cy, C.TIP_BOTTOM + 0.007])
    tip = w.body(s, "/Asset/Anvil", 0.28, position=tip_origin, ccd=True)
    w.cylinder(s, tip + "/Plunger", 0.0055, 0.066, (0, 0, 0.041), STEEL)
    w.cylinder(s, tip + "/FlatSelfLevellingTip", 0.0075, 0.011, (0, 0, -0.0015), STEEL)
    w.cylinder(s, tip + "/TipGrip", 0.0084, 0.005, (0, 0, 0.004), GREY, collide=False)
    joint(w, s, "PressureAnvil", base, tip, tip_origin, (-0.005, 0.004), False)
    return s


def coupon(w):
    s = w.stage("coupon")
    p = w.body(s, "/Asset/Card", 0.0107, ccd=True)
    attrs = w.Px.PhysxRigidBodyAPI.Get(s, p)
    attrs.CreateSolverPositionIterationCountAttr(32)
    attrs.CreateSolverVelocityIterationCountAttr(4)
    attrs.CreateMaxDepenetrationVelocityAttr(0.2)
    attrs.CreateAngularDampingAttr(1.2)
    w.rounded_box(
        s, p + "/PolyethyleneCoupon", C.COUPON_SIZE, (0, 0, 0), PE, 0.0032, 0.00018
    )
    # Small visual arrows mark specimen orientation and remain outside the contact zone.
    w.box(
        s,
        p + "/WitnessLine",
        (0.021, 0.001, 0.0001),
        (0, 0.027, 0.00130),
        TEAL,
        collide=False,
    )
    w.box(
        s,
        p + "/WitnessLine2",
        (0.013, 0.001, 0.0001),
        (0, 0.032, 0.00130),
        TEAL,
        collide=False,
    )
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [
        {"name": "bench", "type": "base", "file": w.save(bench(w))},
        {
            "name": "instrument",
            "type": "articulation",
            "file": w.save(instrument(w)),
            "joints": {"PressureKnob": 0.0, "PressureAnvil": 0.0},
        },
        {
            "name": "coupon",
            "type": "rigid",
            "file": w.save(coupon(w)),
            "position": C.PICK,
        },
    ]
    w.inventory()
    return {"assets": assets, "settle_actions": 7}
