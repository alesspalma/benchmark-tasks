"""Original guide axis, two snap crossbars and jointed paired cable section."""

import numpy as np

from . import config as C
from .usd_scene import Writer

GRAPHITE = (0.095, 0.112, 0.128)
LINK = (0.21, 0.23, 0.25)
STEEL = (0.60, 0.66, 0.68)
BED = (0.37, 0.40, 0.41)
YELLOW = (0.91, 0.67, 0.18)
BLUE = (0.12, 0.48, 0.83)
ORANGE = (0.91, 0.34, 0.09)
GRAY = (0.36, 0.38, 0.36)


def joint(w, stage, name, parent, child, axis, limits, spring):
    p, a = parent
    c, b = child
    j = w.Phy.RevoluteJoint.Define(stage, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([p])
    j.CreateBody1Rel().SetTargets([c])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, np.subtract(b, a))))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    j.CreateAxisAttr(axis)
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    j.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    stiffness, damping, target, cap = spring
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateTargetPositionAttr(target)
    drive.CreateMaxForceAttr(cap)


def bench(w):
    s = w.stage("bench")
    w.box(s, "/Asset/Table", (0.92, 1.20, 0.044), (0.34, 0, C.TABLE_Z - 0.022), GRAY)
    w.box(s, "/Asset/Floor", (4, 4, 0.020), (0, 0, -0.010), (0.31, 0.33, 0.32))
    return s


def carrier(w):
    s = w.stage("carrier")
    origin = (0, 0, 0)
    base = w.body(s, "/Asset/Frame", 3.0, origin, fixed=True, articulation=True)
    w.rounded_box(
        s,
        base + "/AxisPlate",
        (0.272, 0.458, 0.015),
        (0.473, 0.003, C.TABLE_Z + 0.008),
        STEEL,
        0.012,
        0.0015,
    )
    for i, x in enumerate((0.493, 0.552)):
        w.box(s, base + f"/GuideBed{i}", (0.022, 0.424, 0.020), (x, 0.002, 0.775), LINK)
        w.box(
            s, base + f"/SteelGuide{i}", (0.012, 0.408, 0.006), (x, 0.002, 0.788), STEEL
        )
        for n in range(7):
            w.cylinder(
                s,
                base + f"/GuideBolt{i}_{n}",
                0.0034,
                0.0013,
                (x, 0.171 - n * 0.056, 0.791),
                GRAPHITE,
                collide=False,
            )
    w.rounded_box(
        s,
        base + "/Carriage",
        (0.101, 0.113, 0.018),
        (0.522, -0.066, 0.806),
        BED,
        0.006,
        0.0015,
    )
    w.box(
        s, base + "/MotorShell", (0.081, 0.061, 0.065), (0.528, 0.190, 0.810), GRAPHITE
    )
    w.cylinder(s, base + "/MotorBoss", 0.025, 0.007, (0.528, 0.156, 0.810), LINK, "Y")
    w.box(s, base + "/Trough", (0.112, 0.377, 0.009), (C.CHAIN_X, 0.014, 0.781), BED)
    w.box(
        s,
        base + "/CarrierBed",
        (0.086, 0.286, 0.012),
        (C.CHAIN_X, 0.014, C.CHAIN_FLOOR - 0.006),
        GRAPHITE,
    )
    for ix, x in enumerate((C.CHAIN_X - 0.042, C.CHAIN_X + 0.042)):
        for j in range(6):
            y = -0.103 + j * 0.048
            w.rounded_box(
                s,
                base + f"/Link{ix}_{j}",
                (0.012, 0.046, 0.038),
                (x, y, C.CHAIN_FLOOR + 0.0165),
                LINK,
                0.004,
                0.0007,
            )
            outer = x + (-0.0066 if ix == 0 else 0.0066)
            w.cylinder(
                s,
                base + f"/Pivot{ix}_{j}",
                0.008,
                0.0025,
                (outer, y - 0.012, C.CHAIN_FLOOR + 0.017),
                GRAPHITE,
                "X",
                collide=False,
            )
    for i, y in enumerate((0.141,)):
        w.box(
            s,
            base + f"/FixedCrossbar{i}",
            (0.088, 0.017, 0.008),
            (C.CHAIN_X, y, C.BAR_Z),
            GRAPHITE,
        )
    # The lip lies on the outside of the free ends of each flip-open bar.
    for i, y in enumerate(C.GATE_Y):
        start = (C.HINGE_X, y, C.BAR_Z)
        w.box(
            s,
            base + f"/Receiver{i}",
            (0.013, 0.028, 0.009),
            (C.CHAIN_X + 0.047, y, C.BAR_Z - 0.004),
            GRAPHITE,
        )
        w.cylinder(
            s, base + f"/PivotAxle{i}", 0.0058, 0.025, start, STEEL, "Y", collide=False
        )
        gate = w.body(s, f"/Asset/Crossbar{i}", 0.024, start, ccd=True)
        w.rounded_box(
            s,
            gate + "/Bridge",
            (0.085, 0.018, 0.009),
            (0.042, 0, 0),
            YELLOW,
            0.004,
            0.001,
        )
        w.rounded_box(
            s,
            gate + "/PressPad",
            (0.018, 0.025, 0.013),
            (0.073, 0, 0.001),
            YELLOW,
            0.004,
            0.001,
        )
        w.box(
            s,
            gate + "/Catch",
            (0.006, 0.016, 0.005),
            (0.085, 0, -0.005),
            YELLOW,
            collide=False,
        )
        joint(
            w,
            s,
            f"Latch{i}",
            (base, origin),
            (gate, start),
            "Y",
            (-90, 0.4),
            (0, 0.0020, -C.BAR_OPEN, 0.45),
        )
        w.Phy.MassAPI.Get(s, gate).CreateCenterOfMassAttr(w.Gf.Vec3f(0.041, 0, 0))
        w.Phy.MassAPI.Get(s, gate).CreateDiagonalInertiaAttr(
            w.Gf.Vec3f(0.000006, 0.000024, 0.000025)
        )
        w.Px.PhysxRigidBodyAPI.Get(s, gate).CreateDisableGravityAttr(True)
    w.box(
        s,
        base + "/TugWitness",
        (0.048, 0.003, 0.003),
        (C.CHAIN_X, -0.187, 0.860),
        STEEL,
        collide=False,
    )
    w.box(
        s,
        base + "/WitnessStem",
        (0.004, 0.012, 0.065),
        (C.CHAIN_X + 0.026, -0.185, 0.821),
        STEEL,
        collide=False,
    )
    return s


def bundle(w):
    s = w.stage("harness")
    anchor = (C.CHAIN_X, C.ANCHOR_Y, C.FLEX_Z)
    base = w.body(s, "/Asset/Anchor", 0.19, fixed=True, articulation=True)
    w.rounded_box(
        s,
        base + "/StrainRelief",
        (0.062, 0.037, 0.027),
        (C.CHAIN_X, C.ANCHOR_Y, C.FLEX_Z - 0.003),
        GRAPHITE,
        0.006,
        0.002,
    )
    for n, x in enumerate((-0.013, 0.013)):
        w.cylinder(
            s,
            base + f"/Gland{n}",
            0.008,
            0.023,
            (C.CHAIN_X + x, C.ANCHOR_Y - 0.013, C.FLEX_Z),
            GRAPHITE,
            "Y",
            collide=False,
        )
    w.Px.PhysxArticulationAPI.Get(s, base).CreateEnabledSelfCollisionsAttr(False)
    # Each link has two individual round jacket collision surfaces. The two native
    # hinges describe bending over the locally exposed part of the paired harness.
    end1 = (C.CHAIN_X, C.ANCHOR_Y - C.FIRST_SPAN, C.FLEX_Z)
    first = w.body(s, "/Asset/Proximal", 0.021, anchor, ccd=True)
    second = w.body(s, "/Asset/Distal", 0.040, end1, ccd=True)
    end2 = (C.CHAIN_X, C.ANCHOR_Y - C.FIRST_SPAN - C.SECOND_SPAN, C.FLEX_Z)
    tail = w.body(s, "/Asset/Tail", 0.018, end2, ccd=True)
    w.Phy.MassAPI.Get(s, base).CreateDiagonalInertiaAttr(
        w.Gf.Vec3f(0.001, 0.001, 0.001)
    )
    for path, length, moment in (
        (first, C.FIRST_SPAN, 0.000035),
        (second, C.SECOND_SPAN, 0.000105),
        (tail, C.TAIL_SPAN, 0.000021),
    ):
        mass = w.Phy.MassAPI.Get(s, path)
        mass.CreateCenterOfMassAttr(w.Gf.Vec3f(0, -length / 2, 0))
        mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(moment, 0.000013, moment))
    for i, (x, color) in enumerate(zip((-0.013, 0.013), (BLUE, ORANGE))):
        w.cylinder(
            s,
            first + f"/Jacket{i}",
            C.CABLE_RADIUS,
            C.FIRST_SPAN + 0.002,
            (x, -C.FIRST_SPAN / 2, 0),
            color,
            "Y",
        )
        w.cylinder(
            s,
            second + f"/Jacket{i}",
            C.CABLE_RADIUS,
            C.SECOND_SPAN + 0.002,
            (x, -C.SECOND_SPAN / 2, 0),
            color,
            "Y",
        )
        w.cylinder(
            s,
            tail + f"/Jacket{i}",
            C.CABLE_RADIUS,
            C.TAIL_SPAN + 0.002,
            (x, -C.TAIL_SPAN / 2, 0),
            color,
            "Y",
        )
        w.cylinder(
            s,
            tail + f"/ContactCoupler{i}",
            0.0064,
            0.017,
            (x, -C.TAIL_SPAN - 0.003, 0),
            GRAPHITE,
            "Y",
        )
    for body, name, y, width, length in (
        (second, "LayTie", C.SLEEVE_LOCAL_Y, 0.037, 0.016),
        (tail, "TailTie", C.TAIL_LOCAL_Y, 0.038, 0.017),
    ):
        w.rounded_box(
            s,
            body + "/" + name,
            (width, length, 0.011),
            (0, y, 0),
            GRAPHITE,
            0.004,
            0.001,
        )
    joint(
        w,
        s,
        "ProximalFlex",
        (base, (0, 0, 0)),
        (first, anchor),
        "X",
        (-48, 24),
        (0.0100, 0.0042, C.PROXIMAL_REST, 0.42),
    )
    joint(
        w,
        s,
        "ServiceFlex",
        (first, anchor),
        (second, end1),
        "X",
        (-42, 22),
        (0.0140, 0.0049, C.SERVICE_REST, 0.46),
    )
    joint(
        w,
        s,
        "TailFlex",
        (second, end1),
        (tail, end2),
        "X",
        (-45, 25),
        (0.0028, 0.0016, 2.0, 0.11),
    )
    return s


def create_assets(directory):
    w = Writer(directory)
    station = w.save(bench(w))
    rail = w.save(carrier(w))
    cables = w.save(bundle(w))
    w.inventory()
    return {
        "settle_actions": 26,
        "assets": [
            {"name": "bench", "type": "base", "file": station},
            {
                "name": "carrier",
                "type": "articulation",
                "file": rail,
                "joints": {
                    "Latch0": np.deg2rad(-C.BAR_OPEN),
                    "Latch1": np.deg2rad(-C.BAR_OPEN),
                },
            },
            {
                "name": "harness",
                "type": "articulation",
                "file": cables,
                "joints": {
                    "ProximalFlex": np.deg2rad(C.PROXIMAL_REST),
                    "ServiceFlex": np.deg2rad(C.SERVICE_REST),
                    "TailFlex": 0.0,
                },
            },
        ],
    }
