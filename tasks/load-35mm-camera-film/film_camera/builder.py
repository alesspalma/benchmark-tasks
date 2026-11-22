"""Author the camera, cassette, rear door, sprocket and winding lever in SI units."""

from math import cos, pi, sin

from . import config as C
from .film_web import define as film_web
from .usd_scene import Writer

NIGHT = (0.065, 0.075, 0.083)
PAINT = (0.19, 0.22, 0.23)
SILVER = (0.52, 0.55, 0.54)
GATE = (0.31, 0.34, 0.34)
BLACK = (0.12, 0.13, 0.12)
FILM = (0.31, 0.14, 0.065)
ORANGE = (0.98, 0.40, 0.075)
LABEL = (0.89, 0.72, 0.27)


def bearing(w, s, name, parent, child, offset, axis, limits, *, linear=False):
    kind = w.Phy.PrismaticJoint if linear else w.Phy.RevoluteJoint
    joint = kind.Define(s, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*offset))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(limits[0])
    joint.CreateUpperLimitAttr(limits[1])
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear" if linear else "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.0)
    drive.CreateDampingAttr(0.10 if linear else 0.00007)


def bench(w):
    s = w.stage("bench")
    w.box(
        s,
        "/Asset/Worktop",
        (0.92, 1.20, 0.044),
        (0.34, 0, C.TABLE - 0.022),
        (0.46, 0.48, 0.43),
    )
    w.box(s, "/Asset/Floor", (4.0, 4.0, 0.020), (0, 0, -0.010), (0.34, 0.37, 0.36))
    w.rounded_box(
        s,
        "/Asset/CameraMat",
        (0.204, 0.237, 0.004),
        (0.397, -0.018, C.TABLE + 0.002),
        (0.29, 0.39, 0.36),
        0.013,
        0.001,
    )
    return s


def camera(w):
    s = w.stage("camera")
    b = w.body(s, "/Asset/CameraShell", 0.70, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(s, b).CreateEnabledSelfCollisionsAttr(False)
    w.rounded_box(
        s,
        b + "/NonskidCradle",
        (0.102, 0.154, 0.008),
        (C.X, -0.014, 0.759),
        (0.15, 0.23, 0.21),
        0.008,
        0.001,
    )
    w.rounded_box(
        s,
        b + "/Casting",
        (0.085, 0.139, 0.036),
        (C.X, -0.014, 0.782),
        NIGHT,
        0.008,
        0.001,
    )
    w.rounded_box(
        s,
        b + "/LowerTrim",
        (0.087, 0.140, 0.006),
        (C.X, -0.014, 0.772),
        SILVER,
        0.008,
        0.001,
    )
    w.rounded_box(
        s,
        b + "/RearGate",
        (0.085, 0.076, 0.016),
        (C.X, -0.031, C.DECK - 0.008),
        PAINT,
        0.004,
        0.001,
    )
    w.rounded_box(
        s,
        b + "/CartridgeWell",
        (0.047, 0.028, 0.005),
        (C.X, C.SEAT_Y, C.SEAT_Z - 0.015),
        BLACK,
        0.003,
        0.0007,
    )
    w.box(
        s,
        b + "/CassetteGuideLeft",
        (0.048, 0.002, 0.010),
        (C.X, C.SEAT_Y + 0.014, C.SEAT_Z - 0.008),
        GATE,
    )
    w.box(
        s,
        b + "/CassetteGuideRight",
        (0.048, 0.002, 0.009),
        (C.X, C.SEAT_Y - 0.014, C.SEAT_Z - 0.0085),
        GATE,
    )
    for edge in (-1, 1):
        for side in (-1, 1):
            w.box(
                s,
                b + f"/CassetteBearing{int(edge > 0)}{int(side > 0)}",
                (0.005, 0.002, 0.010),
                (C.X + edge * 0.021, C.SEAT_Y + side * 0.0138, C.SEAT_Z + 0.003),
                GATE,
            )
        w.box(
            s,
            b + f"/EndRetainer{int(edge > 0)}",
            (0.006, 0.014, 0.002),
            (C.X + edge * 0.021, C.SEAT_Y + 0.002, C.SEAT_Z + 0.0172),
            GATE,
        )
        w.box(
            s,
            b + f"/WellStop{int(edge > 0)}",
            (0.004, 0.024, 0.016),
            (C.X + edge * 0.029, C.SEAT_Y, C.SEAT_Z + 0.001),
            GATE,
        )
        w.box(
            s,
            b + f"/FilmRail{int(edge > 0)}",
            (0.002, 0.078, 0.0012),
            (C.X + edge * 0.0175, -0.025, C.DECK + 0.0006),
            SILVER,
        )
        w.box(
            s,
            b + f"/LeaderIndex{int(edge > 0)}",
            (0.008, 0.0028, 0.00035),
            (
                C.X + edge * 0.021,
                C.SEAT_Y + C.LEADER_DY - C.PULL - 0.017,
                C.DECK + 0.0014,
            ),
            ORANGE,
            collide=False,
        )
    # The two registration crowns flank the film's imaging surface.
    for x in (-0.015, 0.015):
        pin = int(x > 0)
        center = (C.X + x, -0.055, C.DECK + 0.0004)
        w.cylinder(
            s,
            b + f"/SprocketCrown{pin}",
            0.0044,
            0.0022,
            center,
            SILVER,
            "X",
            collide=False,
        )
        for tooth in range(8):
            a = tooth * pi / 4
            w.box(
                s,
                b + f"/SprocketTooth{pin}_{tooth}",
                (0.0022, 0.001, 0.001),
                (center[0], center[1] + 0.0044 * cos(a), center[2] + 0.0044 * sin(a)),
                SILVER,
                collide=False,
            )
    w.box(
        s,
        b + "/ExposureAperture",
        (0.024, 0.032, 0.0005),
        (C.X, -0.023, C.DECK + 0.0003),
        NIGHT,
        collide=False,
    )
    w.rounded_box(
        s,
        b + "/Viewfinder",
        (0.006, 0.023, 0.014),
        (C.X - 0.046, 0.022, 0.803),
        GATE,
        0.002,
        0.0007,
    )
    w.box(
        s,
        b + "/FinderGlass",
        (0.001, 0.015, 0.008),
        (C.X - 0.0494, 0.022, 0.805),
        (0.15, 0.26, 0.28),
        collide=False,
    )
    w.cylinder(
        s, b + "/FilmSpeedDial", 0.008, 0.004, (C.X - 0.032, 0.062, 0.812), SILVER
    )
    for k in range(10):
        a = k * pi / 5
        w.cylinder(
            s,
            b + f"/DialMark{k}",
            0.00042,
            0.00045,
            (C.X - 0.030 + 0.0065 * cos(a), 0.062 + 0.0065 * sin(a), 0.814),
            NIGHT,
            collide=False,
        )
    w.cylinder(
        s, b + "/LensRing", 0.024, 0.008, (C.X, -0.018, 0.763), SILVER, collide=False
    )
    w.rounded_box(
        s,
        b + "/SpoolControlHousing",
        (0.040, 0.043, 0.023),
        (0.391, 0.069, 0.798),
        NIGHT,
        0.005,
        0.001,
        collide=False,
    )
    w.rounded_box(
        s,
        b + "/AdvanceHousing",
        (0.032, 0.034, 0.023),
        (0.363, -0.087, 0.796),
        NIGHT,
        0.005,
        0.001,
        collide=False,
    )
    door = w.body(s, "/Asset/BackCover", 0.045, C.DOOR, ccd=True)
    w.rounded_box(
        s,
        door + "/PressurePlate",
        (0.090, 0.142, 0.004),
        (0, -0.072, 0.012),
        NIGHT,
        0.003,
        0.0006,
    )
    for x in (-0.031, 0.031):
        w.box(
            s,
            door + f"/EdgePad{int(x > 0)}",
            (0.008, 0.095, 0.004),
            (x, -0.087, 0.007),
            BLACK,
        )
    w.rounded_box(
        s,
        door + "/InsetLeather",
        (0.078, 0.126, 0.0007),
        (0, -0.074, 0.0143),
        PAINT,
        0.002,
        0.0003,
        collide=False,
    )
    # Outer light seal outside the swept film and lever envelope.
    w.rounded_box(
        s,
        door + "/TailSkin",
        (0.089, 0.013, 0.004),
        (0, -0.145, 0.012),
        NIGHT,
        0.002,
        0.0005,
        collide=False,
    )
    w.rounded_box(
        s,
        door + "/LightSealSkirt",
        (0.085, 0.003, 0.017),
        (0, -0.150, 0.004),
        NIGHT,
        0.001,
        0.0004,
        collide=False,
    )
    w.rounded_box(
        s,
        door + "/MoldedLip",
        (0.019, 0.009, 0.008),
        (-0.025, -C.DOOR_REACH, 0.002),
        GATE,
        0.002,
        0.0007,
    )
    for x in (-0.042, 0.042):
        w.cylinder(
            s,
            b + f"/Hinge{int(x > 0)}",
            0.0034,
            0.009,
            (C.DOOR[0] + x, C.DOOR[1], C.DOOR[2]),
            SILVER,
            "X",
            collide=False,
        )
    bearing(w, s, "BackHinge", b, door, C.DOOR, "X", (-101, 0))
    lever = w.body(s, "/Asset/AdvanceLever", 0.018, C.LEVER, ccd=True)
    w.cylinder(s, lever + "/Pivot", 0.008, 0.008, (0, 0, 0), SILVER, collide=False)
    w.rounded_box(
        s,
        lever + "/AdvanceArm",
        (0.010, 0.033, 0.005),
        (0, -0.017, 0.003),
        GATE,
        0.003,
        0.001,
    )
    w.rounded_box(
        s,
        lever + "/FingerGrip",
        (0.017, 0.014, 0.012),
        (0, -C.LEVER_REACH, 0.008),
        SILVER,
        0.003,
        0.001,
    )
    bearing(w, s, "AdvancePivot", b, lever, C.LEVER, "Z", (0, 67))
    rewind_center = (0.386, 0.080, 0.826)
    rewind = w.body(s, "/Asset/RewindKnob", 0.010, rewind_center)
    w.cylinder(s, rewind + "/Wheel", 0.009, 0.006, (0, 0, 0), SILVER)
    w.rounded_box(
        s,
        rewind + "/Crank",
        (0.012, 0.0025, 0.001),
        (0.0018, 0, 0.0033),
        ORANGE,
        0.001,
        0.0003,
        collide=False,
    )
    bearing(w, s, "RewindPivot", b, rewind, rewind_center, "Z", (-4, 160))
    return s


def cartridge(w):
    s = w.stage("film_cartridge")
    b = w.body(s, "/Asset/Cassette", 0.028, articulation=True, ccd=True)
    w.cylinder(s, b + "/FilmCan", 0.0125, 0.042, (0, 0, 0), LABEL, "X")
    for side in (-1, 1):
        w.rounded_box(
            s,
            b + f"/EndPinch{int(side > 0)}",
            (0.0019, 0.014, 0.012),
            (side * 0.025, 0.0, 0.004),
            NIGHT,
            0.002,
            0.0005,
        )
        w.cylinder(
            s,
            b + f"/SteelCap{int(side > 0)}",
            0.0121,
            0.0025,
            (side * 0.021, 0, 0),
            NIGHT,
            "X",
        )
        w.cylinder(
            s,
            b + f"/Spindle{int(side > 0)}",
            0.005,
            0.0038,
            (side * 0.024, 0, 0),
            SILVER,
            "X",
        )
    w.box(s, b + "/CartridgeFlat", (0.037, 0.015, 0.003), (0, 0, -0.0110), NIGHT)
    w.box(
        s,
        b + "/FeltSlot",
        (0.036, 0.004, 0.004),
        (0, -0.010, C.FILM_Z - C.SEAT_Z - 0.001),
        NIGHT,
    )
    film_web(w, s, b, FILM)
    leader_root = (0, C.LEADER_DY, C.FILM_Z - C.SEAT_Z)
    payout = w.body(s, "/Asset/PayoutCarriage", 0.004, leader_root)
    lateral = w.body(s, "/Asset/LeaderCarriage", 0.004, leader_root)
    for body in (payout, lateral):
        w.Phy.MassAPI(s.GetPrimAtPath(body)).CreateDiagonalInertiaAttr(
            w.Gf.Vec3f(1e-7, 1e-7, 1e-7)
        )
    tip = w.body(s, "/Asset/FilmLeader", 0.006, leader_root, ccd=True)
    w.rounded_box(
        s, tip + "/Leader", (0.024, 0.033, 0.0018), (0, -0.001, 0), FILM, 0.002, 0.00025
    )
    w.rounded_box(
        s,
        tip + "/LeaderCurl",
        (0.025, 0.011, 0.009),
        (0, -0.012, 0.0045),
        FILM,
        0.001,
        0.00035,
    )
    for side in (-1, 1):
        for j in range(5):
            w.box(
                s,
                tip + f"/Perforation{int(side > 0)}_{j}",
                (0.0022, 0.00145, 0.0001),
                (side * 0.0104, -0.006 + 0.0038 * j, 0.00093),
                LABEL,
                collide=False,
            )
    bearing(
        w,
        s,
        "FilmPayout",
        b,
        payout,
        leader_root,
        "Y",
        (-C.PULL - C.ADVANCE - 0.008, 0.001),
        linear=True,
    )
    bearing(
        w, s, "LeaderCross", payout, lateral, (0, 0, 0), "X",
        (-0.011, 0.011), linear=True,
    )
    bearing(w, s, "LeaderSkew", lateral, tip, (0, 0, 0), "Z", (-22, 22))
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [
        {"name": "bench", "type": "base", "file": w.save(bench(w))},
        {
            "name": "camera",
            "type": "articulation",
            "file": w.save(camera(w)),
            "joints": {
                "BackHinge": -C.DOOR_OPEN * pi / 180,
                "AdvancePivot": 0.0,
                "RewindPivot": 0.0,
            },
        },
        {
            "name": "film_cartridge",
            "type": "articulation",
            "file": w.save(cartridge(w)),
            "position": (C.X, C.SEAT_Y, C.SEAT_Z + 0.001),
        },
    ]
    return {"assets": assets, "settle_actions": 12}
