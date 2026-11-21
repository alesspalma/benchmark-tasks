"""Native grooved cheese board, pivoting handle, and two finite-cohesion pieces."""

from math import cos, pi, sin

from . import config as C
from .usd_scene import Writer

MAPLE = (0.64, 0.41, 0.23)
END_GRAIN = (0.51, 0.31, 0.17)
STEEL = (0.60, 0.64, 0.68)
WIRE = (0.32, 0.39, 0.44)
GRIP = (0.08, 0.11, 0.14)
CHEDDAR = (0.94, 0.62, 0.22)
PLATE = (0.34, 0.69, 0.72)
MARK = (0.25, 0.15, 0.08)


def slicer(w):
    stage = w.stage("wire_board")
    board = w.body(stage, "/Asset/Board", 0.57, fixed=True, articulation=True)
    geometry = w.Geom.Xform.Define(stage, board + "/Geometry")
    geometry.AddTranslateOp().Set(w.Gf.Vec3d(*map(float, C.PIVOT)))
    board_geometry = board + "/Geometry"
    length = 0.241
    width = 0.151
    gap = 0.0043
    section = (length - gap) / 2
    for side, name in ((-1, "StockSide"), (1, "ServingSide")):
        x = side * (gap + section) / 2
        w.rounded_box(
            stage,
            f"{board_geometry}/{name}",
            (section, width, 0.022),
            (x, -0.076, -0.011),
            MAPLE,
            0.007,
            0.0014,
        )
        for n in range(7):
            y = -0.142 + n * 0.020
            w.box(
                stage,
                f"{board_geometry}/{name}Grain{n}",
                (section - 0.019, 0.00065, 0.00015),
                (x, y, 0.00006),
                END_GRAIN if n % 3 == 0 else (0.70, 0.47, 0.30),
                collide=False,
            )
    # These familiar engraved gauge ticks give a manual thickness reference.
    for i, x in enumerate((0.008, 0.016, 0.024)):
        for j, y in enumerate((-0.120, -0.032)):
            w.box(
                stage,
                f"{board_geometry}/Gauge{i}_{j}",
                (0.0008, 0.014, 0.00028),
                (x, y, 0.00013),
                MARK,
                collide=False,
            )
    for i, x in enumerate((-0.09, 0.09)):
        for j, y in enumerate((-0.127, -0.025)):
            w.cylinder(
                stage,
                f"{board_geometry}/RubberFoot{i}_{j}",
                0.009,
                0.002,
                (x, y, -0.023),
                GRIP,
            )
    w.cylinder(
        stage,
        board_geometry + "/BearingHousing",
        0.010,
        0.026,
        (0, 0.003, -0.004),
        STEEL,
        "X",
    )

    arm = w.body(stage, "/Asset/Arm", 0.074, position=C.PIVOT, ccd=True)
    # The arch keeps the lever's return bar outside the stock path.
    w.cylinder(stage, arm + "/PivotBar", 0.004, 0.085, (0.041, 0.0, 0.005), STEEL, "X")
    w.cylinder(
        stage,
        arm + "/LongBar",
        0.004,
        C.WIRE_LENGTH + 0.008,
        (0.081, -0.5 * C.WIRE_LENGTH, 0.005),
        STEEL,
        "Y",
    )
    w.cylinder(
        stage,
        arm + "/OuterReturn",
        0.004,
        0.084,
        (0.041, -C.WIRE_LENGTH, 0.005),
        STEEL,
        "X",
    )
    w.cylinder(stage, arm + "/GripStem", 0.005, 0.035, (0, -0.193, 0.005), STEEL, "Y")
    w.rounded_box(
        stage, arm + "/Handle", (0.028, 0.043, 0.023), C.HANDLE, GRIP, 0.010, 0.0025
    )
    w.cylinder(
        stage,
        arm + "/StainlessWire",
        0.00052,
        C.WIRE_LENGTH,
        (0, -0.5 * C.WIRE_LENGTH, C.WIRE_Z),
        WIRE,
        "Y",
        collide=False,
    )
    for i, y in enumerate((0.000, -C.WIRE_LENGTH)):
        w.cylinder(
            stage,
            f"{arm}/WireAnchor{i}",
            0.003,
            0.016,
            (0, y, 0.005),
            STEEL,
            collide=False,
        )
    joint = w.Phy.RevoluteJoint.Define(stage, "/Asset/WirePivot")
    joint.CreateBody0Rel().SetTargets([board])
    joint.CreateBody1Rel().SetTargets([arm])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, C.PIVOT)))
    joint.CreateAxisAttr("X")
    joint.CreateLowerLimitAttr(-52.0)
    joint.CreateUpperLimitAttr(0.22)
    return stage


def cheese(w, name, dimensions, mass):
    stage = w.stage(name)
    body = w.body(stage, "/Asset/Cheese", mass, ccd=True)
    # Use flat mating cut surfaces and small exterior bevels.
    w.box(stage, body + "/ContactCore", dimensions, (0, 0, 0), CHEDDAR)
    for i in range(12):
        x = dimensions[0] * (0.5 - (i % 5) * 0.19) - 0.0007
        y = dimensions[1] * ((i * 7 % 13) / 15 - 0.4)
        w.cylinder(
            stage,
            f"{body}/SurfaceSpeck{i}",
            0.0009 + 0.0002 * (i % 3),
            0.00010,
            (x, y, dimensions[2] / 2 + 0.00002),
            (0.81, 0.49, 0.15),
            collide=False,
        )
    return stage


def side_plate(w):
    stage = w.stage("side_plate")
    w.cylinder(stage, "/Asset/Floor", 0.063, 0.005, (0, 0, -0.0025), PLATE)
    for k in range(32):
        angle = k * 2 * pi / 32
        w.rounded_box(
            stage,
            f"/Asset/Rim{k}",
            (0.010, 0.017, 0.008),
            (0.064 * cos(angle), 0.064 * sin(angle), 0.002),
            PLATE,
            0.003,
            0.001,
            collide=True,
        )
    return stage


def worktop(w):
    stage = w.stage("kitchen_bench")
    w.box(
        stage,
        "/Asset/Counter",
        (1.22, 1.12, 0.043),
        (0.45, 0, C.TABLE - 0.0215),
        (0.68, 0.66, 0.61),
    )
    w.box(stage, "/Asset/Floor", (4, 4, 0.02), (0, 0, -0.01), (0.55, 0.59, 0.60))
    return stage


def create_assets(directory):
    w = Writer(directory)
    assets = [
        {"name": "bench", "type": "base", "file": w.save(worktop(w))},
        {
            "name": "slicer",
            "type": "articulation",
            "file": w.save(slicer(w)),
            "joints": {"WirePivot": float(C.OPEN)},
        },
        {
            "name": "stock",
            "type": "rigid",
            "file": w.save(cheese(w, "stock", C.STOCK_SIZE, 0.116)),
            "position": C.STOCK_RESET,
        },
        {
            "name": "slice",
            "type": "rigid",
            "file": w.save(cheese(w, "slice", C.SLICE_SIZE, 0.0145)),
            "position": C.SLICE_RESET,
        },
        {
            "name": "plate",
            "type": "base",
            "file": w.save(side_plate(w)),
            "position": C.PLATE,
        },
    ]
    w.inventory()
    return {"assets": assets, "settle_actions": 12}
