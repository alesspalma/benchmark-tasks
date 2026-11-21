"""Native steel garden gate, lift bolt, timber fence, and articulated walker."""

from . import config as C
from . import pedestrian
from .usd_scene import Writer

GREEN = (0.132, 0.285, 0.224)
TRIM = (0.085, 0.174, 0.139)
TIMBER = (0.57, 0.44, 0.31)
TIMBER_DARK = (0.36, 0.272, 0.183)
STEEL = (0.44, 0.49, 0.51)
GRAY = (0.42, 0.43, 0.42)


def grounds(w):
    s = w.stage("grounds")
    w.box(s, "/Asset/Ground", (6.0, 6.0, 0.06), (0.45, 0, -0.03), (0.21, 0.30, 0.21))
    w.box(s, "/Asset/Walkway", (3.05, 0.88, C.FLOOR), (0.88, -0.022, C.FLOOR / 2), GRAY)
    for i in range(8):
        x = -0.60 + 0.37 * i
        if x < 2.36:
            w.box(
                s,
                f"/Asset/Slab{i}",
                (0.353, 0.841, 0.003),
                (x, -0.02, C.FLOOR + 0.0015),
                (
                    0.58 + 0.015 * (i % 3),
                    0.542 + 0.014 * (i % 3),
                    0.49 + 0.011 * (i % 3),
                ),
                collide=False,
            )
    for index, y in enumerate((-0.305, 0.305)):
        height = C.BASES[("right", "left")[index]][2] - C.FLOOR
        w.box(
            s,
            f"/Asset/Pedestal{index}",
            (0.193, 0.159, height),
            (-0.048, y, C.FLOOR + height / 2),
            (0.25, 0.27, 0.28),
        )
    for side in (-1, 1):
        for i in range(9):
            y = side * (0.586 + 0.091 * i)
            w.box(
                s,
                f"/Asset/Board{int(side > 0)}_{i}",
                (0.034, 0.082, 0.893),
                (C.HINGE[0] + 0.008, y, C.FLOOR + 0.469),
                tuple(x + 0.024 * (i % 3) for x in TIMBER),
            )
        for i, z in enumerate((0.308, 0.803)):
            y = side * 0.961
            w.box(
                s,
                f"/Asset/Stringer{int(side > 0)}_{i}",
                (0.051, 0.84, 0.045),
                (C.HINGE[0] - 0.026, y, z),
                TIMBER_DARK,
            )
    for i, y in enumerate((-0.659, 0.637)):
        w.box(
            s,
            f"/Asset/GardenBed{i}",
            (0.88, 0.35, 0.105),
            (1.54, y, 0.053),
            TIMBER_DARK,
        )
        for j in range(12):
            w.cylinder(
                s,
                f"/Asset/Plant{i}_{j}",
                0.008,
                0.093 + 0.013 * (j % 3),
                (1.15 + 0.063 * j, y + 0.027 * (j % 3), 0.145),
                (0.21, 0.41, 0.16),
                collide=False,
            )
    return s


def gate(w):
    s = w.stage("gate")
    frame = w.body(
        s, "/Asset/Frame", 16.0, position=C.HINGE, fixed=True, articulation=True
    )
    leaf = w.body(s, "/Asset/Leaf", 3.6, position=C.HINGE, ccd=True)
    bolt = w.body(s, "/Asset/Bolt", 0.122, position=C.HINGE + C.BOLT, ccd=True)
    for i, y in enumerate((0.044, -0.931)):
        w.box(
            s,
            frame + f"/Post{i}",
            (0.091, 0.088, 1.057),
            (0.0, y, C.FLOOR + 0.5285),
            TRIM,
        )
        w.rounded_box(
            s,
            frame + f"/Cap{i}",
            (0.104, 0.101, 0.030),
            (0.0, y, C.FLOOR + 1.069),
            GREEN,
            0.006,
            0.003,
        )
    for i, z in enumerate((C.GATE_BOTTOM + 0.098, C.GATE_TOP - 0.090)):
        w.cylinder(
            s,
            frame + f"/SpringBarrel{i}",
            0.021,
            0.073,
            (0.0, 0.0, z),
            TRIM,
            collide=False,
        )
        w.box(
            s,
            leaf + f"/HingeLeaf{i}",
            (0.036, 0.065, 0.032),
            (0.0, -0.026, z - 0.012),
            STEEL,
            collide=False,
        )
    for i, y in enumerate((-0.025, -C.SPAN + 0.010)):
        w.box(
            s,
            leaf + f"/Stile{i}",
            (0.041, 0.034, C.GATE_TOP - C.GATE_BOTTOM),
            (0.0, y, (C.GATE_TOP + C.GATE_BOTTOM) / 2),
            GREEN,
        )
    for i, z in enumerate((C.GATE_BOTTOM + 0.027, C.GATE_TOP - 0.020)):
        w.box(
            s,
            leaf + f"/Rail{i}",
            (0.038, C.SPAN - 0.039, 0.043),
            (0.0, -(C.SPAN + 0.007) / 2, z),
            GREEN,
        )
    for i, r in enumerate((0.123, 0.230, 0.341, 0.451, 0.559, 0.667, 0.753)):
        w.cylinder(
            s,
            leaf + f"/Picket{i}",
            0.0105,
            C.GATE_TOP - C.GATE_BOTTOM - 0.036,
            (0.0, -r, (C.GATE_TOP + C.GATE_BOTTOM) / 2),
            GREEN,
        )
    w.cylinder(
        s,
        leaf + "/PullHandle",
        0.0165,
        0.117,
        (-C.HANDLE_OFFSET, -C.HOLD_RADIUS, C.RAIL_HEIGHT),
        STEEL,
    )
    for i, z in enumerate((C.RAIL_HEIGHT - 0.061, C.RAIL_HEIGHT + 0.061)):
        w.cylinder(
            s,
            leaf + f"/PullHandleReturn{i}",
            0.013,
            C.HANDLE_OFFSET + 0.012,
            (-C.HANDLE_OFFSET / 2, -C.HOLD_RADIUS, z),
            STEEL,
            "X",
        )
    # Keeper cheeks capture the post-mounted lift bolt. Clearance is provided
    # physically by raising the bolt, with no kinematic gate lock.
    for i, x in enumerate((-0.0245, 0.0245)):
        w.box(
            s,
            leaf + f"/KeeperCheek{i}",
            (0.013, 0.050, 0.018),
            (x, C.BOLT[1] + 0.010, C.BOLT[2] + 0.007),
            STEEL,
        )
    w.box(
        s,
        leaf + "/KeeperFoot",
        (0.052, 0.022, 0.017),
        (0.0, C.BOLT[1] + 0.036, C.BOLT[2] + 0.007),
        STEEL,
    )
    w.rounded_box(
        s,
        frame + "/LatchHousing",
        (0.057, 0.043, 0.157),
        (-0.021, -0.874, 0.987),
        TRIM,
        0.005,
        0.002,
        collide=False,
    )
    w.cylinder(s, bolt + "/Pin", 0.009, 0.034, (0.0, 0.0, 0.0), STEEL)
    w.cylinder(
        s, bolt + "/Stem", 0.006, 0.085, (-0.005, 0.0, 0.045), STEEL, collide=False
    )
    w.box(
        s,
        bolt + "/PullStub",
        (0.057, 0.014, 0.013),
        (-0.027, -0.004, 0.080),
        STEEL,
        collide=False,
    )
    w.cylinder(
        s,
        bolt + "/PullKnob",
        0.015,
        0.026,
        (-0.061, -0.004, 0.080),
        (0.70, 0.56, 0.23),
        "X",
    )

    hinge = w.Phy.RevoluteJoint.Define(s, "/Asset/Swing")
    hinge.CreateBody0Rel().SetTargets([frame])
    hinge.CreateBody1Rel().SetTargets([leaf])
    hinge.CreateAxisAttr("Z")
    hinge.CreateLowerLimitAttr(0.0)
    hinge.CreateUpperLimitAttr(C.OPEN_STOP)
    slide = w.Phy.PrismaticJoint.Define(s, "/Asset/LiftBolt")
    slide.CreateBody0Rel().SetTargets([frame])
    slide.CreateBody1Rel().SetTargets([bolt])
    slide.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, C.BOLT)))
    slide.CreateAxisAttr("Z")
    slide.CreateLowerLimitAttr(0.0)
    slide.CreateUpperLimitAttr(C.LIFT)
    return s


def create_assets(directory):
    writer = Writer(directory)
    assets = [
        {"name": "grounds", "type": "base", "file": writer.save(grounds(writer))},
        {
            "name": "walker",
            "type": "base",
            "file": writer.save(pedestrian.build(writer)),
        },
        {"name": "gate", "type": "articulation", "file": writer.save(gate(writer))},
    ]
    writer.inventory()
    return {"assets": assets, "settle_actions": 12}
