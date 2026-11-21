"""Original glazed wall, jointed mounting basket and a filled shampoo bottle."""

from itertools import pairwise
from pathlib import Path

import numpy as np

from . import config as C
from .usd_scene import Writer

TILE = (0.72, 0.83, 0.83)
JOINT = (0.44, 0.54, 0.55)
BASKET = (0.86, 0.88, 0.90)
BODY = (0.94, 0.94, 0.93)
LEVER = (0.14, 0.28, 0.31)
GASKET = (0.27, 0.39, 0.41)
BOTTLE = (0.05, 0.33, 0.64)


def path(name):
    return "/Asset/" + name


def make_wall(w):
    s = w.stage("wall")
    w.box(
        s,
        path("Counter"),
        (1.0, 1.02, 0.043),
        (0.337, 0.0, C.TABLE - 0.0215),
        (0.56, 0.58, 0.62),
    )
    w.box(s, path("Floor"), (6.0, 6.0, 0.02), (0.0, 0.0, -0.01), (0.28, 0.33, 0.35))
    for x in (-0.10, 0.78):
        for y in (-0.46, 0.46):
            w.box(
                s,
                path(f"Leg{x}_{y}"),
                (0.04, 0.04, C.TABLE - 0.043),
                (x, y, (C.TABLE - 0.043) / 2),
                (0.43, 0.48, 0.53),
            )
    w.box(
        s,
        path("GroutBed"),
        (0.025, 0.73, 0.423),
        (C.WALL_X + 0.0155, 0.0, 0.943),
        JOINT,
    )
    xs = (-0.365, *C.SEAM_Y, 0.365)
    zs = (0.732, *C.SEAM_Z, 1.156)
    for i, (y0, y1) in enumerate(pairwise(xs)):
        for j, (z0, z1) in enumerate(pairwise(zs)):
            width = y1 - y0 - C.GROUT_WIDTH
            height = z1 - z0 - C.GROUT_WIDTH
            w.box(
                s,
                path(f"Tile{i}{j}"),
                (0.010, width, height),
                (C.WALL_X + 0.005, (y0 + y1) / 2, (z0 + z1) / 2),
                TILE,
            )
    w.box(
        s,
        path("TopTrim"),
        (0.017, 0.732, 0.009),
        (C.WALL_X + 0.0025, 0.0, 1.1585),
        (0.46, 0.57, 0.58),
    )
    return s


def make_joint(w, s, cls, name, parent, child, anchor, axis, lo, hi):
    j = cls.Define(s, path(name))
    j.CreateBody0Rel().SetTargets([parent])
    j.CreateBody1Rel().SetTargets([child])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, anchor)))
    j.CreateAxisAttr(axis)
    j.CreateLowerLimitAttr(lo)
    j.CreateUpperLimitAttr(hi)
    j.CreateCollisionEnabledAttr(False)
    return j


def make_basket(w):
    s = w.stage("basket")
    p = w.body(s, path("Basket"), 0.405, articulation=True, ccd=True)
    # Draining base, upper rolled rim and open sides.
    for j, y in enumerate(np.linspace(-0.121, 0.121, 17)):
        w.box(s, p + f"/FloorSlat{j}", (0.106, 0.010, 0.007), (0.0, y, 0.0045), BASKET)
    for i, side in enumerate((-1, 1)):
        x = side * 0.053
        w.rounded_box(
            s,
            p + f"/LongRim{i}",
            (0.012, 0.274, 0.010),
            (x, 0.0, C.RIM_Z),
            BASKET,
            radius=0.004,
            bevel=0.0008,
        )
        for j, y in enumerate(np.linspace(-0.130, 0.130, 15)):
            w.box(s, p + f"/Wire{i}{j}", (0.004, 0.004, 0.048), (x, y, 0.029), BASKET)
        y = side * 0.134
        w.rounded_box(
            s,
            p + f"/ShortRim{i}",
            (0.110, 0.008, 0.010),
            (0.0, y, C.RIM_Z),
            BASKET,
            radius=0.003,
            bevel=0.0008,
        )
        for j, x in enumerate(np.linspace(-0.045, 0.045, 5)):
            w.box(
                s, p + f"/SideWire{i}{j}", (0.004, 0.004, 0.048), (x, y, 0.028), BASKET
            )
        # Rounded end sections give the same stiff purchase as the rolled rim.
        w.rounded_box(
            s,
            p + f"/RimEnd{i}",
            (0.014, 0.036, 0.014),
            (C.RIM_X, y * 0.77, C.RIM_Z),
            BASKET,
            radius=0.0055,
            bevel=0.0008,
        )
        y = side * C.CUP_Y
        w.rounded_box(
            s,
            p + f"/MountArm{i}",
            (0.026, 0.028, 0.082),
            (0.054, y, 0.054),
            BODY,
            radius=0.007,
            bevel=0.001,
        )
        w.cylinder(
            s, p + f"/CupShell{i}", 0.0265, 0.019, (0.068, y, C.CUP_Z), BODY, axis="X"
        )
        w.cylinder(
            s,
            p + f"/UpperBumper{i}",
            0.007,
            0.018,
            (0.0785, y, 0.148),
            GASKET,
            axis="X",
        )
        w.cylinder(
            s, p + f"/RimBumper{i}", 0.010, 0.012, (0.081, y, 0.017), GASKET, axis="X"
        )
        center = np.array([C.CUP_X, y, C.CUP_Z])
        cup = w.body(s, path(f"Lip{i}"), 0.016, position=center, ccd=True)
        w.cylinder(
            s,
            cup + "/SiliconeLip",
            C.CUP_RADIUS,
            2 * C.LIP_FORWARD,
            (0.0, 0.0, 0.0),
            GASKET,
            axis="X",
        )
        j = make_joint(
            w,
            s,
            w.Phy.PrismaticJoint,
            f"Compress{i}",
            p,
            cup,
            center,
            "X",
            -C.LIP_TRAVEL,
            0.0,
        )
        d = w.Phy.DriveAPI.Apply(j.GetPrim(), "linear")
        d.CreateStiffnessAttr(1130.0)
        d.CreateDampingAttr(8.0)
        d.CreateMaxForceAttr(24.0)
        d.CreateTargetPositionAttr(0.0)

        pivot = C.LEVER_PIVOT + [0.0, y, 0.0]
        w.rounded_box(
            s,
            p + f"/CamMount{i}",
            (0.026, 0.033, 0.060),
            (0.059, y, 0.123),
            BODY,
            radius=0.008,
            bevel=0.001,
        )
        lever = w.body(s, path(f"Lever{i}"), 0.024, position=pivot, ccd=True)
        w.cylinder(s, lever + "/Pin", 0.009, 0.026, (0.0, 0.0, 0.0), LEVER, axis="Y")
        w.rounded_box(
            s,
            lever + "/CamHandle",
            (0.008, 0.029, 0.044),
            (0.0, 0.0, -0.023),
            LEVER,
            radius=0.0038,
            bevel=0.0008,
        )
        w.box(
            s,
            lever + "/GripLine",
            (0.001, 0.024, 0.009),
            (-0.0045, 0.0, -0.037),
            (0.57, 0.71, 0.70),
            collide=False,
        )
        j = make_joint(
            w,
            s,
            w.Phy.RevoluteJoint,
            f"Cam{i}",
            p,
            lever,
            pivot,
            "Y",
            0.0,
            C.LEVER_ANGLE_OPEN,
        )
        drive = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(0.0012)
        drive.CreateMaxForceAttr(0.12)
        w.Px.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(0.04)
    return s


def make_bottle(w):
    s = w.stage("bottle")
    p = w.body(s, path("Bottle"), 0.218, ccd=True)
    w.rounded_box(
        s,
        p + "/Body",
        (0.038, 0.049, 0.086),
        (0.0, 0.0, 0.043),
        BOTTLE,
        radius=0.012,
        bevel=0.003,
    )
    w.rounded_box(
        s,
        p + "/Shoulder",
        (0.034, 0.042, 0.014),
        (0.0, 0.0, 0.086),
        BOTTLE,
        radius=0.006,
        bevel=0.002,
    )
    w.cylinder(s, p + "/Cap", 0.012, 0.020, (0.0, 0.0, 0.099), (0.87, 0.89, 0.91))
    w.rounded_box(
        s,
        p + "/FrontLabel",
        (0.001, 0.033, 0.045),
        (-0.0194, 0.0, 0.049),
        (0.75, 0.85, 0.90),
        radius=0.0004,
        bevel=0.0001,
        collide=False,
    )
    w.box(
        s,
        p + "/LabelStripe",
        (0.0012, 0.025, 0.006),
        (-0.0200, 0.0, 0.057),
        (0.07, 0.36, 0.59),
        collide=False,
    )
    return s


def create_assets(directory):
    w = Writer(Path(directory))
    stages = (
        ("wall", make_wall(w), "base", (0, 0, 0), {}),
        (
            "basket",
            make_basket(w),
            "articulation",
            C.BASKET_START,
            {"joints": {f"Cam{i}": np.deg2rad(C.LEVER_ANGLE_OPEN) for i in range(2)}},
        ),
        ("bottle", make_bottle(w), "rigid", C.BOTTLE_START, {}),
    )
    return {
        "assets": [
            {"name": n, "file": w.save(s), "type": typ, "position": p, **extra}
            for n, s, typ, p, extra in stages
        ],
        "settle_actions": 15,
    }
