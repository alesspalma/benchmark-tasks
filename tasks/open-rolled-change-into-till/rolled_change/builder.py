"""Author the drawer, paper sleeve halves, and forty nickel collision bodies."""

from math import cos, pi, sin, tan

import numpy as np

from . import config as C
from .usd_scene import Writer

KRAFT = (0.73, 0.49, 0.29)
BLUE = (0.095, 0.38, 0.65)
TILL = (0.10, 0.13, 0.17)
BINS = (0.18, 0.22, 0.26)
STEEL = (0.58, 0.62, 0.65)
NICKEL = (0.70, 0.72, 0.76)
SCRAP = (0.25, 0.40, 0.37)
WHITE = (0.90, 0.94, 0.97)


def wordmark(w, s, label, pattern, origin, pitch):
    for row, line in enumerate(pattern):
        for column, dot in enumerate(line):
            if dot == "1":
                x, y, z = origin
                w.box(
                    s,
                    f"{label}/Cell{row}_{column}",
                    (pitch, pitch, 0.00015),
                    (x + row * pitch, y + column * pitch, z),
                    WHITE,
                    collide=False,
                )


def furniture(w):
    s = w.stage("till")
    z = C.TABLE
    w.rounded_box(
        s,
        "/Asset/CashDrawer",
        (0.340, 0.35, 0.012),
        (0.5465, 0, z + 0.006),
        TILL,
        0.009,
        0.001,
    )
    for i, y in enumerate((-0.1, 0, 0.1)):
        w.box(
            s,
            f"/Asset/CoinFloor{i}",
            (0.162, 0.091, 0.006),
            (0.5395, y, C.BIN_FLOOR - 0.003),
            BINS,
            collide=False,
        )
    # The actual support surface is exactly at the specified bottom plane.
    w.box(
        s,
        "/Asset/RecessBottom",
        (0.251, 0.31, 0.012),
        (0.503, 0, C.BIN_FLOOR - 0.004),
        BINS,
    )
    for i, y in enumerate((-0.153, -0.078, 0.078, 0.153)):
        w.box(
            s,
            f"/Asset/CoinWall{i}",
            (0.247, 0.008, 0.070),
            (0.5005, y, C.BIN_FLOOR + 0.035),
            TILL,
        )
    for label, x in (("Front", 0.377), ("Back", 0.624)):
        w.box(
            s,
            f"/Asset/{label}CoinWall",
            (0.010, 0.315, 0.046 if label == "Front" else 0.070),
            (x, 0, C.BIN_FLOOR + (0.023 if label == "Front" else 0.035)),
            TILL,
        )
    w.rounded_box(
        s,
        "/Asset/FrontDrawerRail",
        (0.022, 0.347, 0.019),
        (0.371, 0, z + 0.013),
        TILL,
        0.006,
        0.001,
    )
    w.box(
        s,
        "/Asset/UnderLipSteelWeb",
        (0.056, 0.005, 0.006),
        (0.404, 0, C.EDGE_Z - 0.015),
        STEEL,
    )
    w.rounded_box(
        s,
        "/Asset/ExposedSteelEdge",
        (0.023, 0.006, 0.019),
        (C.EDGE_X, 0, C.EDGE_Z - 0.0095),
        STEEL,
        0.0010,
        0.0003,
    )
    w.box(
        s,
        "/Asset/NickelPlaque",
        (0.020, 0.071, 0.001),
        (0.377, 0, C.BIN_FLOOR + 0.0465),
        BLUE,
        collide=False,
    )
    wordmark(
        w,
        s,
        "/Asset/NickelLabel",
        ("11100111", "10000110", "11101111", "00110100", "11101111"),
        (0.370, -0.012, C.BIN_FLOOR + 0.0471),
        0.0025,
    )
    for i, y in enumerate((-0.095, 0.0, 0.095)):
        w.box(
            s,
            f"/Asset/NoteBay{i}",
            (0.061, 0.086, 0.015),
            (0.664, y, z + 0.012),
            BINS,
            collide=False,
        )
        w.rounded_box(
            s,
            f"/Asset/SpringBillClip{i}",
            (0.041, 0.014, 0.011),
            (0.675, y, z + 0.024),
            STEEL,
            0.004,
            0.001,
            collide=False,
        )
    w.box(s, "/Asset/RearDrawerWall", (0.013, 0.35, 0.05), (0.709, 0, z + 0.03), TILL)
    w.rounded_box(
        s,
        "/Asset/FeltRollRest",
        (0.039, 0.022, 0.031),
        (C.ROLL_X, 0, z + 0.0155),
        SCRAP,
        0.004,
        0.001,
    )
    for i, sign in enumerate((-1, 1)):
        w.box(
            s,
            f"/Asset/RollRestSide{i}",
            (0.004, 0.022, 0.006),
            (C.ROLL_X + sign * 0.013, 0, z + 0.033),
            SCRAP,
        )
    w.rounded_box(
        s,
        "/Asset/ScrapFloor",
        (0.109, 0.338, 0.006),
        (0.2565, 0, z + 0.003),
        SCRAP,
        0.006,
        0.0008,
    )
    for label, x in (("Near", 0.206), ("Far", 0.307)):
        w.box(
            s,
            f"/Asset/Scrap{label}",
            (0.006, 0.335, 0.027),
            (x, 0, C.SCRAP_FLOOR + 0.0135),
            SCRAP,
        )
    for i, y in enumerate((-0.165, 0.165)):
        w.box(
            s,
            f"/Asset/ScrapSide{i}",
            (0.096, 0.006, 0.027),
            (0.2565, y, C.SCRAP_FLOOR + 0.0135),
            SCRAP,
        )
    return w.save(s)


def sleeve(w, name, sign):
    s = w.stage(name)
    b = w.body(s, "/Asset/Paper", 0.012, ccd=True)
    mat = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    mat.CreateStaticFrictionAttr(0.62)
    mat.CreateDynamicFrictionAttr(0.35)
    side_count = 16
    inner = 0.01165
    wall = C.PAPER_RADIUS - inner
    tangent = 2 * inner * tan(pi / side_count) + 0.00065
    for j in range(side_count):
        angle = 2 * pi * j / side_count
        n = np.array([sin(angle), 0, cos(angle)])
        turn = np.array(
            [[cos(angle), 0, sin(angle)], [0, 1, 0], [-sin(angle), 0, cos(angle)]]
        )
        w.box(
            s,
            f"{b}/PaperPanel{j}",
            (tangent, C.PAPER_LENGTH, wall),
            (inner + wall / 2) * n,
            KRAFT,
            rotation=turn,
        )
        for k, y in enumerate((sign * 0.010, sign * 0.0175)):
            p = (C.PAPER_RADIUS + 0.00011) * n + [0, y, 0]
            w.box(
                s,
                f"{b}/BlueInk{j}_{k}",
                (tangent, 0.0045, 0.00012),
                p,
                BLUE,
                rotation=turn,
                collide=False,
            )
    cap_y = sign * (C.PAPER_LENGTH / 2 - 0.00045)
    w.cylinder(
        s, b + "/CrimpContact", C.PAPER_RADIUS, 0.0009, (0, cap_y, 0), KRAFT, axis="Y"
    )
    for j in range(10):
        angle = 2 * pi * j / 10
        w.cylinder(
            s,
            f"{b}/Fold{j}",
            0.0019,
            0.00017,
            (0.0095 * sin(angle), cap_y + sign * 0.00054, 0.0095 * cos(angle)),
            (0.55, 0.31, 0.17),
            axis="Y",
            collide=False,
        )
    return w.save(s)


def coin(w):
    s = w.stage("nickel")
    b = w.body(s, "/Asset/Nickel", C.COIN_MASS, ccd=True)
    mat = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    mat.CreateStaticFrictionAttr(0.17)
    mat.CreateDynamicFrictionAttr(0.14)
    body = w.Px.PhysxRigidBodyAPI.Get(s, b)
    body.CreateLinearDampingAttr(0.25)
    body.CreateAngularDampingAttr(0.30)
    face = w.cylinder(
        s, b + "/Disc", C.COIN_RADIUS, C.COIN_THICKNESS, (0, 0, 0), NICKEL, axis="Y"
    )
    w.Px.PhysxCollisionAPI.Apply(face.GetPrim()).CreateContactOffsetAttr(0.00017)
    for sign in (-1, 1):
        y = sign * (C.COIN_THICKNESS / 2 + 0.000055)
        tag = "Front" if sign == 1 else "Back"
        w.cylinder(
            s,
            f"{b}/{tag}Field",
            0.0089,
            0.00010,
            (0, y, 0),
            (0.61, 0.64, 0.69),
            axis="Y",
            collide=False,
        )
        for j in range(3):
            w.box(
                s,
                f"{b}/{tag}Relief{j}",
                (0.0095 - 0.0012 * j, 0.00013, 0.0007),
                (0, y + sign * 0.00010, -0.0026 + 0.00145 * j),
                STEEL,
                collide=False,
            )
    return w.save(s)


def create_assets(directory):
    w = Writer(directory)
    d = furniture(w)
    l = sleeve(w, "left_sleeve", 1)
    r = sleeve(w, "right_sleeve", -1)
    nickel = coin(w)
    assets = [
        {"name": "drawer", "type": "base", "file": d},
        {
            "name": C.PAPER_NAMES[0],
            "type": "rigid",
            "file": l,
            "position": (C.ROLL_X, C.HALF_Y, C.ROLL_Z),
        },
        {
            "name": C.PAPER_NAMES[1],
            "type": "rigid",
            "file": r,
            "position": (C.ROLL_X, -C.HALF_Y, C.ROLL_Z),
        },
    ]
    for i, name in enumerate(C.COINS):
        assets.append(
            {
                "name": name,
                "type": "rigid",
                "file": nickel,
                "position": (C.ROLL_X, (i - 19.5) * C.COIN_PITCH, C.ROLL_Z),
            }
        )
    w.inventory()
    return {"assets": assets, "settle_actions": 14}
