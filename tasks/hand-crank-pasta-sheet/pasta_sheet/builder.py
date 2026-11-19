"""Native colliders around an unpowered pasta roller and a loose dough sheet."""

import math
from pathlib import Path

import numpy as np

from . import config as C
from .usd_scene import Writer

STEEL = (0.58, 0.62, 0.66)
DARK = (0.17, 0.21, 0.24)
WOOD = (0.60, 0.43, 0.26)
DOUGH = (0.91, 0.65, 0.25)


def revolute(w, stage, name, base, shaft, origin):
    joint = w.Phy.RevoluteJoint.Define(stage, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([base])
    joint.CreateBody1Rel().SetTargets([shaft])
    joint.CreateAxisAttr("Y")
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, origin)))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateCollisionEnabledAttr(False)
    return joint


def machine(w):
    s = w.stage("machine")
    base = w.body(s, "/Asset/Housing", 2.3, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(s, base).CreateEnabledSelfCollisionsAttr(False)
    w.box(
        s, base + "/Foot", (0.188, 0.202, 0.024), (*C.NIP[:2], C.TABLE + 0.012), STEEL
    )
    for k, sign in enumerate((-1, 1)):
        y = C.NIP[1] + sign * 0.091
        w.box(
            s,
            base + f"/Bearing{k}",
            (0.116, 0.027, 0.149),
            (C.NIP[0], y, C.NIP[2] - 0.068),
            STEEL,
        )
        w.cylinder(
            s,
            base + f"/Dial{k}",
            0.023,
            0.016,
            (C.NIP[0], y + sign * 0.018, C.NIP[2] - 0.051),
            DARK,
            "Y",
            False,
        )
    for name, origin in (("Driven", C.ROLL_NEAR), ("Opposed", C.ROLL_FAR)):
        shaft = w.body(s, "/Asset/" + name + "Shaft", 0.19, origin, ccd=True)
        w.cylinder(
            s,
            shaft + "/SteelRoll",
            C.ROLL_RADIUS,
            C.ROLL_WIDTH,
            (0.0, 0.0, 0.0),
            STEEL,
            "Y",
        )
        if name == "Driven":
            w.cylinder(s, shaft + "/Axle", 0.006, 0.052, (0.0, -0.101, 0.0), STEEL, "Y")
            a = C.CRANK_START
            radial = np.array([math.cos(a), 0.0, -math.sin(a)])
            spin = np.array(
                [
                    [radial[0], 0.0, -radial[2]],
                    [0.0, 1.0, 0.0],
                    [radial[2], 0.0, radial[0]],
                ]
            )
            w.box(
                s,
                shaft + "/CrankArm",
                (0.080, 0.009, 0.013),
                0.5 * C.CRANK_RADIUS * radial + [0.0, -0.127, 0.0],
                STEEL,
                rotation=spin,
            )
            w.cylinder(
                s,
                shaft + "/Grip",
                C.CRANK_KNOB_RADIUS,
                0.037,
                C.CRANK_RADIUS * radial + [0.0, C.CRANK_SPAN, 0.0],
                DARK,
                "Y",
            )
        revolute(w, s, name, base, shaft, origin)
    return w.save(s)


def board(w):
    s = w.stage("board")
    p = "/Asset/Set"
    w.box(
        s,
        p + "/DoughBoard",
        (0.176, 0.170, 0.1655),
        (0.296, 0.038, C.TABLE + 0.08275),
        WOOD,
    )
    w.box(
        s,
        p + "/CatchBoard",
        (0.170, 0.155, 0.012),
        (C.NIP[0], C.NIP[1], C.TABLE + 0.028),
        WOOD,
    )
    return w.save(s)


def dough(w):
    s = w.stage("dough")
    base = w.body(s, "/Asset/LowerFace", 0.022, ccd=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(s, base).CreateEnabledSelfCollisionsAttr(False)
    w.box(
        s,
        base + "/Dough",
        (C.STOCK_LENGTH, C.STOCK_WIDTH, 0.0024),
        (0.0, 0.0, -0.0012),
        DOUGH,
    )
    moving = w.body(s, "/Asset/UpperFace", 0.021, (0.0, 0.0, 0.0012), ccd=True)
    w.box(
        s,
        moving + "/Dough",
        (C.STOCK_LENGTH, C.STOCK_WIDTH, 0.0024),
        (0.0, 0.0, 0.0),
        DOUGH,
    )
    compression = w.Phy.PrismaticJoint.Define(s, "/Asset/Compression")
    compression.CreateBody0Rel().SetTargets([base])
    compression.CreateBody1Rel().SetTargets([moving])
    compression.CreateLocalPos0Attr(w.Gf.Vec3f(0, 0, 0.0012))
    compression.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    compression.CreateAxisAttr("Z")
    compression.CreateLowerLimitAttr(-0.00155)
    compression.CreateUpperLimitAttr(0.0)
    compression.CreateCollisionEnabledAttr(False)
    w.Px.JointStateAPI.Apply(compression.GetPrim(), "linear").CreatePositionAttr(0.0)
    drive = w.Phy.DriveAPI.Apply(compression.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(6800.0)
    drive.CreateDampingAttr(8.0)
    drive.CreateMaxForceAttr(14.0)
    drive.CreateTargetPositionAttr(0.0)

    return w.save(s)


def create_assets(directory):
    w = Writer(Path(directory))
    return {
        "assets": [
            {"name": "machine", "type": "articulation", "file": machine(w)},
            {"name": "board", "type": "base", "file": board(w)},
            {
                "name": "dough",
                "type": "articulation",
                "file": dough(w),
                "position": C.STOCK,
            },
        ],
        "settle_actions": 10,
    }
