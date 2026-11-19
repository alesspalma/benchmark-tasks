"""Procedural baking sheet, free barrel and contact-driven lever."""

import math

import numpy as np

from . import config as C
from .motion import rotation
from .usd_scene import Writer


def machine(w):
    stage = w.stage("press")
    barrel = w.body(stage, "/Asset/Barrel", 0.328, articulation=True, ccd=True)
    lever = w.body(stage, "/Asset/Lever", 0.055, position=C.PIVOT, ccd=True)
    w.Px.PhysxArticulationAPI.Get(stage, barrel).CreateEnabledSelfCollisionsAttr(False)
    for path, inertia in (
        (barrel, (0.0013, 0.0013, 0.00016)),
        (lever, (0.00012, 0.00005, 0.00010)),
    ):
        w.Phy.MassAPI.Get(stage, path).CreateDiagonalInertiaAttr(w.Gf.Vec3f(*inertia))
    w.Phy.MassAPI.Get(stage, barrel).CreateCenterOfMassAttr(w.Gf.Vec3f(0.0, 0.0, 0.054))
    graphite = (0.085, 0.115, 0.127)
    blue = (0.055, 0.31, 0.40)
    alloy = (0.47, 0.53, 0.57)
    dough = (0.75, 0.47, 0.22)
    # The release gap under the die preserves the real standoff to the tray.
    ring = "/Asset/Barrel/Base"
    w.ring(stage, ring, C.BASE_RADIUS, 0.006, 0.012, 0.006, graphite, n=16)
    w.cylinder(stage, barrel + "/Die", 0.026, 0.002, (0, 0, 0.0115), alloy)
    for k in range(6):
        a = math.tau * k / 6
        w.cylinder(
            stage,
            barrel + f"/DieHole{k}",
            0.0039,
            0.00018,
            (0.012 * math.cos(a), 0.012 * math.sin(a), 0.01043),
            (0.09, 0.09, 0.09),
            collide=False,
        )
    w.cylinder(stage, barrel + "/LowerCollar", 0.0305, 0.017, (0, 0, 0.024), blue)
    w.cylinder(
        stage, barrel + "/Charge", C.BARREL_RADIUS, 0.111, (0.0, 0.0, 0.090), dough
    )
    w.cylinder(
        stage,
        barrel + "/GripBand",
        C.BARREL_RADIUS + 0.001,
        0.044,
        (0.0, 0.0, C.BARREL_GRIP_Z),
        (0.16, 0.34, 0.39),
    )
    w.cylinder(stage, barrel + "/Collar", 0.029, 0.024, (0.0, 0.0, 0.149), blue)
    w.cylinder(stage, barrel + "/TopHousing", 0.024, 0.022, (0.0, 0.0, 0.168), graphite)
    w.cylinder(
        stage, barrel + "/Rod", 0.004, 0.045, (0.0, 0.0, 0.195), alloy, collide=False
    )
    for k in range(4):
        w.box(
            stage,
            barrel + f"/FillMark{k}",
            (0.0008, 0.011, 0.001),
            (0.02445, 0.000, 0.056 + k * 0.020),
            (0.36, 0.28, 0.22),
            collide=False,
        )
    for y in (-0.010, 0.010):
        w.cylinder(
            stage,
            barrel + f"/Hinge_{y > 0}",
            0.008,
            0.012,
            (0.0, C.PIVOT[1] + y, C.PIVOT[2]),
            alloy,
            axis="X",
            collide=False,
        )
    L = np.linalg.norm(C.LEVER_VECTOR)
    tilt = -math.atan2(C.LEVER_VECTOR[2], -C.LEVER_VECTOR[1])
    w.box(
        stage,
        lever + "/RatchetLever",
        (0.017, L, 0.012),
        C.LEVER_VECTOR / 2.0,
        graphite,
        rotation=rotation("X", tilt),
    )
    w.box(
        stage,
        lever + "/ThumbPad",
        (0.035, 0.029, 0.011),
        C.LEVER_VECTOR,
        blue,
        rotation=rotation("X", tilt),
    )
    joint = w.Phy.RevoluteJoint.Define(stage, "/Asset/Actuate")
    joint.CreateBody0Rel().SetTargets([barrel])
    joint.CreateBody1Rel().SetTargets([lever])
    joint.CreateAxisAttr("X")
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*C.PIVOT))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLowerLimitAttr(0.0)
    joint.CreateUpperLimitAttr(math.degrees(C.LEVER_LIMIT))
    joint.CreateCollisionEnabledAttr(False)
    return stage


def tray(w):
    stage = w.stage("tray")
    sx, sy = C.TRAY_SIZE
    metal = (0.38, 0.44, 0.49)
    edge = (0.23, 0.28, 0.32)
    w.box(stage, "/Asset/Sheet", (sx, sy, 0.0025), (0, 0, 0.00925), metal)
    for sign in (-1, 1):
        w.box(
            stage,
            f"/Asset/EdgeX{sign > 0}",
            (0.005, sy + 0.010, 0.012),
            (sign * (sx / 2 + 0.0025), 0, 0.014),
            edge,
        )
        w.box(
            stage,
            f"/Asset/EdgeY{sign > 0}",
            (sx, 0.005, 0.012),
            (0, sign * (sy / 2 + 0.0025), 0.014),
            edge,
        )
    # Baked-on low-profile dough extrusion is modelled in the material layer.
    # This parent holds its rendered surface, leaving the sheet contact intact.
    w.Geom.Xform.Define(stage, "/Asset/ExtrudedDough")
    return stage


def create_assets(directory):
    writer = Writer(directory)
    return {
        "assets": [
            {
                "name": "press",
                "type": "articulation",
                "file": writer.save(machine(writer)),
                "position": C.START,
            },
            {
                "name": "tray",
                "type": "base",
                "file": writer.save(tray(writer)),
                "position": C.TRAY,
            },
        ],
        "settle_actions": 22,
    }
