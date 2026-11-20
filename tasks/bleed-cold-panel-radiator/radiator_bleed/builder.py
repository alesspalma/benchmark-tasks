"""Native service radiator, square key and gravity-supported catch cup."""

import math

import numpy as np

from . import config as C
from .motion import rotation
from .usd_scene import Writer

PAINT = (.81, .83, .81)
EDGE = (.63, .68, .67)
STEEL = (.44, .48, .51)
BRASS = (.72, .47, .20)
CUP = (.20, .62, .62)
BENCH = (.58, .43, .30)


def table(w):
    s = w.stage("room")
    w.box(s, "/Asset/Worktop", (1.10, 1.25, .060), (.32, 0, C.TABLE - .030), BENCH)
    for i, x in enumerate((-.16, .80)):
        for j, y in enumerate((-.57, .57)):
            w.box(s, f"/Asset/Leg{i}{j}", (.058, .058, C.TABLE - .06),
                  (x, y, (C.TABLE - .06) / 2), (.20, .21, .23))
    w.box(s, "/Asset/Floor", (8., 8., .02), (0, 0, -.01), (.32, .36, .35))
    return s


def radiator(w):
    s = w.stage("radiator")
    root = w.body(s, "/Asset/Radiator", 9., fixed=True, articulation=True)
    x, y, z = C.VENT
    pad = C.TABLE + .002
    # A 500 mm by 300 mm sheet-steel panel, rotated to expose the end bleed plug.
    w.box(s, root + "/Panel", (.500, .035, .300), (x + .250, y, pad + .150), PAINT)
    for i in range(24):
        xi = x + .017 + i * .0202
        for sign in (-1, 1):
            w.rounded_box(s, f"{root}/Waterway{i}_{sign + 1}", (.013, .013, .277),
                          (xi, y + sign * .017, pad + .151), PAINT,
                          radius=.004, bevel=.002, collide=False)
    for i, xx in enumerate((x + .066, x + .432)):
        w.box(s, f"{root}/BenchSaddle{i}", (.032, .100, .014),
              (xx, y, C.TABLE + .007), EDGE)
    w.box(s, root + "/UpperGrille", (.496, .050, .008), (x + .250, y, pad + .300), EDGE)
    for i in range(21):
        w.box(s, f"{root}/GrilleSlot{i}", (.013, .039, .0007),
              (x + .022 + i * .0225, y, pad + .3043), (.37, .42, .42), collide=False)
    w.cylinder(s, root + "/VentBoss", .0115, .018, (x - .008, y, z), EDGE, axis="X")
    w.cylinder(s, root + "/BleedBody", .0072, .017, (x - .014, y, z), BRASS, axis="Y")
    w.box(s, root + "/VentOutlet", (.009, .012, .010), (x - .011, y + .012, z - .006), BRASS)
    valve = w.body(s, "/Asset/Spindle", .008, position=C.VENT, ccd=True)
    w.cylinder(s, valve + "/Guide", .0055, .009, (-.020, 0, 0), BRASS, axis="X")
    w.box(s, valve + "/SquareStem", (.015, .005, .005), (-.027, 0, 0), STEEL)
    rev = w.Phy.RevoluteJoint.Define(s, "/Asset/VentAngle")
    rev.CreateBody0Rel().SetTargets([root])
    rev.CreateBody1Rel().SetTargets([valve])
    rev.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, C.VENT)))
    rev.CreateAxisAttr("X")
    rev.CreateLowerLimitAttr(-120.)
    rev.CreateUpperLimitAttr(0.)
    rev.CreateCollisionEnabledAttr(False)
    # The passive drive dissipates motion without biasing either turn direction.
    spring = w.Phy.DriveAPI.Apply(rev.GetPrim(), "angular")
    spring.CreateTypeAttr("force")
    spring.CreateStiffnessAttr(0.)
    spring.CreateDampingAttr(.0015)
    spring.CreateMaxForceAttr(.10)
    w.Px.PhysxJointAPI.Apply(rev.GetPrim()).CreateJointFrictionAttr(.022)
    return s


def key(w):
    s = w.stage("key")
    body = w.body(s, "/Asset/BrassKey", .019, ccd=True)
    w.rounded_box(s, body + "/Butterfly", (.010, .043, .012),
                  (-.003, 0, 0), BRASS, radius=.004, bevel=.002)
    socket_len = .0135
    for sign in (-1, 1):
        w.box(s, body + f"/StemChannelSide{sign}", (socket_len, .0014, .0095),
              (.007, sign * .00365, 0.), BRASS)
        w.box(s, body + f"/StemChannelFace{sign}", (socket_len, .0095, .0014),
              (.007, 0, sign * .00365), BRASS)
    return s


def cup(w):
    s = w.stage("cup")
    body = w.body(s, "/Asset/Cup", .079, ccd=True)
    w.cylinder(s, body + "/Bottom", C.CUP_RADIUS + .0005, .003, (0, 0, .0015), CUP)
    for i in range(32):
        angle = i * math.tau / 32
        rad = np.array([math.cos(angle), math.sin(angle), 0.])
        w.box(s, f"{body}/Wall{i}", (.002, .0081, C.CUP_HEIGHT - .003),
              rad * C.CUP_RADIUS + [0, 0, (C.CUP_HEIGHT + .003) / 2],
              CUP, rotation("Z", angle), collide=True)
    for i, height in enumerate((.009, .016, .024, .032, .040, .048, .056, .064)):
        w.box(s, f"{body}/Graduation{i}", (.00055, .009 if i % 2 == 0 else .005, .0013),
              (-C.CUP_RADIUS - .001, -.002, height), (.89, .93, .9), collide=False)
    w.rounded_box(s, body + "/ThumbGrip", (.024, .016, .030),
                  tuple(C.CUP_HANDLE), (.14, .44, .44), radius=.0035, bevel=.001)
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = []
    for name, kind, stage, pos in (
        ("table", "base", table(w), (0, 0, 0)),
        ("radiator", "articulation", radiator(w), (0, 0, 0)),
        ("key", "rigid", key(w), C.KEY_HOME),
        ("cup", "rigid", cup(w), C.CUP_START),
    ):
        assets.append(dict(name=name, type=kind, file=w.save(stage), position=list(pos),
                           quaternion=[0., 0., 0., 1.]))
    return {"assets": assets, "settle_actions": 15}
