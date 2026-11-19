"""Compound aluminum frame, two side hinges and spring-loaded release buttons."""

import numpy as np

from . import config as C
from .usd_scene import Writer

ALUMINUM = (.70, .74, .77)
GRIP = (.09, .12, .15)
RUBBER = (.09, .09, .11)
LATCH = (.15, .17, .21)
BUTTON = (.59, .22, .08)


def tube(writer, stage, path, a, b, radius, color=ALUMINUM, *, collide=True):
    a, b = np.asarray(a), np.asarray(b)
    direction = b - a
    length = np.linalg.norm(direction)
    solid = writer.cylinder(stage, path, radius, length, (a + b) / 2, color, collide=collide)
    rotation = writer.Gf.Rotation(writer.Gf.Vec3d(0, 0, 1), writer.Gf.Vec3d(*(direction/length)))
    solid.AddOrientOp(precision=writer.Geom.XformOp.PrecisionDouble).Set(rotation.GetQuat())
    writer.records[-1].update(endpoints=[a.tolist(), b.tolist()])


def hinge(writer, stage, path, cls, body_a, body_b, p, limits):
    joint = cls.Define(stage, "/Asset/" + path)
    joint.CreateBody0Rel().SetTargets([body_a])
    joint.CreateBody1Rel().SetTargets([body_b])
    joint.CreateAxisAttr("Z")
    joint.CreateLocalPos0Attr(writer.Gf.Vec3f(*p))
    joint.CreateLocalPos1Attr(writer.Gf.Vec3f(0, 0, 0))
    joint.CreateLowerLimitAttr(limits[0])
    joint.CreateUpperLimitAttr(limits[1])
    joint.CreateCollisionEnabledAttr(False)
    return joint


def walker(writer):
    stage = writer.stage("walker")
    root = writer.body(stage, "/Asset/CrossFrame", .49, articulation=True)
    writer.Px.PhysxArticulationAPI.Get(stage, root).CreateEnabledSelfCollisionsAttr(False)
    root_phys = writer.Px.PhysxRigidBodyAPI.Get(stage, root)
    root_phys.CreateAngularDampingAttr(.32)
    root_phys.CreateLinearDampingAttr(.28)
    half = C.HINGE_HALF_WIDTH
    tube(writer, stage, root + "/UpperBridge", (0, -half, .799), (0, half, .799), .0127)
    tube(writer, stage, root + "/LowerBridge", (.024, -half, .453), (.024, half, .453), .0127)
    for sign in (-1, 1):
        y = sign * half
        tube(writer, stage, root + f"/BridgeEnd{sign}", (.024, y, .453), (0, y, .799), .015)
    for side, sign in C.SIDES.items():
        origin = (0., sign * half, 0.)
        swing = writer.body(stage, "/Asset/" + side.capitalize() + "Side", .97, position=origin)
        # The top tube is continuous from the hinge to the rear upright. A sleeve
        # at the user's grip supplies the contact surface for opening either side.
        tube(writer, stage, swing + "/FrontLower", (0., 0., .021), (0., 0., .535), .0115)
        tube(writer, stage, swing + "/FrontUpper", (0., 0., .456), (0., 0., C.GRIP_HEIGHT), .0127)
        tube(writer, stage, swing + "/Top", (0., 0., C.GRIP_HEIGHT), (.338, 0., C.GRIP_HEIGHT), .0127)
        tube(writer, stage, swing + "/Grip", (.109, 0., C.GRIP_HEIGHT),
             (.273, 0., C.GRIP_HEIGHT), .018, GRIP)
        tube(writer, stage, swing + "/RearLeg", (.338, 0., C.GRIP_HEIGHT),
             (C.BACK_FOOT_DISTANCE, sign * C.BACK_FOOT_SPLAY, .020), .0127)
        tube(writer, stage, swing + "/RearLower", (.379, sign * .017, .200),
             (C.BACK_FOOT_DISTANCE, sign * C.BACK_FOOT_SPLAY, .020), .0115)
        tube(writer, stage, swing + "/LowerBrace", (0., 0., .464),
             (.362, sign * .009, .464), .011)
        for foot, xy in (("Front", (0., 0.)), ("Rear", (C.BACK_FOOT_DISTANCE, sign*C.BACK_FOOT_SPLAY))):
            writer.cylinder(stage, swing + "/" + foot + "Ferrule",
                            C.FRONT_FOOT_RADIUS, .035, (*xy, .0195), RUBBER, collide=False)
            writer.box(stage, swing + "/" + foot + "RubberContact",
                       (.028, .028, .008), (*xy, .004), RUBBER)
        for i in range(5):
            writer.cylinder(stage, swing + f"/AdjustmentHole{i}", .0024, .0004,
                            (.0113, 0., .122 + .025 * i), GRIP, axis="X", collide=False)
        writer.box(stage, root + "/" + side.capitalize() + "LatchHousing",
                   (.051, .042, .022), (-.006, sign*half, .829), LATCH)
        angle_limits = (-87., .8) if sign > 0 else (-.8, 87.)
        joint = hinge(writer, stage, side + "Hinge", writer.Phy.RevoluteJoint,
                      root, swing, origin, angle_limits)
        props = writer.Px.PhysxJointAPI.Apply(joint.GetPrim())
        props.CreateJointFrictionAttr(.14)
        props.CreateArmatureAttr(.008)
        button_origin = (-.006, sign*half, .841)
        button = writer.body(stage, "/Asset/" + side.capitalize() + "Button", .018,
                             position=button_origin)
        writer.cylinder(stage, button + "/Cap", .014, .009, (0., 0., .002), BUTTON)
        pin = hinge(writer, stage, side + "Button", writer.Phy.PrismaticJoint, root,
                    button, button_origin, (0., C.BUTTON_TRAVEL))
        spring = writer.Phy.DriveAPI.Apply(pin.GetPrim(), "linear")
        spring.CreateTypeAttr("force")
        spring.CreateStiffnessAttr(520.)
        spring.CreateDampingAttr(4.)
        spring.CreateMaxForceAttr(7.)
        spring.CreateTargetPositionAttr(C.BUTTON_TRAVEL)
    return stage


def fixtures(writer):
    stage = writer.stage("fixtures")
    for name, (x, y, z) in C.BASES.items():
        writer.box(stage, "/Asset/" + name + "Pedestal", (.140, .140, z-C.FLOOR),
                   (x, y, (z+C.FLOOR)/2), (.22, .25, .27))
        writer.box(stage, "/Asset/" + name + "PedestalFoot", (.204, .205, .012),
                   (x, y, C.FLOOR+.006), (.15, .17, .19))
    chair = (.45, .25, .14)
    steel = (.23, .25, .28)
    writer.box(stage, "/Asset/ChairSeat", (.413, .435, .044), (C.CHAIR_X, 0., .449), chair)
    writer.box(stage, "/Asset/ChairBack", (.034, .434, .317), (C.CHAIR_X+.188, 0., .767), chair)
    for yi in (-.189, .189):
        for xi in (-.169, .169):
            tube(writer, stage, f"/Asset/ChairLeg{xi}_{yi}",
                 (C.CHAIR_X+xi, yi, C.FLOOR), (C.CHAIR_X+xi, yi, .441), .012, steel)
        tube(writer, stage, f"/Asset/ChairBackPost{yi}", (C.CHAIR_X+.177, yi, .43),
             (C.CHAIR_X+.177, yi, .901), .012, steel)
    # Rigid clothed training mannequin. The handoff area ends well before its toes.
    pants = (.20, .29, .41)
    skin = (.65, .44, .31)
    shirt = (.27, .38, .35)
    torso_rotation = np.array([[.99, 0., -.141067], [0., 1., 0.], [.141067, 0., .99]])
    writer.box(stage, "/Asset/ManikinTorso", (.165, .336, .448),
               (C.CHAIR_X+.082, 0., .710), shirt, torso_rotation)
    writer.cylinder(stage, "/Asset/ManikinNeck", .043, .074, (C.CHAIR_X+.054, 0., .949), skin)
    head = writer.Geom.Sphere.Define(stage, "/Asset/TrainingManikinHead")
    head.CreateRadiusAttr(.094)
    head.AddTranslateOp().Set(writer.Gf.Vec3d(C.CHAIR_X+.043, 0., 1.058))
    head.AddScaleOp().Set(writer.Gf.Vec3f(.84, .87, 1.10))
    writer._finish(stage, head, skin, False)
    for y in (-.113, .113):
        writer.box(stage, f"/Asset/ManikinThigh{y}", (.290, .148, .107),
                   (C.CHAIR_X-.021, y, .521), pants)
        writer.box(stage, f"/Asset/ManikinShin{y}", (.109, .113, .450),
                   (C.CHAIR_X-.128, y, .271), pants)
        writer.box(stage, f"/Asset/Shoe{y}", (.214, .101, .060),
                   (C.CHAIR_X-.165, y, C.FLOOR+.030), (.103, .112, .126))
    for y in (-.169, .169):
        tube(writer, stage, f"/Asset/ManikinUpperArm{y}", (C.CHAIR_X+.06, y, .864),
             (C.CHAIR_X+.02, y, .576), .052, shirt)
        tube(writer, stage, f"/Asset/ManikinForearm{y}", (C.CHAIR_X+.02, y, .576),
             (C.CHAIR_X-.131, y*.75, .594), .037, skin)
    return stage


def create_assets(directory):
    writer = Writer(directory)
    stages = (
        ("walker", "articulation", walker(writer), C.START.tolist()),
        ("fixtures", "base", fixtures(writer), [0., 0., 0.]),
    )
    assets = [dict(name=name, type=kind, file=writer.save(stage), position=position)
              for name, kind, stage, position in stages]
    writer.inventory()
    return {"assets": assets, "settle_actions": 20}
