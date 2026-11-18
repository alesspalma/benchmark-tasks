"""Compound static props with a native rigid lid and revolute hinge."""

import math
import numpy as np

from . import config as C
from . import worker
from .usd_scene import Writer

WHITE = (.89, .90, .89)
WHITE_TOP = (.96, .965, .965)
METAL = (.55, .58, .60)
RUBBER = (.12, .14, .15)
SHIRT = (.025, .21, .32)
SKIN = (.67, .45, .32)
HAIR = (.15, .09, .055)


def vessel(w):
    stage = w.stage("freezer")
    base = w.body(stage, "/Asset/Cabinet", 24., fixed=True, articulation=True)
    lid = w.body(stage, "/Asset/Lid", C.LID_MASS, position=C.HINGE, ccd=True)
    w.Px.PhysxArticulationAPI.Get(stage, base).CreateEnabledSelfCollisionsAttr(False)
    w.Px.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(lid)).CreateThresholdAttr(0.)
    front, back = C.HINGE[0] - C.DEPTH, C.HINGE[0]
    middle = (front + back) / 2
    for suffix, coordinate in (("Front", front + .025), ("Rear", back - .025)):
        w.box(stage, base + "/" + suffix, (.05, C.WIDTH, .8008), (coordinate, 0., C.FLOOR + .4004), WHITE)
        inner = coordinate + (.026 if suffix == "Front" else -.026)
        w.box(stage, base + "/Foil" + suffix, (.003, C.WIDTH-.10, .701), (inner, 0., C.FLOOR+.396), METAL, collide=False)
    for suffix, side in (("Left", 1), ("Right", -1)):
        w.box(stage, base + "/" + suffix, (C.DEPTH - .10, .05, .8008), (middle, side*(C.WIDTH/2-.025), C.FLOOR+.4004), WHITE)
        w.box(stage, base + "/Foil" + suffix, (C.DEPTH-.10, .003, .701), (middle, side*(C.WIDTH/2-.053), C.FLOOR+.396), METAL, collide=False)
    w.box(stage, base + "/Bottom", (C.DEPTH, C.WIDTH, .10), (middle, 0, C.FLOOR + .05), WHITE)
    w.box(stage, base + "/InteriorFloor", (C.DEPTH-.10, C.WIDTH-.10, .005), (middle, 0, C.FLOOR+.103), METAL, collide=False)
    w.box(stage, base + "/Control", (.006, .11, .046), (front-.002, -.16, C.FLOOR+.13), (.24, .27, .27), collide=False)
    w.cylinder(stage, base + "/Dial", .014, .007, (front-.007, -.175, C.FLOOR+.13), (.89,.89,.88), axis="X", collide=False)
    w.box(stage, lid + "/Panel", (C.DEPTH, C.WIDTH, C.THICKNESS), (-C.DEPTH/2, 0., C.THICKNESS/2), WHITE_TOP)
    for i, y in enumerate((-.176, .176)):
        w.box(stage, lid + f"/HingeCover{i}", (.044,.067,.042), (.003,y,.012), (.62,.64,.64), collide=False)
    w.box(stage, lid + "/Recess", (.047, .232, .011), (-C.DEPTH+.025,0.,.038), (.32,.36,.36), collide=False)
    w.box(stage, lid + "/GripRim", (.012, .247, .013), (-C.DEPTH+.004,0.,.044), WHITE_TOP, collide=False)
    w.box(stage, lid + "/Gasket", (C.DEPTH-.045, C.WIDTH-.045, .006), (-C.DEPTH/2,0.,.002), RUBBER, collide=False)
    mass = w.Phy.MassAPI.Get(stage, lid)
    mass.CreateCenterOfMassAttr(w.Gf.Vec3f(-C.DEPTH/2, 0., C.THICKNESS/2))
    mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(
        C.LID_MASS * (C.WIDTH**2 + C.THICKNESS**2)/12,
        C.LID_MASS * (C.DEPTH**2 + C.THICKNESS**2)/12,
        C.LID_MASS * (C.DEPTH**2 + C.WIDTH**2)/12,
    ))
    hinge = w.Phy.RevoluteJoint.Define(stage, "/Asset/Hinge")
    hinge.CreateBody0Rel().SetTargets([base])
    hinge.CreateBody1Rel().SetTargets([lid])
    hinge.CreateAxisAttr("Y")
    hinge.CreateLocalPos0Attr(w.Gf.Vec3f(*C.HINGE))
    hinge.CreateLocalPos1Attr(w.Gf.Vec3f(0., 0., 0.))
    hinge.CreateLowerLimitAttr(0.)
    hinge.CreateUpperLimitAttr(85.)
    hinge.CreateCollisionEnabledAttr(False)
    return stage


def ellipsoid(w, stage, path, center, radii, color, major=None, collide=True):
    body = path.split("/")[-2]
    center = np.asarray(center) - worker.ANCHORS[body]
    shape = w.Geom.Sphere.Define(stage, path)
    shape.CreateRadiusAttr(1.)
    shape.AddTranslateOp().Set(w.Gf.Vec3d(*center))
    if major is not None:
        from yam_arena.station import aim
        x,y,z,a = aim(major)
        shape.AddOrientOp(precision=w.Geom.XformOp.PrecisionDouble).Set(w.Gf.Quatd(a,x,y,z))
    shape.AddScaleOp().Set(w.Gf.Vec3f(*radii))
    # Nonuniform sphere scales fit render only. Approximate contact with
    # independently sized capsules and balls in the swept upper-body region.
    w._finish(stage, shape, color, False)
    if collide:
        contact = w.Geom.Capsule.Define(stage, path + "Contact")
        radius = min(radii)
        contact.CreateRadiusAttr(radius)
        contact.CreateHeightAttr(max(2.*(radii[2]-radius), 0.))
        contact.AddTranslateOp().Set(w.Gf.Vec3d(*center))
        if major is not None:
            contact.AddOrientOp(precision=w.Geom.XformOp.PrecisionDouble).Set(w.Gf.Quatd(a,x,y,z))
        w._finish(stage, contact, color, True)
        contact.MakeInvisible()
        half_axis = np.zeros(3) if major is None else np.asarray(major) / np.linalg.norm(major)
        capsule_half_height = max(radii[2] - radius, 0.)
        if major is None:
            half_axis[2] = 1.
        return (body, tuple(center - capsule_half_height * half_axis),
                tuple(center + capsule_half_height * half_axis), radius)
    return None


def segment(w, stage, name, a, b, radii, color):
    a,b = np.array(a), np.array(b)
    axis = b-a
    return ellipsoid(w, stage, name, tuple((a+b)/2), (*radii,float(np.linalg.norm(axis))/2+.018), color, axis)


def stocker(w):
    stage = w.stage("stocker")
    for name in worker.BODY_NAMES:
        path = w.body(stage, f"/Asset/{name}", 2., position=worker.ANCHORS[name])
        w.Phy.RigidBodyAPI.Get(stage, path).CreateKinematicEnabledAttr(True)
        w.Px.PhysxContactReportAPI.Apply(stage.GetPrimAtPath(path)).CreateThresholdAttr(0.)
    bounds = []
    torso = "/Asset/MotionTorso/"
    shoulder = (.318, -.343, .942)
    neck = (.320, -.277, .948)
    head = (.324, -.174, .933)
    bounds.append(segment(w, stage, torso + "Shirt", worker.HIP, shoulder, (.189, .088), SHIRT))
    bounds.append(segment(w, stage, torso + "Neck", shoulder, neck, (.049, .046), SKIN))
    bounds.append(segment(w, stage, torso + "Head", neck, head, (.079, .087), SKIN))
    ellipsoid(w, stage, torso + "Crown", (.326, -.221, .968), (.078, .070, .058), HAIR, collide=False)
    for i, x in enumerate((.254, .389)):
        ellipsoid(w, stage, torso + f"Ear{i}", (x, -.211, .927), (.017, .024, .024), SKIN, collide=False)
    for i, (shoulder, elbow, wrist) in enumerate(zip(worker.SHOULDERS, worker.ELBOWS, worker.WRISTS, strict=True)):
        bounds.append(segment(w, stage, f"/Asset/MotionUpper{i}/Sleeve", shoulder, elbow, (.053, .051), SHIRT))
        forearm = f"/Asset/MotionForearm{i}/"
        bounds.append(segment(w, stage, forearm + "Forearm", elbow, wrist, (.038, .038), SKIN))
        bounds.append(ellipsoid(w, stage, forearm + "Hand", wrist + (0., .015, -.020),
                                (.041, .029, .057), SKIN, (0., .35, -.94)))
    for i, (ankle, top) in enumerate(zip(worker.ANKLES, worker.LEG_TOPS, strict=True)):
        leg = f"/Asset/MotionLeg{i}/"
        bounds.append(segment(w, stage, leg + "Leg", ankle, top, (.083, .078), (.12, .15, .20)))
        ellipsoid(w, stage, leg + "Shoe", (ankle[0], -.535, .030),
                  (.084, .13, .041), (.075, .08, .09), collide=False)
    return stage, bounds


def support(w):
    stage = w.stage("supports")
    for name, y in (("Left",.305),("Right",-.305)):
        height = C.BASES[name.lower()][2] - C.FLOOR
        frame = (.20,.20,height)
        w.box(stage, f"/Asset/{name}Pedestal", frame, (0., y, C.FLOOR + height/2), (.45,.49,.51))
    return stage


def stock(w):
    stage = w.stage("stock")
    for i,(x,y,z) in enumerate(((.385,-.140,.621),(.388,-.023,.625),(.312,.102,.621),(.437,.102,.615))):
        w.box(stage, f"/Asset/Packet{i}", (.093,.102,.048), (x,y,z), ((.67,.39,.23),(.22,.47,.67),(.76,.72,.54),(.63,.44,.58))[i])
    w.box(stage, "/Asset/BasketFloor", (.321,.392,.009), (.35775,0,.588), (.56,.60,.61))
    for y in (-.19,.19):
        w.box(stage, f"/Asset/BasketEdge{int(y>0)}", (.321,.009,.116), (.35775,y,.646), (.65,.69,.69))
    return stage


def create_assets(directory):
    w = Writer(directory)
    person, bounds = stocker(w)
    specs = [("freezer", "articulation", vessel(w)), ("supports", "base", support(w)),
             ("stock", "base", stock(w)), ("stocker", "base", person)]
    return {"assets": [dict(name=n, type=k, file=w.save(s)) for n, k, s in specs],
            "worker_bounds": bounds}
