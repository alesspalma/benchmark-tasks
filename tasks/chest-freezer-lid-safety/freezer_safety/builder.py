"""Compound static props with a native rigid lid and revolute hinge."""

import math
import numpy as np

from . import config as C
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


def ellipsoid(w, stage, name, center, radii, color, major=None, collide=True):
    shape = w.Geom.Sphere.Define(stage, f"/Asset/{name}")
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
        contact = w.Geom.Capsule.Define(stage, f"/Asset/{name}Contact")
        radius = min(radii)
        contact.CreateRadiusAttr(radius)
        contact.CreateHeightAttr(max(2.*(radii[2]-radius), 0.))
        contact.AddTranslateOp().Set(w.Gf.Vec3d(*center))
        if major is not None:
            contact.AddOrientOp(precision=w.Geom.XformOp.PrecisionDouble).Set(w.Gf.Quatd(a,x,y,z))
        w._finish(stage, contact, color, True)
        contact.MakeInvisible()


def segment(w, stage, name, a, b, radii, color):
    a,b = np.array(a), np.array(b)
    axis = b-a
    ellipsoid(w, stage, name, tuple((a+b)/2), (*radii,float(np.linalg.norm(axis))/2+.018), color, axis)


def stocker(w):
    stage = w.stage("stocker")
    hip = (.362,-.565,.925)
    shoulder = (.318,-.343,.942)
    neck = (.320,-.277,.948)
    head = (.324,-.174,.933)
    segment(w,stage,"Shirt",hip,shoulder,(.189,.088),SHIRT)
    segment(w,stage,"Neck",shoulder,neck,(.049,.046),SKIN)
    segment(w,stage,"Head",neck,head,(.079,.087),SKIN)
    ellipsoid(w,stage,"Crown",(.326,-.221,.968),(.078,.070,.058),HAIR,collide=False)
    for i, x in enumerate((.254, .389)):
        ellipsoid(w,stage,f"Ear{i}",(x,-.211,.927),(.017,.024,.024),SKIN,collide=False)
    for index, x in enumerate((.274,.402)):
        shoulder_end = (x,-.351,.937)
        elbow = (x,-.196,.828)
        wrist = (x-.005,-.066,.704)
        segment(w,stage,f"Sleeve{index}",shoulder_end,elbow,(.053,.051),SHIRT)
        segment(w,stage,f"Forearm{index}",elbow,wrist,(.038,.038),SKIN)
        ellipsoid(w,stage,f"Hand{index}",(wrist[0],wrist[1]+.015,wrist[2]-.020),(.041,.029,.057),SKIN,(0.,.35,-.94))
    for i,x in enumerate((.281,.453)):
        segment(w,stage,f"Leg{i}",(x,-.59,.010),(x,-.57,.918),(.083,.078),(.12,.15,.20))
        ellipsoid(w,stage,f"Shoe{i}",(x,-.535,.03),(.084,.13,.041),(.075,.08,.09),collide=False)
    return stage


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


def create_assets(directory, occupied):
    w = Writer(directory)
    specs = [("freezer","articulation",vessel(w)),("supports","base",support(w)),("stock","base",stock(w))]
    if occupied:
        specs.append(("stocker","base",stocker(w)))
    return {"assets":[dict(name=n,type=k,file=w.save(s)) for n,k,s in specs]}
