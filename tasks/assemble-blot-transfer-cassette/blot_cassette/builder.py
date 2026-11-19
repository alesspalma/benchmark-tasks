"""Original trainer fixtures and articulated contact surfaces."""

import numpy as np


from . import config as C
from .usd_scene import Writer

ABS = (.072, .15, .20)
GREY = (.42, .47, .51)
TEAL = (.063, .38, .44)
WET = (.21, .64, .62)
RUBBER = (.17, .21, .23)
GOLD = (.90, .47, .105)
CLEAR = (.46, .71, .76)


def slider(w, s, name, fixed, moving, origin, axis, limits, stiffness, damping, cap):
    joint = w.Phy.PrismaticJoint.Define(s, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([fixed])
    joint.CreateBody1Rel().SetTargets([moving])
    joint.CreateAxisAttr(axis)
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, origin)))
    joint.CreateLowerLimitAttr(limits[0])
    joint.CreateUpperLimitAttr(limits[1])
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateTargetPositionAttr(0.)
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateMaxForceAttr(cap)
    return joint


def cassette(w):
    from pxr import Sdf

    s = w.stage("cassette")
    base = w.body(s, "/Asset/Base", .9, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(s, base).CreateEnabledSelfCollisionsAttr(False)
    x, y, z = C.ORIGIN
    w.box(s, base + "/Stand", (.237, .163, z - C.TABLE),
          (x+.023, y-.023, (z+C.TABLE)/2), ABS)
    w.box(s, base + "/CassetteTray", (.139, .106, .009), (x, y, z+.0045), GREY)
    for side in (-1, 1):
        w.box(s, base + f"/Guide{side}", (.124, .005, .015),
              (x+.130, y+side*.054, z+.019), ABS)
    w.box(s, base + "/BackingPad", (.106, .079, .002),
          (x, y, z+.010), TEAL, collide=False)
    carriage_pos = C.ORIGIN + [0., 0., .010]
    carriage = w.body(s, "/Asset/Carriage", .10, position=carriage_pos)
    slider(w, s, "Shear", base, carriage, carriage_pos, "Y",
           (-C.SHEAR_LIMIT, C.SHEAR_LIMIT), C.SHEAR_STIFFNESS, 3., 4.)
    cushion = w.body(s, "/Asset/Layer", .043, position=carriage_pos)
    w.box(s, cushion + "/WetPatch", (.100, .075, .004), (0., 0., .005), WET)
    # The two ends of the wet patch are visible on the backing pad.
    w.box(s, cushion + "/WetMargin", (.102, .084, .004), (0., 0., .002), TEAL)
    slider(w, s, "Compression", carriage, cushion, (0., 0., 0.), "Z",
           (-.005, .0001), C.CUSHION_STIFFNESS, 5., 18.)
    for k, (gx, gy, radius) in enumerate(C.GAPS):
        w.cylinder(s, cushion+f"/Gap{k}", radius, .0003, (gx, gy, .00718),
                   (.90, .91, .86), collide=False)
        w.cylinder(s, cushion+f"/Gap{k}/Center", radius*.66, .00035,
                   (0., 0., .00023), (.66, .86, .89), collide=False)
    lid_pos = C.ORIGIN + [C.LID_TRAVEL, 0., .030]
    lid = w.body(s, "/Asset/Cover", .13, position=lid_pos)
    w.box(s, lid + "/Grip", (.019, .070, .019), (-.058, 0., .011), GOLD)
    for side in (-1, 1):
        w.box(s, lid + f"/Side{side}", (.121, .005, .006),
              (0., side*.045, 0.), CLEAR)
    w.box(s, lid + "/FarCrossbar", (.005, .092, .006), (.058, 0., 0.), CLEAR)
    transparent = w.Shade.Material.Define(s, "/Asset/LidDisplay")
    shader = w.Shade.Shader.Define(s, "/Asset/LidDisplay/Shade")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(*CLEAR))
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(.25)
    transparent.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    w.box(s, lid + "/Window", (.112, .084, .0025), (0., 0., 0.), CLEAR)
    w.Shade.MaterialBindingAPI.Apply(s.GetPrimAtPath(lid + "/Window")).Bind(transparent)
    joint = slider(w, s, "Closing", base, lid, lid_pos, "X",
                   (-C.LID_TRAVEL, 0.), 0., 2., 5.)
    w.Px.PhysxJointAPI.Apply(joint.GetPrim()).CreateJointFrictionAttr(.018)
    return w.save(s)


def cradle(w):
    s = w.stage("cradle")
    x, y, z = C.ROLLER_START
    floor_z = z - C.ROLLER_R - .005
    w.box(s, "/Asset/Foot", (.156, .064, floor_z-C.TABLE),
          (x, y, (floor_z+C.TABLE)/2), GREY)
    for k, side in enumerate((-1, 1)):
        support = z + .0195
        w.box(s, f"/Asset/Shoulder{k}", (.016, .049, support-floor_z),
              (x+side*.066, y, (floor_z+support)/2), ABS)
    return w.save(s)


def roller(w):
    s = w.stage("roller")
    handle = w.body(s, "/Asset/Handle", .11, articulation=True, ccd=True)
    w.Px.PhysxArticulationAPI.Get(s, handle).CreateEnabledSelfCollisionsAttr(False)
    native = w.Px.PhysxRigidBodyAPI.Get(s, handle)
    native.CreateAngularDampingAttr(.8)
    w.cylinder(s, handle + "/Neck", .008, .049, (0., 0., .047), GREY)
    w.box(s, handle + "/Grip", (.020, .023, .044), (0., 0., C.GRIP_HEIGHT), GOLD)
    w.box(s, handle + "/Bridge", (.142, .017, .009), (0., 0., .0245), GREY)
    for side in (-1, 1):
        w.box(s, handle + f"/Fork{side}", (.006, .011, .027),
              (side*.055, 0., .011), GREY)
    drum = w.body(s, "/Asset/Drum", .065, ccd=True)
    w.cylinder(s, drum + "/Rubber", C.ROLLER_R, C.ROLLER_WIDTH,
               (0., 0., 0.), RUBBER, axis="X")
    joint = w.Phy.RevoluteJoint.Define(s, "/Asset/Spin")
    joint.CreateBody0Rel().SetTargets([handle])
    joint.CreateBody1Rel().SetTargets([drum])
    joint.CreateAxisAttr("X")
    joint.CreateCollisionEnabledAttr(False)
    return w.save(s)


def create_assets(directory):
    writer = Writer(directory)
    return {
        "assets": [
            dict(name="cassette", type="articulation", file=cassette(writer)),
            dict(name="cradle", type="base", file=cradle(writer)),
            dict(name="roller", type="articulation", file=roller(writer), position=C.ROLLER_START),
        ],
        "settle_actions": 12,
    }
