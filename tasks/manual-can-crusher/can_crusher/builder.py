"""Native contacts on the lever, closed slider loop and thin-walled can."""

import math

import numpy as np

from . import config as C
from .usd_scene import Writer


def inertia(w, stage, body, values):
    w.Phy.MassAPI.Get(stage, body).CreateDiagonalInertiaAttr(w.Gf.Vec3f(*values))


def joint(w, stage, name, cls, body0, body1, axis, p0, p1, limits, external=False):
    j = cls.Define(stage, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([body0])
    j.CreateBody1Rel().SetTargets([body1])
    j.CreateAxisAttr(axis)
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*p0))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(*p1))
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    j.CreateCollisionEnabledAttr(False)
    if external:
        j.CreateExcludeFromArticulationAttr(True)
    return j


def machine(w):
    stage = w.stage("crusher")
    mount = w.body(stage, "/Asset/Mount", 6., fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(stage, mount).CreateEnabledSelfCollisionsAttr(False)
    handle = w.body(stage, "/Asset/Handle", .23, position=C.PIVOT)
    upper_origin = C.PIVOT + [0., 0., C.DRIVE_RADIUS]
    upper = w.body(stage, "/Asset/Upper", .17, position=upper_origin)
    pin = w.body(stage, "/Asset/Pin", .03, position=upper_origin)
    inertia(w, stage, handle, (.006, .010, .004))
    inertia(w, stage, upper, (.00038, .00027, .00048))
    inertia(w, stage, pin, (.00001, .00001, .00001))
    frame = (.075, .085, .095)
    lever = (.15, .17, .18)
    steel = (.38, .40, .42)
    rubber = w.Shade.Material.Define(stage, "/Asset/Rubber")
    rubber_props = w.Phy.MaterialAPI.Apply(rubber.GetPrim())
    rubber_props.CreateStaticFrictionAttr(1.0)
    rubber_props.CreateDynamicFrictionAttr(.80)
    rubber_props.CreateRestitutionAttr(0.)
    w.box(stage, mount + "/Back", (.015, .144, .240), (.484, 0., .804), frame)
    for side in (-1, 1):
        y = side * .070
        w.box(stage, mount + f"/HingeSide{side}", (.045, .010, .103), (.467, y, .790), frame)
        w.box(stage, handle + f"/PullMember{side}", (.013, .009, C.LEVER_LENGTH),
              (0., side * .076, C.LEVER_LENGTH / 2.), lever)
        w.cylinder(stage, mount + f"/SpringCover{side}", .014, .011,
                   C.PIVOT + [0., side * .082, 0.], steel, axis="Y", collide=False)
        w.cylinder(stage, mount + f"/Guide{side}", .003, .224,
                   C.LOWER + [.010, side * .077, .112], steel)
        w.cylinder(stage, mount + f"/Bolt{side}", .006, .009, (.475, side * .054, .906), steel,
                   axis="X", collide=False)
        for offset in (-.008, .008):
            w.box(stage, upper + f"/Slot{side}_{offset > 0}", (.145, .010, .006),
                  (-.058, side * .072, offset), steel, collide=False)
    w.cylinder(stage, handle + "/Grip", .015, .161, (0., 0., C.LEVER_LENGTH),
               (.21, .22, .23), axis="Y")
    w.cylinder(stage, handle + "/GripInset", .0157, .063, (0., 0., C.LEVER_LENGTH),
               (.31, .37, .43), axis="Y", collide=False)
    w.box(stage, upper + "/PressureFace", (.098, .140, .028), (-.044, 0., .008), frame)
    w.cylinder(stage, upper + "/BucklingBoss", .018, C.BUCKLING_BOSS_HEIGHT,
               (C.CENTER_OFFSET, 0., -C.PLATE_HALF - C.BUCKLING_BOSS_HEIGHT/2.), steel)
    w.box(stage, mount + "/BasePlate", (.098, .140, .012),
          C.LOWER + [-.044, 0., 0.], frame)
    w.box(stage, mount + "/SupportPad", (.086, .083, .0014),
          C.LOWER + [-.048, 0., .0067], (.23, .22, .22))
    for path in (handle + "/Grip", mount + "/SupportPad"):
        w.Shade.MaterialBindingAPI.Get(stage, path).Bind(rubber, materialPurpose="physics")
    # A shallow cup at the cradle locates the can while it is loaded.
    for y in (-.0455, .0455):
        w.box(stage, mount + f"/CradleSide{y > 0}", (.068, .004, .009),
              C.LOWER + [-.047, y, .0095], lever)
    w.box(stage, mount + "/CradleBack", (.006, .090, .012), C.LOWER + [-.008, 0., .010], lever)
    w.cylinder(stage, pin + "/Roller", .008, .159, (0., 0., 0.), steel, axis="Y", collide=False)

    hinge = joint(w, stage, "Lever", w.Phy.RevoluteJoint, mount, handle, "Y",
                  C.PIVOT, (0., 0., 0.), (math.degrees(C.CLOSED), 0.))
    w.Px.PhysxJointAPI.Apply(hinge.GetPrim()).CreateJointFrictionAttr(.28)
    joint(w, stage, "Slide", w.Phy.PrismaticJoint, mount, upper, "Z",
          upper_origin, (0., 0., 0.), (-.153, .002))
    joint(w, stage, "RollerSlot", w.Phy.PrismaticJoint, upper, pin, "X",
          (0., 0., 0.), (0., 0., 0.), (-C.DRIVE_RADIUS - .002, .003))
    joint(w, stage, "DrivePin", w.Phy.RevoluteJoint, handle, pin, "Y",
          (0., 0., C.DRIVE_RADIUS), (0., 0., 0.), (-180., 180.), external=True)
    return stage


def beverage_can(w):
    stage = w.stage("can")
    bottom = w.body(stage, "/Asset/Bottom", .0085, articulation=True, ccd=True)
    top = w.body(stage, "/Asset/Top", .0055, position=(0., 0., C.CAN_HEIGHT), ccd=True)
    inertia(w, stage, bottom, (5e-6, 5e-6, 5e-6))
    inertia(w, stage, top, (3e-6, 3e-6, 3e-6))
    lacquer = (.09, .34, .47)
    alloy = (.57, .59, .60)
    w.cylinder(stage, bottom + "/Heel", .032, .016, (0., 0., .008), lacquer)
    w.cylinder(stage, bottom + "/Rim", .029, .0025, (0., 0., .0015), alloy, collide=False)
    w.cylinder(stage, top + "/Shoulder", .0325, .024, (0., 0., -.013), lacquer)
    w.cylinder(stage, top + "/End", .029, .004, (0., 0., -.002), alloy)
    w.cylinder(stage, top + "/PourOpening", .006, .0003, (.008, 0., .00015),
               (.034, .039, .042), collide=False)
    w.box(stage, top + "/PullTab", (.017, .009, .001), (-.006, 0., .001), (.67, .68, .69), collide=False)
    j = joint(w, stage, "ShellFold", w.Phy.PrismaticJoint, bottom, top, "Z",
              (0., 0., C.CAN_HEIGHT), (0., 0., 0.), (C.CAN_BOTTOM_STOP - C.CAN_HEIGHT, 0.))
    # The authored friction is light. The axial shell force supplies compression resistance.
    w.Px.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(.015)
    drive = w.Phy.DriveAPI.Apply(j.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(C.AXIAL_STIFFNESS)
    drive.CreateDampingAttr(C.AXIAL_DAMPING)
    drive.CreateMaxForceAttr(C.AXIAL_FORCE_LIMIT)
    drive.CreateTargetPositionAttr(0.)
    drive.CreateTargetVelocityAttr(0.)
    mesh = w.Geom.Mesh.Define(stage, bottom + "/ThinWall")
    n = 32
    rings = 13
    faces = [(i*n+k, i*n+(k+1)%n, (i+1)*n+(k+1)%n, (i+1)*n+k)
             for i in range(rings - 1) for k in range(n)]
    mesh.CreateFaceVertexCountsAttr([4] * len(faces))
    mesh.CreateFaceVertexIndicesAttr([k for face in faces for k in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreatePointsAttr(shell_points(w.Gf, C.CAN_HEIGHT))
    mesh.CreateDisplayColorAttr([w.Gf.Vec3f(*lacquer)])
    return stage


def shell_points(gf, height):
    fraction = float(np.clip((C.CAN_HEIGHT - height) / .097, 0., 1.))
    points = []
    for i in range(13):
        t = i / 12
        radius = C.CAN_RADIUS - .003 * (t**8 + (1. - t)**8)
        for k in range(32):
            angle = math.tau * k / 32.
            ripple = .0065 * fraction * math.sin(3 * angle + i * 1.7) * math.sin(math.pi * t)
            points.append(gf.Vec3f((radius + ripple) * math.cos(angle),
                                   (radius + ripple) * math.sin(angle),
                                   .003 + (height - .009) * t))
    return points


def fixtures(w):
    stage = w.stage("fixtures")
    w.box(stage, "/Asset/MountedPanel", (.032, .58, 1.240), (.508, 0., .613), (.65, .66, .68))
    for name, (_, y, z) in C.BASES.items():
        w.box(stage, "/Asset/" + name + "Pedestal", (.160, .160, z-C.FLOOR),
              (0., y, (z+C.FLOOR)/2), (.25, .28, .29))
        w.box(stage, "/Asset/" + name + "Foot", (.220, .220, .014),
              (0., y, C.FLOOR+.007), (.18, .20, .21))
    top_x, top_y, top_z = C.CAN_START
    w.box(stage, "/Asset/LoadingShelf", (.265, .190, .018),
          (top_x+.039, top_y, top_z-.009), (.58, .46, .32))
    w.box(stage, "/Asset/ShelfPost", (.070, .110, top_z - C.FLOOR - .018),
          (top_x+.115, top_y, (top_z + C.FLOOR - .018)/2), (.44, .35, .26))
    x, y, z = C.BIN_CENTER
    d, width, h = C.BIN_DEPTH, C.BIN_WIDTH, C.BIN_HEIGHT
    color = (.08, .22, .37)
    w.box(stage, "/Asset/BinFloor", (d, width, .009), (x, y, z), color)
    for side in (-1, 1):
        w.box(stage, f"/Asset/BinEnd{side}", (.006, width, h),
              (x+side*(d-.006)/2, y, z+h/2), color)
        w.box(stage, f"/Asset/BinSide{side}", (d, .006, h),
              (x, y+side*(width-.006)/2, z+h/2), color)
    w.box(stage, "/Asset/BinShelf", (d+.015, width+.016, .019),
          (x, y, z-.015), (.63, .65, .65))
    w.box(stage, "/Asset/BinSupport", (.085, .120, z-.024-C.FLOOR),
          (.454, y, (z-.024+C.FLOOR)/2), (.39, .41, .43))
    return stage


def create_assets(directory):
    w = Writer(directory)
    stages = [("crusher", "articulation", machine(w), (0., 0., 0.)),
              ("can", "articulation", beverage_can(w), C.CAN_START.tolist()),
              ("fixtures", "base", fixtures(w), (0., 0., 0.))]
    assets = [dict(name=name, type=kind, file=w.save(stage), position=pos)
              for name, kind, stage, pos in stages]
    w.inventory()
    return {"assets": assets, "settle_actions": 20}
