"""Native floor bucket, pedestal mounts, and compliant wet mop bundles."""
import math

import numpy as np
from yam_arena.ik import matrix_to_quat_xyzw
from yam_arena.urdf import axis_angle_matrix

from . import config as C
from .usd_scene import Writer

BLUE = (.067, .28, .52)
PALE_BLUE = (.16, .42, .68)
GRAY = (.36, .40, .42)
AZ = np.array([0., 0., 1.])


def shell(writer, stage, root, z_low, z_high, r_low, r_high, thick, color, facets=24):
    for k in range(facets):
        phi = (k + .5) * math.tau / facets
        slope = math.atan2(r_high - r_low, z_high - z_low)
        rad = (r_low + r_high) / 2
        around = axis_angle_matrix(AZ, phi)
        tilt = axis_angle_matrix(np.array([0., 1., 0.]), slope)
        writer.box(
            stage, f"{root}/Strip{k}", (thick, 2.04 * r_high * math.tan(math.pi / facets),
            math.hypot(z_high - z_low, r_high - r_low)),
            around @ [rad, 0., (z_low + z_high) / 2], color, rotation=around @ tilt,
        )


def bucket(w):
    s = w.stage("bucket")
    p = w.body(s, "/Asset/Bucket", 3.52, ccd=True)
    mass = w.Phy.MassAPI.Get(s, p)
    mass.CreateCenterOfMassAttr(w.Gf.Vec3f(0., 0., .045))
    mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(.060, .060, .050))
    w.cylinder(s, p + "/Base", .139, .008, (0, 0, .004), BLUE)
    shell(w, s, p + "/Walls", .005, C.MOUTH, .137, .176, .005, BLUE)
    w.ring(s, p + "/RolledLip", .174, .012, .011, .288, PALE_BLUE)
    shell(w, s, p + "/Cone", C.CONE_BOTTOM, C.MOUTH, C.CONE_BOTTOM_R,
          C.CONE_TOP_R, .005, GRAY)
    w.ring(s, p + "/ConeLip", .106, .014, .008, .288, (.2, .23, .24))
    for sign in (-1, 1):
        w.box(s, p + f"/Support{sign}", (.071, .025, .011), (.140 * sign, 0., .285), GRAY)
        w.cylinder(s, p + f"/Pivot{sign}", .016, .012, (.171 * sign, 0., .245), GRAY, axis="X")
    for k in range(24):
        phi = k * math.tau / 24
        x = (.175 * math.cos(phi), .175 * math.sin(phi), .248)
        w.box(s, p + f"/Bail{k}", (.009, .046, .009), x, GRAY,
              rotation=axis_angle_matrix(AZ, phi), collide=False)
    w.cylinder(s, p + "/CaughtWater", .141, .003, (0., 0., .055), (.22, .42, .48), collide=False)
    return s


def make_sliding_bundle(w, s, shaft, n):
    phi = n * math.tau / 6
    rot = axis_angle_matrix(AZ, phi)
    anchor = rot @ np.array([C.BUNDLE_R, 0, -.118])
    node = w.body(s, f"/Asset/Bundle{n}", .078, position=anchor, ccd=True)
    mass = w.Phy.MassAPI.Get(s, node)
    mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(.00019, .00019, .000025))
    cream = (.69, .63, .50)
    w.box(s, node + "/NativeContact", (.031, .041, C.BUNDLE_LENGTH), (0, 0, 0), cream,
          rotation=rot)
    w.Geom.Imageable.Get(s, node + "/NativeContact").MakeInvisible()
    for j in range(9):
        lateral = (j % 3 - 1) * .013
        radial = (j // 3 - 1) * .009
        local = rot @ np.array([radial, lateral, .005 + (j % 2) * .004])
        w.cylinder(s, node + f"/Rayon{j}", .007, .170 - .006 * (j % 2), local,
                   (.79, .76, .63) if j % 2 else cream, collide=False)
    joint = w.Phy.PrismaticJoint.Define(s, f"/Asset/Squeeze{n}")
    joint.CreateBody0Rel().SetTargets([shaft])
    joint.CreateBody1Rel().SetTargets([node])
    joint.CreateAxisAttr("X")
    joint.CreateLowerLimitAttr(-C.BUNDLE_TRAVEL)
    joint.CreateUpperLimitAttr(.001)
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*anchor))
    q = matrix_to_quat_xyzw(rot)
    orientation = w.Gf.Quatf(float(q[-1]), *map(float, q[:3]))
    joint.CreateLocalRot0Attr(orientation)
    joint.CreateLocalRot1Attr(orientation)
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(C.BUNDLE_STIFFNESS)
    drive.CreateDampingAttr(1.8)
    drive.CreateTargetPositionAttr(0.)
    drive.CreateTargetVelocityAttr(0.)
    drive.CreateMaxForceAttr(18.)


def mop(w):
    s = w.stage("mop")
    a = w.body(s, "/Asset/Shaft", .43, articulation=True, ccd=True)
    w.Px.PhysxArticulationAPI.Get(s, a).CreateEnabledSelfCollisionsAttr(False)
    mass = w.Phy.MassAPI.Get(s, a)
    mass.CreateCenterOfMassAttr(w.Gf.Vec3f(0, 0, .51))
    mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(.047, .047, .000058))
    w.cylinder(s, a + "/SteelShaft", C.SHAFT_RADIUS, C.SHAFT_LENGTH,
               (0., 0., C.SHAFT_LENGTH / 2), (.88, .91, .93))
    w.cylinder(s, a + "/ThreadedHead", .028, .071, (0., 0., -.015), (.17, .16, .14))
    w.cylinder(s, a + "/HandleGrip", .017, .13, (0., 0., C.SHAFT_LENGTH - .063), (.11, .14, .16))
    for i in range(6):
        make_sliding_bundle(w, s, a, i)
    mat = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    mat.CreateStaticFrictionAttr(.88)
    mat.CreateDynamicFrictionAttr(.58)
    return s


def mounts(w):
    s = w.stage("mounts")
    h = .75 - C.FLOOR_Z
    for side, y in [("Left", .305), ("Right", -.305)]:
        w.box(s, "/Asset/" + side + "Foot", (.272, .245, .037),
              (0., y, C.FLOOR_Z + .0185), (.24, .27, .30))
        w.box(s, "/Asset/" + side + "Post", (.163, .160, h - .054),
              (0., y, C.FLOOR_Z + h/2), (.38, .41, .46))
        w.box(s, "/Asset/" + side + "Top", (.211, .231, .025),
              (0., y, .7375), (.25, .29, .32))
    return s


def drops(w):
    s = w.stage("drops")
    for j in range(28):
        prim = w.Geom.Sphere.Define(s, f"/Asset/Drop{j}")
        prim.CreateRadiusAttr(.0026)
        prim.CreateDisplayColorAttr([w.Gf.Vec3f(.3, .61, .86)])
        prim.AddTranslateOp().Set(w.Gf.Vec3d(0., 0., -1.))
        prim.MakeInvisible()
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [
        dict(name="bucket", type="rigid", file=w.save(bucket(w)), position=C.BUCKET.tolist()),
        dict(name="mop", type="articulation", file=w.save(mop(w)), position=[*C.BUCKET[:2], C.NECK_Z]),
        dict(name="mounts", type="base", file=w.save(mounts(w))),
        dict(name="drops", type="base", file=w.save(drops(w))),
    ]
    w.inventory()
    return dict(assets=assets, settle_actions=20)
