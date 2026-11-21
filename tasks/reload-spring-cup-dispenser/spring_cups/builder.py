"""Original millimetre-scale dispenser and free nested cup geometry."""

from math import cos, pi, sin

import numpy as np
from yam_arena import ik

from . import config as C
from .motion import rotation
from .usd_scene import Writer

COUNTER = (.52, .42, .32)
CABINET = (.17, .20, .23)
TEAL = (.13, .47, .47)
TRIM = (.09, .13, .15)
GUARD = (.25, .28, .28)
NYLON = (.84, .88, .85)
PAPER = (.92, .93, .86)


def ring_segments(w, s, path, profile, color, count=24, collide=True, omit=()):
    """Make a hollow ring from short convex sectors with an r/z profile."""
    moments = []
    for i in range(count):
        if i in omit:
            continue
        a, b = (2 * pi * j / count for j in (i, i + 1))
        points = [(r * cos(angle), r * sin(angle), z) for angle in (a, b) for r, z in profile]
        length = len(profile)
        faces = [
            list(range(length)),
            list(reversed(range(length, 2 * length))),
            *([j, j + length, (j + 1) % length + length, (j + 1) % length] for j in range(length)),
        ]
        prim = convex(w, s, f"{path}/Section{i:02}", points, faces, color, collide)
        moments.append(prim)
    return moments


def convex(w, s, path, points, faces, color, collide=True):
    mesh = w.Geom.Mesh.Define(s, path)
    mesh.CreatePointsAttr([w.Gf.Vec3f(*map(float, v)) for v in points])
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateSubdivisionSchemeAttr("none")
    if collide:
        w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    return w._finish(s, mesh, color, collide, contact=.00018).GetPrim()


def transparency(w, stage, prims, opacity):
    from pxr import Sdf

    mat = w.Shade.Material.Define(stage, "/Asset/Polycarbonate")
    surface = w.Shade.Shader.Define(stage, "/Asset/Polycarbonate/Surface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(*TEAL))
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.28)
    surface.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    mat.CreateSurfaceOutput().ConnectToSource(surface.ConnectableAPI(), "surface")
    for prim in prims:
        w.Shade.MaterialBindingAPI.Apply(prim).Bind(mat)


def bench(w):
    stage = w.stage("counter")
    w.box(stage, "/Asset/Floor", (4., 4., .02), (.40, 0., -.01), (.43, .48, .48))
    hx, hy = C.DISPENSER
    r = C.CUTOUT_RADIUS
    xmin, xmax, ymin, ymax = .075, 1.115, -.59, .59
    for j, (lo, hi, front, back) in enumerate((
        (xmin, hx - r, ymin, ymax),
        (hx + r, xmax, ymin, ymax),
        (hx - r, hx + r, ymin, hy - r),
        (hx - r, hx + r, hy + r, ymax),
    )):
        w.box(stage, f"/Asset/Counter{j}", (hi-lo, back-front, .036),
              ((lo+hi)/2, (front+back)/2, C.TABLE_Z-.018), COUNTER)
    for j, x in enumerate((.103, 1.088)):
        for k, y in enumerate((-.555, .555)):
            w.box(stage, f"/Asset/Leg{j}{k}", (.046, .046, C.TABLE_Z-.036),
                  (x, y, (C.TABLE_Z-.036)/2), CABINET)
    w.box(stage, "/Asset/Backsplash", (.030, 1.18, .145),
          (1.10, 0., C.TABLE_Z+.0725), (.23, .35, .34))
    rest = (.222, .394, .39)
    w.rounded_box(stage, "/Asset/ServingMat", (.130, .130, .002),
                  (*C.MAT, C.TABLE_Z+.001), rest, .012, .001)
    w.rounded_box(stage, "/Asset/SupplyRiser", (.101, .048, C.STOCK_ELEVATION),
                  (*C.STOCK, C.TABLE_Z+C.STOCK_ELEVATION/2), CABINET, .006, .001)
    return stage


def slide(w, s, name, parent, child, at, axis, limits, stiffness, damping, target):
    joint = w.Phy.PrismaticJoint.Define(s, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, at)))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(limits[0])
    joint.CreateUpperLimitAttr(limits[1])
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateTargetPositionAttr(target)
    drive.CreateMaxForceAttr(30.)


def dispenser(w):
    stage = w.stage("dispenser")
    root = "/Asset/Housing"
    w.Geom.Xform.Define(stage, root)
    inner = C.TUBE_RADIUS
    lower = -C.TUBE_LENGTH
    tube = ring_segments(w, stage, root+"/Tube",
                         ((inner, lower), (inner+.003, lower),
                          (inner+.003, -.008), (inner, -.008)), TEAL, collide=False)
    transparency(w, stage, tube, .38)
    for i in range(16):
        a = 2*pi*i/16
        w.box(stage, f"{root}/TubeWall{i}", (.004, .023, .448),
              ((inner+.002)*cos(a), (inner+.002)*sin(a), (lower-.008)/2),
              TEAL, rotation("Z", a))
        w.Geom.Imageable(stage.GetPrimAtPath(f"{root}/TubeWall{i}")).MakeInvisible()
    ring_segments(w, stage, root+"/Flange",
                  ((.051, -.008), (.083, -.008), (.083, .008), (.051, .008)), TRIM)
    ring_segments(w, stage, root+"/GuardFrame",
                  ((.049, .008), (.066, .008), (.062, .028), (.049, .028)), GUARD,
                  omit=(0, 7, 8, 15, 16, 23))
    w.cylinder(stage, root+"/Cap", .058, .010, (0, 0, lower+.005), TRIM)
    w.cylinder(stage, root+"/CapInset", .043, .004, (0, 0, lower-.002), TEAL, collide=False)
    return stage


def anchor(w, stage, position, angle=0.):
    root = w.body(stage, "/Asset/Mount", .25, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(stage, root).CreateEnabledSelfCollisionsAttr(False)
    w.Phy.MassAPI.Get(stage, root).CreateDiagonalInertiaAttr(w.Gf.Vec3f(.001, .001, .001))
    joint = w.Phy.FixedJoint.Get(stage, root+"/WorldFix")
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, position)))
    q = ik.matrix_to_quat_xyzw(rotation("Z", angle))
    joint.CreateLocalRot0Attr(w.Gf.Quatf(float(q[3]), w.Gf.Vec3f(*map(float, q[:3]))))
    return root


def retainer(w, index):
    stage = w.stage(f"retainer{index}")
    root = anchor(w, stage, [*C.DISPENSER, C.FINGER_Z], 2*pi*index/3)
    at = (0., 0., 0.)
    slider = w.body(stage, "/Asset/Lever", .030, at)
    shape = ((.0360, -.001), (.0430, -.003), (.0490, -.003),
             (.0490, .015), (.0460, .015), (.0360, .001))
    points = [(r, y, z) for y in (-.0080, .0080) for r, z in shape]
    n = len(shape)
    faces = [list(range(n)), list(reversed(range(n, 2*n))),
             *([j, j+n, (j+1)%n+n, (j+1)%n] for j in range(n))]
    prim = convex(w, stage, slider+"/SpringTip", points, faces, NYLON)
    mat = w.Shade.Material.Define(stage, "/Asset/LeverMaterial")
    material = w.Phy.MaterialAPI.Apply(mat.GetPrim())
    material.CreateStaticFrictionAttr(.13)
    material.CreateDynamicFrictionAttr(.11)
    w.Px.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr("min")
    w.Shade.MaterialBindingAPI.Apply(prim).Bind(mat, materialPurpose="physics")
    slide(w, stage, "RetainerTravel", root, slider, at, "X", (0., .013), 110., 2.4, 0.)
    return stage


def follower(w):
    stage = w.stage("follower")
    root = anchor(w, stage, [*C.DISPENSER, C.TABLE_Z])
    at = (0., 0., C.FOLLOWER_REST-C.TABLE_Z)
    follower = w.body(stage, "/Asset/Follower", .065, at)
    w.cylinder(stage, follower+"/FollowerDisk", .049, C.FOLLOWER_THICKNESS, (0,0,0), NYLON)
    w.cylinder(stage, follower+"/Guide", .018, .012, (0,0,-.010), TEAL)
    slide(w, stage, "FollowerTravel", root, follower, at, "Z",
          (-.370, 0.001), 25., 4.0, .017)
    return stage


def cup(w):
    stage = w.stage("paper_cup")
    root = w.body(stage, "/Asset/Cup", C.CUP_MASS, ccd=True)
    # In a top-loading station the cups enter mouth-first, with their bases up.
    invert = w.Geom.Xform.Define(stage, root+"/Surfaces")
    invert.AddTranslateOp().Set(w.Gf.Vec3d(0., 0., C.CUP_HEIGHT))
    invert.AddRotateXOp().Set(180.)
    surfaces = str(invert.GetPath())
    # A rigid thin-wall contact envelope keeps neighbouring tapered cups nested.
    wall = .00135
    base, top = C.CUP_BASE_R, C.CUP_WALL_R
    h = C.CUP_HEIGHT
    ring_segments(w, stage, surfaces+"/TaperedWall",
                  ((base-wall, .001), (base, .001), (top, h-.001), (top-wall, h-.001)), PAPER, 20)
    ring_segments(w, stage, surfaces+"/RolledRim",
                  ((top-.0013, h-.0032), (top+.0002, h-.0032),
                   (C.CUP_RIM_R, h-.0017), (C.CUP_RIM_R, h-.0002),
                   (top+.0005, h+.0002), (top-.0013, h+.0002)), PAPER, 20)
    w.cylinder(stage, surfaces+"/Bottom", base-.0004, .0025, (0,0,.0035), PAPER)
    # Two narrow original stock-paper stripes make the nested rims readable.
    for i, z in enumerate((.030, .043)):
        r = base + (top-base)*(z-.001)/(h-.002)
        ring_segments(w, stage, surfaces+f"/PrintBand{i}",
                      ((r+.00018, z), (r+.00019, z),
                       (r+.00051, z+.002), (r+.0005, z+.002)), (.14, .44, .46),
                      20, collide=False)
    return stage


def create_assets(directory):
    w = Writer(directory)
    counter = w.save(bench(w))
    tube = w.save(dispenser(w))
    follower_path = w.save(follower(w))
    cup_path = w.save(cup(w))
    assets = [
        {"name": "counter", "type": "base", "file": counter},
        {"name": "dispenser", "type": "base", "file": tube, "position": [*C.DISPENSER, C.TABLE_Z]},
        {"name": "follower", "type": "articulation", "file": follower_path,
         "position": [*C.DISPENSER, C.TABLE_Z], "joints": {"FollowerTravel": 0.}},
    ]
    for i in range(3):
        retainer_path = w.save(retainer(w, i))
        assets.append({"name": f"retainer{i}", "type": "articulation", "file": retainer_path,
                       "position": [*C.DISPENSER, C.FINGER_Z],
                       "quaternion": ik.matrix_to_quat_xyzw(rotation("Z", 2*pi*i/3)),
                       "joints": {"RetainerTravel": 0.}})
    for i in range(C.NUM_CUPS):
        p = [*C.STOCK, C.TABLE_Z + C.STOCK_ELEVATION + .0004 + C.CUP_PITCH*i]
        assets.append({"name": f"cup{i}", "type": "rigid", "file": cup_path, "position": p})
    w.inventory()
    return {"assets": assets, "settle_actions": 20}
