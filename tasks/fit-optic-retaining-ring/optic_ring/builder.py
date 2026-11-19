"""Native collision geometry for a lens tube, separate optic, ring and spanner."""

import math

import numpy as np

from . import config as C
from .usd_scene import Writer

ANODIZED = (.14, .18, .22)
BLACK = (.07, .10, .13)
STEEL = (.61, .66, .69)
EDGE = (.45, .69, .72)
OPTICAL = (.30, .58, .65)
AMBER = (.60, .31, .11)
WHITE = (.84, .87, .85)
INSPECTION = (.45, .51, .50)


def material(w, stage, name, friction):
    base = w.Shade.Material.Define(stage, "/Asset/" + name)
    api = w.Phy.MaterialAPI.Apply(base.GetPrim())
    api.CreateStaticFrictionAttr(friction)
    api.CreateDynamicFrictionAttr(.85 * friction)
    api.CreateRestitutionAttr(0.)
    return base


def finish(w, stage, shape, color, surface):
    w._finish(stage, shape, color, True, contact=.00015)
    if surface:
        w.Shade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(
            surface, materialPurpose="physics")
    return shape


def convex(w, stage, path, points, z0, z1, color, surface=None):
    """Extrude a convex collision solid while leaving the aperture clear."""
    v = [w.Gf.Vec3f(float(x), float(y), z) for z in (z0, z1) for x, y in points]
    n = len(points)
    faces = [list(range(n-1, -1, -1)), list(range(n, 2*n))]
    faces.extend([[i, (i+1) % n, (i+1) % n+n, i+n] for i in range(n)])
    mesh = w.Geom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(v)
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateSubdivisionSchemeAttr("none")
    finish(w, stage, mesh, color, surface)
    w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")


def polar(r, angle):
    a = math.radians(angle)
    return r*math.cos(a), r*math.sin(a)


def ring(w, stage, root, name, r0, r1, z0, z1, color, spans=((0, 360),), surface=None):
    for j, (start, end) in enumerate(spans):
        count = math.ceil((end - start) / 12.)
        angles = np.linspace(start, end, count+1)
        for i, (a0, a1) in enumerate(zip(angles[:-1], angles[1:])):
            p = [polar(r, a) for r, a in ((r0, a0), (r1, a0), (r1, a1), (r0, a1))]
            convex(w, stage, f"{root}/{name}_{j}_{i}", p, z0, z1, color, surface)


def cylinder(w, stage, path, radius, length, z, color, surface=None):
    shape = w.Geom.Cylinder.Define(stage, path)
    shape.CreateRadiusAttr(radius)
    shape.CreateHeightAttr(length)
    shape.CreateAxisAttr("Z")
    shape.AddTranslateOp().Set(w.Gf.Vec3d(0., 0., z))
    return finish(w, stage, shape, color, surface)


def joint(w, stage, t, name, body0, body1, local0, low, high):
    definition = t.Define(stage, body0 + "/" + name)
    definition.CreateAxisAttr("Z")
    definition.CreateBody0Rel().SetTargets([body0])
    definition.CreateBody1Rel().SetTargets([body1])
    definition.CreateLocalPos0Attr(w.Gf.Vec3f(*local0))
    definition.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    definition.CreateLowerLimitAttr(low)
    definition.CreateUpperLimitAttr(high)
    definition.CreateCollisionEnabledAttr(False)
    return definition


def make_tube(w):
    stage = w.stage("lens_tube")
    root = w.body(stage, "/Asset/LensTube", .096, articulation=True)
    physx = w.Px.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(root))
    physx.CreateEnabledSelfCollisionsAttr(False)
    physx.CreateSolverPositionIterationCountAttr(48)
    carrier = w.body(stage, "/Asset/ThreadAxialCarrier", .012, (0., 0., C.RING_BOTTOM))
    moving = w.body(stage, "/Asset/RetainingRing", .018, (0., 0., C.RING_BOTTOM))
    # This collar has no fixed connection to the light pad or bench.
    shell = material(w, stage, "AnodizedAluminum", .72)
    insert = material(w, stage, "PolishedMetal", .28)
    ring(w, stage, root, "Wall", C.TUBE_BORE, C.TUBE_OUTER, .0001, C.TUBE_HEIGHT,
         ANODIZED, surface=shell)
    ring(w, stage, root, "Shoulder", C.CLEAR_RADIUS + .00014, C.TUBE_BORE,
         C.LENS_SEAT-.0015, C.LENS_SEAT, BLACK, surface=insert)
    ring(w, stage, root, "UpperRim", C.TUBE_BORE, C.TUBE_OUTER+.0004,
         C.TUBE_HEIGHT-.0011, C.TUBE_HEIGHT, STEEL, surface=shell)
    ring(w, stage, root, "BottomRib", C.TUBE_BORE, C.TUBE_OUTER+.0004,
         .001, .003, BLACK, surface=shell)
    ring(w, stage, moving, "LowerLoadFace", C.CLEAR_RADIUS+.00011, .02563,
         0., C.SLOT_BOTTOM, STEEL, surface=insert)
    slot = math.degrees(math.asin(C.SLOT_HALF_WIDTH / C.CLEAR_RADIUS))
    ring(w, stage, moving, "UpperSlotCheeks", C.CLEAR_RADIUS+.00011, .02563,
         C.SLOT_BOTTOM, C.RING_THICKNESS, STEEL,
         ((slot, 180-slot), (180+slot, 360-slot)), insert)
    axial = joint(w, stage, w.Phy.PrismaticJoint, "Lead", root, carrier,
                  (0., 0., C.RING_BOTTOM), -C.THREAD_TRAVEL, .000075)
    drive = w.Phy.DriveAPI.Apply(axial.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(75000.)
    drive.CreateDampingAttr(90.)
    drive.CreateMaxForceAttr(C.THREAD_FORCE_LIMIT)
    spin = joint(w, stage, w.Phy.RevoluteJoint, "Thread", carrier, moving,
                 (0., 0., 0.), -163., 0.65)
    # A compliant friction contact at the thread flank, limited to the
    # breakaway torque even if the tool holds the ring under load.
    brake = w.Phy.DriveAPI.Apply(spin.GetPrim(), "angular")
    brake.CreateTypeAttr("force")
    brake.CreateStiffnessAttr(.047)
    brake.CreateDampingAttr(.0011)
    brake.CreateMaxForceAttr(.032)
    spin_physx = w.Px.PhysxJointAPI.Apply(spin.GetPrim())
    spin_physx.CreateJointFrictionAttr(.06)
    return w.save(stage)


def make_lens(w):
    stage = w.stage("optic")
    root = w.body(stage, "/Asset/Optic", C.LENS_MASS, ccd=True)
    glass = material(w, stage, "OpticalGlass", .19)
    optic = cylinder(w, stage, root + "/Body", C.LENS_RADIUS, C.LENS_THICKNESS, 0., EDGE, glass)
    from pxr import Sdf

    appearance = w.Shade.Material.Define(stage, "/Asset/GlassAppearance")
    shader = w.Shade.Shader.Define(stage, "/Asset/GlassAppearance/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(*OPTICAL))
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(.47)
    shader.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.50)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.105)
    appearance.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    w.Shade.MaterialBindingAPI.Apply(optic.GetPrim()).Bind(appearance)
    return w.save(stage)


def make_spanner(w):
    stage = w.stage("spanner")
    root = w.body(stage, "/Asset/Spanner", .108, ccd=True)
    tips = material(w, stage, "GroundSteel", .27)
    grip = material(w, stage, "Grip", 1.2)
    for sign, key in ((1, "Positive"), (-1, "Negative")):
        pts = [
            (sign*(C.TOOL_TEETH_RADIUS + x), sign*y)
            for x, y in ((-.00053, -.00102), (.00053, -.00102),
                         (.00053, .00102), (-.00053, .00102))
        ]
        convex(w, stage, root + "/Tooth" + key, pts, 0., .0064, STEEL, tips)
        x = sign * .0226
        p = [(x-.00155, -.0031), (x+.00155, -.0031), (x+.00155, .0031), (x-.00155, .0031)]
        convex(w, stage, root + "/Head" + key, p, .0062, .018, STEEL, tips)
        p = [(sign*.002, sign*-.004), (sign*.024, sign*-.004),
             (sign*.024, sign*.004), (sign*.002, sign*.004)]
        convex(w, stage, root + "/Arm" + key, p, .017, .025, STEEL, tips)
    cylinder(w, stage, root + "/Shank", .0043, .051, .049, STEEL, tips)
    flats = [polar(.014, a + 30.) for a in range(0, 360, 60)]
    convex(w, stage, root + "/Handle", flats, .063, .104, AMBER, grip)
    for i in range(4):
        w.cylinder(stage, f"{root}/Knurl{i}", .0118, .001,
                   (0., 0., .073 + .006*i), BLACK, collide=False)
    return w.save(stage)


def make_bench(w):
    stage = w.stage("bench")
    w.box(stage, "/Asset/WorkSurface", (.620, .770, .036),
          (.30, 0., C.TABLE-.018), (.29, .36, .37))
    x, y, _ = C.TUBE
    p = material(w, stage, "PTFE", .07)
    w.Px.PhysxMaterialAPI.Apply(p.GetPrim()).CreateFrictionCombineModeAttr("multiply")
    pad = w.Geom.Cube.Define(stage, "/Asset/InspectionPad")
    pad.CreateSizeAttr(1)
    pad.AddTranslateOp().Set(w.Gf.Vec3d(x, y, C.TABLE+.009))
    pad.AddScaleOp().Set(w.Gf.Vec3f(.092, .090, .018))
    finish(w, stage, pad, WHITE, p)
    for axis in range(2):
        size = (.067, .0014, .00005) if axis == 0 else (.0014, .067, .00005)
        w.box(stage, f"/Asset/IlluminatorFiducial{axis}", size,
              (x, y, C.FIXTURE_TOP+.00001), INSPECTION, collide=False)
    x, y, _ = C.TOOL
    # Two opposite channels support the upright spanner by its fork cheeks.
    for sign in (-1, 1):
        w.box(stage, f"/Asset/RestSide_{sign}", (.014, .009, .019),
              (x+sign*.0226, y+sign*.0155, C.TABLE+.0095), BLACK)
        w.box(stage, f"/Asset/RestOther_{sign}", (.014, .009, .019),
              (x+sign*.0226, y-sign*.0155, C.TABLE+.0095), BLACK)
    w.box(stage, "/Asset/RestLeft", (.006, .037, .015),
          (x+.037, y, C.TABLE+.0075), BLACK)
    w.box(stage, "/Asset/RestRight", (.006, .037, .015),
          (x-.037, y, C.TABLE+.0075), BLACK)
    return w.save(stage)


def create_assets(directory):
    w = Writer(directory)
    return {
        "settle_actions": 16,
        "assets": [
            {"name": name, "type": kind, "file": file, "position": position}
            for name, kind, file, position in (
                ("tube", "articulation", make_tube(w), C.TUBE),
                ("optic", "rigid", make_lens(w), C.TUBE+[0., 0., C.LENS_CENTER]),
                ("spanner", "rigid", make_spanner(w), C.TOOL),
                ("bench", "base", make_bench(w), np.zeros(3)),
            )
        ],
    }
