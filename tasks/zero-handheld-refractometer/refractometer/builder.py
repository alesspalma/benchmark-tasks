"""Native rigid bodies, lid hinge and slotted zero adjustment for the optical instrument."""

from math import cos, pi, sin

import numpy as np

from . import config as C
from .motion import rotation
from .usd_scene import Writer

DARK = (.075, .095, .110)
BLACK = (.045, .055, .064)
STEEL = (.63, .69, .73)
PALE = (.77, .84, .82)
TEAL = (.06, .22, .28)
AMBER = (.92, .49, .12)
WATER = (.14, .63, .80)
BLUE = (.027, .33, .72)


def joint(w, s, cls, name, parent, child, pose, axis, lower, upper, local_rotation=None):
    value = cls.Define(s, "/Asset/" + name)
    value.CreateBody0Rel().SetTargets([parent])
    value.CreateBody1Rel().SetTargets([child])
    value.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, pose)))
    value.CreateAxisAttr(axis)
    value.CreateLowerLimitAttr(lower)
    value.CreateUpperLimitAttr(upper)
    value.CreateCollisionEnabledAttr(False)
    if local_rotation is not None:
        value.CreateLocalRot0Attr(local_rotation)
    return value


def orient(w, stage, path, axis, angle):
    q = quaternion(w, axis, angle)
    p = stage.GetPrimAtPath(path)
    p.GetAttribute("xformOp:orient").Set(
        w.Gf.Quatd(q.GetReal(), w.Gf.Vec3d(*q.GetImaginary()))
    )


def quaternion(w, axis, angle):
    v = np.zeros(3)
    v["XYZ".index(axis)] = sin(angle / 2)
    return w.Gf.Quatf(cos(angle / 2), w.Gf.Vec3f(*v))


def spring(w, p, stiffness, damping, force, target=0.):
    d = w.Phy.DriveAPI.Apply(p.GetPrim(), "linear")
    d.CreateStiffnessAttr(stiffness)
    d.CreateDampingAttr(damping)
    d.CreateMaxForceAttr(force)
    d.CreateTargetPositionAttr(target)


def make_bench(w):
    s = w.stage("bench")
    w.box(s, "/Asset/Bench", (.96, 1.17, .046), (.35, 0., C.TABLE - .023), (.53, .59, .57))
    w.box(s, "/Asset/Floor", (8., 8., .02), (0., 0., -.01), (.30, .34, .36))
    for j, x in enumerate((-.095, .78)):
        for k, y in enumerate((-.53, .53)):
            w.box(s, f"/Asset/Leg{j}{k}", (.043, .043, C.TABLE-.045),
                  (x, y, (C.TABLE-.045)/2), DARK)
    # Two open rails under the rubber barrel stop it from rolling at reset.
    for section, y in enumerate((-.046, .081)):
        for index, dx in enumerate((-.011, .011)):
            height = .018 if section == 0 else .0145
            w.box(s, f"/Asset/InstrumentRail{section}{index}", (.011, .018, height),
                  (C.INSTRUMENT[0] + dx, C.INSTRUMENT[1] + y, C.TABLE + height/2),
                  TEAL, rotation("Y", (-1)**index * .17))
    for name, origin, radius, seat_height in (
        ("Dropper", C.DROPPER, .00875, .0555),
        ("Driver", C.DRIVER, .011, .072),
    ):
        x, y = origin[:2]
        w.box(s, "/Asset/" + name + "Foot", (.047, .046, .010), (x, y, C.TABLE + .005), TEAL)
        for index in range(8):
            theta = index * pi / 4
            r = radius + .0024
            height = seat_height + .005
            w.box(s, f"/Asset/{name}Socket{index}", (.0041, .009, height),
                  (x + r*cos(theta), y + r*sin(theta), C.TABLE + height/2),
                  TEAL, rotation("Z", theta))
        # A short lower socket locates the narrow shaft without gripping it.
        if name == "Driver":
            for index in range(8):
                theta = index * pi / 4
                r = .0054
                w.box(s, f"/Asset/{name}Guide{index}", (.0022, .005, .010),
                      (x + r*cos(theta), y + r*sin(theta), C.TABLE + .046),
                      TEAL, rotation("Z", theta))
    return s


def make_instrument(w):
    s = w.stage("instrument")
    p = w.body(s, "/Asset/Tube", .131, articulation=True, ccd=True)
    w.cylinder(s, p+"/OpticalTube", .014, .100, (0., .022, 0.), TEAL, axis="Y")
    # Squared sides on the rubber handgrip are real facets of the overmold.
    w.rounded_box(s, p+"/RubberGrip", (.029, .049, .030), C.HANDLE,
                  BLACK, radius=.003, bevel=.001)
    w.cylinder(s, p+"/FocusRing", .016, .015, (0., .076, 0.), DARK, axis="Y")
    w.cylinder(s, p+"/Eyecup", .017, .017, (0., .087, 0.), BLACK, axis="Y")
    w.cylinder(s, p+"/Lens", .0114, .0010, (0., .096, 0.), BLUE, axis="Y", collide=False)
    # Prism carrier and its exposed top glass.
    w.rounded_box(s, p+"/PrismCarrier", (.026, .046, .023), (0., -.049, .001),
                  STEEL, radius=.004, bevel=.001)
    w.box(s, p+"/PrismGlass", (C.PRISM_WIDTH, C.PRISM_LENGTH, .0025),
          C.PRISM-[0., 0., .00125], (.23, .45, .53))
    w.box(s, p+"/SampleFilm", (.0001, .0001, .0006),
          C.PRISM+[0., 0., .00035], WATER, collide=False)
    w.Geom.Imageable(s.GetPrimAtPath(p+"/SampleFilm")).MakeInvisible()

    lid_angle = np.radians(C.LID_OPEN_DEG)
    lid = w.body(s, "/Asset/DaylightPlate", .006, position=C.HINGE, ccd=True)
    orient(w, s, lid, "X", lid_angle)
    # A translucent center in a small molded rim, like the hinged working cover.
    w.box(s, lid+"/Acrylic", (.025, .034, .0026), (0., .017, 0.), PALE)
    material = w.Shade.Material.Define(s, "/Asset/Acrylic")
    surf = w.Shade.Shader.Define(s, "/Asset/Acrylic/Surface")
    surf.CreateIdAttr("UsdPreviewSurface")
    from pxr import Sdf
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(.80, .92, .92))
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.12)
    surf.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(.36)
    surf.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    w.Shade.MaterialBindingAPI.Apply(s.GetPrimAtPath(lid+"/Acrylic")).Bind(material)
    for i, x in enumerate((-.013, .013)):
        w.box(s, lid+f"/Side{i}", (.0026, .034, .0028), (x, .017, 0.), PALE)
    w.box(s, lid+"/FingerEdge", (.028, .0044, .0040), (0., .033, 0.), PALE)
    j = joint(w, s, w.Phy.RevoluteJoint, "CoverAngle", p, lid, C.HINGE,
              "X", -C.LID_OPEN_DEG, 0., quaternion(w, "X", lid_angle))
    rb = w.Px.PhysxJointAPI.Apply(j.GetPrim())
    rb.CreateJointFrictionAttr(.008)

    shaft = w.body(s, "/Asset/Screw", .004, position=C.SCREW, ccd=True)
    orient(w, s, shaft, "Z", C.SCREW_INIT)
    # The two steel shoulders leave a real slot for a flat driver.
    w.cylinder(s, p+"/ScrewSeat", .0066, .005, C.SCREW+[0., 0., -.0057], DARK)
    w.cylinder(s, shaft+"/SlotBottom", .0051, .0018, (0., 0., -.0031), STEEL)
    for i, y in enumerate((-.00273, .00273)):
        path = shaft+f"/SlotWall{i}"
        w.box(s, path, (.0076, .00315, .0034), (0., y, -.0005), STEEL)
        w.Px.PhysxCollisionAPI.Get(s, path).GetContactOffsetAttr().Set(.00010)
    w.Px.PhysxCollisionAPI.Get(s, shaft+"/SlotBottom").GetContactOffsetAttr().Set(.00010)
    j = joint(w, s, w.Phy.RevoluteJoint, "ZeroAngle", p, shaft, C.SCREW,
              "Z", -118., 90., quaternion(w, "Z", C.SCREW_INIT))
    w.Px.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(.025)
    damping = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular")
    damping.CreateStiffnessAttr(0.)
    damping.CreateDampingAttr(.0060)
    damping.CreateMaxForceAttr(.019)
    damping.CreateTargetVelocityAttr(0.)
    make_reticle(w, s, p)
    return s


def make_dropper(w):
    s = w.stage("dropper")
    p = w.body(s, "/Asset/Tube", .014, articulation=True, ccd=True)
    w.cylinder(s, p+"/Barrel", .0068, .051, (0., 0., -.041), PALE)
    w.cylinder(s, p+"/Nozzle", .0023, .027, (0., 0., -.067), PALE)
    w.cylinder(s, p+"/Neck", .0095, .006, (0., 0., -.0375), PALE)
    w.rounded_box(s, p+"/BulbFixed", (.012, .026, .028), (-.0085, 0., 0.), AMBER,
                  radius=.0055, bevel=.001)
    center = (.0085, 0., 0.)
    pad = w.body(s, "/Asset/Bulb", .003, position=center, ccd=True)
    w.rounded_box(s, pad+"/BulbSqueeze", (.012, .026, .028), (0., 0., 0.), AMBER,
                  radius=.0055, bevel=.001)
    j = joint(w, s, w.Phy.PrismaticJoint, "BulbTravel", p, pad, center,
              "X", -C.BULB_TRAVEL, 0.)
    spring(w, j, stiffness=600., damping=2.1, force=12., target=.0006)
    return s


def make_driver(w):
    s = w.stage("driver")
    p = w.body(s, "/Asset/Driver", .027, ccd=True)
    w.rounded_box(s, p+"/Handle", (.022, .019, .039), (0., 0., .0015), AMBER,
                  radius=.005, bevel=.001)
    w.cylinder(s, p+"/Collar", .0115, .004, (0., 0., -.022), DARK)
    w.cylinder(s, p+"/Shaft", .002, .033, (0., 0., -.041), STEEL)
    w.box(s, p+"/Blade", (.0044, .00135, .0060), (0., 0., -.060), STEEL)
    w.Px.PhysxCollisionAPI.Get(s, p+"/Blade").GetContactOffsetAttr().Set(.00010)
    return s


def create_assets(directory):
    w = Writer(directory)
    driver_quat = tuple([0., 0., sin(C.DRIVER_REST_YAW/2), cos(C.DRIVER_REST_YAW/2)])
    stages = [("bench", "base", make_bench(w), (0, 0, 0), (0, 0, 0, 1)),
              ("instrument", "articulation", make_instrument(w), C.INSTRUMENT, (0, 0, 0, 1)),
              ("dropper", "articulation", make_dropper(w), C.DROPPER,
               (0., 0., sin(C.DROPPER_YAW / 2), cos(C.DROPPER_YAW / 2))),
              ("driver", "rigid", make_driver(w), C.DRIVER, driver_quat)]
    w.inventory()
    return {"assets": [
        {"name": name, "type": kind, "file": w.save(stage), "position": pos, "quaternion": quat}
        for name, kind, stage, pos, quat in stages
    ], "settle_actions": 12}


def update_film(stage, prefix, water_ml, spread):
    from pxr import Gf, UsdGeom

    prim = stage.GetPrimAtPath(prefix + "/instrument/Tube/SampleFilm")
    vis = UsdGeom.Imageable(prim)
    if water_ml < .010:
        vis.MakeInvisible()
        return
    vis.MakeVisible()
    scale = next(op for op in UsdGeom.Xformable(prim).GetOrderedXformOps()
                 if op.GetOpType() == UsdGeom.XformOp.TypeScale)
    span = min(1., water_ml / .108) ** .5
    size = (.019, .028) if spread else (.008, .008)
    scale.Set(Gf.Vec3f(span*size[0], span*size[1], .0006))


def aperture(radius, cut=-1.):
    points = [
        (radius*cos(k*pi/36), radius*sin(k*pi/36))
        for k in range(72)
        if radius*sin(k*pi/36) >= cut
    ]
    if -radius < cut < radius:
        x = (radius**2 - cut**2)**.5
        points += [(-x, cut), (x, cut)]
        points.sort(key=lambda p: np.arctan2(p[1]-(radius+cut)/2, p[0]))
    return points


def reticle_mesh(w, s, path, color, front, radius):
    mesh = w.Geom.Mesh.Define(s, path)
    points = aperture(radius)
    set_reticle_points(mesh, points, front)
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateSubdivisionSchemeAttr("none")
    w._finish(s, mesh, color, False)
    return mesh


def set_reticle_points(mesh, points, front):
    mesh.CreatePointsAttr([(x, front, z) for x, z in points])
    mesh.CreateFaceVertexCountsAttr([len(points)])
    mesh.CreateFaceVertexIndicesAttr(list(range(len(points))))


def make_reticle(w, s, tube):
    reticle_mesh(w, s, tube + "/OpticalField", (.90, .93, .92), .0972, .0126)
    blue = reticle_mesh(w, s, tube + "/OpticalBoundary", BLUE, .09728, .0126)
    w.Geom.Imageable(blue.GetPrim()).MakeInvisible()
    ink = (.03, .054, .069)
    for i in range(12):
        z = -.0042 + i*.00085
        length = .0093 if i == 0 else (.004 if i % 5 == 0 else .0018)
        w.box(s, tube + f"/Graduation{i}", (length, .00018, .00038),
              (.0018 + length/2, .09741, z), ink, collide=False)
    # A small zero adjacent to the main line depicts the actual reticle index.
    for index, (x, z, lx, lz) in enumerate((
        (.0003, -.00342, .00112, .00025), (.0003, -.00498, .00112, .00025),
        (-.00023, -.00420, .00025, .00155), (.00083, -.00420, .00025, .00155),
    )):
        w.box(s, tube + f"/ZeroMark{index}", (lx, .00018, lz),
              (x, .09741, z), ink, collide=False)


def show_reticle(stage, prefix, reading):
    from pxr import UsdGeom

    mesh = UsdGeom.Mesh.Get(stage, prefix + "/instrument/Tube/OpticalBoundary")
    view = UsdGeom.Imageable(mesh.GetPrim())
    if reading is None:
        view.MakeInvisible()
        return
    cut = -.0042 + .00425 * np.clip(reading, -1.65, 3.60)
    set_reticle_points(mesh, aperture(.0126, cut), .09728)
    view.MakeVisible()
