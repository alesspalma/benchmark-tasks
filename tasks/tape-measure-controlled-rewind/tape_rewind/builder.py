"""Generate a native free case, end hook, sliding blade and spring-loaded button."""

from . import config as C
from .usd_scene import Writer

MATERIAL_CASE = (.025, .032, .041)
MATERIAL_SHELL = (.185, .48, .12)
MATERIAL_STEEL = (.62, .67, .72)
MATERIAL_BLADE = (.92, .67, .10)
MATERIAL_BUTTON = (.86, .28, .08)


def _slide(w, stage, name, base, body, axis, local, limits):
    slide = w.Phy.PrismaticJoint.Define(stage, "/Asset/" + name)
    slide.CreateBody0Rel().SetTargets([base])
    slide.CreateBody1Rel().SetTargets([body])
    slide.CreateAxisAttr(axis)
    slide.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, local)))
    slide.CreateLocalPos1Attr(w.Gf.Vec3f(0., 0., 0.))
    slide.CreateLowerLimitAttr(limits[0])
    slide.CreateUpperLimitAttr(limits[1])
    slide.CreateCollisionEnabledAttr(False)
    return slide


def create_assets(directory):
    w = Writer(directory)
    stage = w.stage("tape")
    case = w.body(stage, "/Asset/Case", .340, articulation=True, ccd=True)
    w.Px.PhysxArticulationAPI.Get(stage, case).CreateEnabledSelfCollisionsAttr(False)
    hook = w.body(stage, "/Asset/Hook", .027, position=C.APERTURE, ccd=True)
    button = w.body(stage, "/Asset/Button", .011, position=C.BUTTON, ccd=True)
    width, depth, height = C.CASE_SIZE
    w.box(stage, case + "/LowerBumper", (width, depth, .018),
          (0., .002, .009), MATERIAL_CASE)
    w.box(stage, case + "/MainCase", (width, .065, height - .018),
          (0., .0035, .018 + (height - .018) / 2.), MATERIAL_CASE)
    w.box(stage, case + "/BladeGuide", (.033, .015, .011),
          (0., -.031, .0295), MATERIAL_CASE)
    for side in (-1, 1):
        w.box(stage, case + f"/Shoulder{side}", (.008, .065, .036),
              (side * (width / 2. - .003), .0035, .034), MATERIAL_SHELL)
        w.cylinder(stage, case + f"/HubCover{side}", .018, .002,
                   (side * (width / 2. + .001), .004, .038), MATERIAL_SHELL,
                   axis="X", collide=False)
    w.box(stage, case + "/ApertureLine", (.026, .001, .0017),
          (0., -.039, .024), MATERIAL_STEEL, collide=False)
    w.box(stage, case + "/Clip", (.002, .037, .027),
          (.0265, .006, .036), MATERIAL_STEEL, collide=False)
    w.box(stage, button + "/ThumbPad", (.022, .029, C.BUTTON_HEIGHT),
          (0., 0., 0.), MATERIAL_BUTTON)
    for offset in (-.008, -.004, 0., .004, .008):
        w.box(stage, button + f"/Ridge{offset}", (.019, .0013, .00065),
              (0., offset, C.BUTTON_HEIGHT / 2.), MATERIAL_CASE, collide=False)
    w.box(stage, hook + "/Tongue", (C.HOOK_WIDTH, .0024, C.HOOK_HEIGHT),
          (0., -.0026, -.006), MATERIAL_STEEL)
    w.box(stage, hook + "/ReturnLip", (C.HOOK_WIDTH, .005, .0028),
          (0., -.0048, -.014), MATERIAL_STEEL)
    # Only the terminal steel strip is rigidly coupled to the hook. The working
    # span's length is the native slider coordinate.
    w.box(stage, hook + "/BladeTip", (C.BLADE_WIDTH, .024, .0009),
          (0., .012, 0.), MATERIAL_BLADE)
    for x in (-.006, .006):
        w.cylinder(stage, hook + f"/Rivet{x}", .0015, .00045, (x, .006, .00055),
                   MATERIAL_STEEL, collide=False)
    blade = _slide(w, stage, "BladeTravel", case, hook, "Y", C.APERTURE, (-.310, 0.))
    brake = w.Phy.DriveAPI.Apply(blade.GetPrim(), "linear")
    brake.CreateTypeAttr("force")
    brake.CreateStiffnessAttr(2400.)
    brake.CreateDampingAttr(16.)
    brake.CreateMaxForceAttr(C.BRAKE_CAPACITY)
    brake.CreateTargetPositionAttr(-C.EXPOSED_AT_RESET)
    press = _slide(w, stage, "Release", case, button, "Z", C.BUTTON,
                   (-C.BUTTON_TRAVEL, 0.))
    drive = w.Phy.DriveAPI.Apply(press.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(760.)
    drive.CreateDampingAttr(4.)
    drive.CreateMaxForceAttr(8.)
    drive.CreateTargetPositionAttr(.0008)

    # The ruler face follows the exposed steel. Its marks enter the housing
    # with the moving blade. Rendering does not feed back into the reel physics.
    strip = w.Geom.Cube.Define(stage, case + "/VisibleBlade")
    strip.CreateSizeAttr(1.)
    strip.AddTranslateOp()
    strip.AddScaleOp()
    strip.CreateDisplayColorAttr([w.Gf.Vec3f(*MATERIAL_BLADE)])
    ticks = w.Geom.Mesh.Define(stage, case + "/Graduations")
    count = 61
    ticks.CreateFaceVertexCountsAttr([4] * count)
    ticks.CreateFaceVertexIndicesAttr(list(range(4 * count)))
    ticks.CreateSubdivisionSchemeAttr("none")
    ticks.CreateDoubleSidedAttr(True)
    ticks.CreateDisplayColorAttr([w.Gf.Vec3f(.035, .028, .019)])
    update_blade(stage, case, C.EXPOSED_AT_RESET)
    w.inventory()
    return {
        "assets": [{"name": "tape", "type": "articulation",
                    "file": w.save(stage), "position": C.CASE.tolist()}],
        "settle_actions": 8,
    }


def update_blade(stage, case, extension):
    from pxr import Gf, UsdGeom

    length = max(.00005, extension)
    x, y, z = C.APERTURE
    strip = UsdGeom.Xformable(stage.GetPrimAtPath(case + "/VisibleBlade"))
    translate, scale = strip.GetOrderedXformOps()
    translate.Set(Gf.Vec3d(float(x), float(y - length / 2.), float(z)))
    scale.Set(Gf.Vec3f(C.BLADE_WIDTH, length, .0008))
    points = []
    for index in range(61):
        marking = .005 * index
        tick_y = y - length + marking
        is_visible = marking < length - .0015
        tick_width = (.010 if index % 2 == 0 else .005) if is_visible else 0.
        half_thickness = .00030 if is_visible else 0.
        tick_y = min(y, tick_y)
        edge = x - C.BLADE_WIDTH / 2. + .001
        points.extend(Gf.Vec3f(px, py, z + .00045) for px, py in (
            (edge, tick_y - half_thickness), (edge + tick_width, tick_y - half_thickness),
            (edge + tick_width, tick_y + half_thickness), (edge, tick_y + half_thickness),
        ))
    mesh = UsdGeom.Mesh.Get(stage, case + "/Graduations")
    mesh.GetPointsAttr().Set(points)
