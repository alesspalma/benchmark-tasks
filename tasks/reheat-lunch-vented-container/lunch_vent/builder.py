"""Full-size box microwave, freely movable food bowl, and native controls."""

import numpy as np
from yam_arena import ik

from . import config as C
from .motion import rotation
from .usd_scene import Writer

STEEL = (.38, .43, .46)
SILVER = (.69, .74, .75)
ENAMEL = (.80, .82, .77)
BLACK = (.065, .080, .083)
GREEN = (.13, .57, .30)
TUB = (.76, .15, .085)
LID = (.89, .78, .58)
VENT = (.89, .55, .105)


def rest(w):
    s = w.stage("bench")
    w.box(s, "/Asset/Floor", (4., 4., .02), (.40, 0., -.01), (.37, .41, .43))
    w.rounded_box(s, "/Asset/Bench", (1.04, 1.18, .036), (.64, 0., C.TABLE_Z - .018),
                  (.62, .49, .35), .013, .002)
    w.box(s, "/Asset/BackWall", (.02, 1.42, .78), (1.13, 0., 1.03),
          (.71, .74, .72), collide=False)
    return s


def angular(w, stage, name, parent, child, origin, axis, limits, friction, initial_yaw=0.):
    joint = w.Phy.RevoluteJoint.Define(stage, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, origin)))
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(limits[0])
    joint.CreateUpperLimitAttr(limits[1])
    joint.CreateCollisionEnabledAttr(False)
    w.Px.PhysxJointAPI.Apply(joint.GetPrim()).CreateJointFrictionAttr(friction)
    if initial_yaw:
        q = ik.matrix_to_quat_xyzw(rotation("Z", np.radians(initial_yaw)))
        r = w.Gf.Quatd(float(q[3]), w.Gf.Vec3d(*map(float, q[:3])))
        w.Geom.Xformable(stage.GetPrimAtPath(child)).GetOrderedXformOps()[1].Set(r)
        joint.CreateLocalRot0Attr(w.Gf.Quatf(r))
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateDampingAttr(.012)


def translucent(w, stage, path, opacity, color):
    from pxr import Sdf

    mat = w.Shade.Material.Define(stage, str(path) + "Material")
    surf = w.Shade.Shader.Define(stage, str(path) + "Material/Surface")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(*color))
    surf.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.24)
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    w.Shade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(mat)


def text_sign(w, s, path, word, points, background, foreground):
    """Print an in-scene control label on the physical appliance surface."""
    from PIL import Image, ImageDraw, ImageFont
    from pxr import Sdf

    image = Image.new("RGB", (256, 68), background)
    draw = ImageDraw.Draw(image)
    draw.text((128, 34), word, fill=foreground, anchor="mm", font=ImageFont.load_default(44))
    texture = w.directory / (word + ".png")
    image.save(texture)
    mesh = w.Geom.Mesh.Define(s, path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateSubdivisionSchemeAttr("none")
    coords = w.Geom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, "vertex"
    )
    coords.Set([(0, 0), (1, 0), (1, 1), (0, 1)])
    mat = w.Shade.Material.Define(s, str(path) + "Material")
    reader = w.Shade.Shader.Define(s, str(path) + "Material/UV")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    rgb = w.Shade.Shader.Define(s, str(path) + "Material/Texture")
    rgb.CreateIdAttr("UsdUVTexture")
    rgb.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(texture))
    rgb.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    rgb.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    shader = w.Shade.Shader.Define(s, str(path) + "Material/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb.ConnectableAPI(), "rgb")
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.85)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    w.Shade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)


def microwave(w):
    s = w.stage("microwave")
    oven = w.body(s, "/Asset/Oven", 10.8, fixed=True, articulation=True)
    w.box(s, oven + "/Base", (.336, .433, .034), (.619, -.042, .769), STEEL)
    w.box(s, oven + "/EnamelFloor", (.287, .300, .006), (.600, 0., .786), ENAMEL)
    w.cylinder(s, oven + "/GlassTurntable", C.TRAY_RADIUS, .006,
               C.TRAY_CENTER - [0., 0., .005], (.63, .76, .75))
    mat_name = oven + "/SiliconeTurntableMat"
    w.cylinder(s, mat_name, .127, .002, C.TRAY_CENTER - [0., 0., .001], (.27, .33, .33))
    mat = w.Shade.Material.Define(s, "/Asset/Silicone")
    physical = w.Phy.MaterialAPI.Apply(mat.GetPrim())
    physical.CreateStaticFrictionAttr(1.20)
    physical.CreateDynamicFrictionAttr(.95)
    w.Px.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr("max")
    w.Shade.MaterialBindingAPI.Apply(s.GetPrimAtPath(mat_name)).Bind(
        mat, materialPurpose="physics"
    )
    w.box(s, oven + "/Roof", (.336, .433, .024), (.619, -.042, .999), STEEL)
    for label, y in (("Left", .165), ("Right", -.165)):
        w.box(s, oven + "/" + label + "Wall", (.336, .023, .216), (.619, y, .884), ENAMEL)
    w.box(s, oven + "/Back", (.028, .433, .216), (.777, -.042, .884), ENAMEL)
    w.box(s, oven + "/ControlPanel", (.316, .085, .216), (.603, -.214, .884), BLACK)
    w.rounded_box(s, oven + "/Display", (.006, .062, .040), (.443, -.206, .946),
                  BLACK, .0028, .0006, collide=False)
    w.box(s, oven + "/RunIndicator", (.002, .035, .012), (.438, -.206, .950),
          (.10, .16, .13), collide=False)
    for z in (.916, .900, .884):
        for y in (-.228, -.208, -.188):
            w.box(s, oven + f"/Key_{int(1000*z)}_{int(-1000*y)}", (.004, .011, .008),
                  (.442, y, z), SILVER, collide=False)
    w.box(s, oven + "/ControlScreenInset", (.004, .062, .010), (.441, -.207, .841),
          BLACK, collide=False)

    start = w.body(s, "/Asset/Start", .010, C.START_POSITION)
    w.rounded_box(s, start + "/Cap", (.010, .038, .020), (0., 0., 0.), GREEN, .003, .001)
    text_sign(w, s, start + "/Legend", "START", [
        (-.0053, .017, -.005), (-.0053, -.017, -.005),
        (-.0053, -.017, .005), (-.0053, .017, .005),
    ], (27, 121, 65), (250, 245, 220))
    slider = w.Phy.PrismaticJoint.Define(s, "/Asset/StartSwitch")
    slider.CreateBody0Rel().SetTargets([oven])
    slider.CreateBody1Rel().SetTargets([start])
    slider.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, C.START_POSITION)))
    slider.CreateAxisAttr("X")
    slider.CreateLowerLimitAttr(0.)
    slider.CreateUpperLimitAttr(C.START_TRAVEL)
    slider.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(slider.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(240.)
    drive.CreateDampingAttr(2.4)

    door = w.body(s, "/Asset/Door", 1.25, C.DOOR_PIVOT, ccd=True)
    for name, dims, pos in (
        ("UpperRail", (.016, .316, .042), (-.010, -.154, .081)),
        ("LowerRail", (.016, .316, .032), (-.010, -.154, -.082)),
        ("HingeStile", (.016, .035, .172), (-.010, -.014, 0.)),
        ("HandleStile", (.016, .056, .172), (-.010, -.283, 0.)),
    ):
        w.box(s, door + "/" + name, dims, pos, STEEL)
    glass = door + "/Window"
    w.box(s, glass, (.008, .222, .121), (-.010, -.142, -.006), (.12, .17, .17))
    translucent(w, s, glass, .40, (.09, .14, .15))
    for z in (-.067, .066):
        w.box(s, door + "/HandleMount" + str(z), (.035, .023, .022),
              (-.026, -.289, z), SILVER)
    w.cylinder(s, door + "/BarGrip", .010, .147, (-.046, -.289, 0.), SILVER)
    angular(w, s, "DoorHinge", oven, door, C.DOOR_PIVOT, "Z",
            (0., C.DOOR_OPEN_DEG), .085, -C.DOOR_OPEN_DEG)
    return s


def cover_section(w, s, path, xmin, xmax, ymin, ymax):
    """A convex cropped disc segment leaves the steam port physically open."""
    polygon = [.0735 * np.array([np.cos(a), np.sin(a)])
               for a in np.linspace(0., 2. * np.pi, 48, endpoint=False)]
    for axis, limit, direction in ((0, xmin, 1), (0, xmax, -1), (1, ymin, 1), (1, ymax, -1)):
        clipped = []
        for a, b in zip(polygon, polygon[1:] + polygon[:1]):
            a_in = direction * (a[axis] - limit) >= 0
            b_in = direction * (b[axis] - limit) >= 0
            if a_in:
                clipped.append(a)
            if a_in != b_in:
                clipped.append(a + (b-a) * (limit-a[axis]) / (b[axis]-a[axis]))
        polygon = clipped
    n = len(polygon)
    faces = [list(reversed(range(n))), list(range(n, 2*n))]
    for i in range(n):
        j = (i+1) % n
        faces.append([i, j, j+n, i+n])
    mesh = w.Geom.Mesh.Define(s, path)
    mesh.CreatePointsAttr([(float(p[0]), float(p[1]), z)
                           for z in (.078, .083) for p in polygon])
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateSubdivisionSchemeAttr("none")
    w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    w._finish(s, mesh, LID, True)


def bowl(w):
    s = w.stage("lunch_bowl")
    tub = w.body(s, "/Asset/Bowl", .34, articulation=True, ccd=True)
    wall = tub + "/PolypropyleneBowl"
    w.cylinder(s, wall, C.BOWL_RADIUS, .060, (0., 0., .030), TUB)
    w.cylinder(s, tub + "/ReusableFoodLoad", .061, .031, (0., 0., .052),
               (.89, .52, .26), collide=False)
    for i, theta in enumerate(np.linspace(0., 2*np.pi, 24, endpoint=False)):
        r = rotation("Z", theta)
        w.box(s, tub + f"/Collar{i}", (.008, .021, .023), r @ [.070, 0., .0715],
              TUB, r)
    # Continuous lid rim with an actual open rocker well and through port.
    cover_section(w, s, tub + "/LidFore", -.075, -.063, -.075, .075)
    cover_section(w, s, tub + "/LidAft", .003, .075, -.075, .075)
    cover_section(w, s, tub + "/LidLeft", -.063, .003, -.006, .075)
    cover_section(w, s, tub + "/LidRight", -.063, .003, -.075, -.036)
    # The cap sits on an actual through-port. The adjacent rocker pocket
    # has a blind bottom, leaving no second path around the closed cap.
    for part, x0, x1, y0, y1 in (
        ("PortForeSeat", -.063, -.059, -.036, -.006),
        ("RockerPocketBottom", -.042, .003, -.036, -.006),
        ("PortRightSeat", -.059, -.042, -.036, -.030),
        ("PortLeftSeat", -.059, -.042, -.012, -.006),
    ):
        w.box(s, tub + "/" + part, (x1-x0, y1-y0, .005),
              ((x0+x1)/2, (y0+y1)/2, .0775), LID)
    handle_yaw = rotation("Z", np.radians(C.HANDLE_YAW))
    for part, size, pos in (
        ("HandleTop", (.037, .043, .010), (0., C.HANDLE_RADIUS-.004, C.HANDLE_HEIGHT)),
        ("HandleBase", (.027, .042, .009), (0., C.HANDLE_RADIUS-.008, .021)),
        ("HandleOutside", (.029, .009, .050), (0., C.HANDLE_RADIUS+.016, .046)),
    ):
        w.box(s, tub + "/" + part, size, handle_yaw @ pos, TUB, handle_yaw)
    for side in (-1, 1):
        w.box(s, tub + ("/LidClipLeft" if side > 0 else "/LidClipRight"),
              (.030, .009, .022), (.020, side*.069, .073), TUB)
    text_sign(w, s, tub + "/VentLegend", "VENT", [
        (.004, .034, .0833), (.004, -.034, .0833),
        (.019, -.034, .0833), (.019, .034, .0833),
    ], (175, 34, 16), (255, 240, 198))

    vent = w.body(s, "/Asset/Vent", .009, C.VENT_PIVOT, ccd=True)
    w.box(s, vent + "/SealFace", (.030, .030, .006), (-.016, 0., .000), VENT)
    w.box(s, vent + "/Rocker", (.025, .025, .009), (.008, 0., .008), VENT)
    w.rounded_box(s, vent + "/PressTail", (.016, .029, .017), C.VENT_TAIL,
                  (.92, .70, .18), .003, .0008)
    angular(w, s, "SteamVent", tub, vent, C.VENT_PIVOT, "Y",
            (0., C.VENT_OPEN_DEG), .006)
    return s


def create_assets(directory):
    w = Writer(directory)
    parts = [
        {"name": "bench", "type": "base", "file": w.save(rest(w))},
        {"name": "microwave", "type": "articulation", "file": w.save(microwave(w)),
         "joints": {"DoorHinge": 0., "StartSwitch": 0.}},
        {"name": "lunch_bowl", "type": "articulation", "file": w.save(bowl(w)),
         "position": C.BOWL_RESET.tolist(), "joints": {"SteamVent": 0.}},
    ]
    w.inventory()
    return {"assets": parts, "settle_actions": 14}
