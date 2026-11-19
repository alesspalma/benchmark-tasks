"""Native contact geometry for the filler, glassware and retort stand."""

import numpy as np

from . import config as C, fluid
from .motion import rotation
from .usd_scene import Writer

RUBBER = (.67, .065, .055)
DARK = (.19, .032, .027)
GLASS = (.57, .74, .78)
WATER = (.025, .60, .87)
BLUE = (.045, .28, .61)
CLEAR = (.61, .75, .76)
INK = (.06, .11, .12)
STAND = (.12, .15, .18)
CLAMP = (.31, .34, .38)


def spring_slide(w, stage, name, base, axis, position, travel, direction,
                 stiffness, damping, max_force, mass):
    body = w.body(stage, f"/Asset/{name}", mass, position=position, ccd=True)
    joint = w.Phy.PrismaticJoint.Define(stage, f"/Asset/Slide{name}")
    joint.CreateBody0Rel().SetTargets([base])
    joint.CreateBody1Rel().SetTargets([body])
    joint.CreateAxisAttr(axis)
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, position)))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0., 0., 0.))
    joint.CreateLowerLimitAttr(min(0., direction * travel))
    joint.CreateUpperLimitAttr(max(0., direction * travel))
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateMaxForceAttr(max_force)
    drive.CreateTargetPositionAttr(-direction * .0005)
    return body


def water_visual(w, stage, path, radius, height=.00001, bottom=0.):
    w.cylinder(stage, path, radius, 1., (0., 0., bottom + height / 2),
               WATER, collide=False)
    w.Geom.Xformable(stage.GetPrimAtPath(path)).AddScaleOp().Set(
        w.Gf.Vec3f(1., 1., height)
    )
    if height < .00002:
        w.Geom.Imageable(stage.GetPrimAtPath(path)).MakeInvisible()


def glass_material(w, stage, tint, opacity):
    from pxr import Sdf

    path = "/Asset/GlassMaterial"
    material = w.Shade.Material.Define(stage, path)
    mdl = w.Shade.Shader.Define(stage, path + "/OmniGlass")
    mdl.SetSourceAsset("OmniGlass.mdl", "mdl")
    mdl.SetSourceAssetSubIdentifier("OmniGlass", "mdl")
    mdl.CreateInput("glass_color", Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(*tint))
    mdl.CreateInput("glass_ior", Sdf.ValueTypeNames.Float).Set(1.10)
    mdl.CreateInput("thin_walled", Sdf.ValueTypeNames.Bool).Set(True)
    mdl.CreateOutput("out", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput("mdl").ConnectToSource(mdl.ConnectableAPI(), "out")
    preview = w.Shade.Shader.Define(stage, path + "/Preview")
    preview.CreateIdAttr("UsdPreviewSurface")
    preview.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(*tint))
    preview.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.10)
    preview.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    preview.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    material.CreateSurfaceOutput().ConnectToSource(preview.ConnectableAPI(), "surface")
    return material


def bind_visual(w, stage, path, material):
    w.Shade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(material)


def cup(w, name, color, ghost):
    stage = w.stage(name)
    tint = (.60, .78, .96) if name == "source" else (.95, .97, .97)
    glass = glass_material(w, stage, tint, ghost)
    w.cylinder(stage, "/Asset/Bottom", C.CUP_RADIUS + C.CUP_WALL, C.CUP_BASE,
               (0., 0., C.CUP_BASE / 2), color)
    for i in range(24):
        theta = 2 * np.pi * i / 24
        r = C.CUP_RADIUS + C.CUP_WALL / 2
        w.box(stage, f"/Asset/Wall{i}", (.0067, C.CUP_WALL, C.CUP_HEIGHT - C.CUP_BASE),
              (r * -np.sin(theta), r * np.cos(theta), (C.CUP_HEIGHT + C.CUP_BASE) / 2),
              color, rotation("Z", theta))
        bind_visual(w, stage, f"/Asset/Wall{i}", glass)
    for z in (.014, .030, .045):
        w.box(stage, f"/Asset/Tick{z}", (.011, .001, .00075),
              (0., -C.CUP_RADIUS - .002, z), INK, collide=False)
    height = fluid.water_height(C.WATER_START) - C.CUP_BASE if name == "source" else .00001
    water_visual(w, stage, "/Asset/Water", C.CUP_RADIUS - .0004, height, C.CUP_BASE)
    return stage


def create_assets(directory):
    w = Writer(directory)
    stage = w.stage("pipette")
    mat = w.Phy.MaterialAPI.Get(stage, "/Asset/Physics")
    mat.GetStaticFrictionAttr().Set(1.15)
    mat.GetDynamicFrictionAttr().Set(.89)
    glass = glass_material(w, stage, (.93, .96, .98), .14)
    pipe = w.body(stage, "/Asset/Pipe", .057, articulation=True, ccd=True)
    w.Px.PhysxArticulationAPI.Get(stage, pipe).CreateEnabledSelfCollisionsAttr(False)
    w.cylinder(stage, pipe + "/Glass", .005, C.PIPETTE_LENGTH - .013,
               (0., 0., (C.PIPETTE_LENGTH + .013) / 2), GLASS)
    bind_visual(w, stage, pipe + "/Glass", glass)
    w.cylinder(stage, pipe + "/Outlet", .00135, .018, (0., 0., .009), GLASS)
    bind_visual(w, stage, pipe + "/Outlet", glass)

    for index in range(101):
        volume = index / 10.
        z = fluid.column_height(volume)
        major = index % 10 == 0
        target = index == 50
        width = .010 if target else (.0048 if major else .002)
        thickness = .00095 if target else .0003
        w.box(stage, pipe + f"/Graduation{index}", (thickness, width, thickness),
              (.0048, 0., z), INK, collide=False)
        if target:
            w.cylinder(stage, pipe + "/TargetRing", .0052, .0010,
                       (0., 0., z), INK, collide=False)
    water_visual(w, stage, pipe + "/Water", .0039)
    w.box(stage, pipe + "/Seal", (.027, .031, .042), C.BODY_GRIP, RUBBER,
          rotation("Z", C.SLEEVE_YAW))
    w.box(stage, pipe + "/SealLip", (.031, .025, .006),
          (0., 0., C.HANGER_Z + .003), DARK)
    w.cylinder(stage, pipe + "/Manifold", .011, .041, (0., 0., .340), RUBBER)
    w.cylinder(stage, pipe + "/AirStem", .008, .039, (0., 0., .421), RUBBER)
    w.cylinder(stage, pipe + "/AirHousing", .0074, .019, (0., .004, .437),
               RUBBER, axis="Y", collide=False)
    w.cylinder(stage, pipe + "/DeliveryHousing", .0077, .042, (.020, -.023, .322),
               RUBBER, axis="X", collide=False)
    w.cylinder(stage, pipe + "/SuctionHousing", .008, .026, (-.005, -.014, .323),
               RUBBER, axis="Y", collide=False)

    # The soft shell's fixed side and the moving side transmit the squeeze to
    # a spring-loaded native degree of freedom. The internal air model uses
    # the measured reduction in width, not the gripper command.
    w.box(stage, pipe + "/LeftRubber", (.010, .033, .044),
          (-C.BULB_HALF_WIDTH + .005, 0., C.BULB[2]), RUBBER)
    squeeze = spring_slide(
        w, stage, "Squeeze", pipe, "X", C.BULB + [C.BULB_HALF_WIDTH - .006, 0., 0.],
        C.BULB_TRAVEL, -1., 290., 2., 11., .011,
    )
    w.box(stage, squeeze + "/RightRubber", (.012, .033, .047), (0., 0., 0.), RUBBER)
    bulb = w.Geom.Sphere.Define(stage, pipe + "/RubberShell")
    bulb.CreateRadiusAttr(1.)
    bulb.AddTranslateOp().Set(w.Gf.Vec3d(*map(float, C.BULB)))
    bulb.AddScaleOp().Set(w.Gf.Vec3f(.027, .025, .041))
    bulb.CreateDisplayColorAttr([w.Gf.Vec3f(*RUBBER)])

    for name, position in C.VALVES.items():
        colors = {"Air": (.75, .20, .11), "Suction": (.86, .33, .18),
                  "Delivery": (.89, .46, .20)}
        actuator = spring_slide(
            w, stage, name, pipe, "X", position + [.008, 0., 0.], C.VALVE_TRAVEL,
            -1., 320., 1.5, 6., .0035,
        )
        w.box(stage, actuator + "/PinchSide", (.006, .017, .014),
              (0., 0., 0.), colors[name])
        w.box(stage, pipe + f"/{name}Counterface", (.006, .017, .014),
              position - [.008, 0., 0.], colors[name])

    # Weighted retort stand with an open pipette support fork, just under the
    # collar at reset. The support permits removal toward the receiving cup.
    stand = w.stage("stand")
    support = C.PARKED[2] + C.HANGER_Z - C.TABLE
    w.box(stand, "/Asset/Base", (.094, .110, .012), (.124, .023, .006), STAND)
    w.cylinder(stand, "/Asset/Post", .007, support - .009,
               (.136, .044, (support + .009) / 2), CLAMP)
    w.box(stand, "/Asset/Arm", (.143, .018, .011), (.089, .017, support - .003), CLAMP)
    for index, x in enumerate((-.0145, .0145)):
        w.box(stand, f"/Asset/Fork{index}", (.010, .036, .007),
              (x, 0., support - .004), STAND)
    for index, x in enumerate((-.0225, .0225)):
        w.box(stand, f"/Asset/Side{index}", (.004, .024, .006),
              (x, 0., support + .0005), STAND)
    w.box(stand, "/Asset/ForkBack", (.030, .004, .004),
          (0., .0170, support + .0005), STAND)
    source = cup(w, "source", BLUE, .55)
    receiver = cup(w, "receiver", CLEAR, .45)
    w.inventory()
    return {"assets": [
        {"name": "pipette", "type": "articulation", "file": w.save(stage),
         "position": C.PARKED.tolist()},
        {"name": "stand", "type": "base", "file": w.save(stand), "position": [*C.PARKED[:2], C.TABLE]},
        {"name": "source", "type": "base", "file": w.save(source), "position": C.SOURCE.tolist()},
        {"name": "receiver", "type": "base", "file": w.save(receiver), "position": C.RECEIVER.tolist()},
    ], "settle_actions": 8}


def update_water(stage, root, water, squeeze):
    from pxr import Gf, UsdGeom

    def liquid(path, height, bottom):
        prim = UsdGeom.Xformable(stage.GetPrimAtPath(root + path))
        translate, scale = prim.GetOrderedXformOps()
        translate.Set(Gf.Vec3d(0., 0., bottom + height / 2))
        scale.Set(Gf.Vec3f(1., 1., max(.00001, height)))
        imageable = UsdGeom.Imageable(prim.GetPrim())
        imageable.MakeVisible() if height > 0.00001 else imageable.MakeInvisible()

    liquid("/pipette/Pipe/Water", fluid.column_height(water.pipette) - .012 if water.pipette else 0., .012)
    liquid("/source/Water", fluid.water_height(water.source) - C.CUP_BASE, C.CUP_BASE)
    liquid("/receiver/Water", fluid.water_height(water.receiver) - C.CUP_BASE, C.CUP_BASE)
    bulb = UsdGeom.Xformable(stage.GetPrimAtPath(root + "/pipette/Pipe/RubberShell"))
    translate, scale = bulb.GetOrderedXformOps()
    translate.Set(Gf.Vec3d(-.5 * squeeze, 0., float(C.BULB[2])))
    scale.Set(Gf.Vec3f(.027 - .5 * squeeze, .025 + .22 * squeeze, .041))
