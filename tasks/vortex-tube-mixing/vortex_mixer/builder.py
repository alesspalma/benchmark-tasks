"""Native rigid tube, spring switch, and motorized eccentric rubber cup."""
import math

from . import config as C
from .materials import CLEAR, DARK, FLUID_LUMINANCE
from .usd_scene import Writer

ORB_AXES = ("X", "Y")
RUBBER = (.075, .084, .096)


def _linear_joint(w, stage, name, parent, child, axis, lower, upper, stiffness, damping, force):
    joint = w.Phy.PrismaticJoint.Define(stage, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(lower)
    joint.CreateUpperLimitAttr(upper)
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateTargetPositionAttr(0.)
    drive.CreateTargetVelocityAttr(0.)
    drive.CreateMaxForceAttr(force)


def mixer(w):
    stage = w.stage("mixer")
    base = w.body(stage, "/Asset/Base", .4, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(stage, base).CreateEnabledSelfCollisionsAttr(False)
    pad_center = C.MIXER + [0., 0., C.PAD_Z]
    mount = w.body(stage, "/Asset/Switch", .020, position=pad_center)
    x_stage = w.body(stage, "/Asset/XCarrier", .012, position=pad_center)
    cup = w.body(stage, "/Asset/Cup", .024, position=pad_center)
    for carrier in (mount, x_stage):
        mass = w.Phy.MassAPI.Get(stage, carrier)
        mass.CreateCenterOfMassAttr(w.Gf.Vec3f(0, 0, 0))
        mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(1e-6, 1e-6, 1e-6))
    for name, body, dims, at, color in [
        ("BaseShell", base, (.099, .094, .028), (0, 0, .022), (.78, .80, .79)),
    ]:
        w.box(stage, body + "/" + name, dims, C.MIXER + at, color)
    housing = (.69, .73, .73)
    for side, sign in (("P", 1), ("N", -1)):
        w.box(stage, base + "/HousingX" + side, (.016, .080, .018),
              C.MIXER + [.033 * sign, 0., .045], housing)
        w.box(stage, base + "/HousingY" + side, (.050, .015, .018),
              C.MIXER + [0., .0325 * sign, .045], housing)
    for x in (-.035, .035):
        for y in (-.032, .032):
            w.cylinder(stage, base + f"/Foot_{'f' if x < 0 else 'b'}{'l' if y > 0 else 'r'}",
                       .008, .008, C.MIXER + [x, y, .004], RUBBER)
    w.cylinder(stage, cup + "/Pad", C.PAD_RADIUS, .007, (0, 0, -.0035), RUBBER)
    w.ring(stage, cup + "/Wall", .0210, .004, .006, .003, RUBBER, n=16)
    w.Phy.MaterialAPI.Get(stage, "/Asset/Physics").CreateDynamicFrictionAttr(.82)
    w.Phy.MaterialAPI.Get(stage, "/Asset/Physics").CreateStaticFrictionAttr(1.0)
    _linear_joint(w, stage, "Press", base, mount, "Z", -C.PRESS_TRAVEL, 0.,
                  C.PRESS_STIFFNESS, 3.5, 35.)
    press = w.Phy.PrismaticJoint.Get(stage, "/Asset/Press")
    press.CreateLocalPos0Attr(w.Gf.Vec3f(*pad_center))
    for axis, parent, child in zip(ORB_AXES, (mount, x_stage), (x_stage, cup)):
        _linear_joint(w, stage, "Orbit" + axis, parent, child, axis,
                      -.004, .004, 14500., 11., 35.)
    return stage


def surface(w, stage, name, color, opacity=1.):
    from pxr import Sdf

    material = w.Shade.Material.Define(stage, "/Asset/" + name)
    shader = w.Shade.Shader.Define(stage, "/Asset/" + name + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.20)
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material, shader


def tube(w):
    stage = w.stage("tube")
    body = w.body(stage, "/Asset/Body", C.BODY_MASS, ccd=True)
    mass = w.Phy.MassAPI.Get(stage, body)
    mass.CreateCenterOfMassAttr(w.Gf.Vec3f(0., 0., .047))
    mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(4.3e-5, 4.3e-5, 5.5e-6))
    w.cylinder(stage, body + "/ShellCollision", C.BODY_RADIUS, .105, (0, 0, .0525), (.95, .97, .98))
    w.Geom.Imageable.Get(stage, body + "/ShellCollision").MakeInvisible()

    material, _ = surface(w, stage, "Polypropylene", (.93, .97, .99), .07)
    shell = w.Geom.Mesh.Define(stage, body + "/TubeWall")
    n = 48
    ring_levels = ((C.BODY_RADIUS, .002), (C.BODY_RADIUS, .106))
    points = [w.Gf.Vec3f(r * math.cos(i * math.tau / n), r * math.sin(i * math.tau / n), z)
              for r, z in ring_levels for i in range(n)]
    quad = [v for i in range(n) for v in (i, (i + 1) % n, n + (i + 1) % n, n + i)]
    shell.CreatePointsAttr(points)
    shell.CreateFaceVertexCountsAttr([4] * n)
    shell.CreateFaceVertexIndicesAttr(quad)
    shell.CreateSubdivisionSchemeAttr("none")
    shell.CreateDoubleSidedAttr(True)
    w.Shade.MaterialBindingAPI.Apply(shell.GetPrim()).Bind(material)
    w.cylinder(stage, body + "/SkirtRim", .015, .003, (0, 0, .0015), (.83, .89, .92), collide=False)

    cap = (.045, .24, .60)
    w.cylinder(stage, body + "/ScrewCap", .0171, .011, (0, 0, .1095), cap)
    for i in range(32):
        angle = i * math.tau / 32
        w.cylinder(stage, body + f"/CapRib{i}", .00055, .009,
                   (.017 * math.cos(angle), .017 * math.sin(angle), .1095),
                   (.075, .29, .64), collide=False)

    for name, bottom, top, initial in (
        ("Lower", C.FILL_LOW, C.SAMPLE_SPLIT, DARK),
        ("Upper", C.SAMPLE_SPLIT, C.FILL_HIGH, CLEAR),
    ):
        w.cylinder(stage, body + "/" + name, .0137, top - bottom, (0, 0, (top + bottom) / 2),
                   initial, collide=False)
        from pxr import Sdf

        fluid, shader = surface(w, stage, name + "Fluid", initial)
        shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
            w.Gf.Vec3f(*map(float, initial * FLUID_LUMINANCE))
        )
        w.Shade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(body + "/" + name)).Bind(fluid)
    return stage


def create_assets(directory):
    w = Writer(directory)
    assets = [
        dict(name="tube", type="rigid", file=w.save(tube(w)), position=C.TUBE.tolist()),
        dict(name="mixer", type="articulation", file=w.save(mixer(w)), position=[0., 0., 0.]),
    ]
    w.inventory()
    return dict(assets=assets, settle_actions=20)
