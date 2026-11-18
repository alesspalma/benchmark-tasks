"""Full-size glass and genuine freely manipulated two-stop beer faucet."""
import math

import numpy as np

from . import config as C
from .usd_scene import Writer


def surface(w, stage, name, color, opacity=1.):
    from pxr import Sdf

    material = w.Shade.Material.Define(stage, "/Asset/" + name)
    shader = w.Shade.Shader.Define(stage, "/Asset/" + name + "/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(*color))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.16)
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return material


def empty_mesh(w, stage, path, material):
    mesh = w.Geom.Mesh.Define(stage, path)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreatePointsAttr([])
    mesh.CreateFaceVertexCountsAttr([])
    mesh.CreateFaceVertexIndicesAttr([])
    w.Shade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    return mesh


def glass_shell(w, stage, path, material):
    # A single thin render surface keeps the beer and foam legible through the
    # vessel. The separate overlapping wall panels are used for contacts.
    n = 64
    points = [w.Gf.Vec3f(r * math.cos(angle), r * math.sin(angle), z)
              for r, z in ((C.R_BASE + C.WALL, C.FLOOR), (C.R_LIP + C.WALL, C.HEIGHT),
                           (C.R_LIP, C.HEIGHT))
              for angle in (i * math.tau / n for i in range(n))]
    quads = []
    for lower in (0, n):
        for i in range(n):
            j = (i + 1) % n
            quads.extend([lower + i, lower + j, lower + n + j, lower + n + i])
    mesh = empty_mesh(w, stage, path, material)
    mesh.GetPointsAttr().Set(points)
    mesh.GetFaceVertexCountsAttr().Set([4] * (2 * n))
    mesh.GetFaceVertexIndicesAttr().Set(quads)


def glass(w):
    stage = w.stage("glass")
    path = w.body(stage, "/Asset/Body", C.DRY_MASS, ccd=True)
    mass = w.Phy.MassAPI.Get(stage, path)
    mass.CreateCenterOfMassAttr(w.Gf.Vec3f(*C.DRY_COM))
    mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(*C.DRY_INERTIA.diagonal()))
    clear = surface(w, stage, "Glass", (.91, .96, .98), .10)
    amber = surface(w, stage, "Lager", (.95, .37, .045))
    foam = surface(w, stage, "Foam", (.98, .914, .735))
    w.cylinder(stage, path + "/Heel", C.R_BASE + C.WALL, C.FLOOR, (0, 0, C.FLOOR / 2), (.71, .88, .91))
    n = 32
    wall_height = C.HEIGHT - C.FLOOR
    slope = (C.R_LIP - C.R_BASE) / wall_height
    width = 2 * (C.R_LIP + C.WALL / 2) * math.tan(math.pi / n)
    for k in range(n):
        angle = k * math.tau / n
        radial = np.array([math.cos(angle), math.sin(angle), 0.])
        tangent = np.array([-math.sin(angle), math.cos(angle), 0.])
        up = np.array([0., 0., 1.])
        axis = (slope * radial + up) / math.sqrt(1. + slope**2)
        normal = np.cross(tangent, axis)
        position = radial * ((C.R_LIP + C.R_BASE + C.WALL) / 2) + up * (C.HEIGHT + C.FLOOR) / 2
        w.box(stage, path + f"/Wall{k:02}", (C.WALL, width, wall_height * math.sqrt(1. + slope**2)),
              position, (.71, .88, .91), rotation=np.column_stack([normal, tangent, axis]))
    for k in range(n):
        w.Geom.Imageable(stage.GetPrimAtPath(path + f"/Wall{k:02}")).MakeInvisible()
    w.Shade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path + "/Heel")).Bind(clear)
    glass_shell(w, stage, path + "/GlassShell", clear)
    empty_mesh(w, stage, path + "/Liquid", amber)
    empty_mesh(w, stage, path + "/Foam", foam)
    return stage


def tap(w):
    stage = w.stage("tap")
    base = w.body(stage, "/Asset/Base", 1.0, fixed=True, articulation=True)
    body = w.body(stage, "/Asset/Lever", .095, position=C.PIVOT)
    bright = (.57, .60, .62)
    dark = (.14, .082, .039)
    wx, wy, wz = C.SPOUT
    w.cylinder(stage, base + "/Foot", .043, .007, (.454, wy, C.TOP + .0035), bright)
    w.cylinder(stage, base + "/Column", .031, .265, (.454, wy, C.TOP + .137), bright)
    w.cylinder(stage, base + "/Shank", .020, .095, (.402, wy, wz + .029), bright, axis="X")
    w.cylinder(stage, base + "/Faucet", .017, .064, (.358, wy, wz + .025), bright, axis="X")
    w.cylinder(stage, base + "/Spout", .0095, .025, (wx, wy, wz + .0125), bright)
    w.cylinder(stage, body + "/Stem", .005, .036, (0, 0, .018), bright)
    w.cylinder(stage, body + "/Handle", .013, .083, (0, 0, .0775), dark)
    w.hinge(stage, "/Asset/ValvePivot", base, body, "Y", C.PIVOT, (0, 0, 0), (-C.TAP_ANGLE, 0.),
            friction=.05, stiffness=.42, damping=.018)
    return stage


def create_assets(directory):
    writer = Writer(directory)
    assets = [
        dict(name="glass", type="rigid", file=writer.save(glass(writer)), position=C.GLASS.tolist()),
        dict(name="tap", type="articulation", file=writer.save(tap(writer)), position=[0., 0., 0.]),
    ]
    stage = writer.stage("fixtures")
    writer.box(stage, "/Asset/DripTray", (.135, .145, .012), (.317, -.040, C.TOP+.006), (.31, .34, .38))
    for k in range(10):
        writer.box(stage, f"/Asset/Grate{k:02}", (.004, .137, .002),
                   (.255 + k * .014, -.04, C.TOP + .0125), (.60, .64, .69))
    writer.cylinder(stage, "/Asset/GlassMat", .050, .004, (*C.GLASS[:2], C.TOP+.002), (.12, .16, .19))
    writer.cylinder(stage, "/Asset/Coaster", .047, .005, (*C.COASTER[:2], C.TOP+.0025), (.12, .35, .50))
    empty_mesh(writer, stage, "/Asset/Stream", surface(writer, stage, "StreamAmber", (.95, .48, .055), .87))
    assets.append(dict(name="fixtures", type="base", file=writer.save(stage), position=[0., 0., 0.]))
    writer.inventory()
    return dict(assets=assets, settle_actions=24)
