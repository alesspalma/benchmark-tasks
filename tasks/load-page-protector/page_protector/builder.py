"""Original USD geometry for a Letter pocket, paper and the reset supports."""

import math

import numpy as np
from yam_arena import ik

from . import config as c
from .motion import rotation
from .usd_scene import Writer

BINDER_BLUE = (.095, .22, .37)
STEEL = (.64, .69, .73)
FILM = (.76, .9, .95)
SEAM = (.63, .77, .84)
PAPER = (.94, .925, .86)
INK = (.12, .16, .23)


def film(w, stage, path, size, translation, opacity):
    w.box(stage, path, size, translation, FILM, collide=False)
    prim = stage.GetPrimAtPath(path)
    w.Geom.Gprim(prim).CreateDisplayOpacityAttr([opacity])
    material = w.Shade.Material.Define(stage, path + "/Clear")
    shader = w.Shade.Shader.Define(stage, path + "/Clear/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    from pxr import Sdf

    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(*FILM))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.3)
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    w.Shade.MaterialBindingAPI.Apply(prim).Bind(material)


def collider(w, stage, path, size, pos):
    w.box(stage, path, size, pos, FILM)
    w.Geom.Imageable(stage.GetPrimAtPath(path)).MakeInvisible()


def revolve(w, stage, name, parent, child, pos_a, pos_b, limits, stiffness, damping):
    j = w.Phy.RevoluteJoint.Define(stage, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([parent])
    j.CreateBody1Rel().SetTargets([child])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*pos_a))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(*pos_b))
    j.CreateAxisAttr("Y")
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    j.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness * math.pi / 180)
    drive.CreateDampingAttr(damping * math.pi / 180)
    drive.CreateTargetPositionAttr(0.)
    return j


def stock_rotation():
    return rotation("Z", math.radians(c.STOCK_YAW)) @ rotation("Y", math.radians(c.STOCK_PITCH))


def workstation(w):
    stage = w.stage("binder")
    depth = .284
    binder_x = (c.MOUTH + c.FAR_EDGE) / 2 + .008
    w.box(stage, "/Asset/Cover", (depth, .254, .006),
          (binder_x, c.SLEEVE_Y - .011, c.TABLE + .003), BINDER_BLUE)
    w.box(stage, "/Asset/FacingCover", (depth, .245, .006),
          (binder_x, c.SLEEVE_Y - .274, c.TABLE + .003), BINDER_BLUE)
    w.box(stage, "/Asset/Spine", (depth, .029, .009),
          (binder_x, c.SLEEVE_Y - .15, c.TABLE + .005), BINDER_BLUE)
    w.box(stage, "/Asset/FiledPages", (.288, .238, c.LOWER_FACE - c.TABLE - .007),
          (binder_x + .004, c.SLEEVE_Y - .008, (c.LOWER_FACE + c.TABLE + .005) / 2),
          (.74, .77, .78))
    w.box(stage, "/Asset/RingBase", (.265, .013, .005),
          (binder_x, c.SLEEVE_Y - .131, c.TABLE + .0085), STEEL)
    for number, x in enumerate((binder_x - .108, binder_x, binder_x + .108)):
        for i in range(22):
            angle = i * math.tau / 22
            center = (x, c.SLEEVE_Y - .127 + .017 * math.cos(angle),
                      c.TABLE + .029 + .022 * math.sin(angle))
            w.box(stage, f"/Asset/Ring{number}_{i}", (.003, .006, .004), center, STEEL,
                  collide=False)
    angle = stock_rotation()
    rear = np.asarray(c.STOCK_REAR)
    length = c.PAPER_LENGTH - c.STOCK_OVERHANG + .008
    center = rear + angle @ np.array([c.STOCK_OVERHANG + length / 2, 0., -.0035])
    w.box(stage, "/Asset/DocumentStand", (length, c.PAPER_WIDTH + .010, .006),
          center, BINDER_BLUE, angle)
    center = rear + angle @ np.array([c.PAPER_LENGTH + .0035, 0., .0035])
    w.box(stage, "/Asset/StandRetainer", (.005, c.PAPER_WIDTH + .010, .010),
          center, BINDER_BLUE, angle)
    for i, dx in enumerate((.08, .22)):
        for j, dy in enumerate((-.075, .075)):
            contact = rear + angle @ np.array([dx, dy, -.009])
            height = contact[2] - c.TABLE
            w.box(stage, f"/Asset/StandLeg{i}_{j}", (.012, .012, height),
                  (*contact[:2], c.TABLE + height / 2), BINDER_BLUE)
    return w.save(stage)


def sleeve(w):
    stage = w.stage("protector")
    base = w.body(stage, "/Asset/LowerFilm", .65, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(stage, base).CreateEnabledSelfCollisionsAttr(False)
    hinge = (c.FAR_EDGE, c.SLEEVE_Y, c.UPPER_FACE)
    upper = w.body(stage, "/Asset/UpperFilm", .006, position=hinge)
    depth = c.FAR_EDGE - c.MOUTH
    half = c.INNER_HALF_WIDTH
    middle = c.MOUTH + depth / 2
    # The binder's supported lower film and the welded perimeter stay flat.
    center = (middle, c.SLEEVE_Y, c.LOWER_FACE - c.FILM_PROXY / 2)
    collider(w, stage, base + "/LowerContact", (depth, 2 * (half + .002), c.FILM_PROXY), center)
    film(w, stage, base + "/LowerSurface", (depth, 2 * (half + .002), .00010),
         (middle, c.SLEEVE_Y, c.LOWER_FACE - .00005), .24)
    for side, label in [(-1, "RingSide"), (1, "OuterSide")]:
        w.box(stage, base + "/" + label, (depth, .002, .0020),
              (middle, c.SLEEVE_Y + side * (half + .001), c.LOWER_FACE + .0002), SEAM)
    w.box(stage, base + "/BottomSeam", (.002, 2 * half, .002),
          (c.FAR_EDGE, c.SLEEVE_Y, c.LOWER_FACE + .0002), SEAM)
    w.box(stage, base + "/ReinforcedMargin", (depth, .015, .00025),
          (middle, c.SLEEVE_Y - half - .009, c.LOWER_FACE), SEAM)
    length = c.FAR_EDGE - c.UPPER_LIP
    collider(w, stage, upper + "/UpperContact", (length, 2 * half, c.FILM_PROXY),
             (-length / 2, 0, 0))
    film(w, stage, upper + "/UpperSurface", (length, 2 * half, .00010),
         (-length / 2, 0, .00002), .28)
    w.box(stage, upper + "/TopEdge", (.00065, 2 * half, .00011),
          (-length, 0, .00004), SEAM, collide=False)
    mass = w.Phy.MassAPI.Get(stage, upper)
    mass.CreateCenterOfMassAttr(w.Gf.Vec3f(-length / 2, 0, 0))
    mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(.000025, .000044, .000069))
    revolve(w, stage, "FilmFlex", base, upper, hinge, (0., 0., 0.),
            (0., 7.4), .013, .005)
    return w.save(stage)


def paper(w):
    stage = w.stage("printed_page")
    length = c.PAPER_LENGTH
    base = w.body(stage, "/Asset/Rear", c.PAPER_MASS, ccd=True)
    collider(w, stage, base + "/PaperContact", (length, c.PAPER_WIDTH, c.PAPER_PROXY), (0, 0, 0))
    w.box(stage, base + "/PaperVisual", (length, c.PAPER_WIDTH, .00010),
          (0, 0, 0), PAPER, collide=False)
    for i in range(23):
        x = .106 - i * .008
        if i in (3, 11, 17):
            continue
        width = .111 if i == 0 else .17 - (i % 5) * .009
        w.box(stage, base + f"/Ink{i}", (.0028 if i < 2 else .0012, width, .00002),
              (x, -.091 + width / 2, .000061), INK, collide=False)
    mat = w.Phy.MaterialAPI.Get(stage, "/Asset/Physics")
    mat.CreateStaticFrictionAttr(.42)
    mat.CreateDynamicFrictionAttr(.31)
    return w.save(stage)


def create_assets(directory):
    w = Writer(directory)
    stock_angle = stock_rotation()
    stock_center = np.asarray(c.STOCK_REAR) + stock_angle @ np.array([c.PAPER_HALF, 0., 0.])
    assets = [
        {"name": "workstation", "type": "base", "file": workstation(w)},
        {"name": "sleeve", "type": "articulation", "file": sleeve(w)},
        {"name": "page", "type": "rigid", "file": paper(w),
         "position": tuple(stock_center),
         "quaternion": tuple(ik.matrix_to_quat_xyzw(stock_angle))},
    ]
    w.inventory()
    return {"assets": assets, "settle_actions": 8}
