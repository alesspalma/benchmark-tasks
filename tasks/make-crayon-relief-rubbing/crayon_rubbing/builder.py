"""Build the unmarked sheet, raised PLA plate and wax block as native solids."""

import numpy as np

from . import config as C
from .plate import fern
from .scoring import CELLS, SPACING
from .usd_scene import Writer

TABLE_COLOR = (.42, .50, .45)
PLATE_COLOR = (.66, .33, .19)
RUBBER = (.16, .19, .17)


def friction(w, stage, static, dynamic=None):
    mat = w.Phy.MaterialAPI.Get(stage, "/Asset/Physics")
    mat.CreateStaticFrictionAttr(static)
    mat.CreateDynamicFrictionAttr(static if dynamic is None else dynamic)


def relief_mesh(w, stage, path, points):
    geo = w.Geom.Mesh.Define(stage, path)
    n = len(points)
    h0 = C.TABLE + C.RUBBER_FEET + C.PLATE_BASE - .00005
    h1 = h0 + C.RELIEF + .00005
    xy = points + C.PAPER_CENTER
    geo.CreatePointsAttr([w.Gf.Vec3f(float(x), float(y), z) for z in (h0, h1) for x, y in xy])
    faces = [list(reversed(range(n))), list(range(n, 2 * n))]
    for i in range(n):
        j = (i + 1) % n
        faces.append([i, j, j + n, i + n])
    geo.CreateFaceVertexCountsAttr([len(face) for face in faces])
    geo.CreateFaceVertexIndicesAttr([idx for face in faces for idx in face])
    geo.CreateSubdivisionSchemeAttr("none")
    w.Phy.MeshCollisionAPI.Apply(geo.GetPrim()).CreateApproximationAttr("convexHull")
    w._finish(stage, geo, PLATE_COLOR, True, contact=.0002)


def bench(w):
    stage = w.stage("bench")
    friction(w, stage, .34, .27)
    w.box(stage, "/Asset/Worktop", (1.08, 1.25, .042), (.30, 0, C.TABLE - .021), TABLE_COLOR)
    for i, x in enumerate((-.19, .77)):
        for j, y in enumerate((-.56, .56)):
            w.box(stage, f"/Asset/Leg{i}{j}", (.055, .055, C.TABLE - .042),
                  (x, y, (C.TABLE - .042) / 2), RUBBER)
    w.box(stage, "/Asset/Floor", (6., 6., .02), (0., 0., -.01), (.58, .57, .53))
    w.box(stage, "/Asset/PlateFeet", (*C.PLATE_SIZE, C.RUBBER_FEET),
          (*C.PAPER_CENTER, C.TABLE + C.RUBBER_FEET / 2), RUBBER, collide=False)
    w.box(stage, "/Asset/Plate", (*C.PLATE_SIZE, C.PLATE_BASE),
          (*C.PAPER_CENTER, C.TABLE + C.RUBBER_FEET + C.PLATE_BASE / 2), PLATE_COLOR)
    for i, part in enumerate(fern()):
        relief_mesh(w, stage, f"/Asset/Leaf{i}", part)
    grip_material = w.Shade.Material.Define(stage, "/Asset/Silicone")
    material = w.Phy.MaterialAPI.Apply(grip_material.GetPrim())
    material.CreateStaticFrictionAttr(1.10)
    material.CreateDynamicFrictionAttr(.83)
    w.Px.PhysxMaterialAPI.Apply(grip_material.GetPrim()).CreateFrictionCombineModeAttr("max")
    for i, sign in enumerate((-1, 1)):
        path = f"/Asset/Margin{i}"
        w.box(stage, path, (.118, .019, C.RELIEF),
              (C.PAPER_CENTER[0], C.PAPER_CENTER[1] + sign * .0885,
               C.PLATE_TOP - C.RELIEF / 2), RUBBER)
        w.Shade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(
            grip_material, materialPurpose="physics",
        )
    w.box(stage, "/Asset/CrayonRest", (.061, .040, C.CRAYON_REST),
          (*C.CRAYON_START, C.TABLE + C.CRAYON_REST / 2), RUBBER)
    return stage


def ink_mesh(w, stage):
    mesh = w.Geom.Mesh.Define(stage, "/Asset/WaxOnPaper")
    x_y = CELLS[:, None, :] + SPACING / 2 * np.array([[-1,-1], [1,-1], [1,1], [-1,1]])
    z = C.PAPER_THICKNESS / 2 + .000045
    mesh.CreatePointsAttr([w.Gf.Vec3f(float(x), float(y), z) for x, y in x_y.reshape(-1, 2)])
    mesh.CreateFaceVertexCountsAttr([4] * len(CELLS))
    mesh.CreateFaceVertexIndicesAttr(list(range(4 * len(CELLS))))
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    colors = mesh.CreateDisplayColorPrimvar("uniform")
    colors.Set([w.Gf.Vec3f(*C.PAPER_COLOR)] * len(CELLS))


def paper(w):
    stage = w.stage("paper")
    friction(w, stage, .18, .14)
    w.body(stage, "/Asset", C.PAPER_MASS, ccd=True)
    geo = w.Geom.Cube.Define(stage, "/Asset/Sheet")
    geo.CreateSizeAttr(1.)
    geo.AddScaleOp().Set(w.Gf.Vec3f(*C.PAPER_SIZE, C.PAPER_THICKNESS))
    w._finish(stage, geo, C.PAPER_COLOR, True, contact=.00015)
    ink_mesh(w, stage)
    return stage


def crayon(w):
    stage = w.stage("crayon")
    friction(w, stage, .56, .42)
    w.body(stage, "/Asset", .010, ccd=True)
    w.rounded_box(stage, "/Asset/Wax", C.CRAYON_SIZE, (0.,0.,0.), C.CRAYON_COLOR, .002, .0008)
    w.Px.PhysxContactReportAPI.Apply(stage.GetPrimAtPath("/Asset")).CreateThresholdAttr(0.)
    return stage


def create_assets(directory):
    w = Writer(directory)
    return {"assets": [
        {"name": "bench", "type": "base", "file": w.save(bench(w))},
        {"name": "paper", "type": "rigid", "file": w.save(paper(w)),
         "position": [*C.PAPER_CENTER, C.PLATE_TOP + C.PAPER_THICKNESS / 2 + .0003]},
        {"name": "crayon", "type": "rigid", "file": w.save(crayon(w)),
         "position": [*C.CRAYON_START, C.TABLE + C.CRAYON_REST + C.CRAYON_SIZE[2] / 2 + .0003]},
    ], "settle_actions": 15}
