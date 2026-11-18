"""Native rigid apple, core, corer and an ordinary cutting board."""

import math

import numpy as np

from . import config as C
from .usd_scene import Writer

PEEL = (.19, .52, .05)
FLESH = (.94, .87, .57)
STEEL = (.59, .65, .69)


def hull(w, stage, path, points, faces, color, collide=False):
    mesh = w.Geom.Mesh.Define(stage, path)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreatePointsAttr([w.Gf.Vec3f(*map(float, p)) for p in points])
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    w._finish(stage, mesh, color, collide, contact=.00035)
    if collide:
        w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
        w.Geom.Imageable(mesh.GetPrim()).MakeInvisible()


def board(w):
    stage = w.stage("board")
    w.box(stage, "/Asset/Wood", C.BOARD_SIZE, (0, 0, .005), (.52, .33, .18))
    for j in range(6):
        w.box(stage, f"/Asset/Grain{j}", (.229, .0008, .00008),
              (0., -.061 + .023*j, .010027), (.45, .26, .13), collide=False)
    return stage


def apple(w):
    stage = w.stage("apple")
    p = w.body(stage, "/Asset/Fruit", .142, ccd=True)
    bands = [(0., .016), (.008, .028), (.022, .036), (.038, .037),
             (.053, .034), (.063, .026), (.066, .0105)]
    n = 48
    points = [
        (r * math.cos(a), r * math.sin(a), z + .0013 * math.cos(5*a) * (r / .037))
        for z, r in bands for a in np.linspace(0, math.tau, n, endpoint=False)
    ]
    faces = [(k*n+j, k*n+(j+1)%n, (k+1)*n+(j+1)%n, (k+1)*n+j)
             for k in range(len(bands)-1) for j in range(n)]
    hull(w, stage, p + "/Peel", points, faces, PEEL)
    # The central region is reserved for the force-driven cutting proxy.
    w.ring(stage, p + "/Bore", .01055, .00018, .065, .033, FLESH, n=36)
    for prim in stage.Traverse():
        if "/Bore/" in str(prim.GetPath()):
            if prim.HasAPI(w.Phy.CollisionAPI):
                prim.RemoveAPI(w.Phy.CollisionAPI)
    for j in range(12):
        lo, hi = (j-.52)*math.tau/12, (j+.52)*math.tau/12
        points = [(r*math.cos(a), r*math.sin(a), z)
                  for z, outer in bands for a in (lo,hi) for r in (.0135, outer)]
        # Convex hull uses the complete set of points, with per-band faces.
        faces = [
            (4*k,4*k+1,4*k+5,4*k+4) for k in range(len(bands)-1)
        ] + [(4*k+1,4*k+3,4*k+7,4*k+5) for k in range(len(bands)-1)] + [
            (4*k+3,4*k+2,4*k+6,4*k+7) for k in range(len(bands)-1)
        ] + [(4*k+2,4*k,4*k+4,4*k+6) for k in range(len(bands)-1)] + [
            (0,2,3,1), (24,25,27,26)
        ]
        hull(w, stage, p + f"/Contact{j}", points, faces, PEEL, collide=True)
    return stage


def core(w):
    stage = w.stage("core")
    p = w.body(stage, "/Asset/Core", .013, ccd=True)
    w.cylinder(stage, p + "/Column", C.CORE_RADIUS, 2*C.CORE_HALF_LENGTH,
               (0., 0., 0.), FLESH)
    w.cylinder(stage, p + "/StemSkin", .0092, .0029, (0., 0., .031), PEEL, collide=False)
    w.cylinder(stage, p + "/Stem", .00155, .008, (.001, -.001, .038),
               (.28, .13, .043), collide=False)
    for j in range(4):
        a = j*math.pi/2
        w.box(stage, p + f"/Seed{j}", (.002, .0035, .005),
              (.0082*math.cos(a), .0082*math.sin(a), .005),
              (.30, .14, .049), collide=False)
    return stage


def tool(w):
    stage = w.stage("corer")
    p = w.body(stage, "/Asset/Tool", .076, ccd=True)
    # The cutting tube is closed at the rim and open at the upper side.
    w.ring(stage, p + "/Ring", C.CUT_RADIUS, .00128, .025, .0127, STEEL, n=24)
    w.ring(stage, p + "/Channel", C.CUT_RADIUS, .00128, .048, .048, STEEL,
           n=24, omit=tuple(range(4,13)))
    w.box(stage, p + "/Tang", (.006, .011, .032), (.007, 0., .086), STEEL)
    w.box(stage, p + "/Handle", (.023, .025, .102), (0., 0., .152), (.063, .067, .074))
    w.cylinder(stage, p + "/HandleEnd", .0105, .010, (0., 0., .100),
               (.063, .067, .074), collide=False)
    friction = w.Phy.MaterialAPI.Get(stage, "/Asset/Physics")
    friction.CreateStaticFrictionAttr(.76)
    friction.CreateDynamicFrictionAttr(.58)
    return stage


def create_assets(directory):
    w = Writer(directory)
    return {"assets": [
        dict(name="board", type="base", position=C.BOARD.tolist(), file=w.save(board(w))),
        dict(name="apple", type="rigid", position=C.APPLE.tolist(), file=w.save(apple(w))),
        dict(name="core", type="rigid", position=(C.APPLE+[0.,0.,C.CORE_CENTER]).tolist(),
             file=w.save(core(w))),
        dict(name="corer", type="rigid", position=C.TOOL_START.tolist(),
             quaternion=C.TOOL_ORIENT, file=w.save(tool(w))),
    ], "settle_actions": 14}
