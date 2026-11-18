"""PhysX applicator, compliant transfer nose and an ordinary clipped sheet."""

import numpy as np

from . import config as C, paper
from .usd_scene import Writer

TEAL = (.075, .44, .50)
DARK = (.036, .16, .19)
WHITE = (.95, .96, .93)


def rectangle_points(rects, z):
    return [
        (px, py, z)
        for x, y, w, h in rects
        for px, py in ((x,y), (x+w,y), (x+w,y+h), (x,y+h))
    ]


def quads(w, stage, path, points, color):
    geom = w.Geom.Mesh.Define(stage, path)
    geom.CreateSubdivisionSchemeAttr("none")
    geom.CreateDoubleSidedAttr(True)
    geom.CreatePointsAttr([w.Gf.Vec3f(*map(float, point)) for point in points])
    geom.CreateFaceVertexCountsAttr([4] * (len(points) // 4))
    geom.CreateFaceVertexIndicesAttr(list(range(len(points))))
    geom.CreateDisplayColorAttr([w.Gf.Vec3f(*color)])
    return geom


def clipboard(w):
    s = w.stage("clipboard")
    body = w.body(s, "/Asset/Board", .280, ccd=True)
    w.box(s, body + "/Fiberboard", C.BOARD_SIZE, (0, 0, .004), (.30, .20, .13))
    lx, ly = C.PAPER_SIZE
    w.box(s, body + "/Sheet", (lx, ly, .00011), (0, 0, C.PAPER_Z - .000055), paper.PAGE)
    w.box(s, body + "/Clip", (.028, .087, .006), (-.093, 0., .011), (.52, .56, .62))
    w.box(s, body + "/ClipBar", (.012, .077, .003), (-.079, 0., .0095), (.25, .31, .38))
    for i, (x, y) in enumerate((x, y) for x in (-.097, .097) for y in (-.066, .066)):
        w.box(s, body + f"/Foot{i}", (.023, .025, .0015), (x, y, .00025), (.17, .17, .16))
    for j, (color, rectangles) in enumerate(paper.artwork()):
        # The small visual offset prevents the print from fighting the paper
        # surface in the native cameras. It does not change contact height.
        quads(w, s, body + f"/Print{j}", rectangle_points(rectangles, C.PAPER_Z + .0003), color)
    quads(w, s, body + "/TransferredFilm", [], paper.COATING)
    return s


def dispenser(w):
    s = w.stage("applicator")
    shell = w.body(s, "/Asset/Housing", .026, articulation=True, ccd=True)
    w.Px.PhysxArticulationAPI.Get(s, shell).CreateEnabledSelfCollisionsAttr(False)
    # Three overlapping collision sections follow the shell taper.
    for name, center, size in (
        ("Grip", (.042, 0., 0.), (.046, .035, .016)),
        ("Tail", (.068, 0., 0.), (.024, .026, .016)),
        ("Neck", (.013, 0., 0.), (.017, .019, .012)),
    ):
        w.box(s, shell + "/" + name, size, center, TEAL)
        w.Geom.Imageable.Get(s, shell + "/" + name).MakeInvisible()
    outline = [
        (.005, -.006), (.011, -.011), (.024, -.016), (.038, -.0175),
        (.054, -.0175), (.070, -.013), (.080, -.008), (.082, 0.),
        (.080, .008), (.070, .013), (.054, .0175), (.038, .0175),
        (.024, .016), (.011, .011), (.005, .006),
    ]
    points = [
        (.044 + scale * (x - .044), scale*y, z)
        for scale, z in ((.90, -.008), (1., -.006), (1., .006), (.90, .008))
        for x,y in outline
    ]
    n = len(outline)
    skin = w.Geom.Mesh.Define(s, shell + "/Shell")
    skin.CreateSubdivisionSchemeAttr("none")
    skin.CreateDoubleSidedAttr(True)
    skin.CreatePointsAttr([w.Gf.Vec3f(*p) for p in points])
    faces = [
        (j*n+i, j*n+(i+1)%n, (j+1)*n+(i+1)%n, (j+1)*n+i)
        for j in range(3) for i in range(n)
    ] + [tuple(reversed(range(n))), tuple(range(3*n,4*n))]
    skin.CreateFaceVertexCountsAttr([len(f) for f in faces])
    skin.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    skin.CreateDisplayColorAttr([w.Gf.Vec3f(*TEAL)])
    w.cylinder(s, shell + "/SpoolWindow", .012, .0011, (.048, 0., .0082), DARK, collide=False)
    w.cylinder(s, shell + "/SpoolHub", .004, .0012, (.048, 0., .00875), WHITE, collide=False)

    tip = w.body(s, "/Asset/NoseLink", .002, ccd=True)
    w.box(s, tip + "/Nose", (.010, .008, .0045), (.006, 0, -.001), WHITE)
    w.box(s, tip + "/TransferEdge", (.004, C.TAPE_WIDTH, .002), (0, 0, -.004), WHITE)
    joint = w.Phy.PrismaticJoint.Define(s, "/Asset/NoseSpring")
    joint.CreateBody0Rel().SetTargets([shell])
    joint.CreateBody1Rel().SetTargets([tip])
    joint.CreateAxisAttr("Z")
    joint.CreateLowerLimitAttr(0.)
    joint.CreateUpperLimitAttr(C.NOSE_LIMIT)
    joint.CreateCollisionEnabledAttr(False)
    spring = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    spring.CreateTypeAttr("force")
    spring.CreateStiffnessAttr(C.NOSE_STIFFNESS)
    spring.CreateDampingAttr(2.9)
    spring.CreateTargetPositionAttr(0.)
    spring.CreateMaxForceAttr(12.)
    friction = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    friction.CreateStaticFrictionAttr(.66)
    friction.CreateDynamicFrictionAttr(.44)
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [
        dict(name="clipboard", type="rigid", position=C.BOARD_START.tolist(), file=w.save(clipboard(w))),
        dict(name="applicator", type="articulation", position=C.TOOL_START.tolist(), file=w.save(dispenser(w))),
    ]
    return {"assets": assets, "settle_actions": 24}
