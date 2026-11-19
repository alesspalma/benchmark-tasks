"""Build the table, clip posts and working two-joint cord adjuster."""

import math

import numpy as np

from . import config as C
from .usd_scene import Writer

TABLE = (.026, .28, .26)
WHITE = (.93, .94, .88)
STEEL = (.19, .22, .24)
SILVER = (.59, .62, .63)
BLACK = (.04, .058, .07)
RUBBER = (.073, .078, .085)
CAP = (.32, .45, .51)
CORD = (.82, .80, .69)
MESH = (.13, .18, .19)


def table(w):
    s = w.stage("table")
    centre = (C.TABLE_EDGE + C.TABLE_WIDTH / 2, C.POST_AXIS[1])
    w.box(s, "/Asset/Deck", (C.TABLE_WIDTH, C.TABLE_LENGTH, C.TABLE_THICKNESS),
          (*centre, C.TABLE_HEIGHT - C.TABLE_THICKNESS / 2), TABLE)
    w.box(s, "/Asset/CentreLine", (.003, C.TABLE_LENGTH, .0004),
          (*centre, C.TABLE_HEIGHT + .0002), WHITE, collide=False)
    for sign in (-1, 1):
        w.box(s, f"/Asset/LongLine{sign}", (.02, C.TABLE_LENGTH, .0004),
              (centre[0] + sign * (C.TABLE_WIDTH / 2 - .01), centre[1], C.TABLE_HEIGHT + .0002),
              WHITE, collide=False)
        w.box(s, f"/Asset/EndLine{sign}", (C.TABLE_WIDTH, .02, .0004),
              (centre[0], centre[1] + sign * (C.TABLE_LENGTH / 2 - .01), C.TABLE_HEIGHT + .0002),
              WHITE, collide=False)
    for i, x in enumerate((centre[0] - .57, centre[0] + .57)):
        for j, y in enumerate((centre[1] - .99, centre[1] + .99)):
            w.box(s, f"/Asset/Leg{i}{j}", (.048, .048, .705), (x, y, .3525), BLACK)
    floor = -.0125
    for name, (_, y, z) in C.BASES.items():
        w.box(s, "/Asset/" + name + "Pedestal", (.150, .150, z - floor),
              (0., y, (z + floor) / 2), STEEL)
        w.box(s, "/Asset/" + name + "Foot", (.205, .205, .014),
              (0., y, floor + .007), BLACK)
    return w.save(s)


def post_base(w, s, root, x, direction):
    y = C.POST_AXIS[1]
    z = C.TABLE_HEIGHT
    w.box(s, root + "/TopJaw", (.224, .047, .013),
          (x + direction * .089, y, z + .009), BLACK)
    w.box(s, root + "/UnderJaw", (.205, .049, .014),
          (x + direction * .086, y, z - .039), STEEL)
    w.box(s, root + "/JawBrace", (.034, .047, .046),
          (x + direction * .041, y, z - .015), STEEL)
    w.box(s, root + "/InnerPad", (.038, .042, .005),
          (x + direction * .177, y, z - .028), RUBBER)
    w.cylinder(s, root + "/Hinge", .013, .050, (x + direction * .044, y, z - .012),
               SILVER, axis="Y", collide=False)
    w.box(s, root + "/UprightBody", (.022, .029, .102), (x, y, z + .066), BLACK)


def drive(w, joint, mode, position, stiffness, damping, cap):
    d = w.Phy.DriveAPI.Apply(joint.GetPrim(), mode)
    d.CreateTypeAttr("force")
    d.CreateTargetPositionAttr(position)
    d.CreateStiffnessAttr(stiffness)
    d.CreateDampingAttr(damping)
    d.CreateMaxForceAttr(cap)


def joint(w, s, cls, name, base, moving, origin, limits):
    j = cls.Define(s, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([base])
    j.CreateBody1Rel().SetTargets([moving])
    j.CreateAxisAttr("Z")
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, origin)))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(0., 0., 0.))
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    j.CreateCollisionEnabledAttr(False)
    return j


def adjuster(w):
    s = w.stage("adjuster")
    base = w.body(s, "/Asset/Stand", .62, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(s, base).CreateEnabledSelfCollisionsAttr(False)
    post_base(w, s, base + "/Near", C.POST_X, 1)
    post_base(w, s, base + "/Far", C.FAR_AXIS[0], -1)
    fixed = C.FAR_AXIS
    w.box(s, base + "/FarGuide", (.021, .025, .052),
          (fixed[0], fixed[1], fixed[2] - .025), STEEL)
    w.box(s, base + "/FarCap", (.029, .031, .007), (*fixed[:2], fixed[2] - .003), CAP)
    for j, dz in enumerate((.001, .008)):
        w.box(s, base + f"/HeightMark{j}", (.001, .024, .0015),
              (C.POST_X - .0115, C.POST_AXIS[1], C.TABLE_HEIGHT + .100 + dz),
              SILVER, collide=False)
    post = w.body(s, "/Asset/Guide", .06, position=C.POST_AXIS, ccd=True)
    w.box(s, post + "/Slide", (.016, .019, .068), (0., 0., -.033), STEEL)
    # The broad cap gives direct access to the friction-set guide.
    w.box(s, post + "/Cap", (.032, .034, .019), (0., 0., -.0055), CAP)
    w.box(s, post + "/Groove", (.024, .004, .001), (0., 0., .0045), CORD,
          collide=False)
    travel = joint(w, s, w.Phy.PrismaticJoint, "Height", base, post,
                   C.POST_AXIS, (-C.POST_TRAVEL, 0.))
    drive(w, travel, "linear", 0., 2500., 13., 4.)

    w.box(s, base + "/SpoolBracket", (.065, .095, .015),
          (C.SPINDLE[0] + .012, C.SPINDLE[1] + .027, C.SPINDLE[2] - .029), BLACK)
    w.cylinder(s, base + "/FixedPawl", .005, .012,
               C.SPINDLE + [.014, 0., 0.], SILVER, collide=False)
    crank = w.body(s, "/Asset/Reel", .058, position=C.SPINDLE, ccd=True)
    w.cylinder(s, crank + "/Spool", .016, .018, (0., 0., 0.), SILVER)
    for index in range(C.TOOTH_COUNT):
        a = index * C.TOOTH
        w.box(s, crank + f"/Tooth{index:02}", (.004, .005, .007),
              (.016 * math.cos(a), .016 * math.sin(a), -.004), BLACK, collide=False)
    arm = np.array([math.cos(C.CRANK_ANGLE), math.sin(C.CRANK_ANGLE), 0.])
    rot = np.array([[arm[0], -arm[1], 0.], [arm[1], arm[0], 0.], [0., 0., 1.]])
    w.box(s, crank + "/Wing", (.036, .010, .008),
          .015 * arm + [0., 0., .010], STEEL, rotation=rot)
    w.cylinder(s, crank + "/Knob", C.KNOB_WIDTH / 2., C.KNOB_HEIGHT,
               C.CRANK_RADIUS * arm + [0., 0., .028], RUBBER)
    spool = joint(w, s, w.Phy.RevoluteJoint, "Wind", base, crank, C.SPINDLE,
                  (0., math.degrees(C.ANGULAR_STOP)))
    drive(w, spool, "angular", 0., .24, .006, .64)
    return w.save(s)


def fabric(w):
    s = w.stage("fabric")
    mesh = w.Geom.Mesh.Define(s, "/Asset/WovenNet")
    # One opaque edge tape and visibly open grid lines.
    sections = 120
    levels = 12
    count = sections * (levels + 1) + (sections + 1) * levels
    for path, faces, color in (
        (mesh, count, MESH),
        (w.Geom.Mesh.Define(s, "/Asset/TopTape"), sections, WHITE),
        (w.Geom.Mesh.Define(s, "/Asset/CordTail"), 2, CORD),
    ):
        path.CreateFaceVertexCountsAttr([4] * faces)
        path.CreateFaceVertexIndicesAttr(list(range(4 * faces)))
        path.CreateSubdivisionSchemeAttr("none")
        path.CreateDoubleSidedAttr(True)
        path.CreateDisplayColorAttr([w.Gf.Vec3f(*color)])
    update_fabric(s, "/Asset", C.NEAR_HEIGHT, 0.)
    return w.save(s)


def update_fabric(stage, root, near_height, force):
    from pxr import Gf, UsdGeom

    u = np.linspace(0., C.POST_SPAN, 121)
    z = C.TABLE_HEIGHT + C.net_height(u, near_height, C.HEIGHT_STOP, force)
    z = np.maximum(C.TABLE_HEIGHT + .014, z)
    xs = C.POST_X + u
    y = C.POST_AXIS[1]
    y_plane = y - .0006
    bottom = C.TABLE_HEIGHT + .002
    points = []
    tape = []
    for i in range(120):
        x_a, x_b = xs[i:i + 2]
        z_a, z_b = z[i:i + 2]
        for row in range(13):
            t = row / 12.
            a = bottom + t * (z_a - bottom)
            b = bottom + t * (z_b - bottom)
            points.extend([(x_a, y_plane, a), (x_b, y_plane, b),
                           (x_b, y_plane, b + .0018), (x_a, y_plane, a + .0018)])
        tape.extend([(x_a, y, z_a - .010), (x_b, y, z_b - .010),
                     (x_b, y, z_b), (x_a, y, z_a)])
    for x, top in zip(xs, z, strict=True):
        for row in range(12):
            a = bottom + row / 12. * (top - bottom)
            b = bottom + (row + 1) / 12. * (top - bottom)
            points.extend([(x, y_plane, a), (x + .0018, y_plane, a),
                           (x + .0018, y_plane, b), (x, y_plane, b)])

    cord = np.array([C.POST_X, y, C.TABLE_HEIGHT + near_height - .001])
    reel = C.SPINDLE + [0., 0., .009]
    def ribbon(q):
        offset = np.asarray(q)
        return [cord - offset, reel - offset, reel + offset, cord + offset]

    for name, values in (("WovenNet", points), ("TopTape", tape),
                         ("CordTail", ribbon([.0012, 0., 0.]) + ribbon([0., .0012, 0.]))):
        UsdGeom.Mesh.Get(stage, root + "/" + name).GetPointsAttr().Set(
            [Gf.Vec3f(*map(float, p)) for p in values]
        )


def create_assets(directory):
    writer = Writer(directory)
    return {
        "assets": [
            dict(name="table", type="base", file=table(writer)),
            dict(name="adjuster", type="articulation", file=adjuster(writer)),
            dict(name="fabric", type="base", file=fabric(writer)),
        ],
        "settle_actions": 8,
    }
