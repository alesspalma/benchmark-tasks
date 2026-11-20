"""Bench accessories and native football service mechanisms."""

import math

import numpy as np

from . import config as C
from .motion import rotation
from .usd_scene import Writer

BENCH = (.64, .56, .40)
RUBBER = (.095, .107, .113)
BALL_WHITE = (.92, .91, .84)
NAVY = (.075, .16, .33)
ORANGE = (.86, .25, .084)
PUMP_BLUE = (.15, .35, .43)
STEEL = (.61, .66, .70)


def display_segment(w, stage, path, first, second, radius, color):
    delta = second - first
    length = np.linalg.norm(delta)
    shape = w.Geom.Cylinder.Define(stage, path)
    shape.CreateRadiusAttr(radius)
    shape.CreateHeightAttr(float(length) + .001)
    shape.CreateDisplayColorAttr([w.Gf.Vec3f(*color)])
    shape.AddTranslateOp().Set(w.Gf.Vec3d(*map(float, .5 * (first + second))))
    direction = w.Gf.Vec3d(*map(float, delta / length))
    shape.AddOrientOp(precision=w.Geom.XformOp.PrecisionDouble).Set(
        w.Gf.Rotation(w.Gf.Vec3d(0, 0, 1), direction).GetQuat()
    )


def ring(w, s, name, centre, inner, outer, height, color, count=24, gap=0., collide=True):
    for i in range(count):
        a = math.tau * i / count
        radial = np.array([math.cos(a), math.sin(a), 0.])
        centre_i = np.asarray(centre) + (.5 * (inner + outer)) * radial
        width = 2 * outer * math.tan(math.pi / count) + gap
        w.box(s, f"{name}/Segment{i}", (outer-inner, width, height), centre_i,
              color, rotation("Z", a), collide=collide)


def table(w):
    s = w.stage("room")
    w.box(s, "/Asset/Worktop", (1.10, 1.25, .06), (.32, 0, C.TABLE - .03), BENCH)
    for i, x in enumerate((-.16, .80)):
        for j, y in enumerate((-.57, .57)):
            w.box(s, f"/Asset/Leg{i}{j}", (.058, .058, C.TABLE - .06),
                  (x, y, (C.TABLE - .06) / 2), (.20, .21, .23))
    w.box(s, "/Asset/Floor", (8., 8., .02), (0, 0, -.01), (.32, .36, .35))
    x, y = C.BALL[:2]
    ring(w, s, "/Asset/BallRing", (x, y, C.RING_TOP-.012), C.RING_INNER, .076, .024, RUBBER, count=36)
    x, y = C.NEEDLE[:2]
    w.cylinder(s, "/Asset/NeedleDockFoot", .029, .008, (x, y, C.TABLE+.004), RUBBER)
    w.cylinder(s, "/Asset/NeedleDockColumn", .011, .042, (x, y, C.TABLE+.029), PUMP_BLUE)
    ring(w, s, "/Asset/NeedleDockSocket", (x, y, C.NEEDLE[2]-.015), .00165, .012,
         .028, PUMP_BLUE, count=16, gap=-.0002)
    x, y, z = C.PUMP
    for i, (shift, rise) in enumerate(((-.010, .026), (-.138, .035))):
        w.box(s, f"/Asset/PumpRest{i}", (.067, .028, rise),
              (x, y+shift, C.TABLE+.5*rise), RUBBER)
        for j, side in enumerate((-1., 1.)):
            w.box(s, f"/Asset/PumpRest{i}/Pad{j}", (.018, .028, .029),
                  (x+side*(.026 if i == 0 else .018), y+shift, z-.023), RUBBER,
                  rotation("Y", -side*.50))
    points = C.hose_points(C.PUMP + C.PUMP_ROT @ C.HOSE_ORIGIN, C.NEEDLE + [0, 0, .027])
    for i, (first, second) in enumerate(zip(points[:-1], points[1:])):
        display_segment(w, s, f"/Asset/PumpDisplay/Hose{i:02d}", first, second, .0037, (.12, .15, .17))
    pointer = C.PUMP + C.PUMP_ROT @ C.GAUGE + [0, 0, .0014]
    display_segment(w, s, "/Asset/PumpDisplay/Pointer", pointer,
                    pointer + [-.012, .012, 0.], .001, (.89, .21, .11))
    return s


def shell(w, s, path, profile, *, decorate):
    n = 40
    points = []
    for z, radius in profile:
        points.extend(w.Gf.Vec3f(radius * math.cos(math.tau * j / n),
                                 radius * math.sin(math.tau * j / n), z)
                      for j in range(n))
    faces = [list(reversed(range(n)))]
    for i in range(len(profile) - 1):
        for j in range(n):
            a, b = i * n + j, i * n + (j + 1) % n
            faces.append([a, b, b + n, a + n])
    if not decorate:
        faces.append(list(range(len(points)-n, len(points))))
    g = w.Geom.Mesh.Define(s, path)
    g.CreatePointsAttr(points)
    g.CreateFaceVertexCountsAttr([len(face) for face in faces])
    g.CreateFaceVertexIndicesAttr([p for face in faces for p in face])
    g.CreateSubdivisionSchemeAttr("none")
    if decorate:
        g.CreateDoubleSidedAttr(True)
        c = w.Geom.PrimvarsAPI(g).CreatePrimvar(
            "displayColor", w.Sdf.ValueTypeNames.Color3fArray, w.Geom.Tokens.uniform
        )
        colors = [NAVY]
        for i in range(len(profile)-1):
            for j in range(n):
                # Feint alternating panel seams and distinct dark shell motifs.
                kind = (j // 8 + (i + 3) // 5) % 4
                inset = j % 8 in (1, 2, 3, 4, 5) and i % 5 in (0, 1, 2)
                colors.append((NAVY if kind == 1 else ORANGE) if inset and kind in (1, 2) else BALL_WHITE)
        c.Set([w.Gf.Vec3f(*color) for color in colors])
    else:
        w.Phy.MeshCollisionAPI.Apply(g.GetPrim()).CreateApproximationAttr("convexHull")
        w._finish(s, g, BALL_WHITE, True)
        g.MakeInvisible()


def football(w):
    from pxr import Sdf

    w.Sdf = Sdf
    s = w.stage("football")
    body = w.body(s, "/Asset/Shell", .427, ccd=True)
    r = C.BALL_R
    contact_z = r - .024
    physical = np.linspace(-r+.0004, contact_z, 17)
    shell(w, s, body+"/RoundedHull",
          [(z, math.sqrt(r*r-z*z)) for z in physical], decorate=False)
    rendered = np.unique(np.r_[np.linspace(-r+.0002, r*np.cos(.061), 27),
                               r-.0004])
    shell(w, s, body+"/PanelledShell",
          [(z, math.sqrt(r*r-z*z)) for z in rendered], decorate=True)
    ring(w, s, body+"/ValveMouth", (0., 0., r-.002), .00355, .0075, .004,
         RUBBER, count=16, gap=.0002)
    ring(w, s, body+"/ValveRubber", (0., 0., r-.0125), .00240, .0075, .017,
         RUBBER, count=16, gap=.0002)
    ring(w, s, body+"/ValveLip", (0., 0., r-.0007), .00260, .0037, .0014,
         ORANGE, count=14, collide=False)
    return s


def needle(w):
    s = w.stage("needle")
    body = w.body(s, "/Asset/Chuck", .018, ccd=True)
    w.cylinder(s, body + "/Stem", C.NEEDLE_R, C.NEEDLE_TIP,
               (0, 0, -C.NEEDLE_TIP/2), STEEL)
    w.rounded_box(s, body + "/ValveChuck", (.015, .017, .018), (0, 0, .0090),
                  PUMP_BLUE, radius=.002, bevel=.0007)
    w.cylinder(s, body + "/HoseTail", .0045, .010, (0, 0, .022), RUBBER)
    return s


def pump(w):
    s = w.stage("pump")
    barrel = w.body(s, "/Asset/Barrel", .160, articulation=True, ccd=True)
    w.cylinder(s, barrel + "/Cylinder", .015, C.PUMP_TOP - .011,
               (0, 0, (C.PUMP_TOP+.004)/2), PUMP_BLUE)
    w.cylinder(s, barrel + "/Foot", .025, .013, (0, 0, .0085), RUBBER)
    w.cylinder(s, barrel + "/UpperCollar", .0175, .012, (0, 0, C.PUMP_TOP-.006), RUBBER)
    w.cylinder(s, barrel + "/LowerCollar", .0175, .010, (0, 0, .033), RUBBER)
    w.cylinder(s, barrel + "/Outlet", .005, .020, (.018, 0, C.HOSE_ORIGIN[2]), STEEL, axis="X")
    w.box(s, barrel + "/GaugeMount", (.025, .016, .013),
          (.022, 0, C.GAUGE[2]), RUBBER, collide=False)
    gauge_front = C.GAUGE_FRAME[:, 2]
    w.cylinder(s, barrel + "/GaugeBody", .023, .011, C.GAUGE - .007*gauge_front,
               RUBBER, axis="Y", collide=False)
    w.cylinder(s, barrel + "/GaugeFace", .020, .002, C.GAUGE - .001*gauge_front,
               BALL_WHITE, axis="Y", collide=False)
    for i in range(17):
        theta = math.radians(135 - 270*i/16)
        r = .017
        p = C.GAUGE + C.GAUGE_FRAME @ [math.cos(theta)*r, math.sin(theta)*r, .0005]
        green = .79 <= .70 + .20*i/16 <= .825
        w.box(s, barrel+f"/DialMark{i}", (.0035, .0012, .0006), p,
              (.11, .55, .17) if green else RUBBER, C.GAUGE_FRAME @ rotation("Z", theta), collide=False)
    rod = w.body(s, "/Asset/Rod", .046,
                 position=(0, 0, C.PISTON_HOME), ccd=True)
    w.cylinder(s, rod + "/PistonRod", .005, C.STROKE + .009,
               (0, 0, -.5 * (C.STROKE + .009)), STEEL, collide=False)
    w.box(s, rod + "/Handle", (.060, .022, .022), (0, 0, 0), ORANGE)
    slide = w.Phy.PrismaticJoint.Define(s, "/Asset/Stroke")
    slide.CreateBody0Rel().SetTargets([barrel])
    slide.CreateBody1Rel().SetTargets([rod])
    slide.CreateLocalPos0Attr(w.Gf.Vec3f(0, 0, C.PISTON_HOME))
    slide.CreateAxisAttr("Z")
    slide.CreateLowerLimitAttr(0.)
    slide.CreateUpperLimitAttr(C.STROKE)
    slide.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(slide.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(0.)
    drive.CreateDampingAttr(19.)
    drive.CreateMaxForceAttr(18.)
    w.Px.PhysxJointAPI.Apply(slide.GetPrim()).CreateJointFrictionAttr(.30)
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = []
    for name, kind, stage, position in (
        ("table", "base", table(w), (0, 0, 0)),
        ("ball", "rigid", football(w), C.BALL),
        ("needle", "rigid", needle(w), C.NEEDLE),
        ("pump", "articulation", pump(w), C.PUMP+[0, 0, .004]),
    ):
        assets.append(dict(name=name, type=kind, file=w.save(stage),
                           position=list(position),
                           quaternion=C.PUMP_QUAT if name == "pump" else [0., 0., 0., 1.]))
    return {"assets": assets, "settle_actions": 15}
