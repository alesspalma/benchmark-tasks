"""Original rigid scale, native hinge, parallel captive poises and load."""

from . import config as C
from .usd_scene import Writer

BODY = (.23, .29, .33)
STEEL = (.65, .69, .72)
BEAM = (.85, .80, .66)
RIDER = (.27, .30, .32)
WHITE = (.92, .93, .91)
INK = (.07, .075, .076)
RED = (.80, .11, .07)


def inertia(w, stage, path, centre, diagonal):
    mass = w.Phy.MassAPI.Get(stage, path)
    mass.CreateCenterOfMassAttr(w.Gf.Vec3f(*map(float, centre)))
    mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(*map(float, diagonal)))


def connection(w, joint, parent, child, axis, origin, limit):
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateAxisAttr(axis)
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, origin)))
    joint.CreateLowerLimitAttr(limit[0])
    joint.CreateUpperLimitAttr(limit[1])
    joint.CreateCollisionEnabledAttr(False)
    return joint


def dash(w, stage, body, name, size, position, color=INK):
    w.box(stage, f"{body}/{name}", size, position, color, collide=False)


def mark(w, stage, body, text, x, y, z, scale=.00125):
    segments = (
        (0, -1, 1, 0), (1, -.5, 0, 1), (1, .5, 0, 1),
        (0, 1, 1, 0), (-1, .5, 0, 1), (-1, -.5, 0, 1), (0, 0, 1, 0),
    )
    glyphs = ("012345", "12", "01643", "01236", "5612", "05623", "056234", "012", "0123456", "012356")
    for digit, char in enumerate(str(text)):
        for idx in glyphs[int(char)]:
            dx, dy, sx, sy = segments[int(idx)]
            dash(w, stage, body, f"Digit{round(x*1e5)}_{round(y*1e5)}_{digit}_{idx}",
                 (scale * (1.6 * sx + .22), scale * (1.1 * sy + .22), .00020),
                 (x + scale * dy, y + scale * (dx + 2.8 * digit), z))


def scale(w):
    stage = w.stage("scale")
    base = w.body(stage, "/Asset/Base", 2.1, fixed=True, articulation=True)
    w.box(stage, base + "/Foot", (.161, .418, .025), C.PIVOT + [0, -.043, -.092], BODY)
    w.box(stage, base + "/PivotHousing", (.109, .051, .081), C.PIVOT + [0, 0, -.053], BODY)
    w.box(stage, base + "/PanHousing", (.102, .143, .057), C.PIVOT + [0, .083, -.065], BODY)
    w.box(stage, base + "/ReadoutTower", (.047, .028, .103), C.PIVOT + [0., -.244, -.053], BODY)
    w.box(stage, base + "/IndexPlate", (.042, .010, .029), C.PIVOT + [0., -.253, .001], WHITE)
    dash(w, stage, base, "ZeroBar", (.038, .011, .0013), C.PIVOT + [0., -.253, 0.], INK)

    beam = w.body(stage, "/Asset/Beam", C.BEAM_MASS, position=C.PIVOT)
    # Factory zero adjustment balances the empty pan with all three riders at zero.
    com_y = -(sum(r.mass * r.start_y for r in C.RIDERS) + C.PAN_MASS * C.PAN_LEVER) / C.BEAM_MASS
    inertia(w, stage, beam, (0, com_y, -.063), (.009, .003, .009))
    w.box(stage, beam + "/RailCarrier", (.078, .238, .008), (0, -.121, -.005), BODY)
    w.box(stage, beam + "/PanArm", (.038, .107, .011), (0, .053, -.004), STEEL)
    # A vertical pan guide represents the instrument's parallel-link
    # support. Its spherical thrust foot loads the real moving lever.
    offset = (0., C.PAN_LEVER, 0.)
    pan = w.body(stage, "/Asset/Pan", C.PAN_MASS, position=C.PIVOT + offset)
    guide = w.Phy.PrismaticJoint.Define(stage, "/Asset/PanGuide")
    connection(w, guide, base, pan, "Z", C.PIVOT + offset, (-.012, .012))
    foot = w.Geom.Sphere.Define(stage, pan + "/ThrustBearing")
    foot.CreateRadiusAttr(.007)
    foot.AddTranslateOp().Set(w.Gf.Vec3d(0, 0, .0085))
    w._finish(stage, foot, STEEL, True)
    w.cylinder(stage, pan + "/PanStem", .018, .039, (0, 0, .027), STEEL)
    w.cylinder(stage, pan + "/PanPlate", C.PAN_RADIUS, .004,
               (0, 0, C.PAN_Z - .002), STEEL)
    w.ring(stage, pan + "/PanRim", .0747, .0022, .003, C.PAN_Z + .001,
           STEEL, n=64, collide=False)
    w.box(stage, beam + "/Pointer", (.006, .026, .0016), (-.009, -.234, 0.), RED)
    pivot = connection(w, w.Phy.RevoluteJoint.Define(stage, "/Asset/Pivot"), base,
                       beam, "X", C.PIVOT, (-C.LIMIT_DEG, C.LIMIT_DEG))
    damp = w.Phy.DriveAPI.Apply(pivot.GetPrim(), "angular")
    damp.CreateTypeAttr("force")
    damp.CreateStiffnessAttr(0.)
    damp.CreateDampingAttr(.045)
    damp.CreateTargetVelocityAttr(0.)
    damp.CreateMaxForceAttr(.20)

    for r in C.RIDERS:
        # The stepped cross-section keeps the three slider paths individually accessible.
        rail_z = r.z - r.size[2] / 2 - .003
        w.box(stage, beam + "/Rail" + r.name, (.019, .225, .005),
              (r.x, -.122, rail_z), BEAM)
        divisions = round(r.capacity_g / r.tick_g)
        for i in range(divisions + 1):
            y = r.start_y - r.travel * i / divisions
            major = divisions <= 10 or i % 10 == 0
            dash(w, stage, beam, f"Tick{r.name}{i}",
                 (.006 if major else .003, .00055, .00018),
                 (r.x - .0054, y, rail_z + .0026))
            if major and (divisions <= 10 or i % 20 == 0):
                label = round(i * r.tick_g)
                mark(w, stage, beam, label, r.x + .004, y, rail_z + .0027,
                     scale=.00067 if divisions == 100 else .00084)

        offset = (r.x, r.start_y, r.z)
        poise = w.body(stage, "/Asset/Poise" + r.name, r.mass,
                       position=C.PIVOT + offset)
        w.box(stage, poise + "/Body", r.size, (0, 0, 0), RIDER)
        dash(w, stage, poise, "Index", (.007, .0010, .00021), (0, 0, r.size[2] / 2), WHITE)
        joint = connection(w, w.Phy.PrismaticJoint.Define(stage, "/Asset/" + r.name),
                           beam, poise, "Y", offset, (-r.travel, 0.))
        w.Px.PhysxJointAPI.Apply(joint.GetPrim()).CreateJointFrictionAttr(.26)
        resistance = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
        resistance.CreateTypeAttr("force")
        resistance.CreateStiffnessAttr(0.)
        resistance.CreateDampingAttr(1.8)
        resistance.CreateTargetVelocityAttr(0.)
        resistance.CreateMaxForceAttr(2.0)
    return stage


def specimen(w):
    stage = w.stage("specimen")
    body = w.body(stage, "/Asset/Body", C.SPECIMEN_MASS, ccd=True)
    w.cylinder(stage, body + "/Specimen", C.SPECIMEN_RADIUS, C.SPECIMEN_HEIGHT,
               (0, 0, C.SPECIMEN_HEIGHT / 2), (.35, .46, .55))
    return stage


def create_assets(directory):
    w = Writer(directory)
    assets = [
        dict(name="scale", type="articulation", file=w.save(scale(w)), position=[0., 0., 0.]),
        dict(name="specimen", type="rigid", file=w.save(specimen(w)),
             position=(C.PIVOT + [0, C.PAN_LEVER, C.PAN_Z + .002]).tolist()),
    ]
    w.inventory()
    return dict(assets=assets, settle_actions=36)
