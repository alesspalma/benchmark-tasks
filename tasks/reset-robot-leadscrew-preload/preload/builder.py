"""Physical bench, floating counterhold housing, coaxial ramp collar and screw."""

import math

from . import config as C
from .motion import rotation
from .usd_scene import Writer

OAK = (.56, .46, .32)
CHARCOAL = (.115, .16, .18)
STEEL = (.62, .64, .66)
BRASS = (.79, .54, .22)
BLUE = (.13, .39, .57)
ORANGE = (.91, .35, .14)


def table(w):
    s = w.stage("room")
    w.box(s, "/Asset/Worktop", (1.10, 1.25, .060), (.32, 0, C.TABLE - .030), OAK)
    for i, x in enumerate((-.16, .80)):
        for j, y in enumerate((-.57, .57)):
            w.box(s, f"/Asset/Leg{i}{j}", (.058, .058, C.TABLE - .06),
                  (x, y, (C.TABLE - .06) / 2), CHARCOAL)
    w.box(s, "/Asset/Floor", (8., 8., .02), (0, 0, -.01), (.32, .36, .35))
    return s


def hinge(w, s, name, a, b, local_a, local_b, limits):
    j = w.Phy.RevoluteJoint.Define(s, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([a])
    j.CreateBody1Rel().SetTargets([b])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, local_a)))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(*map(float, local_b)))
    j.CreateAxisAttr("X")
    j.CreateCollisionEnabledAttr(False)
    j.CreateLowerLimitAttr(float(limits[0]))
    j.CreateUpperLimitAttr(float(limits[1]))
    if name != "HousingAngle":
        damper = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular")
        damper.CreateStiffnessAttr(0.)
        damper.CreateDampingAttr(.008 if name == "RampAngle" else .00014)
        damper.CreateMaxForceAttr(.04)
    return j


def stage(w):
    s = w.stage("drive")
    root = w.body(s, "/Asset/Frame", 3., fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(s, root).CreateEnabledSelfCollisionsAttr(False)
    x, y, z = C.AXIS
    w.rounded_box(s, root + "/Base", (.360, .125, .010),
                  (x+.013, y, C.TABLE+.005), BLUE, radius=.008, bevel=.001)
    for i, dx in enumerate((-.135, .165)):
        w.box(s, f"{root}/BearingStand{i}", (.032, .065, .105),
              (x+dx, y, C.TABLE+.06), BLUE)
        w.cylinder(s, f"{root}/Bearing{i}", .024, .017,
                   (x+dx, y, z), STEEL, axis="X")

    case = w.body(s, "/Asset/NutHousing", .265, position=C.AXIS, ccd=True)
    # The concentric guide sleeve hides the internal ramp thrust path but
    # leaves the rotating collar and the front nut witness marks exposed.
    w.cylinder(s, case+"/GuideSleeve", .016, .018, (.023, 0, 0),
               CHARCOAL, axis="X", collide=False)
    # The service extension models the flanged front nut's gripping flats.
    w.cylinder(s, case+"/Main", .022, .071, (.066, 0, 0), CHARCOAL, axis="X")
    w.rounded_box(s, case+"/Flats", (.036, .023, .047),
                  (.090, 0, 0), CHARCOAL, radius=.004, bevel=.002)
    w.box(s, case+"/Witness", (.013, .0023, .001), (.023, 0, .016), (.90, .9, .86), collide=False)
    j = hinge(w, s, "HousingAngle", root, case, C.AXIS, (0, 0, 0), (-115, 115))
    spin = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular")
    spin.CreateStiffnessAttr(0.)
    spin.CreateDampingAttr(.009)
    spin.CreateMaxForceAttr(.010)

    cam = w.body(s, "/Asset/RampCam", .095, position=C.AXIS, ccd=True)
    w.cylinder(s, cam+"/Ring", .024, .026, (0, 0, 0), BRASS, axis="X")
    # Two axial flats are part of the removable hand collar. Bracing on the
    # left lets the right jaws rotate the ring against its return spring.
    w.rounded_box(s, cam+"/WrenchFlats", (.024, .040, .016),
                  (0, 0, 0), BRASS, radius=.004, bevel=.001)
    for i in range(24):
        a = i * math.tau / 24
        w.box(s, f"{cam}/Knurl{i}", (.021, .0022, .0016),
              (0, .023*math.sin(a), .023*math.cos(a)), BRASS,
              rotation("X", -a), collide=False)
    w.box(s, cam+"/Indicator", (.019, .0023, .001), (0, 0, .025), ORANGE, collide=False)
    hinge(w, s, "RampAngle", case, cam, (0, 0, 0), (0, 0, 0), (-39, 75))

    shaft = w.body(s, "/Asset/LeadScrew", .16, position=C.AXIS, ccd=True)
    w.cylinder(s, shaft+"/Core", .0078, .337, (.014, 0, 0), STEEL, axis="X")
    # The reducer models lead-screw flank drag. The fine ridges only show
    # the steel shaft and its trapezoidal crest in the native observations.
    for i in range(76):
        dx = .004 * i - .142
        w.cylinder(s, f"{shaft}/Thread{i}", .008, .0014, (dx, 0, 0),
                   STEEL, axis="X", collide=False)
    w.cylinder(s, shaft+"/DriveWheel", .018, .010, (-.164, 0, 0), STEEL, axis="X")
    w.box(s, shaft+"/WheelMark", (.012, .002, .002),
          (-.164, 0, .0168), ORANGE, collide=False)
    hinge(w, s, "ScrewAngle", root, shaft, C.AXIS, (0, 0, 0), (-120, 120))
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [
        dict(name="table", type="base", file=w.save(table(w)), position=[0., 0., 0.]),
        dict(name="drive", type="articulation", file=w.save(stage(w)), position=[0., 0., 0.]),
    ]
    return {"assets": assets, "settle_actions": 15}
