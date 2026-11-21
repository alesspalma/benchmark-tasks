"""Original fixtures, tray, jointed pump and vacuum pickup pen."""

from math import cos, pi, sin

import numpy as np

from . import config as C
from .motion import rotation
from .usd_scene import Writer

DARK = (.075, .091, .098)
BLUE = (.032, .27, .53)
GEL = (.59, .31, .18)
ALUMINUM = (.58, .63, .66)
GREEN = (.10, .51, .28)
RED = (.68, .12, .09)
AMBER = (.94, .46, .10)


def name(identifier):
    return "/Asset/" + identifier


def ring(w, s, path, center, radius, width, height, color):
    for i in range(24):
        angle = 2 * pi * i / 24
        pos = np.asarray(center) + [radius*cos(angle), radius*sin(angle), 0.]
        w.box(s, f"{path}{i:02}", (width, .265*radius, height), pos, color,
              rotation("Z", angle))


def make_bench(w):
    s = w.stage("bench")
    w.box(s, name("Top"), (1.0, 1.10, .045), (.35, 0., C.TABLE-.0225), (.57, .63, .62))
    w.box(s, name("ESDMat"), (.37, .44, .002), (.39, -.004, C.TABLE), (.26, .38, .37),
          collide=False)
    w.box(s, name("Floor"), (6., 6., .020), (0., 0., -.010), (.28, .33, .35))
    for i, x in enumerate((-.092, .791)):
        for j, y in enumerate((-.494, .494)):
            w.box(s, name(f"Leg{i}{j}"), (.035, .035, C.TABLE-.045),
                  (x, y, (C.TABLE-.045)/2), DARK)
    for i, side in enumerate((-1, 1)):
        w.box(s, name(f"StagingGuide{i}"), (.084, .003, .007),
              (C.TRAY_START[0]+.006, C.TRAY_START[1]+side*.053, C.TABLE+.0035), ALUMINUM)
    w.box(s, name("StagingGuideEnd"), (.004, .097, .007),
          (C.TRAY_START[0]+.054, C.TRAY_START[1], C.TABLE+.0035), ALUMINUM)
    w.rounded_box(s, name("VacuumPlate"), (.125, .119, .016), C.PLATE-[0., 0., .009],
                  ALUMINUM, radius=.004, bevel=.001)
    ring(w, s, name("BackSeal"), C.PLATE-[0., 0., .0006], C.SEAL_RADIUS, .0035, .0012, DARK)
    w.cylinder(s, name("VacuumPort"), .004, .0007, C.PLATE-[0., 0., .0006], DARK, collide=False)
    # Recessed guides leave the edge of the seated tray available to the gripper.
    for side in (-1, 1):
        w.box(s, name(f"SideGuide{side+1}"), (.065, .004, .006),
              C.PLATE+[.012, .057*side, .0025], ALUMINUM)
    w.box(s, name("EndGuide"), (.005, .096, .006), C.PLATE+[.057, 0., .0025], ALUMINUM)
    # Rigid guide marks on the matched plate and on the pickup destination.
    w.box(s, name("Index"), (.010, .006, .0006), C.PLATE+[-.060, -.049, -.00075],
          BLUE, collide=False)
    w.rounded_box(s, name("WafflePack"), (.057, .052, .006), C.RECEIVER-[0., 0., .003],
                  DARK, radius=.004, bevel=.001)
    half = C.POCKET_WIDTH/2
    for i, side in enumerate((-1, 1)):
        w.box(s, name(f"PocketWide{i}"), (.058, .010, .003), C.RECEIVER+[0., side*(half+.005), .0015],
              BLUE)
        w.box(s, name(f"PocketEnd{i}"), (.010, .023, .003), C.RECEIVER+[side*(half+.005), 0., .0015],
              BLUE)
    for i, side in enumerate((-1, 1)):
        w.box(s, name(f"PenCradle{i}"), (.036, .008, .050),
              (C.PEN[0], C.PEN[1]+.0085*side, C.TABLE+.025), DARK)
    w.box(s, name("PenRestFloor"), (.041, .024, .002), C.PEN*[1., 1., 0.]+[0., 0., C.TABLE+.001],
          DARK)
    for i, side in enumerate((-1, 1)):
        w.box(s, name(f"NozzleGuide{i}"), (.005, .021, .029),
              (C.PEN[0]+.0065*side, C.PEN[1], C.TABLE+.0145), DARK)
    w.box(s, name("PumpMount"), (.092, .051, .027), C.PUMP+[.009, 0., .0135], DARK)
    # A dark hose follows the worktop from the fixed pump body to the plate port.
    for i, (length, center, axis) in enumerate((
        (.073, (.440, .211, .763), "X"),
        (.155, (.475, .135, .763), "Y"),
        (.042, (.454, .057, .763), "X"),
    )):
        w.cylinder(s, name(f"VacuumHose{i}"), .0032, length, center, DARK, axis=axis, collide=False)
    return s


def make_tray(w):
    s = w.stage("tray")
    p = w.body(s, name("Carrier"), .081, ccd=True)
    w.rounded_box(s, p+"/PolypropyleneBody", (.102, .102, .009),
                  (0., 0., -.0005), DARK, radius=.003, bevel=.0007)
    w.box(s, p+"/GelMembrane", (.084, .084, .001), (0., 0., .0045), GEL)
    for i, side in enumerate((-1, 1)):
        w.box(s, p+f"/RailX{i}", (.005, .099, .011), (.047*side, 0., .003), DARK)
        w.box(s, p+f"/RailY{i}", (.089, .004, .008), (0., .048*side, .0015), DARK)
    w.rounded_box(s, p+"/LipBridge", (.019, .038, .004), (-.053, -.006, .0005),
                  DARK, radius=.003, bevel=.0006)
    w.rounded_box(s, p+"/CarrierLip", (.014, .038, .011), (-.061, -.006, .007),
                  DARK, radius=.003, bevel=.001)
    w.box(s, p+"/BlueCornerIndex", (.010, .010, .0006), (-.038, -.038, .0053),
          BLUE, collide=False)
    return s


def make_die(w, identifier, tint):
    s = w.stage(identifier)
    p = w.body(s, name("Die"), C.DIE_MASS, ccd=True)
    w.box(s, p+"/Silicon", (C.DIE_WIDTH, C.DIE_WIDTH, C.DIE_THICKNESS),
          (0., 0., 0.), tint)
    w.Px.PhysxCollisionAPI.Get(s, p+"/Silicon").GetContactOffsetAttr().Set(.00009)
    for i, side in enumerate((-1, 1)):
        for j in range(5):
            w.box(s, p+f"/BondPad{i}{j}", (.0009, .00043, .000025),
                  (side*.0048, -.0038+.0019*j, C.DIE_THICKNESS/2+.000014),
                  (.83, .69, .29), collide=False)
    w.box(s, p+"/BackSurface", (.0119, .0119, .000015), (0., 0., -.000402),
          (.19, .20, .23), collide=False)
    return s


def make_joint(w, s, cls, name_, parent, child, anchor, axis, lo, hi):
    j = cls.Define(s, name(name_))
    j.CreateBody0Rel().SetTargets([parent])
    j.CreateBody1Rel().SetTargets([child])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, anchor)))
    j.CreateAxisAttr(axis)
    j.CreateLowerLimitAttr(lo)
    j.CreateUpperLimitAttr(hi)
    j.CreateCollisionEnabledAttr(False)
    return j


def spring(w, joint, kind, stiffness, damping, limit):
    d = w.Phy.DriveAPI.Apply(joint.GetPrim(), kind)
    d.CreateStiffnessAttr(stiffness)
    d.CreateDampingAttr(damping)
    d.CreateMaxForceAttr(limit)
    d.CreateTargetPositionAttr(0.)


def make_pump(w):
    s = w.stage("pump")
    p = w.body(s, name("Housing"), .30, fixed=True, articulation=True)
    w.cylinder(s, p+"/Cylinder", .015, .087, (.004, 0., .047), ALUMINUM, axis="X")
    w.rounded_box(s, p+"/FixedHandle", (.103, .024, .010), (-.026, 0., .028),
                  RED, radius=.0045, bevel=.001)
    w.cylinder(s, p+"/HoseBarb", .005, .014, (.054, 0., .047), ALUMINUM, axis="X")
    lever = w.body(s, name("Lever"), .033, position=C.PIVOT, ccd=True)
    w.rounded_box(s, lever+"/Grip", (.098, .021, .010), (-.046, 0., 0.),
                  RED, radius=.0045, bevel=.001)
    j = make_joint(w, s, w.Phy.RevoluteJoint, "HandleAngle", p, lever, C.PIVOT,
                   "Y", -np.degrees(C.PUMP_STROKE), 0.)
    spring(w, j, "angular", .008, .0016, .5)
    vent = w.body(s, name("Vent"), .009, position=C.VENT, ccd=True)
    w.cylinder(s, vent+"/ReleaseButton", .009, .006, (0., 0., 0.), GREEN)
    j = make_joint(w, s, w.Phy.PrismaticJoint, "VentTravel", p, vent, C.VENT, "Z",
                   -C.VENT_TRAVEL, 0.)
    spring(w, j, "linear", 890., 2.8, 12.)
    center = np.array([.030, -.022, .052])
    w.cylinder(s, p+"/GaugeRim", .022, .007, center, DARK)
    w.cylinder(s, p+"/GaugeFace", .019, .001, center+[0., 0., .004], (.93, .91, .83),
               collide=False)
    for i, angle in enumerate(np.linspace(-2.10, 2.10, 13)):
        xy = np.array([cos(angle), sin(angle), 0.])
        w.box(s, p+f"/Scale{i}", (.0029, .0011, .0005),
              center+.0163*xy+[0., 0., .0048], GREEN if i>=10 else DARK,
              rotation("Z", angle), collide=False)
    pivot = w.Geom.Xform.Define(s, p+"/PointerPivot")
    pivot.AddTranslateOp().Set(w.Gf.Vec3d(*(center+[0., 0., .0052])))
    w.box(s, p+"/PointerPivot/Needle", (.016, .0014, .0004), (.007, 0., 0.),
          RED, collide=False)
    return s


def make_pen(w):
    s = w.stage("pen")
    p = w.body(s, name("Barrel"), .027, articulation=True, ccd=True)
    w.cylinder(s, p+"/Handle", .009, .092, (-.0015, 0., -.004), ALUMINUM)
    w.cylinder(s, p+"/UpperCap", .0096, .008, (-.0015, 0., .045), DARK)
    w.cylinder(s, p+"/Probe", .0019, .045, (0., 0., -.064), ALUMINUM)
    w.box(s, p+"/StandKey", (.0068, .008, .010), (0., 0., -.065), DARK)
    w.cylinder(s, p+"/Cup", C.PEN_RADIUS, .003, C.PEN_TIP+[0., 0., .0015], DARK)
    w.Px.PhysxCollisionAPI.Get(s, p+"/Cup").GetContactOffsetAttr().Set(.00015)
    w.rounded_box(s, p+"/Countergrip", (.011, .028, .043), (-.0078, 0., .010),
                  DARK, radius=.003, bevel=.001)
    center = (.0105, 0., .010)
    squeeze = w.body(s, name("Squeeze"), .004, position=center, ccd=True)
    w.rounded_box(s, squeeze+"/Button", (.013, .028, .043), (0., 0., 0.), AMBER,
                  radius=.004, bevel=.001)
    j = make_joint(w, s, w.Phy.PrismaticJoint, "BulbTravel", p, squeeze, center, "X",
                   -C.BULB_TRAVEL, 0.)
    spring(w, j, "linear", 690., 2.7, 15.)
    return s


def create_assets(directory):
    w = Writer(directory)
    setup = [
        ("bench", "base", make_bench(w), (0., 0., 0.)),
        ("tray", "rigid", make_tray(w), C.TRAY_START),
        ("pump", "articulation", make_pump(w), C.PUMP),
        ("pen", "articulation", make_pen(w), C.PEN),
    ]
    for identifier, tint in zip(C.DIE_NAMES, ((.68, .71, .72), (.44, .50, .53), (.47, .53, .56))):
        pos = C.TRAY_START+C.DIE_OFFSETS[identifier]+[0., 0., C.DIE_THICKNESS/2+.00003]
        setup.append((identifier, "rigid", make_die(w, identifier, tint), pos))
    return {
        "assets": [
            {"name": n, "type": t, "file": w.save(stage), "position": list(pos)}
            for n, t, stage, pos in setup
        ],
        "settle_actions": 6,
    }
