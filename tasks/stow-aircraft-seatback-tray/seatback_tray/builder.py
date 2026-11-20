"""Full-size, passively hinged mechanical seatback and physical rotary keeper."""

import numpy as np

from . import config as C
from .motion import rotation
from .usd_scene import Writer

WARM_GRAY = (.67, .63, .54)
WARM_EDGE = (.52, .48, .41)
DARK_BLUE = (.12, .20, .28)
SEAT_GRAY = (.28, .31, .33)
ALUMINUM = (.42, .45, .47)
LATCH = (.17, .185, .20)


def revolute(w, stage, name, parent, child, pivot, axis, limits, *, friction, damping):
    joint = w.Phy.RevoluteJoint.Define(stage, f"/Asset/{name}")
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, pivot)))
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(float(limits[0]))
    joint.CreateUpperLimitAttr(float(limits[1]))
    w.Px.PhysxJointAPI.Apply(joint.GetPrim()).CreateJointFrictionAttr(friction)
    damper = w.Phy.DriveAPI.Apply(joint.GetPrim(), "angular")
    damper.CreateTypeAttr("force")
    damper.CreateDampingAttr(damping)
    damper.CreateStiffnessAttr(0.)
    return joint


def station(w):
    s = w.stage("station")
    # The arm bases sit on narrow laboratory stands, leaving the full cabin
    # seat and deployed table clear below their ordinary operating height.
    for i, y in enumerate((-.305, .305)):
        w.box(s, f"/Asset/Pedestal{i}", (.21, .195, .72),
              (-.045, y, .36), (.20, .22, .24))
        w.box(s, f"/Asset/BasePlate{i}", (.22, .20, .03),
              (-.045, y, .735), (.29, .31, .32))
    w.box(s, "/Asset/Floor", (6., 6., .02), (0, 0, -.01), (.36, .40, .42))
    return s


def seat(w):
    s = w.stage("seatback")
    root = w.body(s, "/Asset/Seat", 12.0, fixed=True, articulation=True)
    w.rounded_box(s, root + "/Cushion", (.430, .463, .115),
                  (.785, 0., .463), DARK_BLUE, .040, .009)
    frame = rotation("Y", -np.radians(C.SEAT_LEAN))
    spine = C.BACK_ORIGIN + frame @ [.039, 0., .106]
    w.box(s, root + "/SeatbackShell", (.078, .462, .570), spine, SEAT_GRAY, frame)
    w.box(s, root + "/Upholstery", (.045, .440, .552),
          C.BACK_ORIGIN + frame @ [.094, 0., .110], DARK_BLUE, frame)
    w.box(s, root + "/Headrest", (.082, .362, .100),
          C.BACK_ORIGIN + frame @ [.084, 0., .356], DARK_BLUE, frame)
    for i, y in enumerate((-.209, .209)):
        w.box(s, f"{root}/SeatLeg{i}", (.045, .039, .421), (.747, y, .2105), ALUMINUM)
        w.box(s, f"{root}/FloorRail{i}", (.387, .051, .026), (.785, y, .013), ALUMINUM)
        w.cylinder(s, f"{root}/Hinge{i}", .017, .032, C.PIVOT + [0., y, 0.],
                   ALUMINUM, "Y", collide=False)

    tray = w.body(s, "/Asset/Tray", C.TRAY_MASS, C.PIVOT, ccd=True)
    w.rounded_box(s, tray + "/TrayPanel", (C.DEPTH, C.WIDTH, C.THICK),
                  (-C.DEPTH / 2, 0., 0.), WARM_GRAY, .028, .003)
    # The pair of aluminum pivot ears is rigidly attached to this table type.
    for i, y in enumerate((-.208, .208)):
        w.box(s, f"{tray}/SupportArm{i}", (.132, .014, .012),
              (-.022, y, -.013), ALUMINUM)
    w.rounded_box(s, tray + "/EdgeBand", (.237, .395, .0024),
                  (-.131, 0., -.0087), WARM_EDGE, .022, .0006)
    w.rounded_box(s, tray + "/UpperInset", (.220, .378, .0018),
                  (-.137, 0., .0084), (.70, .66, .58), .025, .0004, collide=False)
    w.cylinder(s, tray + "/CupIndex", .034, .0007,
               (-.081, -.136, .0095), (.60, .57, .51), collide=False)
    revolute(w, s, "TrayPivot", root, tray, C.PIVOT, "Y", (0., C.STOW_ANGLE),
             friction=.075, damping=.003)

    cam = w.body(s, "/Asset/Latch", .029, C.LATCH_PIVOT, ccd=True)
    w.Phy.MassAPI(s.GetPrimAtPath(cam)).CreateCenterOfMassAttr(w.Gf.Vec3f(0., 0., 0.))
    w.cylinder(s, cam + "/SpindleCap", .014, .022, (.002, 0., 0.), LATCH, "X")
    w.box(s, cam + "/Keeper", (.014, C.LATCH_LENGTH, .019),
          (-.003, -C.LATCH_LENGTH / 2, 0.), LATCH)
    w.cylinder(s, cam + "/RoundedTip", .0095, .014,
               (-.003, -C.LATCH_LENGTH, 0.), LATCH, "X")
    revolute(w, s, "LatchPivot", root, cam, C.LATCH_PIVOT, "X", (0., 91.),
             friction=.35, damping=.00022)
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [{"name": "station", "type": "base", "file": w.save(station(w))},
              {"name": "seatback", "type": "articulation", "file": w.save(seat(w))}]
    w.inventory()
    return {"assets": assets, "settle_actions": 16}
