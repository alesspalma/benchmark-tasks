"""Full-size steel platen, sample rest, pressure foot and spring-loaded indenter."""

from math import cos, pi, sin

from . import config as C
from .motion import rotation
from .usd_scene import Writer

STEEL = (.53, .62, .69)
DARK = (.050, .062, .071)
RUBBER = (.36, .12, .066)
TEAL = (.025, .31, .36)
DISPLAY = (.21, .32, .27)
DIGIT = (.055, .12, .087)
AMBER = (.95, .54, .12)

SEGMENTS = [
    (0, .004, .006, .0012), (.003, .002, .0012, .003),
    (.003, -.002, .0012, .003), (0, -.004, .006, .0012),
    (-.003, -.002, .0012, .003), (-.003, .002, .0012, .003),
    (0, 0, .006, .0012),
]
DIGITS = ("012345", "12", "01346", "01236", "1256", "02356", "023456", "012", "0123456", "012356")


def make_bench(w):
    s = w.stage("bench")
    w.box(s, "/Asset/Bench", (.96, 1.17, .046), (.35, 0., C.TABLE - .023), (.50, .55, .53))
    w.box(s, "/Asset/Floor", (8., 8., .02), (0., 0., -.01), (.30, .34, .36))
    w.box(s, "/Asset/Platen", C.PLATEN_SIZE, C.PLATEN - [0., 0., .005], STEEL)
    # A narrow rest frees both sides of the coupon for the gripper.
    w.box(s, "/Asset/CouponRest", (.089, .044, C.TRAY_HEIGHT),
          C.START_COUPON - [0., 0., (.010 + C.TRAY_HEIGHT) / 2], DARK)
    x, y, z = C.TOOL
    # The annular foot stands on two parallel ledges. The spring pin is in air.
    for side, offset in enumerate((-.009, .009)):
        w.box(s, f"/Asset/GaugeRest{side}", (.038, .009, .016),
              (x, y + offset, z - .008), TEAL)
    w.box(s, "/Asset/RestBase", (.051, .045, .003), (x, y, C.TABLE + .0015), TEAL)
    return s


def make_coupon(w):
    s = w.stage("coupon")
    body = w.body(s, "/Asset", .078, ccd=True)
    w.box(s, body + "/Elastomer", C.COUPON_SIZE, (0., 0., 0.), RUBBER)
    return s


def make_gauge(w):
    s = w.stage("durometer")
    body = w.body(s, "/Asset/Gauge", .155, articulation=True, ccd=True)
    w.Px.PhysxRigidBodyAPI.Apply(s.GetPrimAtPath(body)).CreateAngularDampingAttr(1.)
    w.rounded_box(s, body + "/Case", (.037, .043, .090), (0., 0., .053), TEAL, .004, .002)
    # A straight grip below the LCD keeps it visible with the tool in hand.
    for side in (-1, 1):
        w.box(s, body + f"/Grip{side + 1}", (.028, .002, .035),
              (0., side * .022, .049), DARK)
    w.cylinder(s, body + "/FootShoulder", .016, .006, (0., 0., .010), STEEL)
    contact_foot = body + "/PressureFootContact"
    w.cylinder(s, contact_foot, C.FOOT_RADIUS, .007, (0., 0., .0035), STEEL)
    # Flat face contact is equivalent for this specimen. Its annular render
    # geometry keeps the indenter opening visible and has no second collider.
    w.Geom.Imageable(s.GetPrimAtPath(contact_foot)).MakeInvisible()
    for i in range(12):
        angle = i * pi / 6
        w.box(s, body + f"/FootSector{i}", (.007, .00775, .007),
              (.0123 * cos(angle), .0123 * sin(angle), .0035), STEEL, rotation("Z", angle), collide=False)
    w.box(s, body + "/ScreenFrame", (.039, .040, .023), (0., 0., .099), DARK, collide=False)
    w.box(s, body + "/LCD", (.001, .034, .013), (.020, 0., .099), DISPLAY, collide=False)
    for digit, y in enumerate((-.010, 0., .010)):
        for index, (dy, dz, width, height) in enumerate(SEGMENTS):
            w.box(s, body + f"/LCD_{digit}_{index}", (.0003, width, height),
                         (.0207, y + dy, .099 + dz), DIGIT, collide=False)
    for index in range(5):
        w.box(s, body + f"/Memory{index}", (.0004, .0032, .0015),
              (.021, -.012 + index * .006, .089), AMBER, collide=False)
    for axis, y in enumerate((-.007, .007)):
        w.box(s, body + f"/Key{axis}", (.0012, .009, .004), (.019, y, .082),
              (.33, .42, .42), collide=False)
    pin = w.body(s, "/Asset/Indenter", .032)
    # The collision itself is compliant. See mechanics.py for the native load.
    w.cylinder(s, pin + "/Stem", .00125, .010, (0., 0., .0025), STEEL, collide=False)
    w.cylinder(s, pin + "/TruncatedTip", .00040, .0010, (0., 0., -.0020),
               STEEL, collide=False)
    w.Px.PhysxRigidBodyAPI.Apply(s.GetPrimAtPath(pin)).CreateDisableGravityAttr(True)
    slide = w.Phy.PrismaticJoint.Define(s, "/Asset/IndenterTravel")
    slide.CreateBody0Rel().SetTargets([body])
    slide.CreateBody1Rel().SetTargets([pin])
    slide.CreateAxisAttr("Z")
    slide.CreateLowerLimitAttr(0.)
    slide.CreateUpperLimitAttr(C.STROKE)
    slide.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(slide.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(C.SPRING_RATE)
    drive.CreateTargetPositionAttr(-C.SPRING_PRELOAD / C.SPRING_RATE)
    drive.CreateDampingAttr(11.)
    drive.CreateMaxForceAttr(12.)
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = []
    for name, kind, make, position in (
        ("bench", "base", make_bench, (0., 0., 0.)),
        ("coupon", "rigid", make_coupon, C.START_COUPON),
        ("durometer", "articulation", make_gauge, C.TOOL),
    ):
        assets.append({"name": name, "type": kind, "file": w.save(make(w)), "position": position})
    w.inventory()
    return {"assets": assets, "settle_actions": 15}


def show_display(stage, env_root, reading, count):
    from pxr import UsdGeom

    parent = env_root + "/durometer/Gauge"
    if not stage.GetPrimAtPath(parent).IsValid():
        return False
    string = "   " if reading is None else f"{reading:3.0f}"
    changed = False
    for digit, number in enumerate(string):
        active = DIGITS[int(number)] if number.isdigit() else ""
        for segment in range(7):
            value = "inherited" if str(segment) in active else "invisible"
            attr = UsdGeom.Imageable(stage.GetPrimAtPath(f"{parent}/LCD_{digit}_{segment}")).GetVisibilityAttr()
            if attr.Get() != value:
                attr.Set(value)
                changed = True
    for index in range(5):
        value = "inherited" if index < count else "invisible"
        attr = UsdGeom.Imageable(stage.GetPrimAtPath(f"{parent}/Memory{index}")).GetVisibilityAttr()
        if attr.Get() != value:
            attr.Set(value)
            changed = True
    return changed
