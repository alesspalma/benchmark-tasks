"""Original full-size corrugated carton, pull tab and book geometry."""

import numpy as np

from . import config as C
from .usd_scene import Writer

KRAFT = (.57, .37, .18)
CUT = (.66, .46, .27)
INSIDE = (.68, .48, .28)
TAPE = (.85, .16, .09)


def box_asset(w):
    s = w.stage("mailer")
    base = w.body(s, "/Asset/Tray", .061, articulation=True, ccd=True)
    lid = w.body(s, "/Asset/Lid", .025, position=(0, 0, C.HEIGHT), ccd=True)
    d, b, h, t = C.DEPTH, C.WIDTH, C.HEIGHT, C.WALL
    w.box(s, base + "/Bottom", (d, b, t), (0, 0, t / 2), INSIDE)
    w.box(s, base + "/Rear", (t, b, h), ((d - t) / 2, 0, h / 2), KRAFT)
    for i, side in enumerate((-1, 1)):
        w.box(s, base + f"/End{i}", (d - t, 2 * t, h - 2 * t),
              (0, side * (b - 2 * t) / 2, (h - 2 * t) / 2), KRAFT)
    lower = C.SEAM_Z - C.SEAM_WIDTH / 2
    w.box(s, base + "/Front", (t, b, lower), (-(d - t) / 2, 0, lower / 2), KRAFT)
    w.box(s, lid + "/Cover", (d + .004, b, t), (-.002, 0, -t / 2), CUT)
    upper = h - C.SEAM_Z - C.SEAM_WIDTH / 2
    w.box(s, lid + "/FrontFlap", (t, b, upper), (-d / 2 - t / 2, 0, -upper / 2), KRAFT)
    joint = w.Phy.RevoluteJoint.Define(s, "/Asset/Crease")
    joint.CreateBody0Rel().SetTargets([base])
    joint.CreateBody1Rel().SetTargets([lid])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(d / 2, 0., h))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(d / 2, 0., 0.))
    joint.CreateAxisAttr("Y")
    joint.CreateLowerLimitAttr(0.)
    joint.CreateUpperLimitAttr(112.)
    joint.CreateCollisionEnabledAttr(False)
    fold = w.Phy.DriveAPI.Apply(joint.GetPrim(), "angular")
    fold.CreateTypeAttr("force")
    fold.CreateStiffnessAttr(0.)
    fold.CreateDampingAttr(.004)
    fold.CreateMaxForceAttr(1.)
    surf = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    surf.CreateStaticFrictionAttr(.6)
    surf.CreateDynamicFrictionAttr(.46)
    return s


def tab_asset(w):
    s = w.stage("pull_tab")
    b = w.body(s, "/Asset/Tab", .006, ccd=True)
    # The stiffened leader folds toward the near side at the starting corner.
    w.box(s, b + "/Pull", C.TAB_SIZE, (0, 0, 0), TAPE)
    m = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    m.CreateStaticFrictionAttr(.8)
    m.CreateDynamicFrictionAttr(.65)
    return s


def book_asset(w):
    s = w.stage("paperback")
    b = w.body(s, "/Asset/Book", .142, ccd=True)
    x, y, z = C.BOOK_SIZE
    w.box(s, b + "/Pages", (x, y, z), (0, 0, 0), (.88, .84, .69))
    for i, side in enumerate((-1, 1)):
        w.box(s, b + f"/Cover{i}", (x + .001, y + .001, .001),
              (0, 0, side * z / 2), (.10, .29, .41), collide=False)
    w.box(s, b + "/Spine", (.002, y, z), (-x / 2, 0, 0),
          (.14, .32, .47), collide=False)
    return s


def create_assets(directory):
    w = Writer(directory)
    objects = [
        ("mailer", "articulation", C.BOX, box_asset(w)),
        ("pull_tab", "rigid", C.TAB, tab_asset(w)),
        ("paperback", "rigid", C.BOOK, book_asset(w)),
    ]
    return {"assets": [
        dict(name=name, type=kind, position=np.asarray(pose).tolist(), file=w.save(stage))
        for name, kind, pose, stage in objects
    ], "settle_actions": 20}
