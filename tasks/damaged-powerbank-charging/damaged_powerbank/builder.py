"""Original real-size shells, actual plug voids and passive receptacle contacts."""

import math

import numpy as np

from . import config as C
from .usd_scene import Writer


def hull(w, stage, path, points, color):
    from scipy.spatial import ConvexHull

    pts = np.asarray(points, dtype=float)
    triangles = ConvexHull(pts).simplices.tolist()
    centroid = pts.mean(0)
    for tri in triangles:
        a, b, c = pts[tri]
        if np.dot(np.cross(b - a, c - a), (a + b + c) / 3 - centroid) < 0:
            tri[1], tri[2] = tri[2], tri[1]
    w.convex(stage, path, pts.tolist(), triangles, color)


def rounded_section(width, height, radius=0.005, n=6):
    pts = []
    for sy, sz, start in [(1, 1, 0), (-1, 1, 90), (-1, -1, 180), (1, -1, 270)]:
        for a in np.linspace(start, start + 90, n):
            angle = math.radians(a)
            pts.append([sy * (width / 2 - radius) + radius * math.cos(angle),
                        sz * (height / 2 - radius) + radius * math.sin(angle)])
    return pts


def body_shell(w, s, root, damaged):
    gray = (0.135, 0.145, 0.16)
    section = rounded_section(C.BODY_Y, C.BODY_Z)
    if not damaged:
        points = [[x, y, z] for x in (-C.BODY_X / 2, C.BODY_X / 2) for y, z in section]
        hull(w, s, root + "/RoundedEnclosure", points, gray)
    else:
        # Warning cue only: no claimed internal cell chemistry or OEM failure test.
        points = [[x, y, z] for x in (-0.006, C.BODY_X / 2) for y, z in section]
        hull(w, s, root + "/RearEnclosure", points, gray)
        def cover_back(y):
            # One long edge is lifted9mm, opposite edge1mm: an open seam,
            # rather than a uniformly bigger or merely rounded enclosure.
            return -0.011 + 0.004 * y / (C.BODY_Y/2)
        points = [[cover_back(y), y, z] for y, z in section]
        points += [[cover_back(y)-0.0045-0.003*(z/(C.BODY_Z/2)+1), y, z] for y, z in section]
        for y in np.linspace(-C.BODY_Y * 0.40, C.BODY_Y * 0.40, 5):
            for z in np.linspace(-C.BODY_Z * 0.40, C.BODY_Z * 0.40, 5):
                b = 0.008 * (1 - (y / (C.BODY_Y / 2)) ** 2) * (1 - (z / (C.BODY_Z / 2)) ** 2)
                points.append([cover_back(y)-0.0045-b-0.002*(z/(C.BODY_Z/2)+1), y, z])
        hull(w, s, root + "/BowedSeparatedCover", points, (0.16, 0.17, 0.19))
        # The gap is actual empty space, with the rear shell's light inner rim
        # visible along the lifted edge; there is no answer label or black fill.
        w.box(s, root + "/VisibleInnerRim", (0.0015, 0.002, 0.052),
              (-0.0064, -C.BODY_Y/2+0.0025, 0), (0.49, 0.50, 0.50))
    # Ordinary front ports, button and four charging indicator lenses.
    x = -C.BODY_X / 2 - (0.011 if damaged else 0.00012)
    for k, z in enumerate((-0.015, -0.003)):
        w.box(s, root + f"/USBPort{k}", (0.0004, 0.014, 0.006), (x, 0, z), (0.015, 0.018, 0.020), collide=False)
        w.box(s, root + f"/USBContact{k}", (0.0005, 0.009, 0.0014), (x - 0.0001, 0, z), (0.46, 0.43, 0.23), collide=False)
    for k, y in enumerate(np.linspace(-0.009, 0.009, 4)):
        w.cylinder(s, root + f"/LED{k}", 0.0013, 0.0005, (x - 0.0001, y, 0.017), (0.045, 0.08, 0.13), axis="X", collide=False)
    w.cylinder(s, root + "/StatusButton", 0.005, 0.0007, (x, 0, 0.026), (0.22, 0.23, 0.25), axis="X", collide=False)
    # Ordinary integrated blades, already extended, but not engaged at reset.
    for k, side in enumerate((-1, 1)):
        center = side * C.BLADE_PITCH / 2
        points = []
        for x, thick in [(C.BODY_X / 2 - 0.0005, C.BLADE_T),
                         (C.BODY_X / 2 + C.BLADE_L - 0.002, C.BLADE_T),
                         (C.BODY_X / 2 + C.BLADE_L, 0.0005)]:
            points.extend([[x, center + y, z] for y in (-thick / 2, thick / 2)
                           for z in (-C.BLADE_H / 2, C.BLADE_H / 2)])
        hull(w, s, root + f"/ACBlade{k}", points, (0.66, 0.65, 0.59))


def create_assets(directory):
    from pxr import Gf, UsdGeom, UsdPhysics

    w = Writer(directory)
    assets = []
    for name, pos, damaged in [("intact", C.GOOD, False), ("damaged", C.BAD, True)]:
        s = w.stage(name)
        root = w.body(s, "/Asset/Body", C.MASS, ccd=True)
        body_shell(w, s, root, damaged)
        mat = UsdPhysics.MaterialAPI.Get(s, "/Asset/Physics")
        mat.GetStaticFrictionAttr().Set(0.70)
        mat.GetDynamicFrictionAttr().Set(0.55)
        assets.append(dict(name=name, type="rigid", file=w.save(s), position=pos.tolist()))

    s = w.stage("socket")
    root = "/Asset/Socket"
    xf = UsdGeom.Xform.Define(s, root)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, C.SOCKET)))
    contacts = []
    white, brass = (0.81, 0.82, 0.81), (0.62, 0.49, 0.26)
    mat = UsdPhysics.MaterialAPI.Get(s, "/Asset/Physics")
    mat.GetStaticFrictionAttr().Set(C.FRICTION_STATIC)
    mat.GetDynamicFrictionAttr().Set(C.FRICTION_DYNAMIC)
    w.box(s, root + "/RearHousing", (0.025, 0.330, 0.150), (0.040, 0, -0.005), (0.24, 0.25, 0.26))
    w.box(s, root + "/Foot", (0.120, 0.350, 0.010), (0.025, 0, -0.090), (0.32, 0.33, 0.34))
    for iy, cy in enumerate(C.OUTLET_YS):
        # Five plate pieces leave each blade slot physically open.
        w.box(s, root + f"/FaceTop{iy}", (0.004, 0.070, 0.036), (0.002, cy, 0.022), white)
        w.box(s, root + f"/FaceBottom{iy}", (0.004, 0.070, 0.036), (0.002, cy, -0.022), white)
        intervals = [(-0.035, -C.BLADE_PITCH/2-C.SLOT_T/2),
                     (-C.BLADE_PITCH/2+C.SLOT_T/2, C.BLADE_PITCH/2-C.SLOT_T/2),
                     (C.BLADE_PITCH/2+C.SLOT_T/2, 0.035)]
        for k, (a, b) in enumerate(intervals):
            w.box(s, root + f"/SlotWeb{iy}_{k}", (0.004, b-a, C.SLOT_H),
                  (0.002, cy+(a+b)/2, 0), white)
        # The unused ground opening is visual; these two-blade banks are ungrounded.
        w.cylinder(s, root + f"/GroundOpening{iy}", 0.0035, 0.0002, (-0.0002, cy, -0.018), (0.035, 0.035, 0.035), axis="X", collide=False)
        for k, by in enumerate((-C.BLADE_PITCH/2, C.BLADE_PITCH/2)):
            for side in (-1, 1):
                name = f"Leaf{iy}_{k}_{'P' if side > 0 else 'N'}"
                local = np.array([0.009, cy + by + side*(C.LEAF_GAP+C.LEAF_THICK)/2, 0])
                leaf = w.body(s, "/Asset/" + name, 0.002, position=C.SOCKET+local, ccd=True)
                w.box(s, leaf + "/Contact", (0.010, C.LEAF_THICK, 0.007), (0, 0, 0), brass)
                j = UsdPhysics.PrismaticJoint.Define(s, "/Asset/J_" + name)
                # Independent passive contact springs referenced to the fixed
                # receptacle; no articulation or root-reset coupling is needed.
                j.CreateBody1Rel().SetTargets([leaf])
                j.CreateLocalPos0Attr(Gf.Vec3f(*map(float, C.SOCKET + local)))
                j.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
                j.CreateAxisAttr("Y")
                if side < 0:
                    q = Gf.Quatf(0, Gf.Vec3f(0, 0, 1))
                    j.CreateLocalRot0Attr(q)
                    j.CreateLocalRot1Attr(q)
                j.CreateLowerLimitAttr(0.0)
                j.CreateUpperLimitAttr(C.LEAF_TRAVEL)
                j.CreateCollisionEnabledAttr(True)
                drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "linear")
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(C.LEAF_K)
                drive.CreateDampingAttr(C.LEAF_DAMP)
                drive.CreateTargetPositionAttr(0)
                contacts.append(dict(name=name, position=(C.SOCKET+local).tolist(),
                                     sign=side, outlet=iy, pole=k))
    assets.append(dict(name="socket", type="base", file=w.save(s), position=[0, 0, 0]))

    s = w.stage("supports")
    for name, pos in [("IntactPad", C.GOOD), ("DamagedPad", C.BAD)]:
        w.box(s, "/Asset/" + name, (0.110, 0.120, C.PAD_H),
              (float(pos[0]), float(pos[1]), C.TOP + C.PAD_H/2), (0.65, 0.67, 0.67))
    assets.append(dict(name="supports", type="base", file=w.save(s), position=[0, 0, 0]))
    w.inventory()
    return {"assets": assets, "contacts": contacts, "settle_actions": 30}
