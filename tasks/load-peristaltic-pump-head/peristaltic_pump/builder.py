"""Procedural pump with separate native cam, occlusion shoes and tube links."""

import math

from . import config as C
from .usd_scene import Writer

HOUSING = (0.24, 0.34, 0.40)
EDGE = (0.48, 0.58, 0.63)
STEEL = (0.67, 0.73, 0.74)
LEVER = (0.97, 0.68, 0.15)


def joint(w, stage, name, kind, parent, child, axis, offset, lo, hi):
    joint = getattr(w.Phy, kind + "Joint").Define(stage, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateAxisAttr(axis)
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, offset)))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    joint.CreateLowerLimitAttr(lo)
    joint.CreateUpperLimitAttr(hi)
    joint.CreateCollisionEnabledAttr(False)
    return joint


def drive(w, joint, kind, stiffness, damping, force):
    spring = w.Phy.DriveAPI.Apply(joint.GetPrim(), kind)
    spring.CreateTypeAttr("force")
    spring.CreateStiffnessAttr(stiffness)
    spring.CreateDampingAttr(damping)
    spring.CreateMaxForceAttr(force)


def pump(w):
    s = w.stage("pump")
    root = w.body(s, "/Asset/Case", 0.65, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(s, root).CreateEnabledSelfCollisionsAttr(False)
    w.box(s, root + "/Housing", (0.102, 0.101, 0.056), C.HEAD + (0, 0, 0.028), HOUSING)
    # Rounded four-roller rotor. The top roller extends across both parallel lanes.
    for i, (y, z) in enumerate(((-0.018, 0.049), (0, 0.068), (0.018, 0.049), (0, 0.030))):
        w.cylinder(s, root + f"/Roller{i}", 0.009, 0.047, C.HEAD + (0, y, z), STEEL, axis="X")
    for side in (-1, 1):
        y = side * 0.046
        w.box(s, root + f"/RetainerBase{side}", (0.049, 0.011, 0.020), C.HEAD + (0, y, 0.067), EDGE)
        for i, x in enumerate((-0.020, 0.0, 0.020)):
            w.box(s, root + f"/RetainerPin{i}_{side}", (0.004, 0.011, 0.007),
                  C.HEAD + (x, y, 0.0805), EDGE)
    cam = w.body(s, "/Asset/Cam", 0.047, position=C.HEAD + C.PIVOT, ccd=True)
    w.box(s, cam + "/Bed", (0.058, 0.078, 0.008), (-0.044, 0, -0.009), HOUSING)
    w.cylinder(s, cam + "/Hinge", 0.008, 0.093, (0, 0, 0), EDGE, axis="Y")
    w.box(s, cam + "/LeverArm", (0.016, 0.012, 0.045), (0.014, -0.041, 0.022), LEVER)
    w.cylinder(s, cam + "/LeverGrip", 0.0105, 0.024, C.LEVER, LEVER, axis="Y")
    hinge = joint(w, s, "CamClose", "Revolute", root, cam, "Y", C.HEAD + C.PIVOT, 0.0, 70.0)
    drive(w, hinge, "angular", 0.0, 0.025, 0.18)
    for i, x in enumerate(C.LANES):
        p = (x - C.PIVOT[0], 0, -0.0185)
        shoe = w.body(s, f"/Asset/Shoe{i}", 0.008, position=C.HEAD + C.PIVOT + p)
        w.box(s, shoe + "/OcclusionBed", (0.011, 0.061, 0.004), (0, 0, 0), STEEL)
        spring = joint(w, s, f"Compression{i}", "Prismatic", cam, shoe, "Z", p, 0.0, 0.008)
        drive(w, spring, "linear", 850.0, 9.0, 10.0)
    return s


def tube(w, name, color):
    s = w.stage(name)
    for i in range(C.LINKS):
        link = w.body(s, f"/Asset/Link{i}", 0.0012, position=(0, i * C.TUBE_L, 0),
                      articulation=i == 0, ccd=True)
        geom = w.Geom.Capsule.Define(s, link + "/Silicone")
        geom.CreateRadiusAttr(C.TUBE_R)
        geom.CreateHeightAttr(C.TUBE_L)
        geom.CreateAxisAttr("Y")
        w._finish(s, geom, color, True)
        if i == 0:
            w.Px.PhysxArticulationAPI.Get(s, link).CreateEnabledSelfCollisionsAttr(False)
        else:
            bend = joint(w, s, f"Bend{i}", "Revolute", f"/Asset/Link{i-1}", link,
                         "X" if i % 2 else "Z", (0, C.TUBE_L / 2, 0), -40.0, 40.0)
            bend.CreateLocalPos1Attr(w.Gf.Vec3f(0, -C.TUBE_L / 2, 0))
            drive(w, bend, "angular", 0.0018, 0.00035, 0.018)
    return s


def bench(w):
    s = w.stage("bench")
    for i, x in enumerate(C.STAGING):
        for j, y in enumerate((-0.125, -0.017, 0.115)):
            w.box(s, f"/Asset/Rack{i}_{j}", (0.030, 0.013, 0.038), (x, C.HEAD[1] + y, 0.019), EDGE)
            for k in (-1, 1):
                w.box(s, f"/Asset/RackEdge{i}_{j}_{k}", (0.004, 0.013, 0.007),
                      (x + k * 0.007, C.HEAD[1] + y, 0.0415), EDGE)
    for i, x in enumerate(C.LANES):
        x += C.HEAD[0]
        for suffix, y in (("Source", 0.160), ("Receiver", -0.160)):
            path = f"/Asset/{suffix}{i}"
            w.cylinder(s, path + "Bottom", 0.0082, 0.003, (x, y, 0.0015), HOUSING)
            for j in range(16):
                angle = math.tau * j / 16
                from .motion import rotation
                w.box(s, path + f"Wall{j}", (0.001, 0.0033, 0.026),
                      (x + 0.008 * math.cos(angle), y + 0.008 * math.sin(angle), 0.014),
                      (0.50, 0.66, 0.70), rotation=rotation("Z", angle), collide=False)
            if suffix == "Source":
                w.cylinder(s, path + "Water", 0.0071, 0.017, (x, y, 0.011), C.COLORS[i], collide=False)
            else:
                w.cylinder(s, f"/Asset/Collected{i}", 0.0071, 0.0001,
                           (x, y, 0.004), C.COLORS[i], collide=False).MakeInvisible()
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [{"name": "head", "type": "articulation", "file": w.save(pump(w)), "position": (0, 0, 0)},
              {"name": "bench", "type": "base", "file": w.save(bench(w)), "position": (0, 0, C.TABLE)}]
    for i, name in enumerate(C.TUBE_NAMES):
        assets.append({"name": name, "type": "articulation", "file": w.save(tube(w, name, C.COLORS[i])),
                       "position": (C.STAGING[i], C.HEAD[1] - 4 * C.TUBE_L, C.TABLE + 0.043)})
    w.inventory()
    return {"assets": assets, "settle_actions": 18}
