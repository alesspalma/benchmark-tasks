"""PhysX boot panels, cord, and locking metal lace hooks."""

import numpy as np

from . import config as c
from .lace import build_lace
from .usd_scene import Writer

RUBBER = (.065, .059, .050)
LEATHER = (.294, .183, .100)
PANEL = (.473, .300, .166)
PADDING = (.10, .11, .10)
TONGUE = (.34, .235, .138)
CORD = (.100, .46, .33)
METAL = (.50, .52, .50)
AGLET = (.87, .65, .29)


def bench(w):
    stage = w.stage("bench")
    w.box(stage, "/Asset/Bench", (1.10, 1.30, .06), (.375, 0., c.TABLE-.030),
          (.65, .59, .47))
    w.rounded_box(stage, "/Asset/LastSupport", (.097, .091, c.BOOT_RISE+.004),
                  (.316, 0., c.TABLE+(c.BOOT_RISE+.004)/2), (.39, .31, .22), .007, .003)
    return stage


def toe(w, stage, root):
    for name, length, width, height, x, z, color in (
        ("Outsole", .287, .119, .026, .379, .017, RUBBER),
        ("Midsole", .281, .114, .016, .379, .038, PADDING),
        ("Forefoot", .195, .102, .054, .417, .067, LEATHER),
        ("ToeCap", .077, .104, .037, .474, .059, RUBBER),
        ("HeelCounter", .098, .101, .126, .285, .092, LEATHER),
        ("Collar", .081, .098, .043, .278, .157, PADDING),
        ("Tongue", .145, .042, .016, .361, .100, TONGUE),
    ):
        w.rounded_box(stage, root+"/"+name, (length, width, height),
                      (x, 0., c.TABLE+z+c.BOOT_RISE), color, radius=min(length, width)*.42,
                      bevel=min(height*.20, .007))
    for i, x in enumerate(np.linspace(.255, .499, 11)):
        w.box(stage, root+f"/Tread{i}", (.013, .115, .005),
              (x, 0., c.TABLE+.004+c.BOOT_RISE), PADDING, collide=False)


def slide(w, stage, parent, child, side):
    joint = w.Phy.PrismaticJoint.Define(stage, "/Asset/Flap"+side)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateAxisAttr("Y")
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*c.PANEL_ROOT))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0., 0., 0.))
    joint.CreateCollisionEnabledAttr(False)
    left = c.SIDES[side] > 0
    joint.CreateLowerLimitAttr(0. if left else -.016)
    joint.CreateUpperLimitAttr(.016 if left else 0.)
    w.Px.PhysxJointAPI.Apply(joint.GetPrim()).CreateJointFrictionAttr(.002)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(65.)
    drive.CreateDampingAttr(2.)
    drive.CreateTargetPositionAttr(c.SIDES[side]*c.OPEN)


def clamp_plate(w, stage, path, center, frame, sign, throat_top):
    # Both cheeks have a lead-in; the narrowing near the base arrests the
    # tensioned line. The sprung cheek yields only in response to contact.
    section = np.array([[.0044, -.006], [.0015, -.001], [.0015, throat_top],
                        [.0062, throat_top+.009], [.0086, throat_top+.008], [.009, -.006]])
    pts = [center + frame @ [end, sign*width, z]
           for end in (-.009, .009) for width, z in section]
    faces = [(5, 4, 3, 2, 1, 0), (6, 7, 8, 9, 10, 11)]
    faces.extend((i, (i+1) % 6, (i+1) % 6+6, i+6) for i in range(6))
    if sign < 0:
        faces = [face[::-1] for face in faces]
    mesh = w.Geom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([w.Gf.Vec3f(*map(float, p)) for p in pts])
    mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
    mesh.CreateFaceVertexIndicesAttr([idx for face in faces for idx in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    w._finish(stage, mesh, METAL, True, contact=.00025)


def clamping_hook(w, stage, panel, side, center):
    from yam_arena.ik import matrix_to_quat_xyzw

    frame = c.lock_axes(c.SIDES[side])
    w.rounded_box(stage, panel+"/HookSaddle", (.024, .021, .003),
                  center-[0., 0., .0045], METAL, radius=.003, bevel=.0005)
    throat_top = c.THROAT_TOP[side]
    clamp_plate(w, stage, panel+"/FixedCheek", center, frame, -1, throat_top)
    opening = np.array([0., c.SIDES[side]*c.OPEN, 0.])
    jaw = w.body(stage, "/Asset/"+side+"Jaw", .006, c.PANEL_ROOT+opening+center)
    clamp_plate(w, stage, jaw+"/SpringCheek", np.zeros(3), frame, 1, throat_top)
    joint = w.Phy.PrismaticJoint.Define(stage, "/Asset/Jaw"+side)
    joint.CreateBody0Rel().SetTargets([panel])
    joint.CreateBody1Rel().SetTargets([jaw])
    joint.CreateAxisAttr("Y")
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, center)))
    x,y,z,real = map(float, matrix_to_quat_xyzw(frame))
    orientation = w.Gf.Quatf(real, x,y,z)
    joint.CreateLocalRot0Attr(orientation)
    joint.CreateLocalRot1Attr(orientation)
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateLowerLimitAttr(0.)
    joint.CreateUpperLimitAttr(.006)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(1050.)
    drive.CreateDampingAttr(2.4)
    drive.CreateTargetPositionAttr(0.)


def eyelet(w, stage, path, center, *, crosswise):
    # A shallow steel guide surrounds the cord while leaving axial slip free.
    along, across = (1, 0) if crosswise else (0, 1)
    for index, sign in enumerate((-1., 1.)):
        post = center.copy()
        post[across] += sign*.006
        size = np.array([.004, .004, .014])
        size[along] = .008
        w.box(stage, path+f"Side{index}", size, post, METAL)
    for name, sign in (("Base", -1.), ("Bridge", 1.)):
        plate = center+[0., 0., sign*.006]
        size = np.array([.016, .016, .004])
        size[along] = .008
        w.box(stage, path+name, size, plate, METAL)


def boot(w):
    stage = w.stage("boot")
    root = w.body(stage, "/Asset/Last", .75, fixed=True, articulation=True)
    toe(w, stage, root)
    for side, direction in c.SIDES.items():
        opening = np.array([0., direction*c.OPEN, 0.])
        panel = w.body(stage, "/Asset/"+side+"Panel", .043, c.PANEL_ROOT+opening, ccd=True)
        w.rounded_box(stage, panel+"/Upper", (.151, .027, .045),
                      (.007, direction*.042, .016), PANEL, radius=.012, bevel=.005)
        w.rounded_box(stage, panel+"/LaceStay", (.100, .014, .010),
                      (.044, direction*.033, .031), LEATHER, radius=.006, bevel=.001)
        w.rounded_box(stage, panel+"/ToeStay", (.065, .019, .015),
                      (.098, direction*.044, .028), PANEL, radius=.005, bevel=.002)
        slide(w, stage, root, panel, side)
        for index, guide in enumerate((c.ANCHOR, *c.LOWER_GUIDES)):
            center = np.asarray(guide)*[1., direction, 1.] - c.PANEL_ROOT
            eyelet(w, stage, panel+f"/Eyelet{index}", center, crosswise=index == 0)
        lock = c.lock_center(side) - c.PANEL_ROOT
        clamping_hook(w, stage, panel, side, lock)
    build_lace(w, stage)
    return stage


def create_assets(directory):
    w = Writer(directory)
    assets = [
        {"name": "bench", "type": "base", "file": w.save(bench(w))},
        {"name": "boot", "type": "articulation", "file": w.save(boot(w)),
         "initial": {"joint_pos": {"FlapL": c.OPEN, "FlapR": -c.OPEN}}},
    ]
    w.inventory()
    return {"assets": assets, "settle_actions": 25}
