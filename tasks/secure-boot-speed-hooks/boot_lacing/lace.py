"""One continuous native lace threaded through the six forefoot guides.

Joint recipe adapted from the MIT-licensed tie-overhand-knot task at 6567167.
"""
from math import pi

import numpy as np

from . import config as c


def link_name(side, index):
    return f"{side}Lace{index:02d}"


def initial_lace(side):
    sign = c.SIDES[side]
    start = c.ANCHOR * [1., sign, 1.] + [0., sign*c.OPEN, 0.]
    # Reset the same loose shape reached by a stock length hanging at rest
    # from the eyelet. Equal increments match the native joint spacing.
    pitch = np.interp(np.arange(c.LINK_COUNT), [0, 1, 2, 3, 4, 5],
                      [0., .34, .99, 1.47, 1.558, pi/2])
    directions = np.column_stack((np.zeros_like(pitch), sign*np.cos(pitch), -np.sin(pitch)))
    nodes = np.vstack((start, start + np.cumsum(c.LINK_LENGTH*directions, axis=0)))
    positions = (nodes[:-1] + nodes[1:]) * .5
    quats = np.column_stack((np.zeros(len(directions)), -directions[:, 2],
                             directions[:, 1], np.ones(len(directions))))
    return positions, quats / np.linalg.norm(quats, axis=1, keepdims=True)


def _bend(w, stage, previous, body, name, offset, child_half, *, reverse_parent=False):
    joint = w.Phy.Joint.Define(stage, "/Asset/Joint" + name)
    joint.CreateBody0Rel().SetTargets([previous])
    joint.CreateBody1Rel().SetTargets([body])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, offset)))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(-child_half, 0., 0.))
    if reverse_parent:
        joint.CreateLocalRot0Attr(w.Gf.Quatf(0., 0., 0., 1.))
    joint.CreateExcludeFromArticulationAttr(True)
    joint.CreateCollisionEnabledAttr(False)
    prim = joint.GetPrim()
    for axis in ("transX", "transY", "transZ"):
        limit = w.Phy.LimitAPI.Apply(prim, axis)
        limit.CreateLowAttr(1.)
        limit.CreateHighAttr(-1.)
    for axis in ("rotY", "rotZ"):
        limit = w.Phy.LimitAPI.Apply(prim, axis)
        limit.CreateLowAttr(-84.)
        limit.CreateHighAttr(84.)
        drive = w.Phy.DriveAPI.Apply(prim, axis)
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(.00013 * pi / 180)
        drive.CreateDampingAttr(.0000035 * pi / 180)
        drive.CreateTargetPositionAttr(0.)
        drive.CreateTargetVelocityAttr(0.)


def _segment(w, stage, name, p, q, length, neighbors, aglet=False):
    from pxr import Sdf, UsdPhysics

    path = w.body(stage, f"/Asset/Laces/{name}", .00022, p, ccd=True)
    prim = stage.GetPrimAtPath(path)
    x, y, z, real = map(float, q)
    w.Geom.Xformable(prim).GetOrderedXformOps()[1].Set(w.Gf.Quatd(real, x, y, z))
    rb = w.Px.PhysxRigidBodyAPI.Apply(prim)
    rb.CreateSolverPositionIterationCountAttr(48)
    rb.CreateSolverVelocityIterationCountAttr(12)
    rb.CreateLinearDampingAttr(.25)
    rb.CreateAngularDampingAttr(1.)
    rb.CreateSleepThresholdAttr(0.)
    cap = w.Geom.Capsule.Define(stage, path+"/Cord")
    cap.CreateAxisAttr("X")
    cap.CreateHeightAttr(float(length))
    cap.CreateRadiusAttr(c.LACE_RADIUS)
    w._finish(stage, cap, (.93, .72, .31) if aglet else (.09, .53, .33), True, contact=.00030)
    if not name.startswith("Bridge"):
        cord_material = w.Shade.Material.Get(stage, "/Asset/CordFriction")
        w.Shade.MaterialBindingAPI.Apply(cap.GetPrim()).Bind(cord_material, materialPurpose="physics")
    UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(
        [Sdf.Path("/Asset/Laces/" + other) for other in neighbors]
    )
    return path


def bridge_layout():
    """Trace L1–R2–L3–R3–L2–R1, with alternating crosses over the tongue."""
    from yam_arena.ik import matrix_to_quat_xyzw

    x1, y1, z1 = c.ANCHOR + [0., c.OPEN, 0.]
    x2, y2, z2 = np.asarray(c.LOWER_GUIDES[0]) + [0., c.OPEN, 0.]
    x3, y3, z3 = np.asarray(c.LOWER_GUIDES[1]) + [0., c.OPEN, 0.]
    route = np.array([
        [x1, y1, z1], [x1, y1-.010, z1], [.466, 0., .969],
        [x2-.009, -y2, z2], [x2+.009, -y2, z2], [.495, 0., .966],
        [x3-.010, y3, z3], [x3+.007, y3, z3], [.522, 0., .955],
        [x3+.007, -y3, z3], [x3-.010, -y3, z3], [.495, 0., .958],
        [x2+.009, y2, z2], [x2-.009, y2, z2], [.466, 0., .961],
        [x1, -y1+.010, z1], [x1, -y1, z1],
    ])
    smooth = [route[0]]
    for a, b in zip(route[:-1], route[1:]):
        smooth.extend((.76*a+.24*b, .24*a+.76*b))
    smooth.append(route[-1])
    smooth = np.array(smooth)
    dist = np.r_[0., np.linalg.norm(np.diff(smooth, axis=0), axis=1).cumsum()]
    points = np.linspace(0., dist[-1], round(dist[-1]/c.LINK_LENGTH)+1)
    nodes = np.column_stack([np.interp(points, dist, smooth[:, i]) for i in range(3)])
    deltas = np.diff(nodes, axis=0)
    lengths = np.linalg.norm(deltas, axis=1)
    angles = []
    for tangent in deltas/lengths[:, None]:
        y_axis = np.cross([0., 0., 1.], tangent)
        y_axis /= np.linalg.norm(y_axis)
        angles.append(matrix_to_quat_xyzw(np.column_stack((tangent, y_axis, np.cross(tangent, y_axis)))))
    return (nodes[:-1]+nodes[1:])*.5, np.array(angles), lengths


def bridge_name(index):
    return f"Bridge{index:02d}"


def build_lace(w, stage):
    material = w.Shade.Material.Define(stage, "/Asset/CordFriction")
    friction = w.Phy.MaterialAPI.Apply(material.GetPrim())
    friction.CreateStaticFrictionAttr(.90)
    friction.CreateDynamicFrictionAttr(.70)
    friction.CreateRestitutionAttr(0.)
    positions, quats, lengths = bridge_layout()
    count = len(lengths)
    first_segments = {side: _build_tail(w, stage, side, bridge_name(0 if side == "L" else count-1))
                      for side in c.SIDES}
    parent = first_segments["L"]
    offset = (-c.LINK_LENGTH/2, 0., 0.)
    for i, (p, q, length) in enumerate(zip(positions, quats, lengths)):
        prev = link_name("L", 0) if i == 0 else bridge_name(i-1)
        after = link_name("R", 0) if i == count-1 else bridge_name(i+1)
        name = bridge_name(i)
        path = _segment(w, stage, name, p, q, length, (prev, after))
        _bend(w, stage, parent, path, name, offset, length/2, reverse_parent=i == 0)
        parent = path
        offset = (length/2, 0., 0.)
    _bend(w, stage, parent, first_segments["R"], "RightExit", offset, c.LINK_LENGTH/2)
    for side, sign in c.SIDES.items():
        shift = np.array([0., sign*c.OPEN, 0.])
        for index, guide in enumerate((c.ANCHOR, *c.LOWER_GUIDES)):
            center = np.asarray(guide)*[1., sign, 1.] + shift
            nearest = int(np.argmin(np.linalg.norm(positions-center, axis=1)))
            _sliding_guide(w, stage, side, index, center, shift, positions[nearest],
                           quats[nearest], bridge_name(nearest))


def _sliding_guide(w, stage, side, index, center, reset_offset, position, quaternion, name):
    """Keep the continuous cord inside a guide while allowing limited axial slip."""
    from yam_arena.ik import matrix_to_quat_xyzw
    from yam_arena.urdf import quat_xyzw_to_matrix

    basis = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]]) if index == 0 else np.eye(3)
    local = quat_xyzw_to_matrix(quaternion).T
    joint = w.Phy.Joint.Define(stage, f"/Asset/{side}CordGuide{index}")
    joint.CreateBody0Rel().SetTargets([f"/Asset/{side}Panel"])
    joint.CreateBody1Rel().SetTargets([f"/Asset/Laces/{name}"])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, center-c.PANEL_ROOT-reset_offset)))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(*map(float, local @ (center-position))))
    joint.CreateExcludeFromArticulationAttr(True)
    joint.CreateCollisionEnabledAttr(False)
    for setter, rotation in ((joint.CreateLocalRot0Attr, basis), (joint.CreateLocalRot1Attr, local @ basis)):
        x, y, z, real = map(float, matrix_to_quat_xyzw(rotation))
        setter(w.Gf.Quatf(real, x, y, z))
    for axis, travel in (("transX", .011), ("transY", .0012), ("transZ", .0015)):
        limit = w.Phy.LimitAPI.Apply(joint.GetPrim(), axis)
        limit.CreateLowAttr(-travel)
        limit.CreateHighAttr(travel)
    drag = w.Phy.DriveAPI.Apply(joint.GetPrim(), "transX")
    drag.CreateTypeAttr("force")
    drag.CreateDampingAttr(.035)
    drag.CreateTargetVelocityAttr(0.)


def _build_tail(w, stage, side, neighbor):
    positions, quats = initial_lace(side)
    prev = None
    for i, (p, q) in enumerate(zip(positions, quats)):
        name = link_name(side, i)
        neighbors = [link_name(side, j) for j in (i-1, i+1) if 0 <= j < c.LINK_COUNT]
        if i == 0:
            neighbors.append(neighbor)
        path = _segment(w, stage, name, p, q, c.LINK_LENGTH, neighbors, aglet=i >= c.LINK_COUNT-2)
        if prev is None:
            first = path
        else:
            _bend(w, stage, prev, path, name, (c.LINK_LENGTH/2, 0., 0.), c.LINK_LENGTH/2)
        prev = path
    return first


def body_view():
    import omni.usd
    from isaaclab_physx.physics import PhysxManager
    from pxr import UsdPhysics

    stage = omni.usd.get_context().get_stage()
    paths = sorted(str(p.GetPath()) for p in stage.Traverse()
                   if "/boot/Laces/" in str(p.GetPath()) and p.HasAPI(UsdPhysics.RigidBodyAPI))
    if len(paths) != len(c.SIDES) * c.LINK_COUNT + len(bridge_layout()[2]):
        raise RuntimeError(f"Cannot bind native lace bodies: {paths}")
    return PhysxManager.get_physics_sim_view().create_rigid_body_view(paths)


def reset_laces(view, device):
    import warp as wp

    initial = {}
    for side in c.SIDES:
        positions, quats = initial_lace(side)
        for i, row in enumerate(np.column_stack((positions, quats))):
            initial[link_name(side, i)] = row
    positions, quats, _ = bridge_layout()
    for i, row in enumerate(np.column_stack((positions, quats))):
        initial[bridge_name(i)] = row
    poses = np.asarray([initial[p.rsplit("/", 1)[-1]] for p in view.prim_paths], np.float32)
    ids = wp.array(np.arange(len(poses), dtype=np.int32), dtype=wp.int32, device=device)
    view.set_transforms(wp.array(poses, dtype=wp.float32, device=device), ids)
    view.set_velocities(wp.zeros((len(poses), 6), dtype=wp.float32, device=device), ids)
