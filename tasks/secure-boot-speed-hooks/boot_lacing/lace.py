"""Native maximal-coordinate cloth-lace links and their reset layout.

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


def _bend(w, stage, previous, body, name, offset, standing_end):
    joint = w.Phy.Joint.Define(stage, "/Asset/Joint" + name)
    joint.CreateBody0Rel().SetTargets([previous])
    joint.CreateBody1Rel().SetTargets([body])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, offset)))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(-c.LINK_LENGTH/2, 0., 0.))
    joint.CreateExcludeFromArticulationAttr(True)
    joint.CreateCollisionEnabledAttr(False)
    prim = joint.GetPrim()
    for axis in ("transX", "transY", "transZ"):
        limit = w.Phy.LimitAPI.Apply(prim, axis)
        limit.CreateLowAttr(1.)
        limit.CreateHighAttr(-1.)
    if not standing_end:
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


def build_lace(w, stage, panel, side):
    from pxr import Sdf, UsdPhysics

    sign = c.SIDES[side]
    positions, quats = initial_lace(side)
    prev = panel
    for i, (p, q) in enumerate(zip(positions, quats)):
        name = link_name(side, i)
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
        offset = c.ANCHOR * [1., sign, 1.] - c.PANEL_ROOT if i == 0 else (c.LINK_LENGTH/2, 0., 0.)
        _bend(w, stage, prev, path, name, offset, standing_end=i == 0)
        cap = w.Geom.Capsule.Define(stage, path+"/Cord")
        cap.CreateAxisAttr("X")
        cap.CreateHeightAttr(c.LINK_LENGTH)
        cap.CreateRadiusAttr(c.LACE_RADIUS)
        color = (.93, .72, .31) if i >= c.LINK_COUNT-2 else (.09, .53, .33)
        w._finish(stage, cap, color, True, contact=.00030)
        # Only immediately adjacent capsule ends overlap by construction.
        targets = [Sdf.Path("/Asset/Laces/"+link_name(side, j))
                   for j in (i-1, i+1) if 0 <= j < c.LINK_COUNT]
        if i == 0:
            targets.append(Sdf.Path(panel))
        UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(targets)
        prev = path


def body_view():
    import omni.usd
    from isaaclab_physx.physics import PhysxManager

    stage = omni.usd.get_context().get_stage()
    paths = sorted(str(p.GetPath()) for p in stage.Traverse()
                   if "/boot/Laces/" in str(p.GetPath()) and p.GetName().endswith(
                       tuple(f"Lace{i:02d}" for i in range(c.LINK_COUNT))))
    if len(paths) != len(c.SIDES) * c.LINK_COUNT:
        raise RuntimeError(f"Cannot bind native lace bodies: {paths}")
    return PhysxManager.get_physics_sim_view().create_rigid_body_view(paths)


def reset_laces(view, device):
    import warp as wp

    initial = {}
    for side in c.SIDES:
        positions, quats = initial_lace(side)
        for i, row in enumerate(np.column_stack((positions, quats))):
            initial[link_name(side, i)] = row
    poses = np.asarray([initial[p.rsplit("/", 1)[-1]] for p in view.prim_paths], np.float32)
    ids = wp.array(np.arange(len(poses), dtype=np.int32), dtype=wp.int32, device=device)
    view.set_transforms(wp.array(poses, dtype=wp.float32, device=device), ids)
    view.set_velocities(wp.zeros((len(poses), 6), dtype=wp.float32, device=device), ids)
