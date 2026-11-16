"""Real-clearance conveyor joint, closed cover and passive spring clips."""

import itertools
import json
import math
from pathlib import Path

import numpy as np


def create_assets(directory):
    from chain_link import config as C
    from chain_link.config import (
        CHAIN_POSITION,
        DECK_TOP,
        NODES,
        PITCH,
        ROOT_CENTER,
        ROOT_YAW,
    )
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    inventory = []

    def stage_at(name):
        stage = Usd.Stage.CreateNew(str(directory / (name + ".usda")))
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        root = UsdGeom.Xform.Define(stage, "/" + name.title())
        stage.SetDefaultPrim(root.GetPrim())
        mat = UsdShade.Material.Define(stage, str(root.GetPath()) + "/Material")
        api = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        api.CreateStaticFrictionAttr(0.65)
        api.CreateDynamicFrictionAttr(0.5)
        api.CreateRestitutionAttr(0)
        return stage, mat

    def body(stage, path, mass, pos=(0, 0, 0), yaw=0):
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        x.AddOrientOp().Set(
            Gf.Quatf(math.cos(yaw / 2), Gf.Vec3f(0, 0, math.sin(yaw / 2)))
        )
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        rb = PhysxSchema.PhysxRigidBodyAPI.Apply(x.GetPrim())
        rb.CreateSolverPositionIterationCountAttr(32 if path.startswith("/Clip") else 8)
        rb.CreateSolverVelocityIterationCountAttr(8 if path.startswith("/Clip") else 1)
        rb.CreateMaxDepenetrationVelocityAttr(0.2)
        m = UsdPhysics.MassAPI.Apply(x.GetPrim())
        m.CreateMassAttr(mass)
        inventory.append(
            {
                "asset": Path(stage.GetRootLayer().identifier).stem,
                "body": path,
                "mass_kg": mass,
                "initial_local_position_m": list(map(float, pos)),
                "yaw_rad": yaw,
            }
        )

    def contact(mat, prim):
        UsdPhysics.CollisionAPI.Apply(prim)
        p = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        p.CreateContactOffsetAttr(0.00002)
        p.CreateRestOffsetAttr(0)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat, materialPurpose="physics")

    def box(stage, mat, path, size, pos, color, yaw=0):
        x = UsdGeom.Cube.Define(stage, path)
        x.CreateSizeAttr(1)
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        x.AddOrientOp().Set(
            Gf.Quatf(math.cos(yaw / 2), Gf.Vec3f(0, 0, math.sin(yaw / 2)))
        )
        x.AddScaleOp().Set(Gf.Vec3f(*map(float, size)))
        x.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        contact(mat, x.GetPrim())
        inventory.append(
            {
                "asset": Path(stage.GetRootLayer().identifier).stem,
                "solid": path,
                "kind": "box",
                "display_color_rgb": list(color),
                "size_m": list(map(float, size)),
                "center_m": list(map(float, pos)),
                "yaw_rad": yaw,
            }
        )

    def cylinder(stage, mat, path, radius, height, pos, color):
        x = UsdGeom.Cylinder.Define(stage, path)
        x.CreateRadiusAttr(radius)
        x.CreateHeightAttr(height)
        x.CreateAxisAttr("Z")
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        x.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        contact(mat, x.GetPrim())
        inventory.append(
            {
                "asset": Path(stage.GetRootLayer().identifier).stem,
                "solid": path,
                "kind": "cylinder",
                "display_color_rgb": list(color),
                "radius_m": radius,
                "height_m": height,
                "center_m": list(map(float, pos)),
                "yaw_rad": 0,
            }
        )

    def cone(stage, mat, path, radius, height, pos, color):
        x = UsdGeom.Cone.Define(stage, path)
        x.CreateRadiusAttr(radius)
        x.CreateHeightAttr(height)
        x.CreateAxisAttr("Z")
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        x.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        contact(mat, x.GetPrim())
        inventory.append(
            {
                "asset": Path(stage.GetRootLayer().identifier).stem,
                "solid": path,
                "kind": "cone",
                "display_color_rgb": list(color),
                "radius_m": radius,
                "height_m": height,
                "center_m": list(map(float, pos)),
                "yaw_rad": 0,
            }
        )

    def prism(stage, mat, path, vertices, color):
        x = UsdGeom.Mesh.Define(stage, path)
        points = [Gf.Vec3f(*map(float, p)) for p in vertices]
        x.CreatePointsAttr(points)
        x.CreateFaceVertexCountsAttr([4] * 6)
        x.CreateFaceVertexIndicesAttr(
            [0, 3, 2, 1, 4, 5, 6, 7, 0, 1, 5, 4, 1, 2, 6, 5, 2, 3, 7, 6, 3, 0, 4, 7]
        )
        x.CreateSubdivisionSchemeAttr("none")
        x.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        contact(mat, x.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(x.GetPrim()).CreateApproximationAttr(
            "convexHull"
        )
        # Preserve sub-millimetre leaf sections rather than letting the
        # default 1mm convex-cooking minimum expand their collision geometry.
        PhysxSchema.PhysxConvexHullCollisionAPI.Apply(
            x.GetPrim()
        ).CreateMinThicknessAttr(0.00001)
        inventory.append(
            {
                "asset": Path(stage.GetRootLayer().identifier).stem,
                "solid": path,
                "kind": "mesh",
                "vertices_m": [list(map(float, p)) for p in vertices],
            }
        )

    def pin_chamfer(stage, mat, path, y, color):
        # A normal C2 x45-degree machined lead, with an 8mm flat tip.
        angles = np.arange(32) * 2 * math.pi / 32
        vertices = [
            [radius * math.cos(a), y + radius * math.sin(a), z]
            for radius, z in ((0.006, 0.054), (0.004, 0.056))
            for a in angles
        ]
        faces = [list(range(31, -1, -1)), list(range(32, 64))]
        faces.extend([[k, (k + 1) % 32, (k + 1) % 32 + 32, k + 32] for k in range(32)])
        x = UsdGeom.Mesh.Define(stage, path)
        x.CreatePointsAttr([Gf.Vec3f(*map(float, p)) for p in vertices])
        x.CreateFaceVertexCountsAttr([len(f) for f in faces])
        x.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
        x.CreateSubdivisionSchemeAttr("none")
        x.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        contact(mat, x.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(x.GetPrim()).CreateApproximationAttr(
            "convexHull"
        )
        PhysxSchema.PhysxConvexHullCollisionAPI.Apply(
            x.GetPrim()
        ).CreateMinThicknessAttr(0.00001)
        inventory.append(
            {
                "asset": "connector",
                "solid": path,
                "kind": "mesh",
                "vertices_m": vertices,
            }
        )

    def ring(
        stage,
        mat,
        path,
        center,
        inner,
        outer,
        bottom,
        top,
        color,
        start=0,
        end=360,
        segments=32,
        inner_top=None,
    ):
        count = max(1, round((end - start) / 360 * segments))
        # Circumscribed inner polygon preserves the declared minimum bore.
        factor = math.cos(math.radians((end - start) / count) / 2)
        upper = (inner if inner_top is None else inner_top) / factor
        inner = inner / factor
        for k in range(count):
            a, b = np.deg2rad(np.linspace(start, end, count + 1)[k : k + 2])
            vertices = []
            for z, radius in ((bottom, inner), (top, upper)):
                xy = [
                    (radius * math.cos(a), radius * math.sin(a)),
                    (outer * math.cos(a), outer * math.sin(a)),
                    (outer * math.cos(b), outer * math.sin(b)),
                    (radius * math.cos(b), radius * math.sin(b)),
                ]
                vertices.extend([[center[0] + x, center[1] + y, z] for x, y in xy])
            prism(stage, mat, f"{path}/S{k:02}", vertices, color)

    def plate(stage, mat, path, z, color, along_y=False, bevel=0.0):
        # Two true circular bores; the bridge never fills either aperture.
        for k, sign in enumerate((-1, 1)):
            center = (0, sign * PITCH / 2) if along_y else (sign * PITCH / 2, 0)
            radius = C.BORE_DIAMETER / 2
            if bevel:
                ring(
                    stage,
                    mat,
                    path + f"/End{k}/LowerChamfer",
                    center,
                    radius + bevel,
                    0.0175,
                    z - 0.0025,
                    z - 0.0025 + bevel,
                    color,
                    inner_top=radius,
                )
                ring(
                    stage,
                    mat,
                    path + f"/End{k}/Core",
                    center,
                    radius,
                    0.0175,
                    z - 0.0025 + bevel,
                    z + 0.0025 - bevel,
                    color,
                )
                ring(
                    stage,
                    mat,
                    path + f"/End{k}/UpperChamfer",
                    center,
                    radius,
                    0.0175,
                    z + 0.0025 - bevel,
                    z + 0.0025,
                    color,
                    inner_top=radius + bevel,
                )
            else:
                ring(
                    stage,
                    mat,
                    path + f"/End{k}",
                    center,
                    radius,
                    0.0175,
                    z - 0.0025,
                    z + 0.0025,
                    color,
                )
        opening = C.BORE_DIAMETER / 2 + bevel
        size = (
            (0.035, PITCH - 2 * opening, 0.005)
            if along_y
            else (PITCH - 2 * opening, 0.035, 0.005)
        )
        box(stage, mat, path + "/Bridge", size, (0, 0, z), color)
        for sign in (-1, 1):
            size = (
                (0.0175 - opening, PITCH, 0.005)
                if along_y
                else (PITCH, 0.0175 - opening, 0.005)
            )
            pos = (
                (sign * (0.0175 + opening) / 2, 0, z)
                if along_y
                else (0, sign * (0.0175 + opening) / 2, z)
            )
            box(stage, mat, path + f"/Side{sign + 1}", size, pos, color)

    grey, orange, blue, dark = (
        (0.38, 0.41, 0.45),
        (0.95, 0.48, 0.07),
        (0.08, 0.36, 0.85),
        (0.13, 0.16, 0.19),
    )
    chain_grey = (0.075, 0.09, 0.105)
    tooth_grey = (0.13, 0.15, 0.17)
    stage, mat = stage_at("chain")
    yaws = []
    for n, (a, b) in enumerate(itertools.pairwise(NODES)):
        c, s = math.cos(ROOT_YAW), math.sin(ROOT_YAW)
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        center = ((a + b) / 2 - ROOT_CENTER) @ rotation
        yaw = math.atan2(b[1] - a[1], b[0] - a[0]) - ROOT_YAW
        yaws.append(yaw)
        path = f"/Chain/Link{n:02}"
        body(stage, path, 0.56 if n % 2 == 0 else 0.42, center, yaw)
        if n == 0:
            # Explicit root body: otherwise PhysX selects a middle chain link,
            # making the configured spawn pose refer to the wrong body.
            UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath("/Chain"))
            api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath("/Chain"))
            api.CreateEnabledSelfCollisionsAttr(True)
            api.CreateSolverPositionIterationCountAttr(8)
            api.CreateSolverVelocityIterationCountAttr(1)
        layers = (-0.0165, 0.0165) if n % 2 == 0 else (-0.0215, 0.0215)
        for layer, z in enumerate(layers):
            plate(stage, mat, path + f"/Plate{layer}", z, chain_grey)
        for k, x0 in enumerate((-PITCH / 2, PITCH / 2)):
            if n % 2 == 0:
                ring(
                    stage,
                    mat,
                    path + f"/Bush{k}",
                    (x0, 0),
                    C.BORE_DIAMETER / 2,
                    0.009,
                    -0.014,
                    0.014,
                    dark,
                )
                ring(
                    stage,
                    mat,
                    path + f"/Roller{k}",
                    (x0, 0),
                    0.0091,
                    0.0175,
                    -0.0138,
                    0.0138,
                    grey,
                )
            else:
                cylinder(stage, mat, path + f"/Pin{k}", 0.006, 0.048, (x0, 0, 0), dark)
        if n:
            prev = f"/Chain/Link{n - 1:02}"
            j = UsdPhysics.FixedJoint.Define(stage, f"/Chain/FixtureBrace{n:02}")
            j.CreateBody0Rel().SetTargets([prev])
            j.CreateBody1Rel().SetTargets([path])
            j.CreateLocalPos0Attr(Gf.Vec3f(PITCH / 2, 0, 0))
            j.CreateLocalPos1Attr(Gf.Vec3f(-PITCH / 2, 0, 0))
            for attr, angle in (
                (j.CreateLocalRot0Attr(), -yaws[n - 1]),
                (j.CreateLocalRot1Attr(), -yaw),
            ):
                attr.Set(
                    Gf.Quatf(math.cos(angle / 2), Gf.Vec3f(0, 0, math.sin(angle / 2)))
                )
            j.CreateCollisionEnabledAttr(False)
            inventory.append(
                {
                    "asset": "chain",
                    "joint": f"FixtureBrace{n:02}",
                    "type": "fixed representation of the pre-clamped chain fixture",
                    "parent": prev,
                    "child": path,
                    "initial_relative_angle_deg": 0,
                }
            )
    # The whole loop is pre-clamped for this assembly subtask. A fixed-base
    # zero-DOF articulation represents the visible tightened fixture clamps;
    # these constraints never switch during a trial.
    for index in (0,):
        joint = UsdPhysics.FixedJoint.Define(stage, f"/Chain/BenchClamp{index}")
        joint.CreateBody1Rel().SetTargets([f"/Chain/Link{index:02}"])
        point = CHAIN_POSITION + (NODES[index] + NODES[index + 1]) / 2
        joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float, point)))
        yaw = yaws[index] + ROOT_YAW
        joint.CreateLocalRot0Attr(
            Gf.Quatf(math.cos(yaw / 2), Gf.Vec3f(0, 0, math.sin(yaw / 2)))
        )
        joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
    stage.GetRootLayer().Save()

    stage, mat = stage_at("connector")
    body(stage, "/Connector/Body", 0.30)
    plate(stage, mat, "/Connector/Body/Plate", 0, orange, along_y=True)
    for n, y in enumerate((-PITCH / 2, PITCH / 2)):
        for name, r, lo, hi in [
            ("Stem", 0.006, -0.0045, C.GROOVE_Z - C.GROOVE_WIDTH / 2),
            (
                "Groove",
                C.GROOVE_DIAMETER / 2,
                C.GROOVE_Z - C.GROOVE_WIDTH / 2,
                C.GROOVE_Z + C.GROOVE_WIDTH / 2,
            ),
            ("Tip", 0.006, C.GROOVE_Z + C.GROOVE_WIDTH / 2, 0.054),
        ]:
            cylinder(
                stage,
                mat,
                f"/Connector/Body/{name}{n}",
                r,
                hi - lo,
                (0, y, (hi + lo) / 2),
                orange,
            )
        pin_chamfer(stage, mat, f"/Connector/Body/Lead{n}", y, orange)
        cylinder(
            stage,
            mat,
            f"/Connector/Body/RivetedHead{n}",
            0.008,
            0.004,
            (0, y, -0.0045),
            orange,
        )
    stage.GetRootLayer().Save()

    stage, mat = stage_at("keeper")
    body(stage, "/Keeper/Body", 0.16)
    plate(stage, mat, "/Keeper/Body/Plate", 0, blue, along_y=True, bevel=0.0005)
    stage.GetRootLayer().Save()

    for n in range(2):
        name = f"clip{n}"
        stage, mat = stage_at(name)
        root = f"/{name.title()}/Body"
        body(stage, root, 0.0002)
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(root))
        api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(root))
        api.CreateEnabledSelfCollisionsAttr(True)
        api.CreateSolverPositionIterationCountAttr(32)
        api.CreateSolverVelocityIterationCountAttr(8)
        ring(
            stage,
            mat,
            root + "/Back",
            (0, 0),
            0.0054,
            0.0074,
            -C.CLIP_THICKNESS / 2,
            C.CLIP_THICKNESS / 2,
            grey,
            125,
            235,
            segments=64,
        )
        for sign, start, end in [(1, 55, 125), (-1, 235, 305)]:
            angle = math.radians(125 if sign == 1 else 235)
            anchor = np.array([0.006 * math.cos(angle), 0.006 * math.sin(angle), 0.0])
            child = f"/{name.title()}/Jaw{sign + 1}"
            body(stage, child, 0.0001, anchor)
            ring(
                stage,
                mat,
                child + "/Leaf",
                (-anchor[0], -anchor[1]),
                0.0054,
                C.CLIP_LEAF_OUTER_RADIUS,
                -C.CLIP_THICKNESS / 2,
                C.CLIP_THICKNESS / 2,
                grey,
                start,
                end,
                segments=64,
            )
            lo, hi = (55, 75) if sign == 1 else (285, 305)
            ring(
                stage,
                mat,
                child + "/BearingEar",
                (-anchor[0], -anchor[1]),
                C.CLIP_LEAF_OUTER_RADIUS,
                0.0074,
                -C.CLIP_THICKNESS / 2,
                C.CLIP_THICKNESS / 2,
                grey,
                lo,
                hi,
                segments=64,
            )
            joint = UsdPhysics.RevoluteJoint.Define(
                stage, f"/{name.title()}/Spring{sign + 1}"
            )
            joint.CreateBody0Rel().SetTargets([root])
            joint.CreateBody1Rel().SetTargets([child])
            joint.CreateAxisAttr("Z")
            joint.CreateLocalPos0Attr(Gf.Vec3f(*map(float, anchor)))
            joint.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
            joint.CreateLowerLimitAttr(-25)
            joint.CreateUpperLimitAttr(25)
            joint.CreateCollisionEnabledAttr(False)
            drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
            drive.CreateTypeAttr("force")
            drive.CreateStiffnessAttr(C.CLIP_STIFFNESS / math.degrees(1))
            drive.CreateDampingAttr(0.003 / math.degrees(1))
            drive.CreateTargetPositionAttr(0.0)
        stage.GetRootLayer().Save()

    stage, mat = stage_at("tool")
    # A real passive XY guide supports the applicator throughout fitting.
    # Only robot contact moves it; there are no driven fixture joints.
    body(stage, "/Tool/Mount", 0.10)
    UsdPhysics.MassAPI(stage.GetPrimAtPath("/Tool/Mount")).CreateDiagonalInertiaAttr(
        Gf.Vec3f(1e-4, 1e-4, 1e-4)
    )
    body(stage, "/Tool/XSlide", 0.05)
    UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath("/Tool"))
    api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath("/Tool"))
    api.CreateEnabledSelfCollisionsAttr(False)
    api.CreateSolverPositionIterationCountAttr(32)
    api.CreateSolverVelocityIterationCountAttr(8)
    mount = UsdPhysics.FixedJoint.Define(stage, "/Tool/BenchMount")
    mount.CreateBody1Rel().SetTargets(["/Tool/Mount"])
    mount.CreateLocalPos0Attr(Gf.Vec3f(*map(float, C.TOOL_POSITION)))
    mount.CreateLocalRot0Attr(Gf.Quatf(0, Gf.Vec3f(0, 0, 1)))
    mount.CreateLocalPos1Attr(Gf.Vec3f(0, 0, 0))
    mount.CreateLocalRot1Attr(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
    for axis, parent, child, travel in (
        ("X", "Mount", "XSlide", C.PRESS_X_TRAVEL),
        ("Y", "XSlide", "Body", C.PRESS_Y_TRAVEL),
    ):
        joint = UsdPhysics.PrismaticJoint.Define(stage, "/Tool/Slide" + axis)
        joint.CreateBody0Rel().SetTargets(["/Tool/" + parent])
        joint.CreateBody1Rel().SetTargets(["/Tool/" + child])
        joint.CreateAxisAttr(axis)
        joint.CreateLowerLimitAttr(0.0)
        joint.CreateUpperLimitAttr(travel)
        joint.CreateCollisionEnabledAttr(False)
        PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim()).CreateJointFrictionAttr(0.02)
        inventory.append(
            {
                "asset": "tool",
                "joint": "/Tool/Slide" + axis,
                "type": "passive_prismatic",
                "bodies": ["/Tool/" + parent, "/Tool/" + child],
                "axis": axis,
                "limits_m": [0.0, travel],
                "joint_friction": 0.02,
            }
        )
    box(
        stage,
        mat,
        "/Tool/XSlide/Carriage",
        (0.030, 0.070, 0.010),
        (-0.055, 0.040, -0.015),
        grey,
    )
    body(stage, "/Tool/Body", 0.14)
    box(
        stage,
        mat,
        "/Tool/Body/Handle",
        (0.060, 0.045, 0.024),
        (0, 0.175, -0.032),
        dark,
    )
    box(
        stage,
        mat,
        "/Tool/Body/HandleBridge",
        (0.035, 0.012, 0.010),
        (-0.0375, 0.175, -0.032),
        dark,
    )
    # A stepped aluminium frame leaves room for the gripper below the chain.
    box(
        stage,
        mat,
        "/Tool/Body/HandleRiser",
        (0.012, 0.012, 0.0355),
        (-0.055, 0.175, -0.01925),
        dark,
    )
    box(
        stage,
        mat,
        "/Tool/Body/Crossbar",
        (0.012, 0.310, 0.006),
        (-0.055, 0.040, -0.0045),
        dark,
    )
    box(
        stage,
        mat,
        "/Tool/Body/GuideFoot",
        (0.012, 0.020, 0.002),
        (-0.055, 0.040, -0.0085),
        grey,
    )
    for n, y in enumerate((-PITCH / 2, PITCH / 2)):
        # The clips approach along-Y in the world, in line with the handle.
        # Rotating each pocket leaves the two pin sites unchanged.
        def pocket_box(stage, mat, path, size, center, color, center_y=y):
            x, yy, z = center
            box(
                stage,
                mat,
                path,
                size,
                (center_y - yy, center_y + x, z),
                color,
                math.pi / 2,
            )

        pocket_box(
            stage,
            mat,
            f"/Tool/Body/Pusher{n}",
            (0.003, 0.015, 0.004),
            (-0.0095, y, -0.0015),
            grey,
        )
        pocket_box(
            stage,
            mat,
            f"/Tool/Body/Stem{n}",
            (0.014, 0.012, 0.004),
            (-0.018, y, -0.00175),
            dark,
        )
        box(
            stage,
            mat,
            f"/Tool/Body/PocketBridge{n}",
            (0.055, 0.006, 0.004),
            (-0.0275, y - 0.022, -0.00175),
            dark,
        )
        pocket_box(
            stage,
            mat,
            f"/Tool/Body/RearFloor{n}",
            (0.0018, 0.013, 0.0008),
            (-0.0075, y, -0.0010),
            grey,
        )
        # Capture the rigid back of each clip while leaving its spring leaves
        # room to spread. The open-front channel releases the installed clip
        # when the applicator withdraws; no attachment or latch is involved.
        for sign in (-1, 1):
            pocket_box(
                stage,
                mat,
                f"/Tool/Body/BackGuide{n}_{sign + 1}",
                (0.006, 0.001, 0.0025),
                (-0.007, y + sign * 0.0067, -0.001),
                dark,
            )
            pocket_box(
                stage,
                mat,
                f"/Tool/Body/Floor{n}_{sign + 1}",
                (0.018, 0.0041, 0.0008),
                (0.001, y + sign * 0.00845, -0.0010),
                grey,
            )
            # Steel guide caps keep the loose clip's bearing ears in the
            # open-front channel during transport; the pin corridor stays open.
            pocket_box(
                stage,
                mat,
                f"/Tool/Body/GuideCap{n}_{sign + 1}",
                (0.018, 0.0041, 0.0002),
                (0.001, y + sign * 0.00845, 0.0007),
                grey,
            )
            pocket_box(
                stage,
                mat,
                f"/Tool/Body/Side{n}_{sign + 1}",
                (0.018, 0.002, 0.0023),
                (0.001, y + sign * 0.0095, -0.00035),
                dark,
            )
    stage.GetRootLayer().Save()

    stage, mat = stage_at("fixtures")
    box(
        stage,
        mat,
        "/Fixtures/Deck",
        (0.33, 0.48, 0.012),
        (0.555, 0, DECK_TOP - 0.006),
        (0.55, 0.57, 0.60),
    )
    for arm, i, sign in (("right", 0, 1), ("left", 10, -1)):
        a, b = NODES[i], NODES[i + 1]
        yaw = math.atan2(b[1] - a[1], b[0] - a[0])
        R = np.array(
            [
                [math.cos(yaw), -math.sin(yaw), 0],
                [math.sin(yaw), math.cos(yaw), 0],
                [0, 0, 1],
            ]
        )
        center = CHAIN_POSITION + (a + b) / 2
        # Low rear-half nests locate the bushes without covering the service
        # opening or obstructing the separate lower cover and clip applicator.
        local = np.array([sign * 0.025, 0.0, 0.0])
        p = center + R @ local
        box(
            stage,
            mat,
            f"/Fixtures/{arm}NestFloor",
            (0.040, 0.026, 0.005),
            (p[0], p[1], DECK_TOP + 0.0025),
            dark,
            yaw,
        )
        # Put the right pedestal under the front edge of its nest floor.
        # Its former central position intercepted the removable applicator
        # while it travelled below the loop. The floor remains supported.
        support_local = local + np.array([0, -0.0095, 0])
        if arm == "right":
            support_local[0] -= 0.015
        p = center + R @ support_local
        box(
            stage,
            mat,
            f"/Fixtures/{arm}NestSupport",
            (0.006, 0.006, DECK_TOP - C.TABLE_TOP),
            (p[0], p[1], (DECK_TOP + C.TABLE_TOP) / 2),
            dark,
            yaw,
        )
        for side in (-1, 1):
            p = center + R @ (local + [0, side * 0.0191, 0])
            box(
                stage,
                mat,
                f"/Fixtures/{arm}NestSide{side + 1}",
                (0.040, 0.003, 0.006),
                (p[0], p[1], DECK_TOP + 0.008),
                dark,
                yaw,
            )
        p = center + R @ np.array([sign * 0.0691, 0, 0])
        box(
            stage,
            mat,
            f"/Fixtures/{arm}NestBack",
            (0.003, 0.020, 0.006),
            (p[0], p[1], DECK_TOP + 0.008),
            dark,
            yaw,
        )
        for side in (-1, 1):
            p = center + R @ (local + [0, side * 0.0205, 0.0165])
            jaw_length = 0.040
            if arm == "right" and side == -1:
                # Shorten the front-facing jaw toward the rear of the link,
                # leaving access for a hand supporting the connecting plate.
                p += R @ np.array([0.010, 0, 0])
                jaw_length = 0.020
            box(
                stage,
                mat,
                f"/Fixtures/{arm}ClampJaw{side + 1}",
                (jaw_length, 0.006, 0.010),
                p,
                dark,
                yaw,
            )
    # Visible hold-down straps on every outer link keep the service loop in
    # its assembly shape. Chain articulation is not exercised in this subtask.
    for index in (1, 3, 5, 7, 9):
        point = C.CHAIN_POSITION + (NODES[index] + NODES[index + 1]) / 2
        radial = point[:2] - C.CHAIN_POSITION[:2]
        radial /= np.linalg.norm(radial)
        angle = math.atan2(radial[1], radial[0])
        center = point[:2] + radial * 0.010
        front = index in (1, 9)
        bolt = (
            np.array([0.395, math.copysign(0.205, point[1])])
            if front
            else point[:2] + radial * 0.035
        )
        if front:
            delta = bolt - point[:2]
            angle = math.atan2(delta[1], delta[0])
            center = (point[:2] + bolt) / 2
        strap_length = np.linalg.norm(bolt - point[:2]) + 0.020 if front else 0.060
        plate_top = C.CHAIN_POSITION[2] + 0.024
        box(
            stage,
            mat,
            f"/Fixtures/HoldDownStrap{index}",
            (strap_length, 0.020 if front else 0.025, 0.006),
            (*center, plate_top + 0.003),
            dark,
            angle,
        )
        base_z = C.TABLE_TOP + 0.004 if front else C.DECK_TOP
        if front:
            box(
                stage,
                mat,
                f"/Fixtures/HoldDownFoot{index}",
                (0.020, 0.020, 0.004),
                (*bolt, C.TABLE_TOP + 0.002),
                dark,
            )
        height = plate_top - base_z
        cylinder(
            stage,
            mat,
            f"/Fixtures/HoldDownPost{index}",
            0.005,
            height,
            (*bolt, base_z + height / 2),
            grey,
        )
        cylinder(
            stage,
            mat,
            f"/Fixtures/HoldDownBolt{index}",
            0.006,
            0.004,
            (*bolt, plate_top + 0.008),
            grey,
        )
    for name, point, height, R in (
        ("Connector", C.CONNECTOR_PICKUP_POSITION, 0.1475, C.CONNECTOR_ROTATION),
        ("Cover", C.KEEPER_POSITION, 0.1475, np.eye(3)),
    ):
        h = height + 0.0065
        box(
            stage,
            mat,
            f"/Fixtures/{name}LoadingBack",
            (0.004, 0.160, 0.008),
            (point[0] + 0.0210, point[1], point[2]),
            dark,
        )
        for sign in (-1, 1):
            box(
                stage,
                mat,
                f"/Fixtures/{name}LoadingCheek{sign + 1}",
                (0.004, 0.040, 0.008),
                (point[0] - 0.0210, point[1] + sign * 0.060, point[2]),
                dark,
            )
            box(
                stage,
                mat,
                f"/Fixtures/{name}LoadingEnd{sign + 1}",
                (0.046, 0.003, 0.008),
                (point[0], point[1] + sign * 0.0705, point[2]),
                dark,
            )
            for side in (-1, 1):
                box(
                    stage,
                    mat,
                    f"/Fixtures/{name}GuideSupport{sign + 1}_{side + 1}",
                    (0.008, 0.008, h),
                    (
                        point[0] + side * 0.0210,
                        point[1] + sign * 0.075,
                        C.TABLE_TOP + h / 2,
                    ),
                    dark,
                )
        for sign in (-1, 1):
            for side in (-1, 1):
                xy = point + R @ np.array([side * 0.010, sign * 0.020, 0.0])
                box(
                    stage,
                    mat,
                    f"/Fixtures/{name}LoadingPost{sign + 1}_{side + 1}",
                    (0.008, 0.008, height),
                    (xy[0], xy[1], C.TABLE_TOP + height / 2),
                    dark,
                )
    # The pedestal is behind the open joint, outside the cover's insertion
    # path. Fixed rails represent the mount; passive prismatic joints above
    # represent its linear bearings, with a small visual bearing clearance.
    press_center = C.TOOL_POSITION + C.LOADING_ROTATION @ np.array(
        [-0.0325, 0.040, -0.0255]
    )
    box(stage, mat, "/Fixtures/PressBase", (0.100, 0.080, 0.008), press_center, grey)
    pedestal_top = press_center[2] - 0.004
    height = pedestal_top - (C.TABLE_TOP + 0.010)
    box(
        stage,
        mat,
        "/Fixtures/PressPedestal",
        (0.030, 0.030, height),
        (press_center[0], press_center[1], C.TABLE_TOP + 0.010 + height / 2),
        dark,
    )
    box(
        stage,
        mat,
        "/Fixtures/PressFoot",
        (0.080, 0.080, 0.010),
        (press_center[0], press_center[1], C.TABLE_TOP + 0.005),
        grey,
    )
    for sign in (-1, 1):
        pos = C.TOOL_POSITION + C.LOADING_ROTATION @ np.array(
            [-0.0325, 0.040 + sign * 0.020, -0.0210]
        )
        box(
            stage,
            mat,
            f"/Fixtures/PressRail{sign + 1}",
            (0.090, 0.003, 0.001),
            pos,
            dark,
        )
    for i, x in enumerate((0.42, 0.66)):
        for j, y in enumerate((-0.19, 0.19)):
            box(
                stage,
                mat,
                f"/Fixtures/Post{i}{j}",
                (0.025, 0.025, DECK_TOP - 0.012 - 0.7475),
                (x, y, (DECK_TOP - 0.012 + 0.7475) / 2),
                dark,
            )
    # A fixed service former supports the loop during joining. Its low spokes
    # are recessed into the bed; positive teeth enter the open link windows.
    # The front service sector remains open for the two end links and keeper.
    cx, cy = CHAIN_POSITION[:2]
    cylinder(
        stage,
        mat,
        "/Fixtures/FormerHub",
        0.170,
        0.012,
        (cx, cy, CHAIN_POSITION[2]),
        chain_grey,
    )
    cylinder(
        stage,
        mat,
        "/Fixtures/FormerBoss",
        0.025,
        0.032,
        (cx, cy, DECK_TOP + 0.016),
        dark,
    )
    cylinder(
        stage,
        mat,
        "/Fixtures/FormerCap",
        0.012,
        0.003,
        (cx, cy, CHAIN_POSITION[2] + 0.0075),
        chain_grey,
    )
    for n in range(2, 9):
        point = (NODES[n] + NODES[n + 1]) / 2 + CHAIN_POSITION
        delta = point[:2] - CHAIN_POSITION[:2]
        angle = math.atan2(delta[1], delta[0])
        radius = float(np.linalg.norm(delta))
        box(
            stage,
            mat,
            f"/Fixtures/FormerSpoke{n}",
            (radius + 0.012, 0.022, 0.004),
            (cx + delta[0] / 2, cy + delta[1] / 2, DECK_TOP - 0.003),
            chain_grey,
            angle,
        )
        box(
            stage,
            mat,
            f"/Fixtures/FormerTooth{n}",
            (0.036, 0.018, 0.012),
            (point[0], point[1], CHAIN_POSITION[2]),
            tooth_grey,
            angle,
        )
    for n, angle in enumerate(np.deg2rad([45, 90, 135, 225, 270, 315])):
        cylinder(
            stage,
            mat,
            f"/Fixtures/FormerBolt{n}",
            0.0045,
            0.003,
            (
                cx + 0.116 * math.cos(angle),
                cy + 0.116 * math.sin(angle),
                CHAIN_POSITION[2] + 0.0075,
            ),
            dark,
        )
    stage.GetRootLayer().Save()
    (directory / "geometry-inventory.json").write_text(
        json.dumps(inventory, indent=2) + "\n"
    )
    return {
        n: directory / (n + ".usda")
        for n in ("chain", "connector", "keeper", "tool", "clip0", "clip1", "fixtures")
    }
