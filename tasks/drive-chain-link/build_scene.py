"""Original physical chain and headed-pin keeper. No runtime snaps or attachments."""

from pathlib import Path
import json
import math
import numpy as np


def create_assets(directory):
    from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade
    from chain_link.config import (
        DECK_TOP,
        TABLE_TOP,
        CHAIN_Z,
        NODES,
        PITCH,
        ROOT_CENTER,
        ROOT_YAW,
        SPROCKET_CENTERS,
        CONNECTOR_POSITION,
    )

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
        m = UsdPhysics.MassAPI.Apply(x.GetPrim())
        m.CreateMassAttr(mass)
        inventory.append(
            dict(
                asset=Path(stage.GetRootLayer().identifier).stem,
                body=path,
                mass_kg=mass,
                initial_local_position_m=list(map(float, pos)),
                yaw_rad=yaw,
            )
        )

    def contact(mat, prim):
        UsdPhysics.CollisionAPI.Apply(prim)
        p = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        p.CreateContactOffsetAttr(0.0002)
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
            dict(
                asset=Path(stage.GetRootLayer().identifier).stem,
                solid=path,
                kind="box",
                size_m=list(map(float, size)),
                center_m=list(map(float, pos)),
                yaw_rad=yaw,
            )
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
            dict(
                asset=Path(stage.GetRootLayer().identifier).stem,
                solid=path,
                kind="cylinder",
                radius_m=radius,
                height_m=height,
                center_m=list(map(float, pos)),
                yaw_rad=0,
            )
        )

    def cone(stage, mat, path, radius, height, pos, color, direction=1):
        x = UsdGeom.Cone.Define(stage, path)
        x.CreateRadiusAttr(radius)
        x.CreateHeightAttr(height)
        x.CreateAxisAttr("Z")
        x.AddTranslateOp().Set(Gf.Vec3d(*map(float, pos)))
        if direction == -1:
            x.AddOrientOp().Set(Gf.Quatf(0, Gf.Vec3f(1, 0, 0)))
        x.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        contact(mat, x.GetPrim())
        inventory.append(
            dict(
                asset=Path(stage.GetRootLayer().identifier).stem,
                solid=path,
                kind="cone",
                direction_z=direction,
                radius_m=radius,
                height_m=height,
                center_m=list(map(float, pos)),
                yaw_rad=0,
            )
        )

    grey, orange, blue, dark = (
        (0.38, 0.41, 0.45),
        (0.95, 0.48, 0.07),
        (0.08, 0.36, 0.85),
        (0.13, 0.16, 0.19),
    )
    stage, mat = stage_at("chain")
    yaws = []
    for n, (a, b) in enumerate(zip(NODES[:-1], NODES[1:])):
        c, s = math.cos(ROOT_YAW), math.sin(ROOT_YAW)
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        center = ((a + b) / 2 - ROOT_CENTER) @ rotation
        yaw = math.atan2(b[1] - a[1], b[0] - a[0]) - ROOT_YAW
        yaws.append(yaw)
        path = f"/Chain/Link{n:02}"
        body(stage, path, 0.050 if n % 2 == 0 else 0.080, center, yaw)
        if n == 0:
            # Explicit root body: otherwise PhysX selects a middle chain link,
            # making the configured spawn pose refer to the wrong body.
            UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath(path))
            api = PhysxSchema.PhysxArticulationAPI.Apply(stage.GetPrimAtPath(path))
            api.CreateEnabledSelfCollisionsAttr(True)
            api.CreateSolverPositionIterationCountAttr(32)
            api.CreateSolverVelocityIterationCountAttr(8)
        z_plate = 0.010 if n % 2 == 0 else 0.015
        layers = [(-z_plate, 0.004), (z_plate, 0.004)]
        for layer, (z, height) in enumerate(layers):
            for end, x0 in enumerate((-PITCH / 2, PITCH / 2)):
                for j in range(20):
                    theta = j * math.tau / 20
                    box(
                        stage,
                        mat,
                        path + f"/Ring{layer}_{end}_{j}",
                        (0.007, 0.0075, height),
                        (x0 + 0.0195 * math.cos(theta), 0.0195 * math.sin(theta), z),
                        grey,
                        theta,
                    )
            for side in (-1, 1):
                box(
                    stage,
                    mat,
                    path + f"/Rail{layer}_{side + 1}",
                    (PITCH, 0.006, height),
                    (0, side * 0.020, z),
                    grey,
                )
        if n % 2:
            for j, x0 in enumerate((-PITCH / 2, PITCH / 2)):
                cylinder(
                    stage, mat, path + f"/HingePin{j}", 0.010, 0.034, (x0, 0, 0), dark
                )
                for side in (-1, 1):
                    cylinder(
                        stage,
                        mat,
                        path + f"/PinCap{j}_{side + 1}",
                        0.018,
                        0.004,
                        (x0, 0, side * 0.019),
                        dark,
                    )
        if n:
            prev = f"/Chain/Link{n - 1:02}"
            j = UsdPhysics.RevoluteJoint.Define(stage, f"/Chain/Hinge{n:02}")
            j.CreateBody0Rel().SetTargets([prev])
            j.CreateBody1Rel().SetTargets([path])
            j.CreateAxisAttr("Z")
            j.CreateLocalPos0Attr(Gf.Vec3f(PITCH / 2, 0, 0))
            j.CreateLocalPos1Attr(Gf.Vec3f(-PITCH / 2, 0, 0))
            for attr, angle in (
                (j.CreateLocalRot0Attr(), -yaws[n - 1]),
                (j.CreateLocalRot1Attr(), -yaw),
            ):
                attr.Set(
                    Gf.Quatf(math.cos(angle / 2), Gf.Vec3f(0, 0, math.sin(angle / 2)))
                )
            j.CreateLowerLimitAttr(-150)
            j.CreateUpperLimitAttr(150)
            j.CreateCollisionEnabledAttr(False)
            PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(
                0.00005
            )
            inventory.append(
                dict(
                    asset="chain",
                    joint=f"Hinge{n:02}",
                    axis="Z",
                    limits_deg=[-150, 150],
                    parent=prev,
                    child=path,
                    initial_relative_angle_deg=0,
                )
            )
    stage.GetRootLayer().Save()

    stage, mat = stage_at("connector")
    body(stage, "/Connector/Body", 0.20)
    box(stage, mat, "/Connector/Body/Backing", (0.045, 0.140, 0.006), (0, 0, 0), orange)
    for n, y in enumerate((-PITCH / 2, PITCH / 2)):
        cylinder(
            stage, mat, f"/Connector/Body/Stem{n}", 0.005, 0.040, (0, y, -0.023), grey
        )
        cylinder(
            stage, mat, f"/Connector/Body/Head{n}", 0.012, 0.006, (0, y, -0.046), orange
        )
    box(
        stage, mat, "/Connector/Body/Grip", (0.024, 0.030, 0.040), (0, 0, 0.024), orange
    )
    for n, y in enumerate((-PITCH / 2, PITCH / 2)):
        cone(
            stage,
            mat,
            f"/Connector/Body/Lead{n}",
            0.012,
            0.008,
            (0, y, -0.053),
            orange,
            direction=-1,
        )
    stage.GetRootLayer().Save()

    stage, mat = stage_at("keeper")
    body(stage, "/Keeper/Body", 0.075)
    # Two side-entry forks admit the 8 mm stems below the 24 mm heads.
    # The narrow outer outline clears the chain-end grip posts.
    xs = [-0.040, -0.026, 0.040]
    ys = [-0.064, -0.060, -0.040, 0.040, 0.060, 0.064]
    n = 0
    for xa, xb in zip(xs[:-1], xs[1:]):
        for ya, yb in zip(ys[:-1], ys[1:]):
            x, y = (xa + xb) / 2, (ya + yb) / 2
            empty = any(
                -0.026 < x and abs(y - center) < 0.010
                for center in (-PITCH / 2, PITCH / 2)
            )
            if not empty:
                box(
                    stage,
                    mat,
                    f"/Keeper/Body/Cell{n:02}",
                    (xb - xa, yb - ya, 0.012),
                    (x, y, 0),
                    blue,
                )
                n += 1
    box(
        stage,
        mat,
        "/Keeper/Body/HandleBridge",
        (0.024, 0.100, 0.008),
        (0, 0.110, 0),
        blue,
    )
    box(stage, mat, "/Keeper/Body/Grip", (0.024, 0.030, 0.070), (0, 0.140, 0.035), blue)
    stage.GetRootLayer().Save()

    stage, mat = stage_at("fixtures")
    # Raised deck with a real front service opening for the underside keeper.
    box(
        stage,
        mat,
        "/Fixtures/DeckRear",
        (0.50, 0.48, 0.012),
        (0.52, 0, DECK_TOP - 0.006),
        (0.55, 0.57, 0.60),
    )
    box(
        stage,
        mat,
        "/Fixtures/DeckFrontLeft",
        (0.18, 0.06, 0.012),
        (0.18, 0.210, DECK_TOP - 0.006),
        (0.55, 0.57, 0.60),
    )
    box(
        stage,
        mat,
        "/Fixtures/DeckFrontRight",
        (0.18, 0.11, 0.012),
        (0.18, -0.185, DECK_TOP - 0.006),
        (0.55, 0.57, 0.60),
    )
    for i, x in enumerate((0.12, 0.72)):
        for j, y in enumerate((-0.19, 0.19)):
            box(
                stage,
                mat,
                f"/Fixtures/Post{i}{j}",
                (0.025, 0.025, DECK_TOP - 0.012 - TABLE_TOP),
                (x, y, (DECK_TOP - 0.012 + TABLE_TOP) / 2),
                dark,
            )
    for n, (x, y) in enumerate(SPROCKET_CENTERS):
        cylinder(
            stage,
            mat,
            f"/Fixtures/Wheel{n}Shaft",
            0.022,
            CHAIN_Z - DECK_TOP,
            (x, y, (CHAIN_Z + DECK_TOP) / 2),
            orange,
        )
        cylinder(
            stage, mat, f"/Fixtures/Wheel{n}Hub", 0.065, 0.012, (x, y, CHAIN_Z), dark
        )
        for tooth in range(6):
            angle = tooth * math.tau / 6
            box(
                stage,
                mat,
                f"/Fixtures/Wheel{n}Tooth{tooth}",
                (0.050, 0.032, 0.012),
                (x + 0.090 * math.cos(angle), y + 0.090 * math.sin(angle), CHAIN_Z),
                grey,
                angle,
            )
        # A visible rim keeps the chain on its working plane. The first wheel
        # has a service window at the missing connecting link.
        for sector in range(48):
            angle = (sector + 0.5) * math.tau / 48
            if n == 0 and 120 < math.degrees(angle) < 240:
                continue
            box(
                stage,
                mat,
                f"/Fixtures/Wheel{n}Guide{sector}",
                (0.016, 0.017, 0.006),
                (
                    x + 0.124 * math.cos(angle),
                    y + 0.124 * math.sin(angle),
                    CHAIN_Z + 0.025,
                ),
                orange,
                angle,
            )
        # Three radial supports connect the guide rim to the hub. They pass
        # above the chain, with a 1 mm gap over the pin caps.
        for spoke in range(3):
            angle = spoke * math.tau / 3
            if n == 0 and 120 <= math.degrees(angle) <= 240:
                continue
            box(
                stage,
                mat,
                f"/Fixtures/Wheel{n}GuideSupport{spoke}",
                (0.120, 0.009, 0.006),
                (
                    x + 0.065 * math.cos(angle),
                    y + 0.065 * math.sin(angle),
                    CHAIN_Z + 0.025,
                ),
                orange,
                angle,
            )
        cylinder(
            stage,
            mat,
            f"/Fixtures/Wheel{n}TopHub",
            0.025,
            0.030,
            (x, y, CHAIN_Z + 0.010),
            orange,
        )
    # Real service-holder channels keep the two free ends aligned while the
    # missing link is inserted. The underside remains open for the keeper.
    for end, (a, b, inner) in enumerate(
        ((NODES[0], NODES[1], NODES[1]), (NODES[-2], NODES[-1], NODES[-2]))
    ):
        midpoint = (a + b) / 2
        angle = math.atan2((b - a)[1], (b - a)[0])
        normal = np.array([-math.sin(angle), math.cos(angle), 0.0])
        low, high = CHAIN_Z - 0.011, CHAIN_Z + 0.004
        for side in (-1, 1):
            q = midpoint + side * 0.026 * normal
            box(
                stage,
                mat,
                f"/Fixtures/EndHolder{end}Rail{side + 1}",
                (0.100, 0.004, high - low),
                (q[0], q[1], (low + high) / 2),
                dark,
                angle,
            )
            q = inner + side * 0.034 * normal
            box(
                stage,
                mat,
                f"/Fixtures/EndHolder{end}Post{side + 1}",
                (0.012, 0.012, high - DECK_TOP),
                (q[0], q[1], (DECK_TOP + high) / 2),
                dark,
                angle,
            )
            q = inner + side * 0.030 * normal
            box(
                stage,
                mat,
                f"/Fixtures/EndHolder{end}Bridge{side + 1}",
                (0.012, 0.020, 0.004),
                (q[0], q[1], high - 0.002),
                dark,
                angle,
            )
    # Connector pickup stand: supports its backing while downward pins hang free.
    x, y, z = CONNECTOR_POSITION
    for i, dy in enumerate((-0.069, 0.069)):
        height = z - 0.003 - TABLE_TOP
        box(
            stage,
            mat,
            f"/Fixtures/ConnectorStand{i}",
            (0.050, 0.008, height),
            (x, y + dy, TABLE_TOP + height / 2),
            dark,
        )
    box(
        stage,
        mat,
        "/Fixtures/KeeperHandleRest",
        (0.050, 0.030, DECK_TOP - TABLE_TOP),
        (0.18, 0.30, (DECK_TOP + TABLE_TOP) / 2),
        dark,
    )
    stage.GetRootLayer().Save()
    (directory / "geometry-inventory.json").write_text(
        json.dumps(inventory, indent=2) + "\n"
    )
    return {
        n: directory / (n + ".usda")
        for n in ("chain", "connector", "keeper", "fixtures")
    }
