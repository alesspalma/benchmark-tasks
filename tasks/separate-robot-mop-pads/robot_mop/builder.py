"""Native turnover cradle, removable carrier discs, and compliant peel joints."""

import numpy as np

from . import config as C
from .usd_scene import Writer

SHELL = (0.12, 0.14, 0.16)
RIM = (0.23, 0.26, 0.27)
FIXTURE = (0.36, 0.42, 0.45)
HANDLES = (0.55, 0.32, 0.16)
HARD = (0.15, 0.21, 0.24)
FIBRE = (0.67, 0.71, 0.70)
PROPS = (0.66, 0.57, 0.42)
CLOTH_TRAY = (0.39, 0.57, 0.68)
PARTS_TRAY = (0.42, 0.60, 0.46)


def station(w):
    s = w.stage("station")
    w.box(s, "/Asset/Floor", (4.0, 4.0, 0.028), (0.42, 0.0, -0.014), (0.32, 0.35, 0.36))
    w.box(
        s,
        "/Asset/Workbench",
        (0.92, 1.16, 0.035),
        (0.445, 0.0, C.TABLE - 0.0175),
        PROPS,
    )
    for i, x in enumerate((0.22, 0.74)):
        for j, y in enumerate((-0.49, 0.49)):
            w.box(
                s,
                f"/Asset/Leg{i}_{j}",
                (0.065, 0.065, 0.71),
                (x, y, 0.355),
                (0.38, 0.35, 0.29),
            )
    for i, y in enumerate((-0.305, 0.305)):
        w.box(
            s,
            f"/Asset/Pedestal{i}",
            (0.16, 0.16, 0.75),
            (-0.086, y, 0.375),
            (0.31, 0.34, 0.34),
        )
    for index, (x, color) in enumerate(
        ((C.CLOTH_TRAY_X, CLOTH_TRAY), (C.CARRIER_TRAY_X, PARTS_TRAY))
    ):
        if index == 0:
            w.rounded_box(
                s,
                "/Asset/ClothTray",
                (C.CLOTH_TRAY_WIDTH, C.CLOTH_TRAY_LENGTH, 0.008),
                (x, 0.0, C.SORT_Z - 0.004),
                color,
                0.014,
                0.001,
            )
            for mark, y in enumerate((-C.SORT_Y, C.SORT_Y)):
                w.cylinder(
                    s,
                    f"/Asset/ClothMark{mark}",
                    0.067,
                    0.0003,
                    (x, y, C.SORT_Z + 0.0002),
                    tuple(a * 0.85 for a in color),
                    collide=False,
                )
            for rim, sign in enumerate((-1.0, 1.0)):
                w.box(
                    s,
                    f"/Asset/ClothRimX{rim}",
                    (0.006, C.CLOTH_TRAY_LENGTH, 0.008),
                    (
                        x + sign * (C.CLOTH_TRAY_WIDTH / 2 - 0.003),
                        0.0,
                        C.SORT_Z + 0.004,
                    ),
                    color,
                )
                w.box(
                    s,
                    f"/Asset/ClothRimY{rim}",
                    (C.CLOTH_TRAY_WIDTH - 0.012, 0.006, 0.008),
                    (x, sign * (C.CLOTH_TRAY_LENGTH / 2 - 0.003), C.SORT_Z + 0.004),
                    color,
                )
            continue
        for pocket, y in enumerate((-C.SORT_Y, C.SORT_Y)):
            stem = f"/Asset/Tray{index}_{pocket}"
            w.rounded_box(
                s,
                stem,
                (0.194, 0.176, 0.008),
                (x, y, C.SORT_Z - 0.004),
                color,
                0.013,
                0.001,
            )
            w.cylinder(
                s,
                stem + "Mark",
                0.067,
                0.0003,
                (x, y, C.SORT_Z + 0.0002),
                tuple(a * 0.85 for a in color),
                collide=False,
            )
            for i, sign in enumerate((-1.0, 1.0)):
                w.box(
                    s,
                    stem + f"WallX{i}",
                    (0.006, 0.176, 0.015),
                    (x + sign * 0.094, y, C.SORT_Z + 0.0075),
                    color,
                )
                w.box(
                    s,
                    stem + f"WallY{i}",
                    (0.182, 0.006, 0.015),
                    (x, y + sign * 0.085, C.SORT_Z + 0.0075),
                    color,
                )
        # A common strip joins the two wells at the edge of the tray.
        strip_x = x + 0.091
        w.box(
            s,
            f"/Asset/Tray{index}Bridge",
            (0.012, 2 * C.SORT_Y - 0.176, 0.008),
            (strip_x, 0.0, C.SORT_Z - 0.004),
            color,
        )
    return s


def hinge(
    w,
    s,
    name,
    first,
    second,
    anchor,
    axis,
    limits,
    *,
    external=False,
    spring=0.0,
    friction=0.01,
):
    joint = w.Phy.RevoluteJoint.Define(s, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([first])
    joint.CreateBody1Rel().SetTargets([second])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, anchor)))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(limits[0])
    joint.CreateUpperLimitAttr(limits[1])
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateExcludeFromArticulationAttr(external)
    w.Px.PhysxJointAPI.Apply(joint.GetPrim()).CreateJointFrictionAttr(friction)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateDampingAttr(0.028 if not external else 0.008)
    drive.CreateStiffnessAttr(spring)
    return joint


def retainer(w, s, name, first, second, first_anchor, second_anchor, force, torque):
    joint = w.Phy.FixedJoint.Define(s, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([first])
    joint.CreateBody1Rel().SetTargets([second])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, first_anchor)))
    joint.CreateLocalPos1Attr(w.Gf.Vec3f(*map(float, second_anchor)))
    joint.CreateBreakForceAttr(force)
    joint.CreateBreakTorqueAttr(torque)
    joint.CreateCollisionEnabledAttr(False)
    joint.CreateExcludeFromArticulationAttr(True)


def annular_sector(w, s, path, outer, inner, height, start, end, color):
    angles = np.linspace(start, end, 4)
    edges = [
        *(np.c_[outer * np.cos(angles), outer * np.sin(angles)]),
        *(np.c_[inner * np.cos(angles[::-1]), inner * np.sin(angles[::-1])]),
    ]
    points = [(x, y, z) for z in (-height / 2, height / 2) for x, y in edges]
    n = len(edges)
    faces = [list(reversed(range(n))), list(range(n, 2 * n))]
    faces.extend([[i, (i + 1) % n, (i + 1) % n + n, i + n] for i in range(n)])
    mesh = w.Geom.Mesh.Define(s, path)
    mesh.CreatePointsAttr([w.Gf.Vec3f(*map(float, point)) for point in points])
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    w._finish(s, mesh, color, True)
    fabric = w.Shade.Material.Get(s, "/Asset/TextileContact")
    w.Shade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
        fabric, materialPurpose="physics"
    )


def pads(w, s, chassis):
    for side, y in (("L", C.PAD_Y), ("R", -C.PAD_Y)):
        xy = (C.PAD_X, y)
        plate = w.body(
            s, f"/Asset/{side}Carrier", 0.071, C.PIVOT + [*xy, C.PLATE_Z], ccd=True
        )
        w.cylinder(
            s, plate + "/PolymerPlatter", C.CARRIER_RADIUS, 0.012, (0.0, 0.0, 0.0), HARD
        )
        w.cylinder(
            s,
            plate + "/MagnetBoss",
            0.021,
            0.010,
            (0.0, 0.0, 0.008),
            RIM,
            collide=False,
        )
        w.cylinder(s, plate + "/CenterHub", 0.013, 0.029, (0.0, 0.0, -0.020), HARD)
        for i, a in enumerate(np.linspace(0.0, 2 * np.pi, 6, endpoint=False)):
            pp = (0.038 * np.cos(a), 0.038 * np.sin(a), -0.0062)
            w.cylinder(
                s,
                plate + f"/Recess{i}",
                0.010,
                0.0004,
                pp,
                (0.10, 0.13, 0.14),
                collide=False,
            )
        retainer(
            w,
            s,
            side + "MagneticMount",
            chassis,
            plate,
            (*xy, C.PLATE_Z),
            (0.0, 0.0, 0.0),
            47.0,
            2.5,
        )
        names = {
            h: w.body(
                s,
                f"/Asset/{side}Fabric{h}",
                0.018,
                C.PIVOT + [*xy, C.CLOTH_Z],
                ccd=True,
            )
            for h in ("Front", "Back")
        }
        for k in range(20):
            angle = k * np.pi / 10
            # After the turnover, the positive local X half is nearest the arms.
            half = "Front" if np.cos(angle + np.pi / 20) > 0.0 else "Back"
            shade = (k % 3) * 0.021
            annular_sector(
                w,
                s,
                names[half] + f"/Weave{k:02}",
                C.CLOTH_RADIUS,
                0.014,
                0.011,
                angle,
                angle + np.pi / 10,
                tuple(c - shade for c in FIBRE),
            )
        hinge(
            w,
            s,
            side + "ClothFlex",
            names["Back"],
            names["Front"],
            (0.0, 0.0, 0.0),
            "Y",
            (-100.0, 100.0),
            external=True,
            spring=0.14,
            friction=0.003,
        )
        for half, sign in (("Front", 1.0), ("Back", -1.0)):
            retainer(
                w,
                s,
                side + "HookLoop" + half,
                plate,
                names[half],
                (sign * 0.032, 0.0, C.CLOTH_Z - C.PLATE_Z),
                (sign * 0.032, 0.0, 0.0),
                27.0,
                0.44,
            )


def turnover(w):
    s = w.stage("turnover")
    material = w.Shade.Material.Define(s, "/Asset/TextileContact")
    surface = w.Phy.MaterialAPI.Apply(material.GetPrim())
    surface.CreateStaticFrictionAttr(0.94)
    surface.CreateDynamicFrictionAttr(0.78)
    surface.CreateRestitutionAttr(0.0)
    base = w.body(s, "/Asset/Frame", 3.0, fixed=True, articulation=True)
    for k, y in enumerate((-0.208, 0.208)):
        w.rounded_box(
            s,
            base + f"/Foot{k}",
            (0.171, 0.066, 0.013),
            (C.PIVOT[0], y, C.TABLE + 0.0065),
            FIXTURE,
            0.011,
            0.002,
        )
        w.box(
            s,
            base + f"/Upright{k}",
            (0.050, 0.035, C.PIVOT[2] - C.TABLE),
            (C.PIVOT[0], y, (C.PIVOT[2] + C.TABLE) / 2),
            FIXTURE,
        )
        w.cylinder(
            s,
            base + f"/Bearing{k}",
            0.026,
            0.044,
            (C.PIVOT[0], y, C.PIVOT[2]),
            RIM,
            "Y",
        )
    body = w.body(s, "/Asset/Chassis", 4.28, C.PIVOT, ccd=True)
    ballast = w.Phy.MassAPI.Get(s, body)
    ballast.CreateCenterOfMassAttr(w.Gf.Vec3f(-0.008, 0.0, 0.012))
    w.cylinder(s, body + "/LowerShell", 0.173, 0.078, (0.0, 0.0, 0.0), SHELL)
    w.cylinder(s, body + "/Bumper", 0.175, 0.028, (0.0, 0.0, 0.020), RIM)
    w.cylinder(s, body + "/TopCap", 0.170, 0.010, (0.0, 0.0, 0.047), (0.19, 0.23, 0.25))
    w.cylinder(
        s,
        body + "/NavigationTurret",
        0.035,
        0.009,
        (0.035, 0.0, 0.055),
        (0.10, 0.12, 0.14),
    )
    w.cylinder(
        s,
        body + "/ControlButton",
        0.009,
        0.002,
        (-0.107, 0.0, 0.053),
        (0.54, 0.60, 0.60),
        collide=False,
    )
    for i, y in enumerate((-0.126, 0.126)):
        w.cylinder(
            s,
            body + f"/Wheel{i}",
            0.030,
            0.028,
            (0.038, y, -0.034),
            (0.08, 0.09, 0.09),
            "Y",
            False,
        )
        for k, ang in enumerate(np.linspace(0.0, 2 * np.pi, 12, endpoint=False)):
            w.box(
                s,
                body + f"/Tread{i}_{k}",
                (0.005, 0.027, 0.006),
                (0.038 + 0.03 * np.cos(ang), y, -0.034 + 0.03 * np.sin(ang)),
                (0.14, 0.15, 0.15),
                collide=False,
            )
        w.box(
            s,
            body + f"/FoamClamp{i}",
            (0.084, 0.019, 0.058),
            (0.0, np.sign(y) * 0.164, 0.003),
            (0.32, 0.35, 0.34),
        )
        sgn = np.sign(y)
        w.cylinder(
            s,
            body + f"/Axle{i}",
            0.010,
            0.080,
            (0.0, sgn * 0.195, 0.0),
            FIXTURE,
            "Y",
            False,
        )
        w.box(
            s,
            body + f"/HandleArm{i}",
            (C.GRIP_RADIUS, 0.012, 0.018),
            (-C.GRIP_RADIUS / 2, sgn * 0.219, 0.0),
            FIXTURE,
        )
        w.cylinder(
            s,
            body + f"/Grip{i}",
            0.013,
            0.028,
            (-C.GRIP_RADIUS, sgn * C.GRIP_Y, 0.0),
            HANDLES,
            "Y",
        )
    for i, xx in enumerate((0.020, 0.046)):
        w.cylinder(
            s,
            body + f"/Brush{i}",
            0.010,
            0.148,
            (xx, 0.0, -0.042),
            (0.20, 0.43, 0.38),
            "Y",
            False,
        )
    w.cylinder(
        s,
        body + "/Caster",
        0.022,
        0.013,
        (0.105, 0.0, -0.048),
        (0.09, 0.11, 0.11),
        collide=False,
    )
    hinge(w, s, "Turn", base, body, C.PIVOT, "Y", (0.0, 180.0), friction=0.30)
    pads(w, s, body)
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [
        {"name": "station", "type": "base", "file": w.save(station(w))},
        {
            "name": "turnover",
            "type": "articulation",
            "file": w.save(turnover(w)),
            "joints": {"Turn": 0.0},
        },
    ]
    w.inventory()
    return {"assets": assets, "settle_actions": 16}
