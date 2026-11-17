"""Passive tilting head and fork, removable collector, native rigid capsules."""

import math

import numpy as np
from yam_arena.urdf import axis_angle_matrix

from . import config as C
from .usd_scene import Writer


def capsule(w, s, path, color):
    # Original convex tapered cup, OD 33 mm, 27 mm high; 37 mm flange.
    from pxr import Gf, UsdGeom, UsdPhysics

    # Let native PhysX settle residual contact chatter in these light cups.
    # Moving contacts still wake them normally; the scorer is unchanged.
    w.Px.PhysxRigidBodyAPI.Apply(s.GetPrimAtPath(path)).CreateSleepThresholdAttr(
        C.CAPSULE_SLEEP_THRESHOLD
    )
    n = 32
    points = []
    for z, r in ((0.0, 0.012), (0.026, 0.0165)):
        points.extend(
            [
                (r * math.cos(k * math.tau / n), r * math.sin(k * math.tau / n), z)
                for k in range(n)
            ]
        )
    indices = []
    counts = []
    for k in range(n):
        indices.extend([k, (k + 1) % n, (k + 1) % n + n, k + n])
        counts.append(4)
    indices.extend(list(range(n - 1, -1, -1)))
    counts.append(n)
    indices.extend(list(range(n, 2 * n)))
    counts.append(n)
    m = UsdGeom.Mesh.Define(s, path + "/Cup")
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    m.CreateFaceVertexCountsAttr(counts)
    m.CreateFaceVertexIndicesAttr(indices)
    m.CreateSubdivisionSchemeAttr("none")
    w._finish(s, m, color, True)
    UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
    w.cylinder(s, path + "/Flange", 0.0185, 0.0018, (0, 0, 0.0269), color)
    w.cylinder(
        s,
        path + "/Foil",
        0.016,
        0.0003,
        (0, 0, 0.0280),
        (0.34, 0.26, 0.16) if color[0] < 0.6 else (0.88, 0.77, 0.42),
        collide=False,
    )


def collector_guide(w, s, path, side, color):
    """Passive lead-in chamfer: wide at the lip, 6 mm side clearance below."""
    from pxr import Gf, UsdGeom, UsdPhysics

    half_y = C.BIN_INNER[1] / 2 + C.BIN_WALL
    inner, outer = half_y + 0.006, half_y + 0.028
    section = [
        (inner, 0.009),
        (outer, 0.009),
        (outer, 0.025),
        (inner + 0.016, 0.025),
        (inner, 0.012),
    ]
    half_x = C.BIN_INNER[0] / 2 + C.BIN_WALL
    vertices = [
        (x, side * y, C.TOP + z)
        for x in (C.BIN[0] - half_x, C.BIN[0] + half_x)
        for y, z in section
    ]
    faces = [list(range(4, -1, -1)), list(range(5, 10))]
    faces += [[i, (i + 1) % 5, (i + 1) % 5 + 5, i + 5] for i in range(5)]
    if side < 0:
        faces = [face[::-1] for face in faces]
    mesh = UsdGeom.Mesh.Define(s, path)
    mesh.CreatePointsAttr([Gf.Vec3f(*v) for v in vertices])
    mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
    mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    w._finish(s, mesh, color, True)
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr(
        "convexHull"
    )


def create_assets(directory):
    from pxr import Gf, UsdGeom

    w = Writer(directory)
    assets = []
    silver = (0.53, 0.56, 0.58)
    dark = (0.10, 0.12, 0.14)
    case = (0.17, 0.19, 0.20)
    s = w.stage("machine")
    base = w.body(s, "/Asset/Base", 2.8, fixed=True, articulation=True)
    # Cutaway shell and a genuine open ejection path below the loading carrier.
    w.box(
        s,
        base + "/Rear",
        (0.025, 0.19, 0.25 + C.HEAD_RISE),
        (0.625, 0, C.TOP + (0.25 + C.HEAD_RISE) / 2),
        case,
    )
    for y in (-0.096, 0.096):
        w.box(
            s,
            base + f"/SideUpper{y}",
            (0.22, 0.008, 0.060),
            (0.510, y, C.TOP + 0.215 + C.HEAD_RISE),
            case,
        )
        w.box(
            s,
            base + f"/SideRear{y}",
            (0.025, 0.008, 0.175 + C.HEAD_RISE),
            (0.6025, y, C.TOP + 0.0975 + C.HEAD_RISE / 2),
            case,
        )
    w.box(s, base + "/Foot", (0.28, 0.17, 0.008), (0.50, 0, C.TOP + 0.004), dark)
    # Fixed, ordinary coffee-appliance components: the outlet is behind the
    # withdrawn collector's maximum X, and its drip grille stays below it.
    # These do not alter the passive ejection mechanism or its taught path.
    w.box(
        s, base + "/OutletBridge", (0.012, 0.190, 0.024),
        (0.405, 0, C.TOP + 0.217), case,
    )
    w.box(
        s, base + "/CoffeeOutletHousing", (0.020, 0.034, 0.018),
        (0.405, 0, C.TOP + 0.197), dark,
    )
    w.cylinder(
        s, base + "/CoffeeOutlet", 0.006, 0.018,
        (0.405, 0, C.TOP + 0.180), silver,
    )
    for k in range(7):
        w.box(
            s, base + f"/DripGrille{k}", (0.044, 0.004, 0.001),
            (0.385, (k - 3) * 0.011, C.TOP + 0.0085), silver,
        )
    tank = w.cylinder(
        s, base + "/WaterReservoir", 0.025, 0.180,
        (0.655, 0.040, C.TOP + 0.100), (0.56, 0.75, 0.80),
    )
    tank.CreateDisplayOpacityAttr([0.45])
    w.cylinder(
        s, base + "/ReservoirLid", 0.027, 0.008,
        (0.655, 0.040, C.TOP + 0.194), dark,
    )
    w.cylinder(
        s, base + "/ReservoirFoot", 0.025, 0.010,
        (0.655, 0.040, C.TOP + 0.005), dark,
    )
    # Fixed service modules occupy the lower shell. Only the shallow central
    # capsule cassette is removable; the housing does not travel with it.
    w.box(
        s, base + "/ServiceCore", (0.050, 0.125, 0.130), (0.590, 0, C.TOP + 0.073), case
    )
    for side in (-1, 1):
        w.box(
            s,
            base + f"/LowerModule{side + 1}",
            (0.125, 0.030, 0.105),
            (C.BIN[0], side * 0.080, C.TOP + 0.060),
            case,
        )
        w.box(
            s,
            base + f"/CarrierPillar{side + 1}",
            (0.020, 0.030, 0.115),
            (0.540, side * 0.080, C.TOP + 0.1695),
            case,
        )
    # The locating rails and stop are parts of the fixed machine body.
    for side in (-1, 1):
        collector_guide(w, s, base + "/Guide" + str(side + 1), side, dark)
    stop_x = C.BIN[0] + C.BIN_INNER[0] / 2 + C.BIN_WALL + 0.009
    w.box(s, base + "/BinStop", (0.006, 0.10, 0.015), (stop_x, 0, C.TOP + 0.015), dark)
    # Side-supported annular lower carrier; its opening remains physical.
    for n in range(32):
        a = (n + 0.5) * math.tau / 32
        r = 0.0184
        if abs(r * math.sin(a)) > 0.010:
            continue  # Clear slots for the lifting fork.
        rot = axis_angle_matrix(np.array([0, 0, 1]), a)
        w.box(
            s,
            base + f"/Seat{n}",
            (0.0032, 0.0040, 0.005),
            (
                C.SLOT[0] + r * math.cos(a),
                r * math.sin(a),
                C.TOP + 0.1925 + C.HEAD_RISE,
            ),
            silver,
            rotation=rot,
        )
    for y in (-0.085, 0.085):
        w.box(
            s,
            base + f"/Bridge{y}",
            (0.11, 0.008, 0.008),
            (0.445, y, C.TOP + 0.188 + C.HEAD_RISE),
            dark,
        )
    # Cross-beams below the moving fork and short central risers connect the
    # fixed carrier arcs to the housing. The 16 mm-wide risers leave the fork's
    # y=+/-19.5 mm lifting tines unobstructed in the physical reconstruction.
    for x in (0.420, 0.460):
        w.box(
            s, base + f"/CarrierBeam{x}", (0.004, 0.176, 0.004),
            (x, 0, C.TOP + 0.227), silver,
        )
        w.box(
            s, base + f"/CarrierRiser{x}", (0.004, 0.016, 0.008),
            (x, 0, C.TOP + 0.231), silver,
        )
    head = w.body(s, "/Asset/Head", 0.22)
    w.box(
        s,
        head + "/Cover",
        (0.22, 0.17, 0.018),
        (0.48, 0, C.TOP + 0.225 + C.HEAD_RISE),
        (0.21, 0.25, 0.27),
    )
    # Rim-support fork is open toward the rear. Gravity discharges the old
    # capsule down the chute when the head is raised far enough.
    for y in (-0.0195, 0.0195):
        w.box(
            s,
            head + f"/Fork{y}",
            (0.075, 0.005, 0.003),
            (0.445, y, C.TOP + 0.192 + C.HEAD_RISE),
            silver,
        )
    w.box(
        s,
        head + "/ForkFront",
        (0.006, 0.041, 0.006),
        (0.405, 0, C.TOP + 0.194 + C.HEAD_RISE),
        silver,
    )
    # Two solid supports connect the lifting fork to the lid in the supplied
    # physical reconstruction; they remain outside the capsule envelope.
    for side in (-1, 1):
        w.box(
            s, head + f"/ForkSupport{side + 1}", (0.006, 0.006, 0.025),
            (0.405, side * 0.0195, C.TOP + 0.248), silver,
        )
        w.box(
            s, head + f"/HingeEar{side + 1}", (0.018, 0.012, 0.044),
            (C.HINGE[0], side * 0.077, C.HINGE[2] + 0.010), dark,
        )
    # The native revolute bearing supplies the internal pin/bore constraint.
    # Draw the visible shaft; printable parts include actual 6.2 mm bores.
    w.cylinder(
        s, base + "/HingeShaft", 0.003, 0.216,
        C.HINGE, silver, axis="Y",
    )
    # Ordinary bar handle; the right end is clear of the machine's sidewall.
    w.cylinder(
        s,
        head + "/Handle",
        0.009,
        0.24,
        (C.HEAD_HANDLE[0], 0, C.HEAD_HANDLE[2]),
        dark,
        axis="Y",
    )
    for y in (-0.07, 0.07):
        w.box(
            s,
            head + f"/HandleLeg{y}",
            (0.012, 0.012, 0.035),
            (0.38, y, C.TOP + 0.222 + C.HEAD_RISE),
            dark,
        )
    w.hinge(
        s,
        "/Asset/HeadHinge",
        base,
        head,
        "Y",
        C.HINGE,
        C.HINGE,
        (0, math.radians(85)),
        friction=0.30,
        damping=0.008,
    )
    assets.append(
        {
            "name": "machine",
            "type": "articulation",
            "file": w.save(s),
            "initial_joints": {"HeadHinge": C.OPEN_ANGLE},
        }
    )
    s = w.stage("bin")
    body = w.body(s, "/Asset/Body", C.BIN_MASS)
    w.tray(s, body, C.BIN_INNER, C.BIN_H, 0.0, (0.30, 0.34, 0.36), wall=C.BIN_WALL)
    # A front pull bar with an unobstructed 24 mm vertical pinch. Side stems
    # leave the finger pads clear; no lip blocks their approach or release.
    w.box(s, body + "/Handle", C.BIN_GRIP_SIZE, C.BIN_GRIP, dark)
    for side in (-1, 1):
        w.box(
            s,
            body + f"/HandleStem{side + 1}",
            (0.026, 0.008, 0.010),
            (-0.071, side * 0.024, C.BIN_GRIP[2]),
            dark,
        )
    assets.append(
        {"name": "bin", "type": "rigid", "file": w.save(s), "position": C.BIN.tolist()}
    )
    s = w.stage("fresh")
    body = w.body(s, "/Asset/Body", 0.008)
    capsule(w, s, body, (0.76, 0.58, 0.13))
    assets.append(
        {
            "name": "fresh",
            "type": "rigid",
            "file": w.save(s),
            "position": C.FRESH.tolist(),
        }
    )
    used_positions = []
    for k in range(C.USED_COUNT):
        name = f"used{k}"
        s = w.stage(name)
        body = w.body(s, "/Asset/Body", 0.012)
        capsule(w, s, body, (0.42, 0.44, 0.45))
        if k < C.USED_COUNT - 1:
            # Six real-size cups in a 3-by-2 layer; no implausible tall stack
            # is needed to fill the compact collector. The seventh is ejected.
            pos = C.BIN + np.array(
                [(k % 3 - 1) * 0.039, (k // 3 * 2 - 1) * 0.020, 0.0025]
            )
            q = (0, 0, 0, 1)
        else:
            r = axis_angle_matrix(np.array([0, 1, 0]), C.OPEN_ANGLE)
            pos = C.HINGE + r @ (C.SLOT + np.array([0, 0, 0.002]) - C.HINGE)
            q = (0, math.sin(C.OPEN_ANGLE / 2), 0, math.cos(C.OPEN_ANGLE / 2))
        used_positions.append(pos.tolist())
        assets.append(
            {
                "name": name,
                "type": "rigid",
                "file": w.save(s),
                "position": pos.tolist(),
                "quaternion": q,
            }
        )
    s = w.stage("fixtures")
    p = "/Asset/Recycling"
    x = UsdGeom.Xform.Define(s, p)
    x.AddTranslateOp().Set(Gf.Vec3d(*map(float, C.WASTE)))
    w.tray(s, p, C.WASTE_INNER, C.WASTE_H, 0.002, (0.18, 0.44, 0.28), wall=0.004)
    # Stand for the single unused capsule, preventing it rolling before pickup.
    w.cylinder(
        s,
        "/Asset/FreshFoot",
        0.025,
        0.004,
        (C.FRESH[0], C.FRESH[1], C.TOP + 0.002),
        dark,
    )
    w.cylinder(
        s,
        "/Asset/FreshStand",
        0.009,
        0.066,
        (C.FRESH[0], C.FRESH[1], C.TOP + 0.037),
        dark,
    )
    assets.append({"name": "fixtures", "type": "base", "file": w.save(s)})
    w.inventory()
    return {"assets": assets, "settle_actions": 30, "used_initial": used_positions}
