"""Native hose end, sleeve cam, radial locking dogs, valve and two spray heads."""

from math import cos, pi, sin

from . import config as C
from .usd_scene import Writer

ORANGE = (0.93, 0.31, 0.058)
BLACK = (0.085, 0.096, 0.102)
BRASS = (0.72, 0.50, 0.24)
TEAL = (0.035, 0.25, 0.57)
GRAY = (0.30, 0.35, 0.40)
STEEL = (0.60, 0.66, 0.66)
GREEN = (0.147, 0.28, 0.14)
WATER = (0.28, 0.76, 0.91)


def prismatic(
    w, s, name, parent, child, axis, low, high, preload, stiffness, damping, max_force
):
    joint = w.Phy.PrismaticJoint.Define(s, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([parent])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(low)
    joint.CreateUpperLimitAttr(high)
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateTargetPositionAttr(preload)
    drive.CreateMaxForceAttr(max_force)
    return joint


def mesh(w, s, path, points, faces, color, collide):
    g = w.Geom.Mesh.Define(s, path)
    g.CreatePointsAttr([w.Gf.Vec3f(*p) for p in points])
    g.CreateFaceVertexCountsAttr([len(f) for f in faces])
    g.CreateFaceVertexIndicesAttr([i for f in faces for i in reversed(f)])
    g.CreateSubdivisionSchemeAttr("none")
    if collide:
        w.Phy.MeshCollisionAPI.Apply(g.GetPrim()).CreateApproximationAttr("convexHull")
    w._finish(s, g, color, collide, contact=0.00023)


def shell(w, s, path, profile, color, pieces=12, collide=True):
    """An axial sleeve or cam with an open bore, as tiled convex sectors."""
    for i in range(pieces):
        radial = []
        a, b = 2 * pi * (i - 0.015) / pieces, 2 * pi * (i + 1.015) / pieces
        for y, inner, outer in profile:
            radial += [
                (r * cos(theta), y, r * sin(theta))
                for r, theta in ((inner, a), (outer, a), (outer, b), (inner, b))
            ]
        k = len(radial) - 4
        faces = [(0, 1, 2, 3), (k + 3, k + 2, k + 1, k)]
        for level in range(0, k, 4):
            for j in range(4):
                n = (j + 1) % 4
                faces.append((level + j, level + 4 + j, level + 4 + n, level + n))
        mesh(w, s, path + str(i), radial, faces, color, collide)


def tapered(w, s, path, y0, r0, y1, r1, color, collide=True):
    """Solid, hollowless short chamfered male fitting segment."""
    n = 16
    points = [
        (r * cos(i * 2 * pi / n), y, r * sin(i * 2 * pi / n))
        for y, r in ((y0, r0), (y1, r1))
        for i in range(n)
    ]
    faces = [tuple(range(n)), tuple(reversed(range(n, 2 * n)))]
    faces += [(i, n + i, n + (i + 1) % n, (i + 1) % n) for i in range(n)]
    mesh(w, s, path, points, faces, color, collide)


def bench(w):
    s = w.stage("bench")
    w.box(
        s,
        "/Asset/Bench",
        (0.91, 1.19, 0.045),
        (0.34, 0.0, C.TABLE - 0.0225),
        (0.48, 0.51, 0.43),
    )
    w.box(s, "/Asset/Floor", (8.0, 8.0, 0.02), (0.0, 0.0, -0.01), (0.31, 0.35, 0.34))
    x, y, length, width = C.CATCH
    w.rounded_box(
        s,
        "/Asset/CatchTray",
        (length, width, 0.011),
        (x, y, C.TABLE + 0.006),
        (0.21, 0.30, 0.31),
        0.013,
        0.002,
    )
    for i, offset in enumerate((-1.0, 1.0)):
        w.rounded_box(
            s,
            f"/Asset/CatchSide{i}",
            (length - 0.014, 0.006, 0.020),
            (x, y + offset * (width - 0.007) / 2.0, C.TABLE + 0.015),
            (0.25, 0.35, 0.37),
            0.002,
            0.001,
        )
        w.rounded_box(
            s,
            f"/Asset/CatchEnd{i}",
            (0.006, width - 0.010, 0.020),
            (x + offset * (length - 0.010) / 2.0, y, C.TABLE + 0.015),
            (0.25, 0.35, 0.37),
            0.002,
            0.001,
        )
    # Tiny surface disturbances at the measured landing point make the
    # short downward spray visible above the lip of the catch tray.
    impact = w.Geom.Xform.Define(s, "/Asset/SprayImpact")
    impact.AddTranslateOp().Set(w.Gf.Vec3d(0.0, 0.0, C.TABLE + 0.0125))
    for i, (dx, dy, width) in enumerate(
        (
            (-0.030, -0.006, 0.013),
            (-0.021, 0.010, 0.011),
            (-0.012, -0.015, 0.010),
            (0.0, 0.016, 0.012),
            (0.010, -0.014, 0.013),
            (0.020, 0.009, 0.010),
            (0.030, -0.008, 0.013),
        )
    ):
        w.rounded_box(
            s,
            f"/Asset/SprayImpact/Ripple{i}",
            (width, 0.006, 0.001),
            (dx, dy, 0.0),
            (0.55, 0.88, 0.94),
            0.0025,
            0.0002,
            collide=False,
        )
    w.Geom.Imageable(impact).MakeInvisible()
    # A shallow open-sided cradle offers the parked parts a repeatable rest.
    for name, p, color in (
        ("Parts", C.PARK, (0.44, 0.39, 0.27)),
        ("Spare", C.BLUE_READY, (0.31, 0.32, 0.25)),
    ):
        w.rounded_box(
            s,
            f"/Asset/{name}Tray",
            (0.110, 0.185, 0.006),
            (p[0], p[1] - 0.022, C.TABLE + 0.003),
            color,
            0.012,
            0.001,
        )
        w.box(
            s,
            f"/Asset/{name}Pedestal",
            (0.029, 0.102, 0.0475),
            (p[0], p[1] - 0.017, p[2] - 0.019 - 0.0475 / 2),
            color,
        )
    x, y, z = C.SOCKET
    w.box(
        s,
        "/Asset/HoseSupport",
        (0.055, 0.069, 0.048),
        (x, y + 0.110, C.TABLE + 0.024),
        GRAY,
    )
    w.cylinder(s, "/Asset/FeedHose", 0.009, 0.183, (x, y + 0.184, z), GREEN, "Y")
    for i, offset in enumerate((0.099, 0.140, 0.218)):
        w.cylinder(
            s, f"/Asset/FeedBand{i}", 0.0105, 0.008, (x, y + offset, z), BLACK, "Y"
        )
    w.rounded_box(
        s,
        "/Asset/FeedHousing",
        (0.065, 0.076, 0.061),
        (x, y + 0.297, z - 0.006),
        BLACK,
        0.008,
        0.002,
    )
    return s


def coupling(w):
    s = w.stage("coupling")
    anchor = w.body(s, "/Asset/HoseSupportBody", 0.45, articulation=True)
    root = w.Phy.FixedJoint.Define(s, "/Asset/HoseAnchor")
    root.CreateBody1Rel().SetTargets([anchor])
    root.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, C.SOCKET)))
    # The supported short hose end has longitudinal compliance under collar force.
    body = w.body(s, "/Asset/Socket", 0.076)
    prismatic(
        w, s, "HoseCompliance", anchor, body, "Y", -0.007, 0.007, 0.0, 850.0, 4.0, 20.0
    )
    w.cylinder(
        s, body + "/Hose", 0.0092, 0.081, (0.0, 0.073, 0.0), GREEN, "Y", collide=False
    )
    w.cylinder(s, body + "/Back", 0.014, 0.026, (0.0, 0.040, 0.0), BLACK, "Y")
    shell(
        w,
        s,
        body + "/TailGrip",
        ((0.0195, 0.0092, 0.0138), (0.0345, 0.0092, 0.0138)),
        BLACK,
        collide=False,
    )
    shell(
        w,
        s,
        body + "/SocketMouth",
        ((-0.002, 0.012, 0.0146), (0.014, 0.011, 0.0146)),
        BLACK,
        collide=False,
    )
    shell(
        w,
        s,
        body + "/PlugGuide",
        ((0.0105, 0.01075, 0.0138), (0.022, 0.01075, 0.0138)),
        BRASS,
    )
    shell(
        w,
        s,
        body + "/LeadIn",
        ((-0.003, 0.0132, 0.0148), (0.002, 0.0114, 0.0148)),
        BLACK,
    )
    collar = w.body(s, "/Asset/Collar", 0.025)
    prismatic(
        w,
        s,
        "ReleaseTravel",
        body,
        collar,
        "Y",
        0.0,
        C.COLLAR_STROKE,
        -0.002,
        195.0,
        2.0,
        12.0,
    )
    # An annular lock band captures the backs of the radial dogs. The relief
    # ahead of the band gives the dogs their outward travel when retracted.
    shell(
        w,
        s,
        collar + "/Front",
        ((-0.010, 0.020, 0.0215), (0.001, 0.020, 0.0215)),
        ORANGE,
    )
    shell(
        w,
        s,
        collar + "/Cam",
        ((0.001, 0.0198, 0.0215), (0.005, 0.0172, 0.0215)),
        ORANGE,
    )
    shell(
        w,
        s,
        collar + "/LockBand",
        ((0.005, 0.0172, 0.0215), (0.015, 0.0172, 0.0215)),
        ORANGE,
    )
    for sign in (-1.0, 1.0):
        side = "Front" if sign < 0 else "Rear"
        w.rounded_box(
            s,
            collar + f"/{side}GripFlat",
            (0.0065, 0.020, 0.016),
            (sign * 0.0215, 0.001, 0.0),
            ORANGE,
            0.0015,
            0.0005,
        )
    for i in range(12):
        phi = i * pi / 6
        w.box(
            s,
            collar + f"/Rib{i}",
            (0.0035, 0.019, 0.0028),
            (0.022 * cos(phi), 0.003, 0.022 * sin(phi)),
            BLACK,
            collide=False,
        )

    # The cam transfers its clamping load through a bounded radial force
    # in the environment. Keep its surface clear of the dog colliders.
    pairs = w.Phy.FilteredPairsAPI.Apply(
        s.GetPrimAtPath(collar)
    ).CreateFilteredPairsRel()
    for name, sign in (("FrontDog", -1), ("RearDog", 1)):
        dog = w.body(s, "/Asset/" + name, 0.010)
        pairs.AddTarget(dog)
        # In the locked state the inner tooth lies inside the nipple groove
        # while its outer shoulder is backed by the collar's lock band.
        prismatic(
            w,
            s,
            name + "Travel",
            body,
            dog,
            "X",
            min(0, sign * 0.0044),
            max(0, sign * 0.0044),
            -sign * 0.0005,
            150.0,
            1.0,
            4.0,
        )
        w.box(
            s,
            dog + "/Tooth",
            (0.0076, 0.0035, 0.0072),
            (sign * 0.01105, 0.0068, 0.0),
            BRASS,
        )
        w.box(
            s,
            dog + "/Heel",
            (0.0033, 0.0060, 0.0072),
            (sign * 0.0140, 0.0068, 0.0),
            BRASS,
        )

    poppet = w.body(s, "/Asset/Poppet", 0.012)
    guide = w.Phy.FilteredPairsAPI.Apply(
        s.GetPrimAtPath(poppet)
    ).CreateFilteredPairsRel()
    for guided in (anchor, body, collar, "/Asset/FrontDog", "/Asset/RearDog"):
        guide.AddTarget(guided)
    prismatic(
        w,
        s,
        "ValveTravel",
        body,
        poppet,
        "Y",
        0.0,
        C.POPPET_STROKE,
        -0.0012,
        175.0,
        1.2,
        8.0,
    )
    w.cylinder(s, poppet + "/ValveHead", 0.0062, 0.0028, (0.0, 0.0184, 0.0), BRASS, "Y")
    w.cylinder(s, poppet + "/Stem", 0.0028, 0.009, (0.0, 0.0238, 0.0), BRASS, "Y")
    return s


def nozzle(w, name):
    s = w.stage(name)
    body = w.body(s, "/Asset", 0.088 if name == "gray" else 0.102, ccd=True)
    color = GRAY if name == "gray" else TEAL
    w.rounded_box(
        s,
        body + "/Grip",
        (0.038, 0.066, 0.038),
        (0.0, -0.006, 0.0),
        color,
        0.008,
        0.002,
    )
    for i in range(4):
        w.box(
            s,
            body + f"/GripBand{i}",
            (0.0388, 0.003, 0.024),
            (0.0, -0.028 + 0.012 * i, 0.0),
            BLACK,
        )
    w.cylinder(s, body + "/Shoulder", 0.016, 0.017, (0.0, 0.038, 0.0), BLACK, "Y")
    # Three plug sections give a lead-in taper, a real circumferential
    # capture groove and the protruding end that depresses the poppet.
    p = 0.056
    w.cylinder(s, body + "/PlugNeck", 0.0092, 0.005, (0.0, p + 0.001, 0.0), BLACK, "Y")
    w.cylinder(s, body + "/Groove", 0.0070, 0.006, (0.0, p + 0.0065, 0.0), BRASS, "Y")
    tapered(
        w, s, body + "/RetentionBevel", p + 0.0095, 0.007, p + 0.0113, 0.0102, BRASS
    )
    tapered(
        w, s, body + "/RetentionBead", p + 0.0113, 0.0102, p + 0.0120, 0.0102, BRASS
    )
    tapered(w, s, body + "/LeadIn", p + 0.0120, 0.0102, p + 0.0173, 0.0085, BLACK)
    tapered(w, s, body + "/Pilot", p + 0.0173, 0.0085, p + 0.0242, 0.0052, BLACK)
    w.cylinder(s, body + "/OutletBody", 0.014, 0.039, (0.0, -0.056, 0.005), color, "Y")
    w.cylinder(s, body + "/DownTube", 0.012, 0.018, (0.0, -0.074, -0.009), color)
    if name == "gray":
        w.cylinder(s, body + "/Rose", 0.024, 0.010, (0.0, -0.074, -0.016), BLACK)
        w.cylinder(
            s, body + "/Face", 0.020, 0.002, (0.0, -0.074, -0.021), STEEL, collide=False
        )
    else:
        w.rounded_box(
            s,
            body + "/FanHead",
            (0.062, 0.031, 0.013),
            (0.0, -0.078, -0.016),
            TEAL,
            0.010,
            0.002,
        )
        w.rounded_box(
            s,
            body + "/FanFace",
            (0.055, 0.020, 0.004),
            (0.0, -0.078, -0.022),
            BLACK,
            0.007,
            0.001,
            collide=False,
        )

    # Render-only water strands follow the rigid nozzle. Visibility is
    # driven by the valve and seated hose contact, never an action label.
    for i in range(7 if name == "blue" else 5):
        x = (i - 3) * 0.008 if name == "blue" else (i - 2) * 0.006
        path = body + f"/Water{i}"
        w.cylinder(s, path, 0.00082, 0.034, (x, -0.075, -0.044), WATER, collide=False)
        w.Geom.Imageable(s.GetPrimAtPath(path)).MakeInvisible()
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = []
    for name, kind, scene, p in (
        ("bench", "base", bench(w), (0.0, 0.0, 0.0)),
        ("coupling", "articulation", coupling(w), C.SOCKET),
        ("gray", "rigid", nozzle(w, "gray"), C.GRAY_READY),
        ("blue", "rigid", nozzle(w, "blue"), C.BLUE_READY),
    ):
        asset = {"name": name, "type": kind, "position": p, "file": w.save(scene)}
        if kind == "articulation":
            asset["joints"] = {
                "HoseCompliance": 0.0,
                "ReleaseTravel": 0.0,
                "FrontDogTravel": 0.0,
                "RearDogTravel": 0.0,
                "ValveTravel": C.POPPET_STROKE,
            }
        assets.append(asset)
    w.inventory()
    return {"assets": assets, "settle_actions": 20}


def show_flow(stage, env_path, flowing_gray, flowing_blue, impact_xy):
    from pxr import Gf, UsdGeom

    for name, count, flowing in (("gray", 5, flowing_gray), ("blue", 7, flowing_blue)):
        for i in range(count):
            path = f"{env_path}/{name}/Water{i}"
            attr = UsdGeom.Imageable(stage.GetPrimAtPath(path)).GetVisibilityAttr()
            desired = "inherited" if flowing else "invisible"
            if attr.Get() != desired:
                attr.Set(desired)

    impact = UsdGeom.Xform(stage.GetPrimAtPath(f"{env_path}/bench/SprayImpact"))
    visibility = impact.GetVisibilityAttr()
    desired = "inherited" if impact_xy is not None else "invisible"
    if visibility.Get() != desired:
        visibility.Set(desired)
    if impact_xy is not None:
        (translation,) = impact.GetOrderedXformOps()
        point = Gf.Vec3d(*impact_xy, C.TABLE + 0.0125)
        if translation.Get() != point:
            translation.Set(point)
