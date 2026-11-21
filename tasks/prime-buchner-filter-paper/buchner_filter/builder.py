"""Original procedural fixtures and native rigid contact geometry."""

import itertools
from math import cos, pi, sin, tan

import numpy as np

from . import config as C
from .usd_scene import Writer

GLASS = (0.60, 0.82, 0.82)
WHITE = (0.86, 0.89, 0.83)
NAVY = (0.13, 0.25, 0.27)
TEAL = (0.17, 0.52, 0.47)
BLACK = (0.11, 0.14, 0.14)
STEEL = (0.48, 0.56, 0.57)
WATER = (0.25, 0.65, 0.72)
BEAD = (0.74, 0.23, 0.055)


def ring(
    w,
    stage,
    path,
    center,
    radius,
    thick,
    height,
    color,
    count=24,
    spout=False,
    flare=0.0,
):
    """Leave the circular interior open and protect the body ridge with real colliders."""
    for i in range(count):
        theta = 2 * pi * i / count
        extra = 0.008 if spout and abs(theta - pi) < 2.2 * pi / count else 0.0
        r = radius + extra + flare / 2
        c, s = cos(theta), sin(theta)
        slope = float(np.arctan2(flare, height))
        normal = [c * cos(slope), s * cos(slope), -sin(slope)]
        upright = [c * sin(slope), s * sin(slope), cos(slope)]
        transform = np.column_stack((normal, [-s, c, 0.0], upright))
        part = w.box(
            stage,
            f"{path}{i}",
            (
                thick,
                (r + thick / 2 + flare / 2) * 2 * tan(pi / count),
                height / cos(slope),
            ),
            np.asarray(center) + [r * c, r * s, 0],
            color,
            transform,
        )
        part.MakeInvisible()


def revolve(w, s, path, center, profile, color, *, spout=False):
    xy = np.asarray(center)
    n = 48
    points = []
    for radius, z in profile:
        for i in range(n):
            angle = 2 * pi * i / n
            extension = 0.0
            if spout and abs(angle - pi) < pi / 7:
                extension = 0.008 * min(1, z / 0.04) * cos((angle - pi) * 3.5) ** 2
            r = radius + extension
            if radius == 0:
                r = 0.00001
            points.append((xy[0] + r * cos(angle), xy[1] + r * sin(angle), z))
    faces = [
        (j * n + i, j * n + (i + 1) % n, (j + 1) * n + (i + 1) % n, (j + 1) * n + i)
        for j in range(len(profile) - 1)
        for i in range(n)
    ]
    return mesh(w, s, path, points, faces, color)


def mesh(w, s, path, points, faces, color):
    part = w.Geom.Mesh.Define(s, path)
    part.CreatePointsAttr([w.Gf.Vec3f(*map(float, v)) for v in points])
    part.CreateFaceVertexCountsAttr([len(face) for face in faces])
    part.CreateFaceVertexIndicesAttr([int(i) for face in faces for i in face])
    part.CreateSubdivisionSchemeAttr("none")
    part.CreateDoubleSidedAttr(True)
    w._finish(s, part, color, False)
    return part


def material(w, stage, name, color, opacity, parts):
    from pxr import Sdf

    mat = w.Shade.Material.Define(stage, "/Asset/" + name)
    shader = w.Shade.Shader.Define(stage, "/Asset/" + name + "/Surface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        w.Gf.Vec3f(*color)
    )
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.25)
    shader.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.47)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    for part in parts:
        w.Shade.MaterialBindingAPI.Apply(part.GetPrim()).Bind(mat)


def tube(w, stage, path, points, radius, color):
    """Generate joined hose or wire segments oriented along their centerline."""
    from yam_arena.ik import matrix_to_quat_xyzw

    for i, (a, b) in enumerate(itertools.pairwise(points)):
        a, b = np.asarray(a), np.asarray(b)
        axis = b - a
        dist = np.linalg.norm(axis)
        axis /= dist
        u = np.cross(axis, [0, 0, 1])
        if np.linalg.norm(u) < 1e-5:
            u = np.cross(axis, [1, 0, 0])
        u /= np.linalg.norm(u)
        q = matrix_to_quat_xyzw(np.column_stack((u, np.cross(axis, u), axis)))
        part = w.cylinder(
            stage,
            f"{path}{i}",
            radius,
            dist + radius * 0.75,
            (a + b) / 2,
            color,
            collide=False,
        )
        part.AddOrientOp().Set(w.Gf.Quatf(float(q[3]), w.Gf.Vec3f(*map(float, q[:3]))))


def perforated_plate(w, s, location):
    """Triangulate the porcelain plate around actual round perforations."""
    from scipy.spatial import Delaunay

    fx, fy, z = location
    points = [
        (0.0378 * cos(a), 0.0378 * sin(a)) for a in np.linspace(0, 2 * pi, 81)[:-1]
    ]
    holes = np.array(
        [
            (i * 0.0095, j * 0.0095)
            for i in range(-3, 4)
            for j in range(-3, 4)
            if i * i + j * j <= 10
        ]
    )
    hr = 0.00180
    for dx, dy in holes:
        points.extend(
            (dx + hr * cos(a), dy + hr * sin(a))
            for a in np.linspace(0, 2 * pi, 15)[:-1]
        )
    points = np.asarray(points)
    triangles = Delaunay(points).simplices
    keep = (
        np.min(
            np.linalg.norm(
                points[triangles].mean(1)[:, None, :] - holes[None, :, :], axis=2
            ),
            axis=1,
        )
        >= hr * 0.96
    )
    surface = mesh(
        w,
        s,
        "/Asset/PerforatedCeramic",
        [(fx + x, fy + y, z) for x, y in points],
        triangles[keep],
        WHITE,
    )
    surface.CreateDoubleSidedAttr(True)
    # Paper-scale support colliders follow the porcelain webs between hole rows.
    for i in range(-4, 5):
        x = i * 0.0095 + 0.00475
        if abs(x) >= 0.0377:
            continue
        half = (0.0378**2 - (abs(x) + 0.0018) ** 2) ** 0.5
        if isinstance(half, complex):
            continue
        p = w.box(
            s,
            f"/Asset/PorcelainWeb{i + 5}",
            (0.0056, 2 * half, 0.0025),
            (fx + x, fy, z - 0.00128),
            WHITE,
        )
        p.MakeInvisible()


def fixture(w):
    s = w.stage("fixture")
    tx, ty = C.FUNNEL
    t = C.TABLE
    w.box(
        s,
        "/Asset/Worktop",
        (1.03, 1.22, 0.045),
        (0.34, 0, t - 0.0225),
        (0.53, 0.60, 0.58),
    )
    w.box(s, "/Asset/Floor", (6, 6, 0.02), (0, 0, -0.01), (0.29, 0.35, 0.36))
    w.box(
        s,
        "/Asset/Backdrop",
        (0.02, 1.10, 0.39),
        (0.771, 0, t + 0.195),
        (0.74, 0.79, 0.76),
    )
    px, py, _ = C.PAPER
    w.cylinder(s, "/Asset/FilterPaperRest", 0.027, 0.0295, (px, py, t + 0.01475), TEAL)
    w.cylinder(s, "/Asset/RestFoot", 0.038, 0.005, (px, py, t + 0.0025), TEAL)

    w.rounded_box(
        s,
        "/Asset/StandBase",
        (0.146, 0.135, 0.012),
        (tx + 0.05, ty, t + 0.006),
        NAVY,
        0.008,
        0.002,
    )
    post = (tx + 0.111, ty + 0.044)
    w.cylinder(s, "/Asset/StandPost", 0.007, 0.25, (*post, t + 0.133), STEEL)
    neck_at = t + 0.124
    tube(
        w,
        s,
        "/Asset/ClampSpine",
        [
            (*post, neck_at),
            (tx + 0.019, ty + 0.031, neck_at),
            (tx + 0.004, ty + 0.023, neck_at),
        ],
        0.006,
        STEEL,
    )
    w.rounded_box(
        s,
        "/Asset/ClampBoss",
        (0.024, 0.024, 0.027),
        (*post, neck_at),
        NAVY,
        0.004,
        0.002,
    )
    w.cylinder(
        s,
        "/Asset/ClampKnob",
        0.010,
        0.014,
        (post[0], post[1] + 0.016, neck_at),
        (0.68, 0.24, 0.10),
        "Y",
    )
    collar = revolve(
        w,
        s,
        "/Asset/NeckClamp",
        (tx, ty),
        [
            (0.016, neck_at - 0.007),
            (0.028, neck_at - 0.007),
            (0.028, neck_at + 0.007),
            (0.016, neck_at + 0.007),
            (0.016, neck_at - 0.007),
        ],
        NAVY,
    )
    collar.CreateDoubleSidedAttr(True)

    flask_profile = [
        (0, t + 0.014),
        (0.039, t + 0.014),
        (0.050, t + 0.022),
        (0.049, t + 0.042),
        (0.021, t + 0.107),
        (0.016, t + 0.117),
        (0.016, t + 0.148),
        (0.023, t + 0.151),
        (0.023, t + 0.154),
        (0.016, t + 0.155),
        (0.013, t + 0.147),
        (0.013, t + 0.117),
        (0.018, t + 0.104),
        (0.045, t + 0.039),
        (0.046, t + 0.024),
        (0.037, t + 0.018),
        (0, t + 0.018),
    ]
    glass = revolve(w, s, "/Asset/SidearmFlask", (tx, ty), flask_profile, GLASS)
    glass.CreateDoubleSidedAttr(True)
    material(w, s, "Glass", GLASS, 0.33, [glass])
    support = w.cylinder(
        s, "/Asset/FlaskCollider", 0.040, 0.072, (tx, ty, t + 0.052), GLASS
    )
    support.MakeInvisible()
    w.cylinder(
        s,
        "/Asset/SideOutlet",
        0.006,
        0.028,
        (tx + 0.021, ty, t + 0.135),
        GLASS,
        "X",
        collide=False,
    )
    tube(
        w,
        s,
        "/Asset/VacuumHose",
        [
            (tx + 0.033, ty, t + 0.135),
            (tx + 0.067, ty + 0.005, t + 0.128),
            (tx + 0.10, ty - 0.027, t + 0.110),
            (tx + 0.134, ty - 0.052, t + 0.056),
            (tx + 0.214, ty - 0.070, t + 0.034),
            (tx + 0.253, ty - 0.072, t + 0.034),
        ],
        0.008,
        BLACK,
    )
    w.rounded_box(
        s,
        "/Asset/VacuumRegulator",
        (0.124, 0.133, 0.088),
        (tx + 0.245, ty - 0.105, t + 0.044),
        NAVY,
        0.010,
        0.005,
    )
    w.cylinder(
        s, "/Asset/Gauge", 0.020, 0.007, (tx + 0.214, ty - 0.113, t + 0.092), WHITE
    )
    w.box(
        s,
        "/Asset/GaugeNeedle",
        (0.022, 0.0018, 0.0008),
        (tx + 0.215, ty - 0.113, t + 0.096),
        BLACK,
        collide=False,
    )
    w.cylinder(
        s,
        "/Asset/VacuumControl",
        0.013,
        0.010,
        (tx + 0.268, ty - 0.105, t + 0.092),
        TEAL,
    )

    w.cylinder(s, "/Asset/BlackRubberAdapter", 0.024, 0.015, (tx, ty, t + 0.152), BLACK)
    w.cylinder(s, "/Asset/FunnelSpout", 0.010, 0.020, (tx, ty, t + 0.157), WHITE)
    pl, rim = C.PLATE, C.RIM
    profile = [
        (0.010, pl - 0.019),
        (0.020, pl - 0.016),
        (0.041, pl - 0.010),
        (0.043, pl - 0.005),
        (0.051, rim),
        (0.050, rim + 0.0015),
        (0.0465, rim + 0.0015),
        (C.FUNNEL_MOUTH, rim),
        (C.FUNNEL_INNER, pl),
    ]
    revolve(
        w, s, "/Asset/PorcelainFunnel", (tx, ty), profile, WHITE
    ).CreateDoubleSidedAttr(True)
    ring(
        w,
        s,
        "/Asset/FunnelWall",
        (tx, ty, (pl + rim) / 2),
        0.0405,
        0.0050,
        rim - pl,
        WHITE,
        count=32,
        flare=C.FUNNEL_MOUTH - C.FUNNEL_INNER,
    )
    perforated_plate(w, s, (tx, ty, pl))

    for index, level in enumerate((0.023, 0.031, 0.040, 0.050)):
        w.box(
            s,
            f"/Asset/FlaskMark{index}",
            (0.0008, 0.014, 0.001),
            (tx + 0.049 - max(0, level - 0.040) * 0.35, ty, t + level),
            TEAL,
            collide=False,
        )
    w.cylinder(s, "/Asset/Filtrate", 0.039, 1, (tx, ty, t + 0.02), WATER, collide=False)
    w.cylinder(s, "/Asset/FunnelPool", 0.0365, 1, (tx, ty, pl), WATER, collide=False)
    for i in range(36):
        a = i * 2.39996323
        r = 0.032 * ((i + 0.35) / 36) ** 0.5
        w.cylinder(
            s,
            f"/Asset/Bead{i}",
            0.00118,
            0.00155,
            (tx + r * cos(a), ty + r * sin(a), pl + 0.00165),
            BEAD,
            collide=False,
        ).MakeInvisible()
        wd = r * 1.2
        w.cylinder(
            s,
            f"/Asset/Escaped{i}",
            0.00118,
            0.00155,
            (tx + wd * cos(a), ty + wd * sin(a), t + 0.020),
            BEAD,
            collide=False,
        ).MakeInvisible()
    for name in ("Filtrate", "FunnelPool"):
        w.Geom.Imageable(s.GetPrimAtPath("/Asset/" + name)).MakeInvisible()
    for i in range(15):
        mesh(w, s, f"/Asset/Trace{i}", [], [], WATER)
    return s


def paper(w):
    s = w.stage("paper")
    root = w.body(s, "/Asset", 0.0018, ccd=True)
    w.cylinder(s, root + "/FilterDisk", C.PAPER_RADIUS, C.PAPER_THICK, (0, 0, 0), WHITE)
    return s


def beaker(w, v):
    s = w.stage(v.name)
    root = w.body(s, "/Asset", v.mass, ccd=True)
    r, h = v.radius, v.height
    w.cylinder(
        s, root + "/Foot", r - 0.001, 0.005, (0, 0, 0.0025), GLASS
    ).MakeInvisible()
    ring(w, s, root + "/Wall", (0, 0, h / 2), r, 0.0036, h, GLASS, spout=True)
    wall = revolve(
        w,
        s,
        root + "/Jug",
        (0, 0),
        [
            (0, 0),
            (r - 0.001, 0),
            (r + 0.002, 0.005),
            (r + 0.002, h),
            (r - 0.002, h + 0.001),
            (r - 0.003, h - 0.001),
            (r - 0.003, 0.005),
            (0, 0.005),
        ],
        GLASS,
        spout=True,
    )
    wall.CreateDoubleSidedAttr(True)
    material(w, s, "JugPlastic", GLASS, 0.40, [wall])
    sign = np.sign(v.handle)
    # The upper and lower rails meet an inclined molded D-handle grip.
    bar = w.cylinder(s, root + "/HandleGrip", 0.010, h * 0.71, v.grip, NAVY)
    beta = float(np.arctan2(0.43, 0.90))
    bar.AddOrientOp().Set(w.Gf.Quatf(cos(beta / 2), w.Gf.Vec3f(0, -sin(beta / 2), 0)))
    for index, n in enumerate((0.24, 0.74)):
        w.rounded_box(
            s,
            root + f"/Joint{index}",
            (0.017, abs(v.handle) - r + 0.009, 0.011),
            (-(h * n - v.grip[2]) * tan(beta), (v.handle + sign * r) / 2, h * n),
            NAVY,
            0.003,
            0.0015,
        )
    for i in range(1, 5):
        w.box(
            s,
            root + f"/LevelMark{i}",
            (0.001, 0.011, 0.001),
            (r + 0.0025, 0, h * (0.11 + 0.13 * i)),
            NAVY,
            collide=False,
        )
    mesh(w, s, root + "/WaterSurface", [], [], v.color)
    if v.name == C.SUSPENSION.name:
        mesh(w, s, root + "/SuspendedBeads", [], [], BEAD)
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [
        {"name": "fixture", "type": "base", "file": w.save(fixture(w))},
        {
            "name": "paper",
            "type": "rigid",
            "file": w.save(paper(w)),
            "position": C.PAPER.tolist(),
        },
    ]
    for v in C.VESSELS:
        assets.append(
            {
                "name": v.name,
                "type": "rigid",
                "file": w.save(beaker(w, v)),
                "position": v.position.tolist(),
            }
        )
    w.inventory()
    return {"assets": assets, "settle_actions": 18}
