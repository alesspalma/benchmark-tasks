"""Glazed cistern, native trip and flapper hinges, and a loose spring chain clip."""

from math import cos, pi, sin

from . import config as C
from .usd_scene import Writer

WHITE = (0.83, 0.87, 0.86)
GLAZE = (0.93, 0.94, 0.92)
STEEL = (0.38, 0.45, 0.49)
BRASS = (0.52, 0.34, 0.12)
BLUE = (0.085, 0.25, 0.31)
RUBBER = (0.64, 0.135, 0.08)
WATER = (0.29, 0.70, 0.73)
CLIP = (0.64, 0.46, 0.16)


def join(w, s, base, member, name, position, axis, lower, upper):
    j = w.Phy.RevoluteJoint.Define(s, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([base])
    j.CreateBody1Rel().SetTargets([member])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, position)))
    j.CreateAxisAttr(axis)
    j.CreateLowerLimitAttr(lower)
    j.CreateUpperLimitAttr(upper)


def marker_text(w, s, path, position):
    x, y, z = position
    for i, shift in enumerate((-0.009, 0.002, 0.013)):
        w.box(
            s,
            f"{path}{i}",
            (0.0009, 0.010 + 0.004 * i, 0.001),
            (x, y + shift, z),
            BLUE,
            collide=False,
        )


def tank(w):
    s = w.stage("flapper_cistern")
    fixed = w.body(s, "/Asset/CisternBase", 17.0, fixed=True, articulation=True)
    cx = (C.WALL_FRONT + C.WALL_BACK) / 2
    height = C.RIM - C.FLOOR
    for label, x in [("Front", C.WALL_FRONT), ("Rear", C.WALL_BACK)]:
        w.rounded_box(
            s,
            f"{fixed}/{label}Porcelain",
            (0.016, 0.399, height),
            (x, 0, C.FLOOR + height / 2),
            WHITE,
            0.007,
            0.003,
        )
        w.rounded_box(
            s,
            f"{fixed}/{label}Rim",
            (0.027, 0.416, 0.015),
            (x, 0, C.RIM - 0.005),
            GLAZE,
            0.009,
            0.002,
        )
    for label, y in [("Left", C.WALL_LEFT), ("Right", -C.WALL_LEFT)]:
        w.rounded_box(
            s,
            f"{fixed}/{label}Porcelain",
            (0.217, 0.015, height),
            (cx, y, C.FLOOR + height / 2),
            WHITE,
            0.007,
            0.003,
        )
        w.rounded_box(
            s,
            f"{fixed}/{label}Rim",
            (0.232, 0.023, 0.015),
            (cx, y, C.RIM - 0.005),
            GLAZE,
            0.010,
            0.002,
        )
    w.rounded_box(
        s,
        fixed + "/CisternBottom",
        (0.223, 0.413, 0.016),
        (cx, 0, C.FLOOR - 0.006),
        WHITE,
        0.010,
        0.003,
    )
    for i, y in enumerate((-0.198, 0.198)):
        w.rounded_box(
            s,
            f"{fixed}/Support{i}",
            (0.244, 0.022, 0.062),
            (cx, y, C.TABLE + 0.031),
            GLAZE,
            0.010,
            0.004,
        )
    # Sectioned catch basin sits below the stock 2-inch tank outlet.
    w.rounded_box(
        s,
        fixed + "/CatchBasin",
        (0.283, 0.407, 0.013),
        (cx, 0, C.TABLE + 0.007),
        BLUE,
        0.020,
        0.004,
    )
    for i, x in enumerate((cx - 0.137, cx + 0.137)):
        w.rounded_box(
            s,
            f"{fixed}/CatchEnd{i}",
            (0.014, 0.396, 0.056),
            (x, 0, C.TABLE + 0.038),
            BLUE,
            0.006,
            0.002,
        )
    for i, y in enumerate((-0.195, 0.195)):
        w.rounded_box(
            s,
            f"{fixed}/CatchSide{i}",
            (0.279, 0.013, 0.056),
            (cx, y, C.TABLE + 0.038),
            BLUE,
            0.006,
            0.002,
        )
    # Overflow standpipe and fill valve distinguish the gravity cistern.
    w.cylinder(
        s, fixed + "/OverflowTube", 0.016, 0.148, (0.522, -0.030, C.FLOOR + 0.077), BLUE
    )
    w.cylinder(
        s, fixed + "/OverflowLip", 0.019, 0.007, (0.522, -0.030, C.FLOOR + 0.153), GLAZE
    )
    w.cylinder(
        s,
        fixed + "/OverflowMouth",
        0.013,
        0.001,
        (0.522, -0.030, C.FLOOR + 0.157),
        BLUE,
    )
    w.cylinder(
        s, fixed + "/FillValve", 0.016, 0.170, (0.514, 0.150, C.FLOOR + 0.085), STEEL
    )
    w.cylinder(
        s, fixed + "/FillFloat", 0.027, 0.059, (0.514, 0.150, C.FLOOR + 0.114), BLUE
    )
    w.cylinder(
        s, fixed + "/FillCap", 0.020, 0.017, (0.514, 0.150, C.FLOOR + 0.178), BRASS
    )
    w.cylinder(s, fixed + "/OutletSeat", 0.034, 0.012, C.OUTLET + [0, 0, -0.005], BLUE)
    w.cylinder(s, fixed + "/ValveStem", 0.028, 0.052, C.OUTLET + [0, 0, -0.035], BLUE)
    for i, y in enumerate((-0.045, -0.007)):
        w.cylinder(
            s, f"{fixed}/HingePost{i}", 0.006, 0.024, (0.487, y, 0.827), STEEL, "Z"
        )
    shelf = C.CLIP_HOME + [C.CLIP_GRIP[0], 0, -0.0025]
    w.box(s, fixed + "/ClipSill", (0.022, 0.027, 0.008), shelf, GLAZE)
    w.box(
        s,
        fixed + "/SillMount",
        (0.063, 0.019, 0.014),
        shelf + [0.040, -0.004, -0.011],
        WHITE,
    )
    w.box(
        s,
        fixed + "/SillFoot",
        (0.022, 0.027, 0.025),
        shelf + [0.077, -0.004, -0.0245],
        WHITE,
    )
    w.cylinder(
        s,
        fixed + "/HandleEscutcheon",
        0.025,
        0.016,
        C.LEVER + [-0.008, 0, 0],
        STEEL,
        "X",
        collide=False,
    )

    trip = w.body(s, "/Asset/TripLever", 0.14, position=C.LEVER, ccd=True)
    w.cylinder(
        s, trip + "/Spindle", 0.006, 0.127, (-0.002, 0, 0), STEEL, "X", collide=False
    )
    w.rounded_box(
        s,
        trip + "/Handle",
        (0.022, 0.087, 0.016),
        C.HANDLE + [0, -0.021, 0],
        STEEL,
        0.006,
        0.0025,
    )
    w.rounded_box(
        s,
        trip + "/HandleBand",
        (0.025, 0.025, 0.020),
        C.HANDLE,
        (0.19, 0.29, 0.34),
        0.008,
        0.0025,
    )
    w.box(s, trip + "/InnerBend", (0.078, 0.010, 0.007), (0.060, 0, 0), BRASS)
    for i, pin in enumerate(C.PINS):
        x, y, z = pin
        # Open eyelet, with a short transverse load-bearing rail exposed to the hook.
        w.cylinder(s, f"{trip}/EyePin{i}", 0.00165, 0.013, pin, STEEL, "Y")
        w.box(
            s,
            f"{trip}/EyeLeg{i}a",
            (0.012, 0.0025, 0.004),
            (x - 0.005, y - 0.006, z),
            BRASS,
        )
        w.box(
            s,
            f"{trip}/EyeLeg{i}b",
            (0.012, 0.0025, 0.004),
            (x - 0.005, y + 0.006, z),
            BRASS,
        )
        w.box(s, f"{trip}/EyeBack{i}", (0.003, 0.016, 0.004), (x - 0.010, y, z), BRASS)
    span = C.PINS[-1, 1]
    w.box(
        s,
        trip + "/LeverArm",
        (0.010, -span + 0.011, 0.008),
        (0.070, span / 2, 0),
        BRASS,
    )
    for i, pin in enumerate(C.PINS):
        x, y, z = pin
        w.box(
            s,
            f"{trip}/EyeWeb{i}",
            (x - 0.062, 0.012, 0.006),
            ((x + 0.062) / 2 - 0.004, y, z),
            BRASS,
        )
    marker_text(w, s, trip + "/TripNotch", (0.044, -0.027, 0.0045))

    flapper = w.body(s, "/Asset/Flapper", 0.036, position=C.FLAPPER, ccd=True)
    w.cylinder(s, flapper + "/RubberDisc", 0.034, 0.008, (-0.045, 0, 0), RUBBER)
    w.cylinder(
        s, flapper + "/Cup", 0.018, 0.010, (-0.045, 0, -0.005), RUBBER, collide=False
    )
    w.box(s, flapper + "/FlexBridge", (0.039, 0.030, 0.004), (-0.018, 0, 0), RUBBER)
    w.box(s, flapper + "/LeftEar", (0.016, 0.009, 0.006), (-0.004, 0.019, 0), RUBBER)
    w.box(s, flapper + "/RightEar", (0.016, 0.009, 0.006), (-0.004, -0.019, 0), RUBBER)
    w.cylinder(
        s,
        flapper + "/ChainEye",
        0.0035,
        0.005,
        C.FLAPPER_ANCHOR,
        STEEL,
        "Y",
        collide=False,
    )
    join(w, s, fixed, trip, "TripPivot", C.LEVER, "X", -32.0, 10.0)
    join(w, s, fixed, flapper, "FlapperPivot", C.FLAPPER, "Y", 0.0, 77.0)
    return s


def clip(w):
    s = w.stage("flapper_clip")
    b = w.body(s, "/Asset/SpringClip", 0.028, ccd=True)
    w.box(s, b + "/SqueezeTab", (0.026, 0.021, 0.010), C.CLIP_GRIP, CLIP)
    w.box(s, b + "/ThumbPlate", (0.007, 0.013, 0.026), (0.013, 0, 0.012), CLIP)
    w.box(s, b + "/ClipFoot", (0.012, 0.018, 0.010), (0.015, 0, 0.004), STEEL)
    # Small C-wire housing. The open spring tongue is represented by finite closure forces.
    for i in range(13):
        angle = -0.12 + i * pi / 12
        w.cylinder(
            s,
            f"{b}/Wire{i}",
            0.00175,
            0.004,
            (0.0052 * cos(angle), 0, 0.0052 * sin(angle)),
            STEEL,
            "Y",
            collide=False,
        )
    for i, x in enumerate((-0.006, 0.006)):
        w.box(
            s,
            f"{b}/Lead{i}",
            (0.0034, 0.007, 0.008),
            (x, 0, -0.0015),
            STEEL,
            collide=False,
        )
    w.box(
        s,
        b + "/HookBridge",
        (0.017, 0.006, 0.003),
        (0.006, 0, 0.0048),
        STEEL,
        collide=False,
    )
    w.cylinder(
        s, b + "/ChainEye", 0.0035, 0.005, C.CLIP_CHAIN, STEEL, "Y", collide=False
    )
    return s


def chain_links(w, s):
    points, faces = [], []
    long_steps, wire_sides = 16, 6
    width = 0.0025
    length = C.CHAIN_LENGTH / (2 * C.CHAIN_LINK_COUNT) + 0.0011
    for i in range(long_steps):
        t = 2 * pi * i / long_steps
        for j in range(wire_sides):
            phase = 2 * pi * j / wire_sides
            radius = 0.00059 * cos(phase)
            points.append(
                w.Gf.Vec3f(
                    (width + radius) * cos(t),
                    0.00059 * sin(phase),
                    (length + radius) * sin(t),
                )
            )
            start = i * wire_sides
            neighbor = ((i + 1) % long_steps) * wire_sides
            next_side = (j + 1) % wire_sides
            faces.extend(
                (start + j, neighbor + j, neighbor + next_side, start + next_side)
            )
    w.Geom.Xform.Define(s, "/Asset/Chain")
    for index in range(C.CHAIN_LINK_COUNT):
        mesh = w.Geom.Mesh.Define(s, f"/Asset/Chain/Link{index:02}")
        mesh.AddTranslateOp().Set(w.Gf.Vec3d(*map(float, C.FLAPPER)))
        mesh.AddOrientOp(precision=w.Geom.XformOp.PrecisionDouble).Set(w.Gf.Quatd(1.0))
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([4] * (long_steps * wire_sides))
        mesh.CreateFaceVertexIndicesAttr(faces)
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([w.Gf.Vec3f(*STEEL)])


def visuals(w):
    s = w.stage("flapper_water_visuals")
    cx = (C.WALL_FRONT + C.WALL_BACK) / 2
    for name, position, size in (
        ("WaterSurface", (cx, 0, C.FLOOR + 0.09), (*C.WATER_FOOTPRINT, 0.0018)),
        ("CatchWater", (cx, 0, C.TABLE + 0.011), (*C.CATCH_FOOTPRINT, 0.0018)),
    ):
        root = w.Geom.Xform.Define(s, f"/Asset/{name}")
        root.AddTranslateOp().Set(w.Gf.Vec3d(*position))
        w.rounded_box(
            s,
            f"/Asset/{name}/Surface",
            size,
            (0, 0, 0),
            WATER,
            0.015,
            0.0003,
            collide=False,
        )
    root = w.Geom.Xform.Define(s, "/Asset/Outflow")
    root.AddTranslateOp().Set(w.Gf.Vec3d(*map(float, C.OUTLET + [0, 0, -0.057])))
    root.AddScaleOp().Set(w.Gf.Vec3d(0.01, 0.01, 0.01))
    w.cylinder(s, "/Asset/Outflow/Jet", 0.015, 0.05, (0, 0, 0), WATER, collide=False)
    chain_links(w, s)
    return s


def worktop(w):
    s = w.stage("plumbing_bench")
    w.box(
        s,
        "/Asset/Counter",
        (1.22, 1.12, 0.043),
        (0.45, 0, C.TABLE - 0.0215),
        (0.62, 0.65, 0.62),
    )
    w.box(s, "/Asset/Floor", (4, 4, 0.02), (0, 0, -0.01), (0.50, 0.54, 0.54))
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [
        {"name": "bench", "type": "base", "file": w.save(worktop(w))},
        {
            "name": "tank",
            "type": "articulation",
            "file": w.save(tank(w)),
            "joints": {"TripPivot": 0.0, "FlapperPivot": 0.0},
        },
        {
            "name": "clip",
            "type": "rigid",
            "file": w.save(clip(w)),
            "position": C.CLIP_HOME,
        },
        {"name": "water", "type": "base", "file": w.save(visuals(w))},
    ]
    w.inventory()
    return {"assets": assets, "settle_actions": 12}
