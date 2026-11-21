"""Native locked input, compliant reduction and bidirectional proof handle."""

import math

import numpy as np

from . import config as C
from .usd_scene import Writer

GRAPHITE = (0.10, 0.13, 0.16)
STEEL = (0.57, 0.63, 0.65)
ALUMINUM = (0.73, 0.75, 0.76)
GOLD = (0.83, 0.52, 0.12)
BLUE = (0.10, 0.42, 0.65)
WHITE = (0.87, 0.90, 0.85)
GREEN = (0.30, 0.86, 0.53)
GREEN_DIM = (0.13, 0.23, 0.20)
RED = (0.87, 0.29, 0.15)


def joint(w, s, name, parent, child, anchor, axis, limits, spring, rotational=False):
    parent_path, parent_origin = parent
    child_path, child_origin = child
    api = w.Phy.RevoluteJoint if rotational else w.Phy.PrismaticJoint
    j = api.Define(s, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([parent_path])
    j.CreateBody1Rel().SetTargets([child_path])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, np.subtract(anchor, parent_origin))))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(*map(float, np.subtract(anchor, child_origin))))
    j.CreateAxisAttr(axis)
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    j.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(j.GetPrim(), "angular" if rotational else "linear")
    drive.CreateTypeAttr("force")
    stiffness, damping, target, cap = spring
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateTargetPositionAttr(target)
    drive.CreateMaxForceAttr(cap)


def bench(w):
    s = w.stage("bench")
    w.box(
        s,
        "/Asset/Bench",
        (0.92, 1.19, 0.044),
        (0.35, 0, C.TABLE_Z - 0.022),
        (0.47, 0.50, 0.45),
    )
    w.box(s, "/Asset/Floor", (4, 4, 0.020), (0, 0, -0.010), (0.31, 0.35, 0.34))
    return s


def fixture(w):
    s = w.stage("fixture")
    center = (C.CENTER_X, C.CENTER_Y)
    base = w.body(s, "/Asset/Frame", 1.5, (0, 0, 0), fixed=True, articulation=True)
    shell = base + "/Housing"
    w.rounded_box(
        s,
        shell,
        (0.178, 0.175, 0.044),
        (*center, C.TABLE_Z + 0.030),
        GRAPHITE,
        0.012,
        0.002,
    )
    w.rounded_box(
        s,
        base + "/Plate",
        (0.258, 0.335, 0.009),
        (*center, C.TABLE_Z + 0.005),
        STEEL,
        0.009,
        0.001,
    )
    for dx in (-0.071, 0.071):
        for dy in (-0.067, 0.067):
            w.cylinder(
                s,
                base + f"/Bolt_{dx}_{dy}",
                0.005,
                0.004,
                (center[0] + dx, center[1] + dy, C.TABLE_Z + 0.054),
                ALUMINUM,
                collide=False,
            )

    input_origin = (*center, C.INPUT_Z)
    input_body = w.body(s, "/Asset/InputGear", 0.16, input_origin)
    w.cylinder(
        s, input_body + "/InputDisk", 0.060, 0.026, (0, 0, 0), ALUMINUM, collide=False
    )
    w.cylinder(
        s,
        input_body + "/DarkRing",
        0.063,
        0.010,
        (0, 0, 0.010),
        GRAPHITE,
        collide=False,
    )
    # The yoke physically encloses the end faces of this reaction key.
    w.box(
        s,
        input_body + "/ReactionKey",
        (0.032, 0.039, 0.020),
        (0, 0.084, 0.021),
        ALUMINUM,
    )
    w.box(
        s,
        input_body + "/InputArm",
        (0.019, 0.040, 0.012),
        (0, 0.056, 0.018),
        STEEL,
        collide=False,
    )
    joint(
        w,
        s,
        "InputAngle",
        (base, (0, 0, 0)),
        (input_body, input_origin),
        input_origin,
        "Z",
        (-12.0, 12.0),
        (0.012, 0.012, 0, 1.0),
        rotational=True,
    )

    output_origin = (*center, C.OUTPUT_Z)
    output = w.body(s, "/Asset/OutputFlange", 0.18, output_origin)
    w.cylinder(s, output + "/Face", 0.056, 0.021, (0, 0, 0), BLUE, collide=False)
    for index in range(8):
        phi = index * math.tau / 8
        w.cylinder(
            s,
            output + f"/Screw{index}",
            0.004,
            0.002,
            (0.045 * math.cos(phi), 0.045 * math.sin(phi), 0.011),
            ALUMINUM,
            collide=False,
        )
    w.box(
        s,
        output + "/TorqueArm",
        (0.036, 0.088, 0.016),
        (0, -0.071, 0),
        STEEL,
        collide=False,
    )
    w.rounded_box(
        s,
        output + "/ForceTrack",
        (0.084, 0.051, 0.019),
        (0, -C.LOAD_RADIUS, 0.009),
        GRAPHITE,
        0.004,
        0.001,
        collide=False,
    )
    for index, x in enumerate((-0.0116, 0.0, 0.0116)):
        w.box(
            s,
            output + f"/Witness{index}",
            (0.0019, 0.010, 0.001),
            (x, -C.LOAD_RADIUS - 0.019, 0.019),
            WHITE,
            collide=False,
        )
    # USD angular drives are parameterized per degree.
    joint(
        w,
        s,
        "Torsion",
        (input_body, input_origin),
        (output, output_origin),
        output_origin,
        "Z",
        (-7.0, 7.0),
        (1.75, 0.045, 0, 3.0),
        rotational=True,
    )

    slide_origin = (C.CENTER_X, C.CENTER_Y - C.LOAD_RADIUS, C.LOAD_Z)
    slide = w.body(s, "/Asset/LoadHandle", 0.045, slide_origin, ccd=True)
    w.rounded_box(
        s, slide + "/Grip", (0.041, 0.026, 0.036), (0, 0, 0), BLUE, 0.004, 0.002
    )
    w.box(
        s, slide + "/Stem", (0.018, 0.018, 0.024), (0, 0, -0.022), STEEL, collide=False
    )
    for idx, dx in enumerate((-0.013, -0.007, 0.0, 0.007, 0.013)):
        w.box(
            s,
            slide + f"/GripStripe{idx}",
            (0.001, 0.020, 0.001),
            (dx, 0, 0.018),
            WHITE,
            collide=False,
        )
    w.box(
        s,
        slide + "/Pointer",
        (0.002, 0.018, 0.008),
        (0, -0.017, -0.015),
        GOLD,
        collide=False,
    )
    joint(
        w,
        s,
        "ProofSlide",
        (output, output_origin),
        (slide, slide_origin),
        slide_origin,
        "X",
        (-C.LOAD_TRAVEL, C.LOAD_TRAVEL),
        (C.SPRING_N_M, 4.0, 0, 15.0),
    )

    lock_origin = (C.CENTER_X, C.LOCK_Y, C.LOCK_Z)
    lock = w.body(s, "/Asset/InputLock", 0.065, lock_origin, ccd=True)
    w.rounded_box(
        s, lock + "/Pad", (0.053, 0.049, 0.016), (0, 0, 0), GOLD, 0.006, 0.001
    )
    w.box(
        s,
        lock + "/Bridge",
        (0.049, 0.056, 0.009),
        (0, -0.020, -0.013),
        GOLD,
        collide=False,
    )
    for index, dx in enumerate((-0.02065, 0.02065)):
        w.box(
            s,
            lock + f"/YokeJaw{index}",
            (0.009, 0.028, 0.024),
            (dx, -0.038, -0.030),
            ALUMINUM,
        )
    w.cylinder(s, lock + "/LockStem", 0.006, 0.048, (0, 0, -0.029), GOLD, collide=False)
    w.cylinder(
        s,
        base + "/LockGuide",
        0.011,
        0.049,
        (C.CENTER_X, C.LOCK_Y, 0.823),
        GRAPHITE,
        collide=False,
    )
    joint(
        w,
        s,
        "LockStroke",
        (base, (0, 0, 0)),
        (lock, lock_origin),
        lock_origin,
        "Z",
        (-C.LOCK_TRAVEL, 0.0),
        (280, 2.7, 0.0025, 12.0),
    )
    # Omit weight on the guided movable parts. Their diagonal inertia and joint
    # drives remain native. The load and lock springs apply physical resistance.
    for body in (slide, lock):
        w.Px.PhysxRigidBodyAPI.Get(s, body).CreateDisableGravityAttr(True)
    return s


SEGMENTS = (
    (0.010, 0, 0.0023, 0.011),
    (0.005, -0.006, 0.010, 0.0023),
    (-0.005, -0.006, 0.010, 0.0023),
    (-0.010, 0, 0.0023, 0.011),
    (-0.005, 0.006, 0.010, 0.0023),
    (0.005, 0.006, 0.010, 0.0023),
    (0, 0, 0.0023, 0.011),
)
DIGITS = (
    "012345",
    "12",
    "01643",
    "01623",
    "5612",
    "05623",
    "056234",
    "012",
    "0123456",
    "012356",
)


def meter(w):
    s = w.stage("meter")
    height = C.TABLE_Z + 0.035
    w.rounded_box(
        s,
        "/Asset/Case",
        (0.138, 0.212, 0.056),
        (0.623, 0.032, C.TABLE_Z + 0.029),
        GRAPHITE,
        0.011,
        0.002,
        collide=False,
    )
    z = height + 0.024
    for channel, cy in enumerate((0.096, 0.029, -0.038)):
        w.box(
            s,
            f"/Asset/Screen{channel}",
            (0.050, 0.057, 0.001),
            (0.620, cy, z),
            GREEN_DIM,
            collide=False,
        )
        accent = (RED, BLUE, GREEN)[channel]
        w.box(
            s,
            f"/Asset/Mark{channel}",
            (0.003, 0.018, 0.002),
            (0.585, cy, z),
            accent,
            collide=False,
        )
        if channel == 1:
            w.box(
                s,
                "/Asset/PlusVertical",
                (0.014, 0.003, 0.002),
                (0.585, cy, z),
                accent,
                collide=False,
            )
        if channel == 2:
            for idx, tilt in enumerate((-0.55, 0.55)):
                diagonal = np.array(
                    [
                        [math.cos(tilt), -math.sin(tilt), 0],
                        [math.sin(tilt), math.cos(tilt), 0],
                        [0, 0, 1],
                    ]
                )
                w.box(
                    s,
                    f"/Asset/DeltaSide{idx}",
                    (0.016, 0.002, 0.002),
                    (0.592, cy + (idx - 0.5) * 0.008, z),
                    GREEN,
                    rotation=diagonal,
                    collide=False,
                )
        for position, digit_y in enumerate((cy + 0.010, cy - 0.010)):
            for index, (dx, dy, sx, sy) in enumerate(SEGMENTS):
                path = f"/Asset/LCD_{channel}_{position}_{index}"
                w.box(
                    s,
                    path,
                    (sx, sy, 0.001),
                    (0.620 + dx, digit_y + dy, z + 0.001),
                    GREEN,
                    collide=False,
                )
                w.Geom.Imageable(s.GetPrimAtPath(path)).MakeInvisible()
    return s


def show_display(stage, env_path, values):
    from pxr import UsdGeom

    for channel, value in enumerate(values):
        digits = "--" if value is None else f"{min(99, round(abs(value))):02}"
        for position, digit in enumerate(digits):
            active = "6" if digit == "-" else DIGITS[int(digit)]
            for index in range(7):
                path = f"{env_path}/meter/LCD_{channel}_{position}_{index}"
                prim = stage.GetPrimAtPath(path)
                if prim:
                    attr = UsdGeom.Imageable(prim).GetVisibilityAttr()
                    visible = "inherited" if str(index) in active else "invisible"
                    if attr.Get() != visible:
                        attr.Set(visible)


def create_assets(directory):
    w = Writer(directory)
    return {
        "settle_actions": 16,
        "assets": [
            {"name": "bench", "type": "base", "file": w.save(bench(w))},
            {
                "name": "fixture",
                "type": "articulation",
                "file": w.save(fixture(w)),
                "joints": {
                    "InputAngle": 0.0,
                    "Torsion": 0.0,
                    "ProofSlide": 0.0,
                    "LockStroke": 0.0,
                },
            },
            {"name": "meter", "type": "base", "file": w.save(meter(w))},
        ],
    }
