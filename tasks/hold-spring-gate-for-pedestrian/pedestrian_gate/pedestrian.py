"""Native contact geometry and fixed voluntary gate passage of the walker."""

from math import cos, pi, sin

import numpy as np

from . import config as C

JACKET = (0.64, 0.30, 0.15)
DARK_JACKET = (0.43, 0.17, 0.09)
SHOES = (0.11, 0.094, 0.076)
TROUSERS = (0.095, 0.15, 0.27)
SKIN = (0.71, 0.48, 0.33)
HAIR = (0.115, 0.074, 0.059)

# The walker faces -X. Each moving part has an anatomical pivot and a local
# cross-section. The units are metres measured relative to the walking surface.
PARTS = {
    "MotionTorso": ((0.0, 0.0, 0.944), 0.15),
    "MotionThigh0": ((0.0, -0.092, 0.907), 0.84),
    "MotionThigh1": ((0.0, 0.092, 0.907), 0.84),
    "MotionLowerLeg0": ((0.0, -0.092, 0.477), 0.67),
    "MotionLowerLeg1": ((0.0, 0.092, 0.477), 0.67),
    "MotionArm0": ((0.0, -0.205, 1.365), 0.38),
    "MotionArm1": ((0.0, 0.205, 1.365), 0.38),
}
BODY_NAMES = tuple(PARTS)


def smooth(u):
    u = np.clip(u, 0.0, 1.0)
    return u * u * (3.0 - 2.0 * u)


def targets(time_s):
    u = np.clip(time_s / C.WALK_DURATION, 0.0, 1.0)
    # Short speed ramps at the ends preserve a roughly constant walking pace.
    phase = smooth(u / 0.13) * smooth((1.0 - u) / 0.14)
    pos = np.array(
        (
            C.START_X + (C.FINISH_X - C.START_X) * u,
            C.WALK_Y,
            C.FLOOR + 0.006 * phase * sin(6 * pi * u),
        )
    )
    # Step away from the gate's path after clearing the posts. Stop between
    # and walk between the pedestals to the near-side waiting mark.
    pos[1] += 0.081 * smooth((u - 0.46) / 0.12)
    sway = 0.015 * sin(3 * pi * u) * phase
    result = {"MotionTorso": transform(pos + PARTS["MotionTorso"][0], sway)}
    for side, offset in enumerate((0.0, pi)):
        swing = 0.43 * sin(5.60 * pi * u + offset) * phase
        knee = -0.17 * max(0.0, sin(5.60 * pi * u + offset)) * phase
        upper = pos + PARTS[f"MotionThigh{side}"][0]
        result[f"MotionThigh{side}"] = transform(upper, swing)
        # The shank starts at the actual rotated knee and carries its shoe.
        lower = upper + [-0.43 * sin(swing), 0.0, -0.43 * cos(swing)]
        result[f"MotionLowerLeg{side}"] = transform(lower, swing + knee)
        shoulder = pos + PARTS[f"MotionArm{side}"][0]
        result[f"MotionArm{side}"] = transform(shoulder, -0.54 * swing - 0.075)
    return np.asarray([result[name] for name in BODY_NAMES], dtype=np.float32)


def transform(position, angle):
    return (*position, 0.0, sin(angle / 2), 0.0, cos(angle / 2))


def ellipsoid(w, s, path, position, radii, color, collision=False):
    shape = w.Geom.Sphere.Define(s, path)
    shape.CreateRadiusAttr(1.0)
    shape.AddTranslateOp().Set(w.Gf.Vec3d(*position))
    shape.AddScaleOp().Set(w.Gf.Vec3f(*radii))
    w._finish(s, shape, color, False)
    if collision:
        # A native capsule prevents the leaf from visually passing through the
        # body. Scoring also checks the shoulder-width envelope across the bars.
        hit = w.Geom.Capsule.Define(s, path + "Contact")
        radius = min(radii)
        hit.CreateRadiusAttr(radius)
        hit.CreateHeightAttr(max(0.0, 2.0 * (radii[2] - radius)))
        hit.AddTranslateOp().Set(w.Gf.Vec3d(*position))
        w._finish(s, hit, color, True)
        hit.MakeInvisible()


def build(w):
    s = w.stage("walker")
    initial = targets(0.0)
    for index, (name, (_, mass)) in enumerate(PARTS.items()):
        path = w.body(s, f"/Asset/{name}", mass, position=initial[index, :3])
        w.Phy.RigidBodyAPI.Get(s, path).CreateKinematicEnabledAttr(True)
        w.Px.PhysxContactReportAPI.Apply(s.GetPrimAtPath(path)).CreateThresholdAttr(0.0)
        if name == "MotionTorso":
            ellipsoid(
                w,
                s,
                path + "/Coat",
                (0.008, 0.0, 0.230),
                (0.114, 0.226, 0.286),
                JACKET,
                True,
            )
            ellipsoid(
                w,
                s,
                path + "/Hips",
                (0.020, 0.0, -0.009),
                (0.100, 0.166, 0.100),
                TROUSERS,
                True,
            )
            ellipsoid(
                w, s, path + "/Neck", (-0.005, 0.0, 0.477), (0.050, 0.054, 0.081), SKIN
            )
            ellipsoid(
                w, s, path + "/Head", (-0.022, 0.0, 0.590), (0.088, 0.077, 0.111), SKIN
            )
            ellipsoid(
                w, s, path + "/Hair", (0.012, 0.0, 0.643), (0.075, 0.0785, 0.073), HAIR
            )
            ellipsoid(
                w, s, path + "/Nose", (-0.105, 0.0, 0.576), (0.026, 0.021, 0.021), SKIN
            )
            for i, y in enumerate((-0.077, 0.077)):
                ellipsoid(
                    w,
                    s,
                    path + f"/Ear{i}",
                    (-0.009, y, 0.581),
                    (0.022, 0.016, 0.037),
                    SKIN,
                )
            # High contrast cuffs and a placket make the person readable in
            # both the overview and the wrist cameras.
            w.box(
                s,
                path + "/Zip",
                (0.004, 0.009, 0.325),
                (-0.103, 0.0, 0.215),
                DARK_JACKET,
                collide=False,
            )
        elif "Thigh" in name:
            ellipsoid(
                w,
                s,
                path + "/Leg",
                (0.0, 0.0, -0.213),
                (0.075, 0.084, 0.255),
                TROUSERS,
                True,
            )
        elif "LowerLeg" in name:
            ellipsoid(
                w,
                s,
                path + "/Leg",
                (0.0, 0.0, -0.198),
                (0.062, 0.065, 0.233),
                TROUSERS,
                True,
            )
            ellipsoid(
                w,
                s,
                path + "/Shoe",
                (-0.078, 0.0, -0.425),
                (0.137, 0.065, 0.040),
                SHOES,
            )
        else:
            ellipsoid(
                w,
                s,
                path + "/Sleeve",
                (0.0, 0.0, -0.175),
                (0.058, 0.054, 0.216),
                JACKET,
                True,
            )
            ellipsoid(
                w,
                s,
                path + "/Cuff",
                (0.0, 0.0, -0.365),
                (0.049, 0.047, 0.052),
                DARK_JACKET,
            )
            ellipsoid(
                w,
                s,
                path + "/Hand",
                (-0.008, 0.0, -0.426),
                (0.034, 0.035, 0.066),
                SKIN,
                True,
            )
    return s
