"""Native dry skate, compound channel guard with spring slider, and open bag."""

import numpy as np

from . import config as C
from .usd_scene import Writer

BOOT = (0.15, 0.22, 0.36)
SOLE = (0.06, 0.073, 0.086)
LINING = (0.078, 0.11, 0.16)
STEEL = (0.61, 0.69, 0.73)
GUARD = (0.94, 0.28, 0.08)
HEEL = (0.89, 0.51, 0.095)
BAG_COLOR = (0.10, 0.41, 0.39)
DARK = (0.07, 0.13, 0.13)
WHITE = (0.86, 0.85, 0.77)


def _rotation_x(degrees):
    a = np.deg2rad(degrees)
    return ((1, 0, 0), (0, np.cos(a), -np.sin(a)), (0, np.sin(a), np.cos(a)))


def skate(w):
    s = w.stage("skate")
    b = w.body(s, "/Asset/Boot", 0.61, ccd=True)
    for i, (cy, width, length, top) in enumerate((
        (-0.113, 0.063, 0.037, 0.101),
        (-0.076, 0.066, 0.065, 0.157),
        (-0.014, 0.077, 0.075, 0.105),
        (0.049, 0.088, 0.074, 0.081),
        (0.101, 0.075, 0.055, 0.066),
        (0.131, 0.057, 0.018, 0.057),
    )):
        w.box(s, f"{b}/BootSections{i}", (width, length, top - 0.036),
              (0, cy, (top + 0.036) / 2), BOOT)
        w.box(s, f"{b}/Sole{i}", (width + 0.004, length, 0.009), (0, cy, 0.034), SOLE)
    w.box(s, b + "/Tongue", (0.057, 0.079, 0.012), (0, -0.001, 0.119),
          LINING, _rotation_x(-30), collide=False)
    for i in range(7):
        y = -0.048 + 0.012 * i
        z = 0.136 - 0.0075 * i
        w.box(s, f"{b}/Lace{i}", (0.054, 0.0042, 0.0022), (0, y, z), WHITE,
              _rotation_x(-24), collide=False)
    for i, y in enumerate((-0.091, 0.092)):
        w.box(s, f"{b}/Holder{i}", (0.014, 0.041, 0.026), (0, y, 0.0215), WHITE)
    w.box(s, b + "/Runner", (0.0048, 0.222, 0.014), (0, 0.003, 0.007), STEEL)
    # Short rounded-off profiles at the tip and heel retain under the end hooks.
    for sign, start, end in ((-1, -0.108, C.RUNNER_HEEL), (1, 0.114, C.RUNNER_TOE)):
        span = abs(end - start)
        w.box(s, f"{b}/End{sign}", (0.0048, span + 0.001, 0.012),
              (0, (end + start) / 2, 0.009), STEEL)
    return s


def _channel(w, s, path, lo, hi, y_offset, color):
    centre = (lo + hi) / 2 - y_offset
    length = hi - lo
    w.box(s, path + "/Sole", (0.031, length, 0.011), (0, centre, 0.0055), color)
    for i, x in enumerate((-0.011, 0.011)):
        w.box(s, f"{path}/LipSide{i}", (0.006, length, 0.016),
              (x, centre, 0.018), color)


def _end_hook(w, s, path, y, y_offset, sign, color):
    w.box(s, path + "/Stop", (0.028, 0.007, 0.032),
          (0, y + sign * 0.0035 - y_offset, 0.016), color)
    w.box(s, path + "/Roof", (0.016, 0.009, 0.004),
          (0, y - sign * 0.0045 - y_offset, 0.029), color)


def guard(w):
    s = w.stage("guard")
    toe = w.body(s, "/Asset/Toe", 0.077, articulation=True, ccd=True)
    heel = w.body(s, "/Asset/Heel", 0.071, position=(0, C.HEEL_BODY_Y, 0), ccd=True)
    _channel(w, s, toe, -0.016, C.GUARD_TOE_INNER + 0.006, 0, GUARD)
    _channel(w, s, heel, C.HEEL_AT_REST - 0.006, 0.006, C.HEEL_BODY_Y, HEEL)
    _end_hook(w, s, toe, C.GUARD_TOE_INNER, 0, 1, GUARD)
    _end_hook(w, s, heel, C.HEEL_AT_REST, C.HEEL_BODY_Y, -1, HEEL)
    # Narrow overlapping track bridges the junction as the two cups spread.
    w.box(s, toe + "/TrackBottom", (0.013, 0.072, 0.0032),
          (0, -0.034, 0.0035), GUARD, collide=False)
    for i, x in enumerate((-0.0158, 0.0158)):
        w.cylinder(s, f"{toe}/Spring{i}", 0.0026, 0.066, (x, -0.033, 0.009),
                   STEEL, axis="Y", collide=False)
    j = w.Phy.PrismaticJoint.Define(s, "/Asset/Extension")
    j.CreateBody0Rel().SetTargets([toe])
    j.CreateBody1Rel().SetTargets([heel])
    j.CreateAxisAttr("Y")
    j.CreateLocalPos0Attr(w.Gf.Vec3f(0, C.HEEL_BODY_Y, 0))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(0))
    j.CreateLowerLimitAttr(-C.GUARD_TRAVEL)
    j.CreateUpperLimitAttr(0.)
    spring = w.Phy.DriveAPI.Apply(j.GetPrim(), "linear")
    spring.CreateTypeAttr("force")
    spring.CreateTargetPositionAttr(C.SPRING_PRELOAD_EXTENSION)
    spring.CreateStiffnessAttr(C.SPRING_RATE)
    spring.CreateDampingAttr(C.SPRING_DAMPING)
    spring.CreateMaxForceAttr(7.4)
    # Channel halves overlap by design and the guided joint keeps them aligned.
    w.Px.PhysxArticulationAPI.Get(s, toe).CreateEnabledSelfCollisionsAttr(False)
    return s


def rest(w):
    s = w.stage("bench_rest")
    w.box(s, "/Asset/Shim", (0.031, 0.251, 0.029),
          (-.009, .013, .0145), (0.55, 0.30, 0.14))
    return s


def bag(w):
    s = w.stage("bag")
    x, y, z = C.BAG_SIZE
    w.box(s, "/Asset/PaddedBase", (x, y, C.BAG_FLOOR),
          (0, 0, C.BAG_FLOOR / 2), DARK)
    for sign in (-1, 1):
        w.box(s, f"/Asset/LongPanel{sign}", (0.004, y, z),
              (sign * (x / 2 - 0.002), 0, z / 2), BAG_COLOR)
        w.box(s, f"/Asset/ShortPanel{sign}", (x, 0.004, z),
              (0, sign * (y / 2 - 0.002), z / 2), BAG_COLOR)
        w.cylinder(s, f"/Asset/ZipperEdge{sign}", 0.003, y,
                   (sign * (x / 2 - 0.002), 0, z), DARK, axis="Y", collide=False)
        for i, py in enumerate((-0.092, 0.092)):
            w.box(s, f"/Asset/Webbing{sign}_{i}", (0.002, 0.025, z),
                  (sign * x / 2, py, z / 2), DARK, collide=False)
    return s


def create_assets(directory):
    from yam_arena.ik import matrix_to_quat_xyzw

    w = Writer(directory)
    assets = []
    for name, kind, factory, pos, rotation in (
        ("skate", "rigid", skate, C.BOOT_START, C.SIDE),
        ("guard", "articulation", guard, C.GUARD_START, C.SIDE),
        ("bench_rest", "base", rest, np.r_[C.GUARD_START[:2], C.TABLE], np.eye(3)),
        ("bag", "base", bag, C.BAG, np.eye(3)),
    ):
        assets.append(dict(name=name, type=kind, position=pos.tolist(),
                           quaternion=matrix_to_quat_xyzw(rotation).tolist(), file=w.save(factory(w))))
    w.inventory()
    return {"assets": assets, "settle_actions": 15}
