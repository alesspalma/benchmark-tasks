"""Hollow sleeves capture and retain actual independently simulated granules."""

import math

import numpy as np

from . import config as C
from .usd_scene import Writer


def create_assets(directory):
    w = Writer(directory)
    assets = []
    steel = (0.50, 0.55, 0.59)
    dark = (0.12, 0.15, 0.18)
    blue = (0.06, 0.32, 0.65)
    s = w.stage("probe")
    from pxr import UsdPhysics

    mat = UsdPhysics.MaterialAPI.Get(s, "/Asset/Physics")
    mat.GetStaticFrictionAttr().Set(0.35)
    mat.GetDynamicFrictionAttr().Set(0.25)
    outer = w.body(s, "/Asset/Outer", 0.28, articulation=True)
    # Original 32 mm OD tool, with a 70 mm axial window at working depth.
    w.cylinder(s, outer + "/Bottom", 0.016, 0.004, (0, 0, 0.002), steel)
    from pxr import Gf, UsdGeom

    cone = UsdGeom.Cone.Define(s, outer + "/Tip")
    cone.CreateRadiusAttr(0.016)
    cone.CreateHeightAttr(0.012)
    cone.CreateAxisAttr("Z")
    cone.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.004))
    cone.AddRotateXOp().Set(180)
    w._finish(s, cone, steel, True)
    for i, (lo, hi, holes) in enumerate(
        (
            (0.004, C.WINDOW[0], False),
            (C.WINDOW[0], C.WINDOW[1], True),
            (C.WINDOW[1], 0.300, False),
        )
    ):
        w.ring(
            s,
            outer + f"/Tube{i}",
            0.01525,
            0.0015,
            hi - lo,
            (lo + hi) / 2,
            steel,
            omit=tuple(k % 32 for k in range(-4, 4)) if holes else (),
        )
    w.ring(s, outer + "/Grip", 0.0165, 0.003, 0.045, 0.255, dark)
    # Square lower guide indexes the probe in its removable storage rack.
    for axis in (0, 1):
        for side in (-1, 1):
            dims = [0.036, 0.036, 0.010]
            dims[axis] = 0.002
            pos = [0.0, 0.0, 0.010]
            pos[axis] = side * 0.017
            w.box(s, outer + f"/Guide{axis}{side + 1}", dims, pos, dark)
    # Ordinary opposing support grips, outside the material/discharge lumen.
    for side in (-1, 1):
        w.box(
            s,
            outer + f"/SupportHandle{side + 1}",
            (0.018, 0.062, 0.018),
            (0, side * 0.048, 0.255),
            dark,
        )
    inner = w.body(s, "/Asset/Inner", 0.09)
    w.cylinder(s, inner + "/Bottom", 0.0138, 0.002, (0, 0, 0.005), steel)
    for i, (lo, hi, holes) in enumerate(
        (
            (0.006, C.WINDOW[0], False),
            (C.WINDOW[0], C.WINDOW[1], True),
            (C.WINDOW[1], 0.310, False),
        )
    ):
        w.ring(
            s,
            inner + f"/Tube{i}",
            0.01375,
            0.0010,
            hi - lo,
            (lo + hi) / 2,
            steel,
            omit=tuple(range(4, 12)) if holes else (),
        )
    # Exterior turning lever leaves the proximal discharge opening clear.
    w.box(s, inner + "/Lever", (0.014, 0.065, 0.010), (0, -0.040, 0.309), blue)
    w.cylinder(s, inner + "/LeverGrip", 0.010, 0.024, (0, -0.068, 0.312), dark)
    w.hinge(
        s,
        "/Asset/Sleeve",
        outer,
        inner,
        "Z",
        (0, 0, 0),
        (0, 0, 0),
        (-math.pi / 2, 0),
        friction=0.015,
        damping=0.015,
    )
    assets.append(
        {
            "name": "probe",
            "type": "articulation",
            "file": w.save(s),
            "position": C.PROBE.tolist(),
        }
    )
    s = w.stage("fixtures")
    for name, center, inner, height, color in (
        ("Source", C.TUB, C.TUB_INNER, C.TUB_H, (0.72, 0.73, 0.70)),
        ("Jar", C.JAR, (2 * C.JAR_R, 2 * C.JAR_R), C.JAR_H, (0.76, 0.86, 0.90)),
    ):
        # A real square laboratory jar allows simple repeatable ruler scoring.
        p = "/Asset/" + name
        from pxr import Gf, UsdGeom

        xf = UsdGeom.Xform.Define(s, p)
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        w.tray(s, p, inner, height, 0.002, color, wall=0.003)
    # A square rack holds the matching guide upright without attaching it.
    p = "/Asset/Stand"
    xf = UsdGeom.Xform.Define(s, p)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, C.PROBE)))
    w.box(s, p + "/Foot", (0.09, 0.09, 0.006), (0, 0, -0.014), dark)
    for axis in (0, 1):
        for side in (-1, 1):
            dims = [0.046, 0.046, 0.120]
            dims[axis] = 0.004
            pos = [0.0, 0.0, 0.050]
            pos[axis] = side * 0.021
            w.box(s, p + f"/Guide{axis}{side + 1}", dims, pos, dark)
    grains = []
    from pxr import UsdPhysics, UsdShade

    grain_material = UsdShade.Material.Define(s, "/Asset/GranuleMaterial")
    grain_physics = UsdPhysics.MaterialAPI.Apply(grain_material.GetPrim())
    grain_physics.CreateStaticFrictionAttr(0.30)
    grain_physics.CreateDynamicFrictionAttr(0.20)
    grain_physics.CreateRestitutionAttr(0.0)
    pitch = 2 * C.RADIUS + 0.0003
    for k in range(10):
        for i in range(14):
            for j in range(14):
                position = C.TUB + np.array(
                    [(i - 6.5) * pitch, (j - 6.5) * pitch, 0.005 + C.RADIUS + k * pitch]
                )
                name = f"Grain{len(grains):04}"
                body = w.body(s, "/Asset/Grains/" + name, C.MASS, position=position)
                color = (0.76, 0.55, 0.22) if k < 5 else (0.85, 0.67, 0.32)
                w.sphere(s, body + "/Shape", C.RADIUS, (0, 0, 0), color)
                UsdShade.MaterialBindingAPI.Apply(
                    s.GetPrimAtPath(body + "/Shape")
                ).Bind(grain_material, materialPurpose="physics")
                grains.append({"name": name, "position": position.tolist()})
    assets.append({"name": "fixtures", "type": "base", "file": w.save(s)})
    w.inventory()
    return {"assets": assets, "grains": grains, "settle_actions": 50}
