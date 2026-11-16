"""Upright sampling chamber with a passive spring-return sliding shutter."""

from . import config as C
from .usd_scene import Writer


def create_assets(directory):
    w = Writer(directory)
    steel, dark, blue = (0.50, 0.55, 0.59), (0.12, 0.15, 0.18), (0.06, 0.32, 0.75)
    s = w.stage("probe")
    from pxr import Gf, UsdGeom, UsdPhysics, UsdShade

    mat = UsdPhysics.MaterialAPI.Get(s, "/Asset/Physics")
    mat.GetStaticFrictionAttr().Set(0.35)
    mat.GetDynamicFrictionAttr().Set(0.25)
    outer = w.body(s, "/Asset/Outer", 0.28, articulation=True)
    # Closed chamber except for the +X side port. The sloping floor drains
    # through that same port while the probe stays upright inside the jar.
    w.box(s, outer + "/Base", (0.054, 0.054, 0.004), (0, 0, 0.002), steel)
    w.box(s, outer + "/Back", (0.003, 0.048, 0.050), (-0.0255, 0, 0.025), steel)
    for side in (-1, 1):
        w.box(
            s,
            outer + f"/Side{side + 1}",
            (0.054, 0.003, 0.050),
            (0, side * 0.0255, 0.025),
            steel,
        )
    w.box(s, outer + "/Cap", (0.054, 0.054, 0.003), (0, 0, 0.0515), steel)
    wedge = [
        [-0.024, -0.024, 0.004],
        [0.024, -0.024, 0.004],
        [-0.024, -0.024, 0.020],
        [-0.024, 0.024, 0.004],
        [0.024, 0.024, 0.004],
        [-0.024, 0.024, 0.020],
    ]
    triangles = [
        [0, 2, 1],
        [3, 4, 5],
        [0, 1, 4],
        [0, 4, 3],
        [0, 3, 5],
        [0, 5, 2],
        [2, 5, 4],
        [2, 4, 1],
    ]
    w.convex(s, outer + "/SlopedFloor", wedge, [t[::-1] for t in triangles], steel)
    roof = [
        [-0.027, -0.027, 0.053],
        [0.027, -0.027, 0.053],
        [0.027, 0.027, 0.053],
        [-0.027, 0.027, 0.053],
        [0, 0, 0.085],
    ]
    w.convex(
        s,
        outer + "/ShedRoof",
        roof,
        [[0, 2, 1], [0, 3, 2], [0, 1, 4], [1, 2, 4], [2, 3, 4], [3, 0, 4]],
        steel,
    )
    # Hollow square stem and a broad ordinary support grip on the left side.
    for axis in (0, 1):
        for side in (-1, 1):
            dims = [0.018, 0.018, 0.215]
            dims[axis] = 0.002
            pos = [0.0, 0.0, 0.1925]
            pos[axis] = side * 0.008
            w.box(s, outer + f"/Stem{axis}{side + 1}", dims, pos, steel)
    w.box(
        s, outer + "/SupportGrip", (0.024, 0.076, 0.018), (-0.010, 0.052, 0.255), dark
    )
    # Two visible guide bearings; the native prismatic constraint represents
    # their linear bearing. Its spring is constant, never switched or latched.
    for k, z in enumerate((0.090, 0.130)):
        w.box(s, outer + f"/GuideArmY{k}", (0.006, 0.035, 0.006), (0, -0.0175, z), dark)
        w.box(
            s,
            outer + f"/GuideArmX{k}",
            (0.029, 0.006, 0.006),
            (0.0145, -0.035, z),
            dark,
        )
        for side in (-1, 1):
            w.box(
                s,
                outer + f"/GuideX{k}{side + 1}",
                (0.002, 0.012, 0.006),
                (0.033 + side * 0.005, -0.035, z),
                dark,
            )
            w.box(
                s,
                outer + f"/GuideY{k}{side + 1}",
                (0.008, 0.002, 0.006),
                (0.033, -0.035 + side * 0.005, z),
                dark,
            )
    gate = w.body(s, "/Asset/Gate", 0.070)
    w.box(s, gate + "/Shutter", (0.0025, 0.054, 0.054), (0.02875, 0, 0.027), blue)
    w.box(s, gate + "/Connector", (0.008, 0.008, 0.008), (0.031, -0.029, 0.058), blue)
    w.box(s, gate + "/PullRod", (0.006, 0.006, 0.132), (0.033, -0.035, 0.124), steel)
    w.box(s, gate + "/PullArmY", (0.006, 0.045, 0.006), (0.033, -0.0575, 0.190), blue)
    w.box(s, gate + "/PullArmX", (0.083, 0.006, 0.006), (-0.0085, -0.080, 0.190), blue)
    w.box(s, gate + "/PullTab", (0.060, 0.034, 0.018), tuple(C.GATE_GRASP), blue)
    w.slide(
        s,
        "/Asset/GateSlide",
        outer,
        gate,
        C.GATE_TRAVEL,
        C.GATE_SPRING,
        -C.GATE_PRELOAD,
    )
    assets = [
        {
            "name": "probe",
            "type": "articulation",
            "file": w.save(s),
            "position": C.PROBE.tolist(),
        }
    ]

    s = w.stage("fixtures")
    for name, center, inner, height, color in (
        ("Source", C.TUB, C.TUB_INNER, C.TUB_H, (0.72, 0.73, 0.70)),
        ("Jar", C.JAR, (2 * C.JAR_R, 2 * C.JAR_R), C.JAR_H, (0.76, 0.86, 0.90)),
    ):
        p = "/Asset/" + name
        xf = UsdGeom.Xform.Define(s, p)
        xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, center)))
        w.tray(s, p, inner, height, 0.002, color, wall=0.003)
    p = "/Asset/Stand"
    xf = UsdGeom.Xform.Define(s, p)
    xf.AddTranslateOp().Set(Gf.Vec3d(*map(float, C.PROBE)))
    w.box(s, p + "/Foot", (0.116, 0.116, 0.006), (0, 0, -0.003), dark)
    for axis in (0, 1):
        for side in (-1, 1):
            dims = [0.108, 0.108, 0.030]
            dims[axis] = 0.004
            pos = [0.0, 0.0, 0.015]
            pos[axis] = side * 0.052
            w.box(s, p + f"/Guide{axis}{side + 1}", dims, pos, dark)
    # Secondary containment catches loose exterior granules; only material
    # physically sampled and settled inside the separate jar earns credit.
    p = "/Asset/Containment"
    xf = UsdGeom.Xform.Define(s, p)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.40, 0.16, float(C.TOP)))
    w.tray(s, p, (0.520, 0.620), 0.050, 0.001, (0.30, 0.34, 0.38), wall=0.003)
    # Eight millimetres of solid base, with its top flush to the containers.
    w.box(
        s, p + "/ThickBase", (0.526, 0.626, 0.008), (0, 0, -0.0005), (0.30, 0.34, 0.38)
    )
    grains = []
    grain_material = UsdShade.Material.Define(s, "/Asset/GranuleMaterial")
    gm = UsdPhysics.MaterialAPI.Apply(grain_material.GetPrim())
    gm.CreateStaticFrictionAttr(0.30)
    gm.CreateDynamicFrictionAttr(0.20)
    gm.CreateRestitutionAttr(0.0)
    pitch = 2 * C.RADIUS + 0.0003
    for k in range(10):
        for i in range(14):
            for j in range(14):
                position = C.TUB + [
                    (i - 6.5) * pitch,
                    (j - 6.5) * pitch,
                    0.005 + C.RADIUS + k * pitch,
                ]
                name = f"Grain{len(grains):04}"
                body = w.body(
                    s, "/Asset/Grains/" + name, C.MASS, position=position, ccd=True
                )
                color = (0.76, 0.55, 0.22) if k < 5 else (0.85, 0.67, 0.32)
                w.sphere(s, body + "/Shape", C.RADIUS, (0, 0, 0), color)
                UsdShade.MaterialBindingAPI.Apply(
                    s.GetPrimAtPath(body + "/Shape")
                ).Bind(grain_material, materialPurpose="physics")
                grains.append({"name": name, "position": position.tolist()})
    assets.append({"name": "fixtures", "type": "base", "file": w.save(s)})
    w.inventory()
    return {"assets": assets, "grains": grains, "settle_actions": 50}
