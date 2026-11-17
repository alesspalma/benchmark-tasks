"""A notched collecting rod extends from its sleeve, inspired by Buerkle Mini."""

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
    # Fixed 48 mm sleeve, open at its lower end. A notched rod retracts
    # entirely into this sleeve; no floor or cross plate moves through it.
    w.clear_panel(s, outer + "/SleeveBack", (0.048, 0.003, 0.029), (0, -0.0225, 0.0205))
    # The bell-mouth guides coarse calibration beads into the sleeve as
    # the filled pocket retracts, instead of trapping them at a square lip.
    lip = [
        [x, y, z]
        for x in (-0.024, 0.024)
        for y, z in ((-0.024, 0), (-0.024, 0.006), (-0.021, 0.006))
    ]
    lip_faces = [
        [0, 1, 2],
        [3, 5, 4],
        [0, 3, 4],
        [0, 4, 1],
        [1, 4, 5],
        [1, 5, 2],
        [0, 2, 5],
        [0, 5, 3],
    ]
    w.convex(s, outer + "/NotchLipChamfer", lip, lip_faces, steel)
    w.box(s, outer + "/SleeveFront", (0.048, 0.003, 0.035), (0, 0.0225, 0.0175), steel)
    for side in (-1, 1):
        w.box(
            s,
            outer + f"/SleeveSide{side + 1}",
            (0.003, 0.042, 0.035),
            (side * 0.0225, 0, 0.0175),
            steel,
        )
        w.box(
            s,
            outer + f"/CapX{side + 1}",
            (0.017, 0.048, 0.003),
            (side * 0.0155, 0, 0.0365),
            steel,
        )
        w.box(
            s,
            outer + f"/CapY{side + 1}",
            (0.014, 0.017, 0.003),
            (0, side * 0.0155, 0.0365),
            steel,
        )
        w.box(
            s,
            outer + f"/StemX{side + 1}",
            (0.002, 0.018, 0.232),
            (side * 0.008, 0, 0.154),
            steel,
        )
        w.box(
            s,
            outer + f"/StemY{side + 1}",
            (0.014, 0.002, 0.232),
            (0, side * 0.008, 0.154),
            steel,
        )
    # Ordinary rubber-covered barrel grip, not an added grasping handle.
    for side in (-1, 1):
        w.box(
            s,
            outer + f"/BarrelGripX{side + 1}",
            (0.004, 0.024, 0.080),
            (side * 0.010, 0, 0.180),
            dark,
        )
        w.box(
            s,
            outer + f"/BarrelGripY{side + 1}",
            (0.016, 0.004, 0.080),
            (0, side * 0.010, 0.180),
            dark,
        )
    # Two ordinary square guide bushings keep the6mm rod aligned in the barrel.
    for k, z in enumerate((0.043, 0.260)):
        for side in (-1, 1):
            w.box(
                s,
                outer + f"/RodBushingX{k}{side + 1}",
                (0.00375, 0.014, 0.010),
                (side * 0.005125, 0, z),
                dark,
            )
            w.box(
                s,
                outer + f"/RodBushingY{k}{side + 1}",
                (0.0065, 0.00375, 0.010),
                (0, side * 0.005125, z),
                dark,
            )
    # Finger-rest collar on the barrel takes the axial pushbutton reaction.
    # It is part of the handheld tool, with a central hole for the moving rod.
    for side in (-1, 1):
        w.box(
            s,
            outer + f"/FingerRestX{side + 1}",
            (0.011, 0.036, 0.004),
            (side * 0.0125, 0, 0.194),
            dark,
        )
        w.box(
            s,
            outer + f"/FingerRestY{side + 1}",
            (0.014, 0.011, 0.004),
            (0, side * 0.0125, 0.194),
            dark,
        )
    rod = w.body(s, "/Asset/Rod", 0.085)
    # Pocket walls and floor belong to ONE rigid collecting rod.
    # The -Y notch is the only inlet/outlet; the floor always stays beneath it.
    w.box(s, rod + "/PocketFloor", (0.040, 0.040, 0.004), (0, 0, 0.002), steel)
    w.box(s, rod + "/PocketBack", (0.034, 0.003, 0.020), (0, 0.0185, 0.010), steel)
    for side in (-1, 1):
        w.box(
            s,
            rod + f"/PocketSide{side + 1}",
            (0.003, 0.040, 0.020),
            (side * 0.0185, 0, 0.010),
            steel,
        )
    w.box(s, rod + "/PocketRoof", (0.040, 0.040, 0.003), (0, 0, 0.0215), steel)
    w.box(s, rod + "/InnerRod", (0.006, 0.006, 0.287), (0, 0, 0.1665), steel)
    w.cylinder(s, rod + "/PushButton", 0.015, 0.010, (0, 0, 0.310), blue)
    w.slide(
        s,
        "/Asset/SampleSlide",
        outer,
        rod,
        C.GATE_TRAVEL,
        C.GATE_SPRING,
        -C.GATE_PRELOAD,
        axis="Z",
        reverse=True,
    )
    rubber = UsdShade.Material.Define(s, "/Asset/RubberGrip")
    rm = UsdPhysics.MaterialAPI.Apply(rubber.GetPrim())
    rm.CreateStaticFrictionAttr(1.2)
    rm.CreateDynamicFrictionAttr(1.0)
    rm.CreateRestitutionAttr(0.0)
    for suffix in ("BarrelGripX0", "BarrelGripX2", "BarrelGripY0", "BarrelGripY2"):
        UsdShade.MaterialBindingAPI.Apply(s.GetPrimAtPath(outer + "/" + suffix)).Bind(
            rubber, materialPurpose="physics"
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
