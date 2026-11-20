"""PhysX aperture, loose tool plate, and compliant cutting tool."""

import numpy as np

from . import config as C
from .usd_scene import Writer

BENCH = (.51, .41, .30)
ALUMINIUM = (.28, .34, .40)
BURR = (.79, .51, .20)
FRAME = (.10, .12, .15)
STEEL = (.47, .51, .54)
GRIP = (.15, .47, .74)


def prism(w, s, path, vertices, low, high, color, collides):
    n = len(vertices)
    mesh = w.Geom.Mesh.Define(s, path)
    mesh.CreatePointsAttr([w.Gf.Vec3f(float(x), float(y), height)
                           for height in (low, high) for x, y in vertices])
    faces = [list(reversed(range(n))), list(range(n, 2 * n))]
    faces += [[i, (i + 1) % n, (i + 1) % n + n, i + n] for i in range(n)]
    mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
    mesh.CreateFaceVertexIndicesAttr([p for face in faces for p in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    if collides:
        w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    w._finish(s, mesh, color, collides)


def layout(w):
    s = w.stage("layout")
    w.box(s, "/Asset/Bench", (1.10, 1.22, .06), (.36, 0., C.TABLE - .03), BENCH)
    w.box(s, "/Asset/Floor", (5., 5., .02), (0., 0., -.01), (.32, .35, .37))
    for i, (x, y) in enumerate(((-.16, -.56), (-.16, .56), (.83, -.56), (.83, .56))):
        w.box(s, f"/Asset/BenchLeg{i}", (.05, .05, .674), (x, y, .346), FRAME)
    for i, x in enumerate(C.PLATE_START[0] + np.array([-1., 1.]) * C.PARALLEL_OFFSET_X):
        w.box(s, f"/Asset/Parallel{i}", (.020, .076, C.STOCK_SUPPORT),
              (x, C.PLATE_START[1] + .013, C.TABLE + C.STOCK_SUPPORT / 2), FRAME)
    x, y, z = C.TOOL_START
    w.cylinder(s, "/Asset/Stand", .032, .0025, (x, y, C.TABLE + .00125), FRAME)
    for i in range(8):
        phi = 2 * np.pi * i / 8
        xx, yy = x + .012 * np.cos(phi), y + .012 * np.sin(phi)
        w.cylinder(s, f"/Asset/HolsterWall{i}", .0045, .045, (xx, yy, z + .0215), FRAME)
    return s


def plate(w):
    s = w.stage("plate")
    path = w.body(s, "/Asset", .565, ccd=True)
    half = np.array([C.PLATE_X, C.PLATE_Y]) / 2
    contour = C.outline()
    rays = contour - C.PORT_CENTER
    boundary = np.where(rays > 0, half, -half)
    divisions = np.divide(boundary - C.PORT_CENTER, rays,
                          out=np.full_like(rays, np.inf), where=np.abs(rays) > 1e-9)
    outer = C.PORT_CENTER + rays * divisions.min(axis=1, keepdims=True)
    half_z = C.PLATE_THICKNESS / 2
    for i, a in enumerate(contour):
        j = (i + 1) % len(contour)
        b = contour[j]
        prism(w, s, path + f"/MilledRing{i}", [a, b, outer[j], outer[i]],
              -half_z, half_z, ALUMINIUM, True)
        # A clearly lit witness lip represents the remaining machining burr.
        inward_a = a - .0011 * rays[i] / np.linalg.norm(rays[i])
        inward_b = b - .0011 * rays[j] / np.linalg.norm(rays[j])
        prism(w, s, path + f"/Burr{i}", [inward_a, inward_b, b, a],
              half_z, half_z + .00035, BURR, False)
    # Counterbore rings and the dowel references identify the flange interface.
    for i, (x, y) in enumerate(((-.052, -.055), (.052, -.055), (-.052, .055), (.052, .055))):
        w.cylinder(s, path + f"/CounterboreShade{i}", .0065, .00018, (x, y, half_z + .00008),
                   FRAME, collide=False)
        w.cylinder(s, path + f"/CounterboreBottom{i}", .0042, .00019, (x, y, half_z + .00012),
                   (.16, .19, .22), collide=False)
    return s


def tool(w):
    s = w.stage("tool")
    handle = w.body(s, "/Asset/Handle", .065, ccd=True, articulation=True)
    material = w.Phy.MaterialAPI.Get(s, "/Asset/Physics")
    material.CreateStaticFrictionAttr(.95)
    material.CreateDynamicFrictionAttr(.76)
    w.rounded_box(s, handle + "/NylonGrip", (.023, .024, .094), (0., 0., .094),
                  GRIP, .004, .0018)
    w.cylinder(s, handle + "/GripCollar", .011, .012, (0., 0., .047), FRAME)
    w.cylinder(s, handle + "/GripCap", .0123, .007, (0., 0., .142), FRAME)
    shaft = w.body(s, "/Asset/FlexShaft", .006, ccd=True)
    w.cylinder(s, shaft + "/SteelShank", .00165, .016, (0., 0., .0335), STEEL)
    flex = w.Phy.PrismaticJoint.Define(s, "/Asset/Flex")
    flex.CreateBody0Rel().SetTargets([handle])
    flex.CreateBody1Rel().SetTargets([shaft])
    flex.CreateAxisAttr("Z")
    flex.CreateCollisionEnabledAttr(False)
    flex.CreateLowerLimitAttr(-.0008)
    flex.CreateUpperLimitAttr(.0060)
    spring = w.Phy.DriveAPI.Apply(flex.GetPrim(), "linear")
    spring.CreateTypeAttr("force")
    spring.CreateStiffnessAttr(C.FLEX_K)
    spring.CreateDampingAttr(21.)
    spring.CreateMaxForceAttr(48.)
    spring.CreateTargetPositionAttr(0.)
    blade = w.body(s, "/Asset/Knife", .004, ccd=True)
    w.cylinder(s, blade + "/SwivelShank", .0016, .019, (0., 0., .018), STEEL)
    # The cutting toe is modelled with a narrow convex edge rather than a
    # razor surface. The final 1.2 mm of the toe bears on the machined lip.
    prism(w, s, blade + "/CuttingToe",
          [(-.0016, -.001), (.0032, -.001), (.0042, 0.), (.0024, .001), (-.0016, .001)],
          C.KNIFE[2] - .0015, C.KNIFE[2] + .0070, STEEL, True)
    cutting_material = w.Shade.Material.Define(s, "/Asset/SteelContact")
    steel = w.Phy.MaterialAPI.Apply(cutting_material.GetPrim())
    steel.CreateStaticFrictionAttr(.45)
    steel.CreateDynamicFrictionAttr(.34)
    for path in (shaft + "/SteelShank", blade + "/SwivelShank", blade + "/CuttingToe"):
        w.Shade.MaterialBindingAPI.Apply(s.GetPrimAtPath(path)).Bind(
            cutting_material, materialPurpose="physics"
        )
    spin = w.Phy.RevoluteJoint.Define(s, "/Asset/Swivel")
    spin.CreateBody0Rel().SetTargets([shaft])
    spin.CreateBody1Rel().SetTargets([blade])
    spin.CreateAxisAttr("Z")
    spin.CreateCollisionEnabledAttr(False)
    return s


def create_assets(directory):
    w = Writer(directory)
    rows = (
        ("layout", "base", layout(w), (0., 0., 0.)),
        ("plate", "rigid", plate(w), C.PLATE_START),
        ("tool", "articulation", tool(w), C.TOOL_START),
    )
    assets = [dict(name=name, type=kind, file=w.save(stage), position=list(pos))
              for name, kind, stage, pos in rows]
    w.inventory()
    return {"assets": assets, "settle_actions": 24}
