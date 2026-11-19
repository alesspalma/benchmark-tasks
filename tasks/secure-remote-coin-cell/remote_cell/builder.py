"""Free bodies for the remote, slotted cover and screwdriver."""

import math

import numpy as np

from . import config as C
from .usd_scene import Writer


BODY = (.24, .35, .43)
RIM = (.18, .29, .36)
CAP = (.58, .65, .70)
TAB = (.44, .51, .55)
SILVER = (.74, .77, .76)
TOOL = (.36, .17, .08)
RUBBER = (.18, .24, .23)
WOOD = (.43, .30, .21)
WHITE = (.92, .86, .70)


def convex(w, stage, path, points, z0, z1, color, friction=None):
    """A planar polygon extruded to a convex native collision shape."""
    vertices = [
        w.Gf.Vec3f(float(x), float(y), float(z))
        for z in (z0, z1)
        for x, y in points
    ]
    n = len(points)
    faces = [list(reversed(range(n))), list(range(n, 2 * n))]
    faces.extend([[i, (i + 1) % n, (i + 1) % n + n, i + n] for i in range(n)])
    mesh = w.Geom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(vertices)
    mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
    mesh.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    w._finish(stage, mesh, color, True, contact=.00018)
    w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    if friction:
        w.Shade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(
            friction, materialPurpose="physics"
        )
    return mesh


def polar(r, angle, cy=0.):
    a = math.radians(angle)
    return r * math.cos(a), cy + r * math.sin(a)


def ring(w, stage, root, name, inner, outer, z0, z1, intervals, color, friction=None):
    """Give each sector its own convex collider, keeping the bore empty."""
    for start, end in intervals:
        for i, a0 in enumerate(range(start, end, 10)):
            a1 = min(a0 + 10, end)
            vertices = [
                polar(inner, a0, C.SEAT_CENTER[1]),
                polar(outer, a0, C.SEAT_CENTER[1]),
                polar(outer, a1, C.SEAT_CENTER[1]),
                polar(inner, a1, C.SEAT_CENTER[1]),
            ]
            convex(w, stage, f"{root}/{name}_{start + 360}_{i}", vertices, z0, z1, color, friction)


def make_remote(w):
    stage = w.stage("remote")
    root = w.body(stage, "/Asset/Body", .073, ccd=True)
    low = w.Shade.Material.Define(stage, "/Asset/Bayonet")
    material = w.Phy.MaterialAPI.Apply(low.GetPrim())
    material.CreateStaticFrictionAttr(.20)
    material.CreateDynamicFrictionAttr(.15)
    material.CreateRestitutionAttr(0.)
    y = C.SEAT_CENTER[1]
    w.box(stage, root + "/HousingBase", (.040, .143, .004), (0, .020, .002), BODY)
    w.cylinder(stage, root + "/RoundedEnd", .020, .004, (0., y, .002), BODY)
    w.box(stage, root + "/FrontHousing", (.040, .113, .013), (0, .035, .0105), BODY)
    w.cylinder(stage, root + "/Battery", .010, .0032, (0, y, .0071), SILVER)
    w.box(stage, root + "/PlusLong", (.006, .0012, .00010), (0, y, .00875), RIM, collide=False)
    w.box(stage, root + "/PlusShort", (.0012, .006, .00010), (0, y, .00875), RIM, collide=False)
    # Seat at 11.9 mm, an annular pocket underneath a roof with two entry gaps.
    ring(w, stage, root, "Seat", .01040, .020, .004, .0119, [(0, 360)], RIM, low)
    ring(w, stage, root, "GrooveWall", .0165, .020, .0119, .0148, [(0, 360)], RIM, low)
    ring(w, stage, root, "RetainingLip", .0133, .020, .0148, .017, [(22, 158), (202, 338)], RIM, low)
    ring(w, stage, root, "WindowEnd", .01265, .020, .0119, .0148, [(101, 109), (281, 289)], RIM, low)
    ring(w, stage, root, "EntryEdges", .0165, .020, .0148, .017, [(-22, 22), (158, 202)], RIM, low)
    # Above-seat witness lines for entry and full engagement.
    for name, a, color in (
        ("Entry", 0., WHITE),
        ("Closed", math.degrees(C.LOCK_ANGLE), (.24, .73, .49)),
    ):
        x, py = polar(.0183, a, y)
        w.cylinder(stage, root + "/" + name, .00135, .00015, (x, py, .0171), color, collide=False)
    # Recessed button shapes on the reverse face and an IR window at the nose.
    for i, col in enumerate([(.74, .31, .25), (.35, .57, .71), (.65, .66, .55)]):
        w.cylinder(stage, f"{root}/Button{i}", .0048, .001, (0, .011 + i*.013, .0006), col, collide=False)
    w.box(stage, root + "/InfraredWindow", (.021, .0014, .009), (0., .0915, .0105), (.14, .15, .17), collide=False)
    return w.save(stage)


def make_cap(w):
    stage = w.stage("cover")
    root = w.body(stage, "/Asset/Cover", .006, ccd=True)
    w.cylinder(stage, root + "/Neck", .0113, .0036, (0, 0, -.0018), TAB)
    w.cylinder(stage, root + "/SlotFloor", C.CAP_RADIUS, .0012, (0, 0, .0006), CAP)
    for sign, label in ((1, "North"), (-1, "South")):
        # A clipped circular segment leaves a 3 mm slot between the cheeks.
        rad = C.CAP_RADIUS
        low = math.degrees(math.asin(.00150 / rad))
        angles = np.linspace(low, 180 - low, 13)
        poly = [polar(rad, a) for a in angles]
        if sign < 0:
            poly = [(-x, -y) for x, y in poly]
        convex(w, stage, root + "/" + label, poly, .0012, C.CAP_TOP, CAP)
    for sign, label in ((1, "Positive"), (-1, "Negative")):
        w.box(stage, root + "/Ear" + label, (.0057, .0033, .0016),
              (sign * .0123, 0, -.0022), TAB)
    w.cylinder(stage, root + "/Witness", .0011, .00015,
               (.0085, -.0037, C.CAP_TOP + .00006), (.18, .24, .25), collide=False)
    return w.save(stage)


def make_driver(w):
    stage = w.stage("screwdriver")
    root = w.body(stage, "/Asset/Driver", .052, ccd=True)
    w.box(stage, root + "/SlottedBlade", (.0080, .00125, .0035),
          (0, 0, .00175), SILVER)
    w.cylinder(stage, root + "/Shaft", .00245, .037, (0, 0, .0217), SILVER)
    w.cylinder(stage, root + "/Handle", .013, .033, (0, 0, .055), TOOL)
    for i in range(3):
        w.cylinder(stage, f"{root}/GripRib_{i}", .0132, .003,
                   (0, 0, .047 + .008*i), RUBBER, collide=False)
    return w.save(stage)


def make_fixtures(w):
    stage = w.stage("fixtures")
    w.cylinder(stage, "/Asset/SiliconeCapPad", .0095, .0055,
               tuple(C.CAP_PAD + [0., 0., .00275]), RUBBER)
    for name, bore, bottom, height in (
        ("Base", .0048, C.TABLE, .007),
        ("Guide", .00295, C.TABLE + .007, .024),
    ):
        for i in range(12):
            a = i * 30
            gap = .0048 if name == "Guide" and i % 6 in (0, 5) else bore
            pts = [polar(r, b) for r, b in ((gap, a), (.018, a), (.018, a+30), (gap, a+30))]
            pts = [(x + C.DRIVER[0], y + C.DRIVER[1]) for x, y in pts]
            convex(w, stage, f"/Asset/ToolRest{name}_{i}", pts, bottom, bottom + height, RUBBER)
    w.box(stage, "/Asset/LowShelf", (C.SHELF_WIDTH, C.SHELF_LENGTH, .019),
          C.SHELF + [0, 0, .0095], WOOD)
    w.box(stage, "/Asset/ShelfBack", (.018, C.SHELF_LENGTH, .034),
          C.SHELF + [C.SHELF_WIDTH/2 + .009, 0, .030], WOOD)
    w.box(stage, "/Asset/ShelfSide", (C.SHELF_WIDTH+.018, .012, .034),
          C.SHELF + [0, C.SHELF_LENGTH/2 + .006, .030], WOOD)
    # Two small wooden toys share the shelf behind the return area.
    for i, (pos, radius, color) in enumerate([
        ((.066, .042), .011, (.51, .26, .16)),
        ((.066, -.037), .009, (.61, .47, .16)),
    ]):
        w.box(stage, f"/Asset/ToyCarBody_{i}", (.033, .045, .013),
              C.SHELF + [pos[0], pos[1], .039], color)
        for dx in (-.012, .012):
            for dy in (-.016, .016):
                w.cylinder(stage, f"/Asset/Wheel_{i}_{dx}_{dy}", radius, .003,
                           C.SHELF + [pos[0] + dx, pos[1] + dy, .030],
                           RUBBER, axis="X")
    return w.save(stage)


def create_assets(directory):
    w = Writer(directory)
    return {
        "assets": [
            {"name": name, "type": kind, "file": file, "position": p}
            for name, kind, file, p in (
                ("remote", "rigid", make_remote(w), C.REMOTE),
                ("cover", "rigid", make_cap(w), C.CAP),
                ("driver", "rigid", make_driver(w), C.DRIVER),
                ("fixtures", "base", make_fixtures(w), np.zeros(3)),
            )
        ]
    }
