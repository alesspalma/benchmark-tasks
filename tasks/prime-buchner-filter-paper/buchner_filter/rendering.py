"""Native geometry witnesses show measured wetting, solids and water volume."""

from math import pi

import numpy as np

from . import config as C
from .filtration import pose_parts


def _surface(stage, path, points):
    from pxr import Gf, UsdGeom

    mesh = UsdGeom.Mesh.Get(stage, path)
    mesh.GetPointsAttr().Set([Gf.Vec3f(*map(float, point)) for point in points])
    mesh.GetFaceVertexCountsAttr().Set([len(points)] if len(points) > 2 else [])
    mesh.GetFaceVertexIndicesAttr().Set(
        list(range(len(points))) if len(points) > 2 else []
    )


def _show_feed_beads(stage, path, axes, surface, proportion):
    from pxr import Gf, UsdGeom

    mesh = UsdGeom.Mesh.Get(stage, path)
    num = round(28 * proportion) if len(surface) >= 3 else 0
    beads = []
    if num:
        center = np.mean(surface, axis=0)
        normal = axes[2, :]
        u = np.cross(normal, [0, 1, 0])
        u /= max(np.linalg.norm(u), 1e-6)
        v = np.cross(normal, u)
        radius = min(float(np.linalg.norm(side - center)) for side in surface) * 0.72
        angle = np.arange(6) * pi / 3
        points = (np.cos(angle)[:, None] * u + np.sin(angle)[:, None] * v) * 0.0011
        for i in range(num):
            t = i * 2.39996323
            q = (
                center
                + (np.cos(t) * u + np.sin(t) * v) * radius * ((i + 0.4) / 28) ** 0.5
            )
            beads.extend(q + points + normal * 0.00035)
    mesh.GetPointsAttr().Set([Gf.Vec3f(*map(float, bead)) for bead in beads])
    mesh.GetFaceVertexCountsAttr().Set([6] * num)
    mesh.GetFaceVertexIndicesAttr().Set(list(range(len(beads))))


def update(stage, root, state, materials, metrics):
    from pxr import Gf, UsdGeom

    fixture = root + "/fixture"
    for name, bottom, volume, radius in (
        ("Filtrate", C.TABLE + 0.018, sum(materials.flask.values()), 0.042),
        ("FunnelPool", C.PLATE + 0.00075, sum(materials.funnel.values()), 0.0365),
    ):
        shape = UsdGeom.Cylinder.Get(stage, f"{fixture}/{name}")
        height = max(0.00001, volume * 1e-6 / (pi * radius**2))
        shape.GetRadiusAttr().Set(radius)
        shape.GetHeightAttr().Set(height)
        shape.GetOrderedXformOps()[0].Set(
            Gf.Vec3d(*map(float, [*C.FUNNEL, bottom + height / 2]))
        )
        shape.MakeVisible() if volume > 0.28 else shape.MakeInvisible()
    disk = UsdGeom.Cylinder.Get(stage, root + "/paper/FilterDisk")
    wet = min(1.0, metrics["wetting_ml"] / 6.0)
    disk.GetDisplayColorAttr().Set(
        [Gf.Vec3f(0.86 - 0.17 * wet, 0.89 - 0.03 * wet, 0.83 + 0.09 * wet)]
    )
    for label, amount in (
        ("Bead", materials.retained),
        ("Escaped", materials.bypassed),
    ):
        for i in range(36):
            bead = UsdGeom.Cylinder.Get(stage, f"{fixture}/{label}{i}")
            bead.MakeVisible() if round(amount) > i else bead.MakeInvisible()
    # Clip a polygonal cylindrical interior at the horizontal water level.
    for vessel in C.VESSELS:
        name = vessel.name
        source = materials.sources[name]
        path = f"{root}/{name}/WaterSurface"
        if source.ml < 0.015:
            _surface(stage, path, [])
            if name == C.SUSPENSION.name:
                _show_feed_beads(
                    stage, f"{root}/{name}/SuspendedBeads", np.eye(3), [], 0.0
                )
            continue
        _, axes = pose_parts(state["objects"][name]["pose"])
        phi = np.arange(30) * 2 * pi / 30
        xy = source.radius * np.column_stack((np.cos(phi), np.sin(phi)))
        low = np.column_stack((xy, np.full(len(xy), 0.005)))
        high = np.column_stack((xy, np.full(len(xy), vessel.height)))
        edges = (
            list(zip(low, high))
            + list(zip(low, np.roll(low, 1, axis=0)))
            + list(zip(high, np.roll(high, 1, axis=0)))
        )
        points = []
        for a, b in edges:
            za, zb = a @ axes[2], b @ axes[2]
            if (za - source.level) * (zb - source.level) < 0:
                points.append(a + (b - a) * (source.level - za) / (zb - za))
        if len(points) >= 3:
            points = np.asarray(points)
            delta = (points - points.mean(axis=0)) @ axes.T
            points = points[np.argsort(np.arctan2(delta[:, 1], delta[:, 0]))]
        _surface(stage, path, points)
        if name == C.SUSPENSION.name:
            _show_feed_beads(
                stage,
                f"{root}/{name}/SuspendedBeads",
                axes,
                points,
                source.ml / vessel.fill_ml,
            )
    for i in range(15):
        if i < len(materials.paths):
            a, b = materials.paths[i]
            x = np.array([0.0018, 0.0, 0.0])
            _surface(stage, f"{fixture}/Trace{i}", (a - x, a + x, b + x, b - x))
        else:
            _surface(stage, f"{fixture}/Trace{i}", [])
