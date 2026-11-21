"""Paint geometry follows the simulated level, flight and collected volume."""

from __future__ import annotations

from math import pi

import numpy as np

from . import config as C
from .mechanics import parts


class PaintVisuals:
    def __init__(self, stage, path):
        from pxr import Gf, UsdGeom

        self.Gf = Gf
        self.meshes = {
            name: UsdGeom.Mesh.Get(stage, f"{path}/{name}")
            for name in ("InsidePaint", "TrayPaint", "Stream", "Spill")
        }
        t = np.arange(40) * pi / 20
        xy = np.column_stack(
            (C.INNER_RADIUS * 0.997 * np.cos(t), C.INNER_RADIUS * 0.997 * np.sin(t))
        )
        self.floor = np.column_stack((xy, np.full(len(xy), C.FLOOR)))
        self.ceiling = np.column_stack((xy, np.full(len(xy), C.HEIGHT)))

    def _write(self, name, points, faces):
        mesh = self.meshes[name]
        mesh.GetPointsAttr().Set(
            [self.Gf.Vec3f(*map(float, point)) for point in points]
        )
        mesh.GetFaceVertexCountsAttr().Set([len(face) for face in faces])
        mesh.GetFaceVertexIndicesAttr().Set([i for face in faces for i in face])

    def update(self, state, paint):
        can, rotation = parts(state["objects"]["can"]["pose"])
        low = can + self.floor @ rotation.T
        high = can + self.ceiling @ rotation.T
        height = can[2] + paint.level
        pairs = (
            list(zip(low, high))
            + list(zip(low, np.roll(low, 1, axis=0)))
            + list(zip(high, np.roll(high, 1, axis=0)))
        )
        points = [
            a + (b - a) * (height - a[2]) / (b[2] - a[2])
            for a, b in pairs
            if (a[2] <= height < b[2]) or (b[2] <= height < a[2])
        ]
        if len(points) > 2 and paint.source > 0.005:
            points = np.asarray(points)
            delta = points - points.mean(0)
            order = np.argsort(np.arctan2(delta[:, 1], delta[:, 0]))
            self._write(
                "InsidePaint", points[order].tolist(), [list(range(len(points)))]
            )
        else:
            self._write("InsidePaint", [], [])
        xmin, xmax = C.WELL_X
        ymin, ymax = C.WELL_Y
        if paint.tray > 0.1:
            z = C.WELL_FLOOR + paint.tray * 1e-6 / ((xmax - xmin) * (ymax - ymin))
            self._write(
                "TrayPaint",
                [(xmin, ymin, z), (xmax, ymin, z), (xmax, ymax, z), (xmin, ymax, z)],
                [(0, 1, 2, 3)],
            )
        else:
            self._write("TrayPaint", [], [])
        points, faces = [], []
        phi = np.arange(8) * pi / 4
        for a, b in paint.segments:
            tangent = b - a
            length = np.linalg.norm(tangent)
            if length < 0.0002:
                continue
            tangent /= length
            u = np.cross(tangent, [1.0, 0.0, 0.0])
            if np.linalg.norm(u) < 0.001:
                u = np.cross(tangent, [0.0, 1.0, 0.0])
            u /= np.linalg.norm(u)
            v = np.cross(tangent, u)
            radial = 0.0033 * (np.cos(phi)[:, None] * u + np.sin(phi)[:, None] * v)
            offset = len(points)
            points.extend((a + radial).tolist() + (b + radial).tolist())
            faces.extend(
                (
                    offset + k,
                    offset + (k + 1) % 8,
                    offset + 8 + (k + 1) % 8,
                    offset + 8 + k,
                )
                for k in range(8)
            )
        self._write("Stream", points, faces)
        points, faces = [], []
        for p in paint.spill_points[::3]:
            offset = len(points)
            points.extend(
                (p[0] + 0.009 * np.cos(a), p[1] + 0.009 * np.sin(a), C.TABLE + 0.001)
                for a in phi
            )
            faces.append(list(range(offset, offset + 8)))
        self._write("Spill", points, faces)
