"""A tension-driven tear front in reinforced paperboard."""

import numpy as np

from . import config as C

STIFFNESS = 410.
TEAR_FORCE = 4.0
MAX_FORCE = 11.


class Tape:
    def __init__(self):
        self.reset()

    def reset(self):
        self.progress = 0.
        self.tension = 0.
        self.peak_tension = 0.
        self.front = C.BOX + C.FIRST
        self.start = self.front.copy()
        self.end = self.front + [0, C.TEAR_LENGTH, 0]
        self.tab = C.TAB.copy()
        self.up = np.array([0., 0., 1.])
        self.last_length = None

    def advance(self, dt, origin, rotation, tab):
        self.start = origin + rotation @ C.FIRST
        self.end = self.start + rotation[:, 1] * C.TEAR_LENGTH
        self.front = self.start + rotation[:, 1] * self.progress
        self.tab = tab
        self.up = rotation[:, 2]
        delta = tab - self.front
        distance = float(np.linalg.norm(delta))
        if distance < 1e-5 or self.progress >= C.TEAR_LENGTH:
            self.tension = 0.
            return np.zeros(3)
        direction = delta / distance
        extension = distance - (C.LEADER + self.progress)
        rate = 0. if self.last_length is None else (distance - self.last_length) / dt
        self.last_length = distance
        # Distance stores elastic work. A pulled-off-axis strip drives fracture,
        # and the newly released length unloads that same tension.
        self.tension = float(np.clip(STIFFNESS * extension + .10 * max(rate, 0.), 0., MAX_FORCE))
        self.peak_tension = max(self.peak_tension, self.tension)
        outward = float(np.linalg.norm(direction - rotation[:, 1] * (direction @ rotation[:, 1])))
        if self.tension > TEAR_FORCE and outward > .20:
            delta_length = min(.45, .14 * (self.tension - TEAR_FORCE)) * dt
            self.progress = min(C.TEAR_LENGTH, self.progress + delta_length)
        force = -self.tension * direction
        if self.progress < .001:
            original = self.start + rotation @ (C.TAB - C.BOX - C.FIRST)
            # The folded cardboard leader supports the tag near the seam.
            bending = (original - tab) * 285.
            force += np.clip(bending, -4., 4.)
        return force

    def snapshot(self):
        return dict(progress_m=self.progress, tension_n=self.tension,
                    peak_tension_n=self.peak_tension)

    def render(self, stage):
        from pxr import Gf, UsdGeom

        mesh = UsdGeom.Mesh.Define(stage, "/World/TearTape")
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([Gf.Vec3f(.84, .16, .07)])
        points = []
        for a, b in [(self.front, self.end), (self.tab, self.front)]:
            if self.progress >= C.TEAR_LENGTH:
                break
            half = self.up * C.SEAM_WIDTH * .47
            points.extend([a - half, a + half, b + half, b - half])
        mesh.CreatePointsAttr().Set([Gf.Vec3f(*map(float, p)) for p in points])
        mesh.CreateFaceVertexCountsAttr().Set([4] * (len(points) // 4))
        mesh.CreateFaceVertexIndicesAttr().Set(list(range(len(points))))
