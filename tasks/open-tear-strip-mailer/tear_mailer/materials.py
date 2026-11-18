"""A tension-driven tear front in reinforced paperboard."""

import numpy as np

from . import config as C

STIFFNESS = 410.
TEAR_FORCE = 4.0
MAX_FORCE = 11.
FOLD_STIFFNESS = 80.
FOLD_SLACK = .003


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
        self.free_time = 0.
        self.free_direction = None
        self.tab_rotation = np.eye(3)

    def advance(self, dt, origin, rotation, tab, tab_rotation):
        self.start = origin + rotation @ C.FIRST
        self.end = self.start + rotation[:, 1] * C.TEAR_LENGTH
        self.front = self.start + rotation[:, 1] * self.progress
        self.tab = tab
        self.up = rotation[:, 2]
        self.tab_rotation = tab_rotation
        fold_offset = tab - (self.start + rotation @ (C.TAB - C.BOX - C.FIRST))
        delta = tab - self.front
        distance = float(np.linalg.norm(delta))
        if distance < 1e-5 or self.progress >= C.TEAR_LENGTH:
            if self.progress >= C.TEAR_LENGTH:
                self.free_time += dt
            self.tension = 0.
            return np.zeros(3)
        direction = delta / distance
        extension = distance - (C.LEADER + FOLD_SLACK + self.progress)
        # Distance stores elastic work. A pulled-off-axis strip drives fracture,
        # and the newly released length unloads that same tension.
        self.tension = float(np.clip(STIFFNESS * extension, 0., MAX_FORCE))
        self.peak_tension = max(self.peak_tension, self.tension)
        outward = float(np.linalg.norm(direction - rotation[:, 1] * (direction @ rotation[:, 1])))
        if self.tension > TEAR_FORCE and outward > .20:
            delta_length = min(.45, .14 * (self.tension - TEAR_FORCE)) * dt
            self.progress = min(C.TEAR_LENGTH, self.progress + delta_length)
            if self.progress >= C.TEAR_LENGTH:
                self.free_direction = -tab_rotation.T @ direction
        force = -self.tension * direction
        if self.progress < .008:
            # The doubled-over end of the strip bears the tab's weight and
            # resists bending until the advancing tear frees that corner.
            bending = -FOLD_STIFFNESS * fold_offset
            bending[2] += C.TAB_MASS * 9.81
            strength = np.linalg.norm(bending)
            bending *= min(1., 2. / max(strength, 1e-6))
            force += (1. - self.progress / .008) * bending
        return force

    def snapshot(self):
        return dict(progress_m=self.progress, tension_n=self.tension,
                    peak_tension_n=self.peak_tension)

    def _detached_panels(self):
        # Draw the compliant backing curling behind its native leading piece.
        direction = self.tab_rotation @ self.free_direction
        forward = direction.copy()
        forward[2] = 0.
        forward /= max(np.linalg.norm(forward), 1e-6)
        side = np.cross([0., 0., 1.], forward)
        curl = min(1., self.free_time / .38)
        count = 12
        length = (C.LEADER + C.TEAR_LENGTH) / count
        start = self.tab
        for i in range(count):
            theta = 4.4 * curl * i / (count - 1)
            tangent = (
                (1. - curl) * direction
                + curl * (np.cos(theta) * forward + np.sin(theta) * side - [0., 0., .24])
            )
            tangent /= max(np.linalg.norm(tangent), 1e-6)
            end = start + tangent * length
            end[2] = max(C.TABLE + .0006, end[2])
            across = np.cross([0., 0., 1.], tangent)
            across /= max(np.linalg.norm(across), 1e-6)
            yield start, end, (1. - curl) * self.up + curl * across
            start = end

    def render(self, stage):
        from pxr import Gf, UsdGeom

        mesh = UsdGeom.Mesh.Define(stage, "/World/TearTape")
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([Gf.Vec3f(.84, .16, .07)])
        points = []
        panels = (self._detached_panels() if self.free_direction is not None else
                  [(self.front, self.end, self.up), (self.tab, self.front, self.up)])
        for a, b, across in panels:
            half = across * C.SEAM_WIDTH * .47
            points.extend([a - half, a + half, b + half, b - half])
        mesh.CreatePointsAttr().Set([Gf.Vec3f(*map(float, p)) for p in points])
        mesh.CreateFaceVertexCountsAttr().Set([4] * (len(points) // 4))
        mesh.CreateFaceVertexIndicesAttr().Set(list(range(len(points))))
