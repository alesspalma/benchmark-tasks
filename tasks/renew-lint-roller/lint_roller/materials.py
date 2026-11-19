"""Compliant sheet release and contact-driven lint transfer."""

import math

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C

PEEL_FORCE = 0.85
TAPE_STIFFNESS = 230.
FOLD_STIFFNESS = 22.
NAP_STIFFNESS = 1300.
LOAD_REQUIRED = .8
SHEET = (.91, .80, .56)
DIRT = (.89, .76, .57)


class Tape:
    def __init__(self):
        self.reset()

    def reset(self):
        self.progress = 0.
        self.tension = 0.
        self.peak_tension = 0.
        self.peak_contact = 0.
        self.roll_angle = 0.
        self.travel = 0.
        self.previous = None
        self.roller = C.ROLLER.copy()
        self.rotation = np.eye(3)
        self.tab = C.TAB.copy()
        self.tab_rotation = np.eye(3)
        self.front = C.ROLLER + C.SEAM
        self.cloth = C.CLOTH.copy()
        self.cloth_rotation = np.eye(3)
        self.captured = {}
        self.load = 0.

    def radial(self, radians):
        return self.rotation @ np.array([math.sin(radians), 0., math.cos(radians)])

    def advance(self, dt, hub, edge, cloth, angle):
        self.roller, self.rotation = hub[:3], quat_xyzw_to_matrix(hub[3:])
        self.tab, self.tab_rotation = edge[:3], quat_xyzw_to_matrix(edge[3:])
        self.cloth, self.cloth_rotation = cloth[:3], quat_xyzw_to_matrix(cloth[3:])
        self.roll_angle = angle
        released = self.progress >= C.SHEET_LENGTH
        self.load = 0.
        if released:
            self.tension = 0.
            lift = min(3.0, NAP_STIFFNESS * max(0., self.interference()))
            self.load = lift
            self.peak_contact = max(self.peak_contact, lift)
            self.collect(angle)
            # Gravity acts on the removed flexible sheet through its leading edge.
            return np.array([0., 0., -C.TAB_MASS * 9.81]), lift

        self.front = (
            self.roller + self.rotation[:, 1] * C.TAB_Y
            + C.RADIUS * self.radial(self.progress / C.RADIUS)
        )
        delta = self.tab - self.front
        length = float(np.linalg.norm(delta))
        direction = delta / max(length, 1e-7)
        leader = float(np.linalg.norm(C.TAB_OFFSET - np.array([0., C.TAB_Y, C.RADIUS])))
        stretch = length - leader - .004 - self.progress
        self.tension = float(np.clip(TAPE_STIFFNESS * stretch, 0., 5.5))
        self.peak_tension = max(self.tension, self.peak_tension)
        peel_angle = float(direction @ self.radial(self.progress / C.RADIUS))
        if self.tension >= PEEL_FORCE and peel_angle >= -.30:
            speed = min(.30, .24 * (self.tension - PEEL_FORCE))
            self.progress = min(C.SHEET_LENGTH, self.progress + speed * dt)
        fold_offset = self.tab - (self.roller + self.rotation @ C.TAB_OFFSET)
        bend = -FOLD_STIFFNESS * fold_offset
        bend *= min(1., 1.2 / max(np.linalg.norm(bend), .00001))
        fold = max(0., 1. - self.progress / .010)
        return -self.tension * direction + fold * bend, 0.

    def interference(self):
        axis = self.rotation[:, 1]
        normal = self.cloth_rotation[:, 2]
        if abs(axis @ normal) > .11:
            return 0.
        offset = self.cloth_rotation.T @ (self.roller - self.cloth)
        if abs(offset[0]) > C.CLOTH_SIZE[0] / 2 + .010 or abs(offset[1]) > C.CLOTH_SIZE[1] / 2:
            return 0.
        return C.RADIUS + C.PILE + C.CLOTH_SIZE[2] / 2 - offset[2]

    def collect(self, angle):
        current = self.roller - self.cloth, angle
        previous, self.previous = self.previous, current
        if previous is None or self.load < LOAD_REQUIRED:
            return
        tangent = np.cross(self.rotation[:, 1], self.cloth_rotation[:, 2])
        distance = abs(float((current[0] - previous[0]) @ tangent))
        roll = C.RADIUS * abs(math.remainder(angle - previous[1], math.tau))
        if distance < .00001 or not .43 <= roll / distance <= 1.85:
            return
        self.travel += distance
        for number, local in enumerate(C.DIRT):
            if number in self.captured:
                continue
            fiber = self.cloth + self.cloth_rotation @ local
            local = self.rotation.T @ (fiber - self.roller)
            if abs(local[1]) < C.WIDTH / 2 - .002 and abs((fiber - self.roller) @ tangent) < .006:
                local[[0, 2]] *= (C.RADIUS + .00115) / max(.0001, np.linalg.norm(local[[0, 2]]))
                self.captured[number] = local

    def snapshot(self):
        return dict(peeled_m=self.progress, tension_n=self.tension,
                    peak_tension_n=self.peak_tension, contact_n=self.load,
                    peak_contact_n=self.peak_contact, rolling_m=self.travel,
                    collected=len(self.captured), total=len(C.DIRT))

    def sheet_panels(self):
        axis = self.rotation[:, 1] * C.WIDTH / 2
        if self.progress < C.SHEET_LENGTH:
            angles = np.linspace(self.progress / C.RADIUS, math.tau, 33)
            seam = self.roller + (C.RADIUS + .0005) * self.radial(angles[0])
            yield self.tab - self.rotation[:, 1] * C.TAB_Y, seam, axis
            for a, b in zip(angles, angles[1:]):
                yield (
                    self.roller + (C.RADIUS + .0005) * self.radial(a),
                    self.roller + (C.RADIUS + .0005) * self.radial(b), axis,
                )
            return
        start = self.tab - self.rotation[:, 1] * C.TAB_Y * .45
        across = self.tab_rotation[:, 1] * C.WIDTH / 2
        for i in range(20):
            angle = i * .26
            direction = self.tab_rotation @ np.array([math.cos(angle), 0., -.75 + .45 * math.sin(angle)])
            end = start + direction * C.SHEET_LENGTH / 20
            end[2] = max(C.TABLE + .001, end[2])
            yield start, end, across
            start = end

    def render(self, stage):
        from pxr import Gf, UsdGeom

        mesh = UsdGeom.Mesh.Define(stage, "/World/RollerSheet")
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorAttr([Gf.Vec3f(*SHEET)])
        points = []
        for a, b, axis in self.sheet_panels():
            points.extend([a-axis, a+axis, b+axis, b-axis])
        mesh.CreatePointsAttr().Set([Gf.Vec3f(*p) for p in points])
        mesh.CreateFaceVertexCountsAttr().Set([4] * (len(points) // 4))
        mesh.CreateFaceVertexIndicesAttr().Set(list(range(len(points))))
        for i, local in enumerate(C.DIRT):
            if i in self.captured:
                point = self.roller + self.rotation @ self.captured[i]
            else:
                point = self.cloth + self.cloth_rotation @ local
            speck = UsdGeom.Sphere.Define(stage, f"/World/Lint/f{i:02}")
            speck.CreateRadiusAttr(.0024 if i % 3 else .0031)
            speck.CreateDisplayColorAttr([Gf.Vec3f(*DIRT)])
            attr = speck.GetPrim().GetAttribute("xformOp:translate")
            if not attr:
                attr = speck.AddTranslateOp().GetAttr()
            attr.Set(Gf.Vec3d(*point))
