"""Distributed snap-gasket forces and conservative gravity-driven flow."""

from __future__ import annotations

from dataclasses import dataclass
from math import pi, sqrt

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C

GRAVITY = 9.81


def parts(pose):
    return np.asarray(pose[:3]), quat_xyzw_to_matrix(np.asarray(pose[3:]))


class SnapGasket:
    """Four compressible rim sectors load and retain the separate native lid."""

    def __init__(self):
        self.engaged = np.zeros(4, dtype=bool)
        self.minimum_gap = float("inf")
        self.maximum_force = 0.0
        self.gaps = np.zeros(4)

    def step(self, can_pose, lid_pose, can_velocity, lid_velocity):
        can, canopy = parts(can_pose)
        lid, cover = parts(lid_pose)
        z = canopy[:, 2]
        can_points = can + (C.CLIP_POINTS + C.LID_SEAT) @ canopy.T
        lid_points = lid + C.CLIP_POINTS @ cover.T
        offset = lid_points - can_points
        # The circular brim can accept a yaw rotation without breaking its seal.
        # Pair points by the lid's physical projected radial bearing.
        delta = lid - can
        lateral = delta - z * (delta @ z)
        centered = (
            np.linalg.norm(lateral) < (0.0095 if self.engaged.any() else 0.007)
            and z @ cover[:, 2] > 0.96
        )
        self.gaps = offset @ z
        if centered:
            self.minimum_gap = min(self.minimum_gap, float(np.min(self.gaps)))
        else:
            self.engaged[:] = False
        self.engaged &= self.gaps < 0.014
        self.engaged |= centered & (self.gaps <= 0.00065) & (self.gaps > -0.004)
        if not centered:
            return np.zeros(3), np.zeros(3), np.zeros(3)
        can_v = np.asarray(can_velocity)
        lid_v = np.asarray(lid_velocity)
        rel = (
            lid_v[:3]
            + np.cross(lid_v[3:], lid_points - lid)
            - can_v[:3]
            - np.cross(can_v[3:], can_points - can)
        )
        axial_speed = rel @ z
        load = np.zeros(4)
        for i, gap in enumerate(self.gaps):
            if self.engaged[i]:
                load[i] = np.clip(-1650 * gap - 11 * axial_speed[i], -14.0, 0.0)
            elif 0.00065 < gap < 0.0045:
                travel = np.clip((0.0045 - gap) / 0.00385, 0, 1)
                load[i] = 0.6 * travel - 0.8 * min(0.0, axial_speed[i])
        forces = load[:, None] * z
        if self.engaged.any():
            tangent_speed = (lid_v[:3] - can_v[:3]) - z * ((lid_v[:3] - can_v[:3]) @ z)
            seated_fraction = float(self.engaged.mean())
            shear = seated_fraction * (-2300 * lateral - 17.0 * tangent_speed)
            norm = np.linalg.norm(shear)
            if norm > 17:
                shear *= 17 / norm
            forces[self.engaged] += shear / self.engaged.sum()
        self.maximum_force = max(
            self.maximum_force, float(np.linalg.norm(forces, axis=1).sum())
        )
        # The brass/steel wall collision bears compression; these finite forces
        # represent the compliant gasket lip. They never reposition a body.
        lid_moment = np.cross(lid_points - (lid + 0.004 * cover[:, 2]), forces).sum(0)
        can_moment = np.cross(can_points - (can + 0.062 * z), -forces).sum(0)
        return forces.sum(0), lid_moment, can_moment

    @property
    def fitted(self):
        return bool(self.engaged.all() and np.max(self.gaps) < 0.003)


class FreeSurface:
    """Circular-section quadrature with a level defined in world coordinates."""

    def __init__(self):
        q, self.weights = np.polynomial.legendre.leggauss(28)
        length = C.HEIGHT - C.FLOOR
        self.z = C.FLOOR + length / 2 * (q + 1)
        self.weights *= length / 2

    def volume(self, rotation, height):
        r = C.INNER_RADIUS
        slope = np.linalg.norm(rotation[2, :2])
        c = rotation[2, 2]
        if slope < 1e-7:
            fullness = np.clip((height / c - C.FLOOR) / (C.HEIGHT - C.FLOOR), 0, 1)
            if c < 0:
                fullness = 1 - fullness
            return float(fullness * pi * r * r * (C.HEIGHT - C.FLOOR) * 1e6)
        cutoff = np.clip((height - self.z * c) / (r * slope), -1.0, 1.0)
        segment = r * r * (np.arccos(-cutoff) + cutoff * np.sqrt(1 - cutoff * cutoff))
        return float(self.weights @ segment * 1e6)

    def level(self, rotation, volume_ml):
        if rotation[2, 2] > 0.999999:
            return C.FLOOR + volume_ml * 1e-6 / (pi * C.INNER_RADIUS**2)
        low, high = -0.222, 0.222
        for _ in range(21):
            mid = (low + high) / 2
            if self.volume(rotation, mid) < volume_ml:
                low = mid
            else:
                high = mid
        return (low + high) / 2


@dataclass
class Packet:
    position: np.ndarray
    velocity: np.ndarray
    volume_ml: float
    through_spout: bool


class PaintFlow:
    def __init__(self):
        self.free_surface = FreeSurface()
        self.source = C.VOLUME_ML
        self.tray = 0.0
        self.through_spout = 0.0
        self.spill = 0.0
        self.rim_bypass = 0.0
        self.packets = []
        self.segments = []
        self.spill_points = []
        self.level = 0.0

    def _emit(self, amount, origin, velocity, through_spout):
        if amount <= 1e-5:
            return
        self.source -= amount
        self.packets.append(Packet(origin, velocity, amount, through_spout))
        if not through_spout:
            self.rim_bypass += amount

    def _flow(self, state, gasket, dt):
        can, rotation = parts(state["objects"]["can"]["pose"])
        if self.source <= 0.005:
            self.level = 0.0
            return
        self.level = self.free_surface.level(rotation, self.source)
        can_velocity = np.asarray(state["objects"]["can"]["velocity"])[:3]
        lid, lid_rotation = parts(state["objects"]["lid"]["pose"])
        lip_delta = lid - (can + rotation @ C.LID_SEAT)
        on_rim = (
            np.linalg.norm(lip_delta) <= 0.012
            and rotation[:, 2] @ lid_rotation[:, 2] > 0.94
        )
        if on_rim:
            mouth = lid + lid_rotation @ C.SPOUT_MOUTH
            inlet = can + rotation @ C.SPOUT_INLET
            sill = max(mouth[2], inlet[2]) - can[2]
            head = max(0.0, self.level - sill)
            velocity = can_velocity + (lid_rotation @ C.SPOUT_AXIS) * min(
                0.48, 0.55 * sqrt(2 * GRAVITY * head)
            )
            rate = min(160, 54000 * head**1.5)
            self._emit(min(self.source, rate * dt), mouth, velocity, True)
        if not gasket.fitted:
            points = np.column_stack(
                (
                    0.079 * np.cos(C.RIM_ANGLES),
                    0.079 * np.sin(C.RIM_ANGLES),
                    np.full(16, C.HEIGHT),
                )
            )
            rim = points @ rotation.T
            index = int(np.argmin(rim[:, 2]))
            head = max(0.0, self.level - rim[index, 2])
            direction = rotation @ np.array(
                [np.cos(C.RIM_ANGLES[index]), np.sin(C.RIM_ANGLES[index]), 0.25]
            )
            fraction = (
                max(0.16, 1.0 - np.count_nonzero(gasket.engaged) / 4) if on_rim else 1.0
            )
            self._emit(
                min(self.source, fraction * min(200, 3.5e4 * head**1.5) * dt),
                can + rim[index],
                can_velocity + direction * min(0.33, sqrt(2 * GRAVITY * head)),
                False,
            )

    def advance(self, state, gasket, dt):
        self.segments = []
        active = []
        for packet in self.packets:
            old = packet.position
            new = (
                old
                + packet.velocity * dt
                + np.array([0.0, 0.0, -0.5 * GRAVITY * dt**2])
            )
            plane = C.WELL_RIM if old[2] >= C.WELL_RIM else C.WELL_FLOOR
            self.segments.append((old, new))
            if new[2] >= plane:
                packet.position = new
                packet.velocity = packet.velocity + np.array([0.0, 0.0, -GRAVITY * dt])
                active.append(packet)
                continue
            fraction = np.clip((old[2] - plane) / (old[2] - new[2]), 0.0, 1.0)
            point = old + fraction * (new - old)
            x, y, _ = point
            if (
                C.WELL_X[0] + 0.004 <= x <= C.WELL_X[1] - 0.004
                and C.WELL_Y[0] + 0.004 <= y <= C.WELL_Y[1] - 0.004
            ):
                self.tray += packet.volume_ml
                self.through_spout += packet.volume_ml if packet.through_spout else 0.0
            else:
                self.spill += packet.volume_ml
                self.spill_points.append(point)
        self.packets = active
        self._flow(state, gasket, dt)

    def metrics(self):
        airborne = sum(p.volume_ml for p in self.packets)
        return {
            "remaining_ml": self.source,
            "tray_ml": self.tray,
            "through_spout_ml": self.through_spout,
            "spilled_ml": self.spill,
            "bypass_ml": self.rim_bypass,
            "airborne_ml": airborne,
            "mass_balance_ml": self.source + self.tray + self.spill + airborne,
            "free_surface_m": self.level,
        }
