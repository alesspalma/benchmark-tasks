"""Conservative flow and free surfaces for the one open tapered glass.

The faucet emits continuously at a rate set by its native joint angle. Gravity
defines the stream and free surfaces. Foam contains the same beverage mass as
the captured liquid, plus massless gas that expands its occupied volume.
"""
import math

import numpy as np

from . import config as C


def radius(z):
    fraction = (z - C.FLOOR) / (C.HEIGHT - C.FLOOR)
    return C.R_BASE + fraction * (C.R_LIP - C.R_BASE)


class GlassVolume:
    def __init__(self):
        dz = (C.HEIGHT - C.FLOOR) / 128
        self.heights = C.FLOOR + (np.arange(128) + .5) * dz
        self.radii = radius(self.heights)
        self.slices_ml = math.pi * self.radii**2 * dz * 1e6
        self.capacity_ml = float(self.slices_ml.sum())

    def volume(self, up, level):
        lateral = np.linalg.norm(up[:2])
        cut = level - self.heights * up[2]
        if lateral < 1e-6:
            local_height = level / up[2]
            slices = np.clip((local_height - self.heights) / (self.heights[1] - self.heights[0]) + .5, 0., 1.)
            if up[2] < 0:
                slices = 1. - slices
            return float(slices @ self.slices_ml)
        x = np.clip(cut / (lateral * self.radii), -1., 1.)
        fractions = .5 + (np.arcsin(x) + x * np.sqrt(1. - x*x)) / math.pi
        return float(fractions @ self.slices_ml)

    def rim_level(self, up):
        return float(C.HEIGHT * up[2] - C.R_LIP * np.linalg.norm(up[:2]))

    def level(self, up, volume_ml):
        low, high = -.20, .20
        for _ in range(19):
            mid = (low + high) / 2.
            if self.volume(up, mid) < volume_ml:
                low = mid
            else:
                high = mid
        return (low + high) / 2.

    def properties(self, up, beer_ml, foam_ml, liquid_level, total_level):
        # Numerical moments of clipped conical volume for native loading.
        # Liquid moves within the vessel; the handle and its grip are never moved here.
        n = 18
        hz = C.FLOOR + (np.arange(n) + .5) * (C.HEIGHT - C.FLOOR) / n
        azimuth = np.arange(24) * math.tau / 24
        fr = np.sqrt((np.arange(6) + .5) / 6)
        radii = radius(hz)[:, None, None] * fr[None, :, None]
        x = radii * np.cos(azimuth)[None, None, :]
        y = radii * np.sin(azimuth)[None, None, :]
        points = np.stack([x, y, np.broadcast_to(hz[:, None, None], x.shape)], -1).reshape(-1, 3)
        z = points @ up
        weights = np.zeros(len(points))
        for mask, volume in ((z <= liquid_level, beer_ml), ((z > liquid_level) & (z <= total_level), foam_ml)):
            if volume and np.any(mask):
                weights[mask] = volume * .001 / np.count_nonzero(mask)
        load = (beer_ml + foam_ml) * .001
        if weights.sum() > 0:
            weights *= load / weights.sum()
        elif load:
            weights[0] = load
        mass = C.DRY_MASS + load
        com = (C.DRY_MASS * C.DRY_COM + weights @ points) / mass
        dr = points - com
        d = C.DRY_COM - com
        second = (dr * weights[:, None]).T @ dr + C.DRY_MASS * np.outer(d, d)
        return mass, com, C.DRY_INERTIA + np.eye(3) * np.trace(second) - second

    def mesh(self, up, level, lower=None):
        from scipy.spatial import ConvexHull
        n = 36
        angles = np.arange(n) * math.tau / n
        points = np.array([[radius(z)*math.cos(a), radius(z)*math.sin(a), z]
                           for z in (C.FLOOR, C.HEIGHT) for a in angles])
        edges = [(i, (i + 1) % n) for i in range(n)]
        edges += [(n + i, n + (i + 1) % n) for i in range(n)] + [(i, n + i) for i in range(n)]
        q = points @ up
        keep = q <= level
        if lower is not None:
            keep &= q >= lower
        clipped = list(points[keep])
        for cut in ((level,) if lower is None else (lower, level)):
            for i, j in edges:
                if (q[i] > cut) != (q[j] > cut):
                    clipped.append(points[i] + (points[j] - points[i]) * (cut - q[i]) / (q[j] - q[i]))
        if len(clipped) < 4:
            return [], []
        hull = ConvexHull(clipped)
        pts = hull.points
        center = np.mean(pts, axis=0)
        triangles = []
        for face in hull.simplices:
            a, b, c = pts[face]
            if np.dot(np.cross(b-a, c-a), a-center) < 0:
                face = face[::-1]
            triangles.append(face.tolist())
        return pts.tolist(), triangles


def stream_path(position, rotation, liquid_level):
    """Ray contact with the real opening, then the wall, pool or base."""
    origin = rotation.T @ (C.SPOUT - position)
    direction = -rotation[2, :]
    if direction[2] >= -1e-6:
        return None
    opening = (C.HEIGHT - origin[2]) / direction[2]
    if opening < 0:
        return None
    rim_point = origin + opening * direction
    if np.linalg.norm(rim_point[:2]) >= C.R_LIP:
        return None
    to_floor = (C.FLOOR - origin[2]) / direction[2]
    if liquid_level is not None:
        to_pool = C.SPOUT[2] - position[2] - liquid_level
        drop = min(to_floor, max(opening, to_pool))
    else:
        drop = to_floor
    bottom = origin + drop * direction
    wall_flow = np.linalg.norm(bottom[:2]) > radius(bottom[2]) + 1e-6
    if wall_flow:
        lo, hi = opening, drop
        for _ in range(14):
            mid = (lo + hi) / 2
            p = origin + mid * direction
            if np.linalg.norm(p[:2]) > radius(p[2]):
                hi = mid
            else:
                lo = mid
        drop = (lo + hi) / 2
    impact = C.SPOUT - np.array([0., 0., drop])
    return impact, wall_flow


class MaterialModel:
    def __init__(self):
        self.shape = GlassVolume()
        self.reset()

    def reset(self):
        self.liquid_ml = self.foam_ml = self.spill_ml = self.dispensed_ml = 0.
        self.liquid_level = self.total_level = C.FLOOR
        self.last_stream = None
        self.last_flow_ml_s = 0.
        self.wall_capture_ml = self.direct_capture_ml = 0.

    def advance(self, dt, position, rotation, valve, hands):
        from .geometry import segment_robot_hit

        up = rotation[2, :]
        self.last_stream = None
        self.last_flow_ml_s = 0.
        collapsed = self.foam_ml * (1. - math.exp(-dt / 50.))
        self.foam_ml -= collapsed
        self.liquid_ml += collapsed
        rate = C.FULL_FLOW_ML_S * np.clip((valve - .055) / (C.TAP_ANGLE - .055), 0., 1.)
        volume = float(dt * rate)
        if volume:
            self.dispensed_ml += volume
            self.last_flow_ml_s = float(rate)
            path = stream_path(position, rotation, self.shape.level(up, self.liquid_ml) if self.liquid_ml > .1 else None)
            end = path[0] if path is not None else np.array([*C.SPOUT[:2], C.TOP + .015])
            hit = segment_robot_hit(C.SPOUT, end, hands)
            if hit is not None:
                end = C.SPOUT + hit * (end - C.SPOUT)
            self.last_stream = end
            if path is None or hit is not None:
                self.spill_ml += volume
            else:
                wall_flow = path[1]
                # More aeration on direct impact and at a throttled faucet.
                # These two coefficients are engineering estimates using the BA
                # qualitative guidance, not measurements of a specific lager.
                ratio = (.026 if wall_flow else .151) + .14 * (1. - rate / C.FULL_FLOW_ML_S)
                self.foam_ml += volume * ratio
                self.liquid_ml += volume * (1. - ratio)
                self.wall_capture_ml += volume if wall_flow else 0.
                self.direct_capture_ml += 0. if wall_flow else volume

        surface_capacity = self.shape.volume(up, self.shape.rim_level(up))
        excess = max(0., self.liquid_ml + C.FOAM_EXPANSION * self.foam_ml - surface_capacity)
        foam_spill = min(self.foam_ml, excess / C.FOAM_EXPANSION)
        self.foam_ml -= foam_spill
        liquid_spill = max(0., excess - C.FOAM_EXPANSION * foam_spill)
        self.liquid_ml -= liquid_spill
        self.spill_ml += foam_spill + liquid_spill
        self.liquid_level = self.shape.level(up, self.liquid_ml) if self.liquid_ml > .01 else C.FLOOR
        expanded = self.liquid_ml + self.foam_ml * C.FOAM_EXPANSION
        self.total_level = self.shape.level(up, expanded) if expanded > .01 else C.FLOOR

    def snapshot(self):
        return {
            "beer_ml": self.liquid_ml + self.foam_ml,
            "liquid_ml": self.liquid_ml,
            "head_mm": 1000. * max(0., self.total_level - self.liquid_level),
            "rim_gap_mm": 1000. * (C.HEIGHT - self.total_level),
            "spilled_ml": self.spill_ml,
            "dispensed_ml": self.dispensed_ml,
            "wall_capture_ml": self.wall_capture_ml,
            "direct_capture_ml": self.direct_capture_ml,
        }
