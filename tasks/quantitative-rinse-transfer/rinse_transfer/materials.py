"""Pressure-driven water, local wall wetting, and conserved dye recovery."""

import math

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C

ANGLES = (np.arange(8) + 0.5) * math.tau / 8
FILM_POINTS = np.array(
    [
        (C.RADIUS * 0.98 * math.cos(a), C.RADIUS * 0.98 * math.sin(a), z)
        for z in C.FILM_HEIGHTS
        for a in ANGLES
    ]
)
GRID = np.array(
    [
        (x, y)
        for x in np.linspace(-C.RADIUS, C.RADIUS, 27)
        for y in np.linspace(-C.RADIUS, C.RADIUS, 27)
        if x * x + y * y < C.RADIUS**2
    ]
)
WEIGHT_ML = math.pi * C.RADIUS**2 / len(GRID) * 1e6


def basis(pose):
    return np.asarray(pose[:3]), quat_xyzw_to_matrix(np.asarray(pose[3:]))


def mesh_values(mesh, points, faces):
    from pxr import Gf

    mesh.GetPointsAttr().Set([Gf.Vec3f(*map(float, p)) for p in points])
    mesh.GetFaceVertexCountsAttr().Set([len(f) for f in faces])
    mesh.GetFaceVertexIndicesAttr().Set([i for f in faces for i in f])


def crossed_plane(p, speed, normal, position, max_distance=0.45):
    """Ballistic contact with a plane whose positive side initially contains the drop."""
    initial = float(normal @ (p - position))
    linear = float(normal @ speed)
    quadratic = -0.5 * 9.81 * normal[2]
    if initial <= 0:
        return None
    if abs(quadratic) < 1e-8:
        t = -initial / linear if linear < 0 else float("inf")
    else:
        disc = linear**2 - 4 * quadratic * initial
        if disc < 0:
            return None
        roots = [(-linear + s * math.sqrt(disc)) / (2 * quadratic) for s in (-1, 1)]
        t = min((t for t in roots if t > 0), default=float("inf"))
    if not 0 < t < max_distance:
        return None
    return p + t * speed + np.array([0.0, 0.0, -0.5 * 9.81 * t * t])


class Rinse:
    def __init__(self):
        self.reset()

    def reset(self):
        self.in_bottle = 70.0
        self.film = np.full(16, 1.0 / 16)
        self.water = 0.0
        self.dissolved = 0.0
        self.received_ml = 0.0
        self.received_dye = 0.0
        self.spill_ml = 0.0
        self.lost_dye = 0.0
        self.pressure_volume = 0.0
        self.peak_squeeze_mm = 0.0
        self.total_dispensed = 0.0
        self.base_disp = 0.0
        self.spray = None
        self.pour = None
        self.water_plane = C.FLOOR

    def _dispense(self, dt, bottle_pose, compression, p, R):
        # A moving panel sweeps its projected area. A small elastic headspace
        # takes up the initial compression before the water column rises.
        displacement = max(0.0, compression * 1550.0 - 0.46)
        extension = displacement - self.base_disp
        self.base_disp = displacement
        self.pressure_volume = max(0.0, self.pressure_volume + extension)
        amount = min(self.in_bottle, self.pressure_volume * (1 - math.exp(-dt / 0.065)))
        self.pressure_volume -= amount
        self.in_bottle -= amount
        self.total_dispensed += amount
        self.spray = None
        if amount < 1e-6:
            return
        b, S = basis(bottle_pose)
        start = b + S @ C.NOZZLE
        speed = (S @ C.JET) * np.clip(amount / dt / 0.85, 0.60, 3.4)
        rim = p + R[:, 2] * C.HEIGHT
        hit = crossed_plane(start, speed, R[:, 2], rim)
        if hit is not None and np.linalg.norm((R.T @ (hit - p))[:2]) < C.RADIUS:
            bottom = p + R[:, 2] * (C.FLOOR + 0.001)
            projected = crossed_plane(start, speed, R[:, 2], bottom)
            end = hit if projected is None else projected
            delta = R.T @ (end - hit)
            origin = R.T @ (hit - p)
            if np.linalg.norm((origin + delta)[:2]) > C.RADIUS:
                # Clip entry-to-base trajectory at the inner cylindrical wall.
                denom = float(delta[:2] @ delta[:2])
                mixed = float(delta[:2] @ origin[:2])
                length = float(origin[:2] @ origin[:2] - C.RADIUS**2)
                ratio = (
                    -mixed + math.sqrt(max(0.0, mixed * mixed - denom * length))
                ) / denom
                end = hit + ratio * (end - hit)
            self.water += amount
            near = np.linalg.norm(FILM_POINTS - R.T @ (end - p), axis=1) < 0.008
            removal = self.film * near * (1 - math.exp(-amount * 2.0))
            self.film -= removal
            self.dissolved += float(sum(removal))
        else:
            floor = crossed_plane(
                start, speed, np.array([0.0, 0.0, 1.0]), np.array([0.0, 0.0, C.TOP])
            )
            end = start + S @ C.JET * 0.15 if floor is None else floor
            in_funnel = crossed_plane(start, speed, np.array([0.0, 0.0, 1.0]), C.FUNNEL)
            if (
                in_funnel is not None
                and np.linalg.norm((in_funnel - C.FUNNEL)[:2]) < C.FUNNEL_RADIUS
            ):
                self.received_ml += amount
                end = in_funnel
            else:
                self.spill_ml += amount
        if amount / dt > 0.015:
            self.spray = (start, end)

    def _capacity(self, normal, plane):
        if normal[2] < 0.01:
            return 0.0
        return float(
            np.clip(
                (plane - GRID @ normal[:2]) / normal[2] - C.FLOOR,
                0.0,
                C.HEIGHT - C.FLOOR,
            ).sum()
            * WEIGHT_ML
        )

    def advance(self, dt, bottle_pose, compression, beaker_pose):
        self.peak_squeeze_mm = max(self.peak_squeeze_mm, compression * 1000)
        p, R = basis(beaker_pose)
        self._dispense(dt, bottle_pose, compression, p, R)
        self.pour = None
        if self.water < 1e-7:
            return
        u = R[2]
        rim_angle = math.atan2(-u[1], -u[0])
        lip = np.array(
            [C.RADIUS * math.cos(rim_angle), C.RADIUS * math.sin(rim_angle), C.HEIGHT]
        )
        if u @ C.SPOUT <= u @ lip:
            lip = C.SPOUT
        threshold = float(u @ lip)
        full = self._capacity(u, threshold)
        if u[2] > 0.01:
            low = float(u[2] * C.FLOOR - np.linalg.norm(u[:2]) * C.RADIUS)
            high = threshold
            for _ in range(13):
                middle = (low + high) / 2
                if self._capacity(u, middle) < self.water:
                    low = middle
                else:
                    high = middle
            self.water_plane = (low + high) / 2
            wetted = FILM_POINTS @ u < self.water_plane
        else:
            wetted = np.zeros(len(self.film), dtype=bool)
        recovered = self.film * wetted * (1 - math.exp(-dt * 7.5))
        self.film -= recovered
        self.dissolved += float(sum(recovered))
        if self.water > full + 1e-6:
            q = min(self.water - full, max(0.05, (self.water - full) * 7.0) * dt)
            dye = self.dissolved * q / self.water
            point = p + R @ lip
            if (
                point[2] > C.FUNNEL[2]
                and np.linalg.norm((point - C.FUNNEL)[:2]) < C.FUNNEL_RADIUS
            ):
                self.received_ml += q
                self.received_dye += dye
                end = np.array([*point[:2], C.FUNNEL[2] - 0.009])
            else:
                self.spill_ml += q
                self.lost_dye += dye
                end = np.array([*point[:2], C.TOP])
            self.water -= q
            self.dissolved -= dye
            if q / dt > 0.012:
                self.pour = (point, end)

    def snapshot(self):
        return {
            "rinse_ml": self.total_dispensed,
            "bottle_remaining_ml": self.in_bottle,
            "beaker_water_ml": self.water,
            "dissolved_dye": self.dissolved,
            "wall_dye": float(sum(self.film)),
            "recovered_dye": self.received_dye,
            "flask_ml": self.received_ml,
            "spilled_ml": self.spill_ml,
            "lost_dye": self.lost_dye,
            "peak_squeeze_mm": self.peak_squeeze_mm,
        }

    def render(
        self, stage, beaker_path, bottle_path, receiver_path, beaker_pose, compression
    ):
        from pxr import Gf, UsdGeom, UsdShade

        # Use the same live wall compression to show the smooth bottle skin
        # over the native segmented collision surface.
        bottle = UsdGeom.Mesh.Get(stage, bottle_path + "/Base/BottleSkin")
        a = np.arange(64) * math.tau / 64
        n = np.array([math.cos(math.radians(-10)), math.sin(math.radians(-10))])
        directions = np.column_stack((np.cos(a), np.sin(a)))
        shell = []
        r = C.BOTTLE_RADIUS
        for radius, height in (
            *((r, h) for h in (0.002, 0.019, 0.033, 0.048, 0.063, 0.075)),
            (r - 0.004, 0.082),
            (0.012, 0.091),
        ):
            weight = max(
                0.0, math.sin(math.pi * np.clip((height - 0.012) / 0.063, 0, 1))
            )
            xy = (
                radius * directions
                - compression * 0.5 * weight * (directions @ n)[:, None] * n
            )
            shell.extend(Gf.Vec3f(float(x), float(y), height) for x, y in xy)
        bottle.GetPointsAttr().Set(shell)

        def stream(path, segment, size):
            geo = UsdGeom.Mesh.Get(stage, path)
            if segment is None:
                mesh_values(geo, [], [])
                return
            p, q = segment
            length = q - p
            perpendicular = np.cross(length, [1.0, 0.0, 0.0])
            if np.linalg.norm(perpendicular) < 1e-6:
                perpendicular = np.cross(length, [0.0, 1.0, 0.0])
            normal = size * perpendicular / max(np.linalg.norm(perpendicular), 1e-7)
            mesh_values(
                geo, (p - normal, p + normal, q + normal, q - normal), [(0, 1, 2, 3)]
            )

        def shade_liquid(path, dye, volume):
            blue = np.array([0.06, 0.29, 0.87])
            clear = np.array([0.78, 0.91, 0.96])
            strength = min(1.0, max(0.0, dye * 20.0 / max(volume, 0.01)))
            shader = UsdShade.Shader.Get(stage, path + "/Shader")
            shader.GetInput("diffuseColor").Set(
                Gf.Vec3f(*map(float, clear * (1 - strength) + blue * strength))
            )

        for i, fraction in enumerate(self.film):
            strip, sector = divmod(i, 8)
            shade = UsdShade.Shader.Get(
                stage, f"{beaker_path}/Residue{strip}_{sector}/Shader"
            )
            shade.GetInput("opacity").Set(
                float(np.clip(fraction * 16.0, 0.0, 1.0) * 0.86)
            )
        pool = UsdGeom.Mesh.Get(stage, beaker_path + "/Body/Pool")
        _, R = basis(beaker_pose)
        u = R[2]
        if self.water > 0.04 and u[2] > 0.02:
            shade_liquid(beaker_path + "/Pool", self.dissolved, self.water)
            n = 48
            perimeter = np.array(
                [
                    (C.RADIUS * 0.985 * math.cos(a), C.RADIUS * 0.985 * math.sin(a))
                    for a in np.arange(n) * math.tau / n
                ]
            )
            tops = np.clip(
                (self.water_plane - perimeter @ u[:2]) / u[2], C.FLOOR, C.HEIGHT
            )
            points = [
                *np.column_stack((perimeter, np.full(n, C.FLOOR))),
                *np.column_stack((perimeter, tops)),
            ]
            faces = [(i, (i + 1) % n, (i + 1) % n + n, i + n) for i in range(n)] + [
                tuple(range(n, 2 * n))
            ]
            mesh_values(pool, points, faces)
        else:
            mesh_values(pool, [], [])
        collected = UsdGeom.Mesh.Get(stage, receiver_path + "/Collected")
        if self.received_ml > 0.04:
            shade_liquid(
                receiver_path + "/CollectedMaterial",
                self.received_dye,
                self.received_ml,
            )
            r = 0.025
            cap = 0.005 + self.received_ml * 1e-6 / (math.pi * r * r)
            n = 48
            points = [
                C.FLASK
                + [r * math.cos(i * math.tau / n), r * math.sin(i * math.tau / n), h]
                for h in (0.005, cap)
                for i in range(n)
            ]
            faces = [(i, (i + 1) % n, (i + 1) % n + n, i + n) for i in range(n)] + [
                tuple(range(n, 2 * n))
            ]
            mesh_values(collected, points, faces)
        else:
            mesh_values(collected, [], [])
        stream(receiver_path + "/Spray", self.spray, 0.0010)
        stream(receiver_path + "/Pour", self.pour, 0.0015)
