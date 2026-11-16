"""Actual mass admitted at the side and discharged through the separate bottom."""

from itertools import product

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C
from .geometry import contained


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.admitted = set()
        self.captured = set()
        self.discharged = set()
        self.previous_inside = None
        self.previous_local = None
        self.previous_points = None
        self.violations = set()
        self.samples = 0
        self.held = 0.0
        self.last_t = -1.0
        self.inside_count = 0

    def observe(self, s):
        self.samples += 1
        points = np.asarray(s["grains"])
        probe = s["objects"]["probe"]
        p = np.asarray(probe["pose"])
        r = quat_xyzw_to_matrix(p[3:])
        if not np.isfinite(points).all() or not np.isfinite(p).all():
            self.violations.add("invalid_state")
            return
        local = (points - p[:3]) @ r
        margin = C.RADIUS - 0.0005
        floor = C.FLOOR_MID_HEIGHT - C.FLOOR_SLOPE * local[:, 0]
        inside = (
            (local[:, 0] > C.INNER_BACK + margin)
            & (local[:, 0] < C.INNER_FRONT - margin)
            & (np.abs(local[:, 1]) < C.INNER_HALF_WIDTH - margin)
            & (local[:, 2] > floor + margin * np.sqrt(1 + C.FLOOR_SLOPE**2))
            & (local[:, 2] < C.CHAMBER_TOP - margin)
        )
        gate = probe["joints"]["GateSlide"]
        bottom = probe["joints"]["BottomSlide"]
        buried = (
            abs(p[0] - C.TUB[0]) < 0.025
            and abs(p[1] - C.TUB[1]) < 0.025
            and C.TOP <= p[2] < C.TOP + C.SAMPLING_MAX_BASE_HEIGHT
            and r[2, 2] > 0.97
        )
        if self.previous_inside is not None and gate >= C.GATE_OPEN and buried:
            entry = inside & ~self.previous_inside
            side = (
                (self.previous_local[:, 0] >= C.INNER_FRONT - margin)
                & (local[:, 0] < self.previous_local[:, 0])
                & (local[:, 2] + C.RADIUS < gate + 0.0005)
                & (local[:, 2] > C.WINDOW[0] - C.RADIUS)
                & (local[:, 2] < C.WINDOW[1] + C.RADIUS)
            )
            old = self.previous_points - C.TUB
            from_source = (
                (np.abs(old[:, 0]) < C.TUB_INNER[0] / 2)
                & (np.abs(old[:, 1]) < C.TUB_INNER[1] / 2)
                & (old[:, 2] > 0)
                & (old[:, 2] < C.TUB_H)
            )
            self.admitted.update(np.flatnonzero(entry & side & from_source).tolist())
        # Only a real grain still enclosed after closing becomes eligible.
        if gate <= C.GATE_CLOSED and bottom <= C.DUMP_CLOSED:
            self.captured.update(i for i in self.admitted if inside[i])
        # Whole-grain crossing through the physically uncovered bottom.
        # Holding a full sampler above or inside the jar earns no delivery.
        if self.previous_local is not None and bottom > 2 * C.RADIUS:
            boundary = -C.RADIUS + 0.0005
            exits = (
                (self.previous_local[:, 2] > boundary)
                & (local[:, 2] <= boundary)
                & (local[:, 0] > C.INNER_BACK - C.RADIUS)
                & (local[:, 0] < C.INNER_FRONT + C.RADIUS)
                & (np.abs(local[:, 1]) < C.INNER_HALF_WIDTH + C.RADIUS)
                & (local[:, 0] - C.RADIUS > 0.027 - bottom - 0.0005)
            )
            self.discharged.update(
                i for i in np.flatnonzero(exits) if i in self.captured
            )
        self.previous_inside = inside
        self.previous_local = local
        self.previous_points = points.copy()
        self.inside_count = int(inside.sum())
        if np.any(points[:, 2] - C.RADIUS < C.TOP - 0.005):
            self.violations.add("granule_off_table")
        for box in s["bounds"].values():
            if not contained(box, [-0.19, -0.65, C.TOP - 0.005], [0.91, 0.65, 2.0]):
                self.violations.add("probe_off_table")

    def evaluate(self, s):
        points = np.asarray(s["grains"])
        lo = C.JAR + [-C.JAR_R, -C.JAR_R, 0.0035]
        hi = C.JAR + [C.JAR_R, C.JAR_R, C.JAR_H + 0.0035]
        in_jar = np.all(
            (points - C.RADIUS >= lo - 0.0005) & (points + C.RADIUS <= hi + 0.0005),
            axis=1,
        )
        eligible = np.array([i in self.discharged for i in range(len(points))])
        velocity = np.asarray(
            s.get("grain_velocities", np.full((len(points), 6), np.inf))
        )
        settled_grains = np.linalg.norm(velocity[:, :3], axis=1) < 0.015
        delivered = int(np.sum(in_jar & eligible & settled_grains))
        grams = delivered * C.MASS * 1000
        p = s["objects"]["probe"]
        rotation = quat_xyzw_to_matrix(np.asarray(p["pose"][3:]))
        corners = np.asarray(
            list(product((-0.027, 0.027), (-0.027, 0.027), (0.0, 0.054)))
        )
        head = corners @ rotation.T + p["pose"][:3]
        ready = bool(
            np.all(np.abs(head[:, :2] - C.PROBE[:2]) <= 0.0505)
            and abs(p["pose"][2] - C.PROBE[2]) < 0.005
        )
        ready &= (
            quat_xyzw_to_matrix(np.asarray(p["pose"][3:]))[2, 2] > 0.98
            and p["joints"]["GateSlide"] <= C.GATE_CLOSED
            and p["joints"]["BottomSlide"] <= C.DUMP_CLOSED
        )
        settled = (
            np.linalg.norm(p["velocity"][:3]) < 0.015
            and np.linalg.norm(p["velocity"][3:]) < 0.15
        )
        released = (
            len(s.get("hands", {})) == 2
            and all(h["opening_m"] > 0.04 for h in s["hands"].values())
            and s.get("robot_clearance_m", 0) > 0.003
        )
        completed = (
            grams >= 5 and ready and settled and released and not self.violations
        )
        if s["time_s"] != self.last_t:
            dt = (
                min(0.05, max(0, s["time_s"] - self.last_t))
                if self.last_t >= 0
                else 0.0
            )
            self.held = self.held + dt if completed else 0.0
            self.last_t = s["time_s"]
        value = (
            0.0
            if self.violations
            else (
                1.0
                if self.held >= 1.0 - 1e-8
                else 0.7
                if grams >= 5
                else 0.4
                if grams >= 2
                else 0.0
            )
        )
        return {
            "score": value,
            "success": value == 1.0,
            "delivered_grams": round(grams, 3),
            "delivered_grains": delivered,
            "admitted_grains": len(self.admitted),
            "captured_grains": len(self.captured),
            "discharged_grains": len(self.discharged),
            "inside_grains": self.inside_count,
            "gate_open_m": float(p["joints"]["GateSlide"]),
            "bottom_open_m": float(p["joints"]["BottomSlide"]),
            "probe_parked": bool(ready),
            "physics_samples": self.samples,
            "violations": sorted(self.violations),
        }
