"""Real mass delivery with native side-window capture and closed retention."""

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
        inside = (
            (np.linalg.norm(local[:, :2], axis=1) < 0.0096)
            & (local[:, 2] > 0.006 + C.RADIUS - 0.0005)
            & (local[:, 2] < 0.310 - C.RADIUS + 0.0005)
        )
        angle = probe["joints"]["Sleeve"]
        buried = (
            abs(p[0] - C.TUB[0]) < 0.025
            and abs(p[1] - C.TUB[1]) < 0.025
            and C.TOP <= p[2] < C.TOP + 0.025
            and r[2, 2] > 0.97
        )
        if self.previous_inside is not None and angle < -1.2 and buried:
            entry = inside & ~self.previous_inside
            side = (
                (self.previous_local[:, 0] > 0.002)
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
        if angle > -0.08:
            self.captured.update(i for i in self.admitted if inside[i])
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
        eligible = np.array([i in self.captured for i in range(len(points))])
        velocity = np.asarray(
            s.get("grain_velocities", np.full((len(points), 6), np.inf))
        )
        settled_grains = np.linalg.norm(velocity[:, :3], axis=1) < 0.015
        delivered = int(np.sum(in_jar & eligible & settled_grains))
        grams = delivered * C.MASS * 1000
        p = s["objects"]["probe"]
        ready = np.linalg.norm(np.asarray(p["pose"][:3]) - C.PROBE) < 0.012
        ready &= (
            quat_xyzw_to_matrix(np.asarray(p["pose"][3:]))[2, 2] > 0.98
            and p["joints"]["Sleeve"] > -0.08
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
            "inside_grains": self.inside_count,
            "probe_parked": bool(ready),
            "physics_samples": self.samples,
            "violations": sorted(self.violations),
        }
