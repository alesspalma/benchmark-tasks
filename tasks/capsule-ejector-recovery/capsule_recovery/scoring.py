"""Outcome scoring and permanent workspace validity at every physics step."""

import math

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C
from .geometry import contained


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.violations = set()
        self.samples = 0
        self.held = 0.0
        self.last_t = -1

    def observe(self, s):
        self.samples += 1
        for name, o in s["objects"].items():
            if name == "machine":
                continue
            p = np.asarray(o["pose"])
            if not np.isfinite(o["velocity"]).all():
                self.violations.add("invalid_state")
            if not np.isfinite(p).all():
                self.violations.add("invalid_state")
            for key, box in s["bounds"].items():
                if key.startswith(name + ":") and not contained(
                    box, [-0.19, -0.65, C.TOP - 0.01], [0.91, 0.65, 2.0]
                ):
                    self.violations.add("part_off_table")

    def evaluate(self, s):
        o = s["objects"]
        collected = []
        for k in range(C.USED_COUNT):
            lo = C.WASTE + [-C.WASTE_INNER[0] / 2, -C.WASTE_INNER[1] / 2, 0.0035]
            hi = C.WASTE + [
                C.WASTE_INNER[0] / 2,
                C.WASTE_INNER[1] / 2,
                C.WASTE_H + 0.0035,
            ]
            cap = o[f"used{k}"]
            collected.append(
                contained(s["bounds"][f"used{k}:Body"], lo, hi)
                and np.linalg.norm(cap["velocity"][:3]) < 0.015
                and np.linalg.norm(cap["velocity"][3:]) < 0.15
            )
        all_used = all(collected)
        b = o["bin"]
        bin_fit = (
            np.linalg.norm(np.asarray(b["pose"][:3]) - C.BIN) < 0.009
            and quat_xyzw_to_matrix(np.asarray(b["pose"][3:]))[2, 2] > 0.99
        )
        fresh = o["fresh"]
        f = np.asarray(fresh["pose"][:3]) - C.SLOT
        seated = (
            np.linalg.norm(f[:2]) < 0.0035
            and abs(f[2]) < 0.003
            and quat_xyzw_to_matrix(np.asarray(fresh["pose"][3:]))[2, 2] > 0.99
        )
        head = o["machine"]["joints"]["HeadHinge"]
        closed = abs(head) < math.radians(4)
        quiet = all(
            np.linalg.norm(v["velocity"][:3]) < 0.015
            and np.linalg.norm(v["velocity"][3:]) < 0.15
            for n, v in o.items()
            if n != "machine"
        )
        released = (
            len(s.get("hands", {})) == 2
            and all(h["opening_m"] > 0.04 for h in s["hands"].values())
            and s.get("robot_clearance_m", 0) > 0.003
        )
        done = (
            all_used
            and bin_fit
            and seated
            and closed
            and quiet
            and released
            and not self.violations
        )
        if self.last_t != s["time_s"]:
            dt = (
                min(0.05, max(0, s["time_s"] - self.last_t))
                if self.last_t >= 0
                else 0.0
            )
            self.held = self.held + dt if done else 0.0
            self.last_t = s["time_s"]
        value = (
            0.0
            if self.violations
            else 1.0
            if self.held >= 1.0 - 1e-8
            else 0.65
            if all_used and bin_fit
            else 0.35
            if all_used
            else 0.0
        )
        return {
            "score": value,
            "success": value == 1.0,
            "used_in_recycling": sum(collected),
            "bin_fitted": bool(bin_fit),
            "fresh_seated": bool(seated),
            "head_angle_deg": round(math.degrees(head), 2),
            "physics_samples": self.samples,
            "violations": sorted(self.violations),
        }
