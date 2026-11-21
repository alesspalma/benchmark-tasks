"""Score verifiable dwell readings of the actual indenter at distinct material sites."""

from collections import deque
from itertools import product

import numpy as np

from . import config as C
from .mechanics import frame


def pose(state, name):
    return frame(np.asarray(state["objects"][name]["pose"]))


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.readings = []
        self.sites = []
        self.anchor = None
        self.measurement_started = None
        self.dwell = 0.
        self.values = deque(maxlen=5)

    def evaluate(self, state):
        sample, sample_rotation = pose(state, "coupon")
        tool, gauge_rotation = pose(state, "durometer")
        sample_z = sample_rotation[:, 2]
        gauge_z = gauge_rotation[:, 2]
        corners = [
            sample + sample_rotation @ np.array([xs, ys, -C.COUPON_SIZE[2] / 2])
            for xs, ys in product(*[(-size / 2, size / 2) for size in C.COUPON_SIZE[:2]])
        ]
        supported = (
            all(np.all(np.abs(p[:2] - C.PLATEN[:2]) <= np.array(C.PLATEN_SIZE[:2]) / 2)
                and abs(p[2] - C.PLATEN[2]) <= .001 for p in corners)
            and float(sample_z[2]) >= np.cos(np.radians(3.))
        )
        local = sample_rotation.T @ (tool - sample)
        gap = float(local[2] - C.COUPON_SIZE[2] / 2)
        angle = float(np.degrees(np.arccos(np.clip(sample_z @ gauge_z, -1., 1.))))
        gauge = state["objects"]["durometer"]
        stroke = gauge["joints"]["IndenterTravel"]
        reading = float(np.clip(100 * stroke / C.STROKE, 0., 100.))
        material_margin = np.min(C.COUPON_SIZE[:2] / 2 - np.abs(local[:2]))
        healthy = (
            supported and material_margin >= C.EDGE_MARGIN and angle <= C.TILT_DEG
            and -.0003 <= gap <= .001 and C.MIN_READING <= reading <= C.MAX_READING
            and state["contact_load_n"] >= .5
        )
        if healthy:
            if self.anchor is None or np.linalg.norm(local[:2] - self.anchor) > .0015:
                self.anchor = local[:2].copy()
                self.measurement_started = state["time_s"]
                self.dwell = 0.
                self.values = deque(maxlen=5)
            self.dwell = state["time_s"] - self.measurement_started
            self.values.append(reading)
            if self.dwell >= C.DWELL - 1e-6 and len(self.sites) < 5:
                fresh = all(np.linalg.norm(local[:2] - xy) >= C.SPACING for xy in self.sites)
                if fresh and len(self.values) == 5 and np.ptp(self.values) <= 4.:
                    self.sites.append(local[:2].copy())
                    self.readings.append(round(float(np.mean(self.values)), 1))
        else:
            self.anchor = None
            self.measurement_started = None
            self.dwell = 0.
            self.values = deque(maxlen=5)
        return {
            "score": len(self.sites) / 5,
            "success": len(self.sites) == 5,
            "count": len(self.sites),
            "readings_shore_a": self.readings.copy(),
            "sites_mm": [(xy * 1000).round(2).tolist() for xy in self.sites],
            "coupon_supported": bool(supported),
            "edge_clearance_mm": float(material_margin * 1000),
            "foot_gap_mm": gap * 1000,
            "foot_tilt_deg": angle,
            "dwell_s": self.dwell,
            "indenter_retraction_mm": float(stroke * 1000),
            "contact_load_n": state["contact_load_n"],
            "valid_placement": bool(healthy),
            "last_reading": self.readings[-1] if self.readings else None,
        }
