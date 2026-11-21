"""Geometry-derived wetting and stable placement score with no arm labels."""

from dataclasses import dataclass, field
from math import cos, pi, sin

import numpy as np
from scipy.spatial.transform import Rotation

from . import config as C


def pose(state, name):
    values = state["objects"][name]["pose"]
    return np.asarray(values[:3]), Rotation.from_quat(values[3:]).as_matrix()


_HALF = C.SLIP_SIZE / 2
_CORNERS = np.asarray(
    [(x, y, -C.SLIP_THICK / 2) for x in (-_HALF, _HALF) for y in (-_HALF, _HALF)]
)
_CAP_SAMPLES = np.asarray(
    [
        (0, 0),
        *[
            (r * cos(theta * pi / 4), r * sin(theta * pi / 4))
            for r in (0.40, 0.72, 0.94)
            for theta in range(8)
        ],
    ]
)
_SAMPLE_HEIGHT = C.DROP_HEIGHT * (1.0 - np.sum(_CAP_SAMPLES**2, axis=1))


def relative_slip(state):
    slide, frame = pose(state, "slide")
    slip, attitude = pose(state, "coverslip")
    return frame.T @ (slip - slide), frame.T @ attitude


@dataclass
class WetMount:
    volume_ml: float = 0.0
    spilled_ml: float = 0.0
    remaining: int = 1
    center: np.ndarray = field(default_factory=lambda: C.DOSE_OFFSET[:2].copy())
    previous_press: float = 0.0
    stroke: float = 0.0
    angle_deg: float | None = None
    coverage: float = 0.0
    air_fraction: float = 0.0
    edge_gap_mm: float = 0.0

    def dispense(self, state):
        press = max(0.0, -state["objects"]["dropper"]["joints"]["BulbTravel"] - 0.0043)
        self.stroke += max(0.0, press - self.previous_press)
        self.previous_press = press
        if self.remaining == 0 or self.stroke < C.DROP_TRAVEL:
            return
        self.remaining -= 1
        slide, frame = pose(state, "slide")
        tube, attitude = pose(state, "dropper")
        tip = tube + attitude @ C.DROPPER_TIP
        top = slide + frame[:, 2] * C.SLIDE_SIZE[2] / 2
        height = frame[:, 2] @ (tip - top)
        if frame[2, 2] > 0.86:
            land = frame.T @ (tip - [0.0, 0.0, height / frame[2, 2]] - top)
            if (
                0.004 <= height <= 0.042
                and attitude[2, 2] > 0.83
                and abs(land[0]) <= 0.010
                and abs(land[1]) <= 0.007
            ):
                self.volume_ml = C.DROP_ML
                self.center = land[:2]
                return
        self.spilled_ml = C.DROP_ML

    def update(self, state, dt):
        self.dispense(state)
        center, attitude = relative_slip(state)
        normal = attitude[:, 2]
        corners = center + _CORNERS @ attitude.T
        gap = np.min(corners[:, 2]) - C.SLIDE_SIZE[2] / 2
        self.edge_gap_mm = float(gap * 1000)
        if not self.volume_ml or normal[2] < 0.35:
            return
        x_y = self.center + C.DROP_RADIUS * _CAP_SAMPLES
        bottom = center - normal * C.SLIP_THICK / 2
        heights = bottom[2] - (x_y - bottom[:2]) @ normal[:2] / normal[2]
        geometry = np.column_stack((x_y, heights))
        within = np.all(
            np.abs(((geometry - center) @ attitude)[:, :2]) <= _HALF, axis=1
        )
        touches = np.any(within & (heights <= C.SLIDE_SIZE[2] / 2 + _SAMPLE_HEIGHT))
        if self.angle_deg is None and touches and gap <= 0.0025:
            self.angle_deg = float(np.degrees(np.arccos(np.clip(normal[2], 0, 1))))
            # An advancing edge vents air. A low-angle parallel landing
            # encloses a central pocket early, which remains after settling.
            escape = np.clip((self.angle_deg - 4.0) / 9.0, 0.0, 1.0)
            self.air_fraction = float(0.0012 + 0.12 * (1.0 - escape) ** 2)
        if (
            self.angle_deg is not None
            and max(corners[:, 2]) <= C.SLIDE_SIZE[2] / 2 + 0.002
        ):
            capacity = min(
                0.98, self.volume_ml * 1.0e-6 / (C.FILM_DEPTH * C.SLIP_SIZE**2)
            )
            self.coverage = min(capacity - self.air_fraction, self.coverage + dt * 0.93)


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.consecutive = 0.0

    def evaluate(self, state, water, elapsed):
        slide, attitude = pose(state, "slide")
        slip, _ = pose(state, "coverslip")
        relative, frame = relative_slip(state)
        top = C.SLIDE_SIZE[2] / 2
        corners = relative + _CORNERS @ frame.T
        centered = bool(abs(relative[0]) <= 0.0030 and abs(relative[1]) <= 0.0025)
        seated = bool(centered and np.max(np.abs(corners[:, 2] - top)) <= 0.0010)
        on_pad = bool(
            np.max(np.abs(slide - C.SLIDE)) <= 0.014 and attitude[2, 2] > 0.99
        )
        clear = all(
            np.linalg.norm(np.asarray(h["points"][0]) - slip) >= 0.039
            for h in state["hands"].values()
        )
        speed = np.linalg.norm(state["objects"]["coverslip"]["velocity"][:3])
        bubble = float(2000 * C.SLIP_SIZE * np.sqrt(water.air_fraction / np.pi))
        quality = bool(seated and water.coverage >= 0.82 and bubble <= 2.0)
        if quality and on_pad and clear and speed < 0.018:
            self.consecutive += elapsed
        else:
            self.consecutive = 0.0
        done = self.consecutive >= 0.5
        score = (
            1.0
            if done
            else 0.8
            if quality and on_pad
            else 0.55
            if seated and water.coverage > 0.6
            else 0.25
            if water.volume_ml
            else 0.0
        )
        return {
            "score": score,
            "success": done,
            "water_microlitres": water.volume_ml * 1000,
            "missed_microlitres": water.spilled_ml * 1000,
            "landing_position_mm": (water.center * 1000).tolist(),
            "first_contact_deg": water.angle_deg,
            "lower_edge_gap_mm": water.edge_gap_mm,
            "wetted_fraction": water.coverage,
            "air_fraction": water.air_fraction,
            "largest_bubble_mm": bubble,
            "slip_relative_mm": (relative * 1000).tolist(),
            "slip_centered": centered,
            "slip_seated": seated,
            "hands_clear": bool(clear),
            "slide_on_pad": on_pad,
            "stable_quality_seconds": self.consecutive,
        }
