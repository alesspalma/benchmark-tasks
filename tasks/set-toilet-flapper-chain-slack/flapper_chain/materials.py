"""Finite spring-clip, tension-only chain and head-driven gravity discharge."""

from math import pi, sqrt

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C

TIMESTEP = 1.0 / C.PHYSICS_HZ
TANK_AREA = C.WATER_FOOTPRINT[0] * C.WATER_FOOTPRINT[1]
DRAIN_AREA = pi * (C.DRAIN_DIAMETER / 2) ** 2


def cap(vector, limit):
    return vector * min(1.0, limit / max(float(np.linalg.norm(vector)), 1e-10))


class Mechanism:
    def __init__(self):
        self.reset()

    def reset(self):
        self.latch = None
        self.litres = C.CAPACITY_L
        self.delivered = 0.0
        self.handle_discharge = 0.0
        self.max_flush_angle = 0.0
        self.max_flapper_angle = 0.0
        self.flush_started = False
        self.flow = 0.0
        self.tension = 0.0
        self.chain_span = 0.0
        self.lever_error = float("inf")
        self.previous_pins = None
        self.anchor = C.FLAPPER + C.FLAPPER_ANCHOR
        self.end = C.CLIP_HOME + C.CLIP_CHAIN

    def _clip_force(self, lever, lever_axes, lv, clip, clip_axes, cv):
        pins = lever[:3] + C.PINS @ lever_axes.T
        hook_frame = (pins - clip[:3]) @ clip_axes
        if self.previous_pins is None:
            self.previous_pins = hook_frame
        if self.latch is None:
            for i, (point, previous) in enumerate(zip(hook_frame, self.previous_pins)):
                if (
                    abs(point[0]) < 0.0065
                    and abs(point[1]) < 0.010
                    and -0.002 <= point[2] <= 0.009
                    and previous[2] < 0.002
                    and point[2] > previous[2] + 0.00004
                    and clip_axes[2, 2] > 0.81
                ):
                    self.latch = i
                    break
        self.previous_pins = hook_frame.copy()
        if self.latch is None:
            self.lever_error = float(np.min(np.linalg.norm(hook_frame, axis=1)))
            return np.zeros(3), np.zeros(3)
        index = self.latch
        offset = lever_axes @ (C.PINS[index] + C.HOOK_REST)
        displacement = lever[:3] + offset - clip[:3]
        self.lever_error = float(np.linalg.norm(displacement))
        # Enough axial travel opens the spring tongue and frees the clip.
        if self.lever_error > 0.022:
            self.latch = None
            return np.zeros(3), np.zeros(3)
        point_velocity = lv[:3] + np.cross(lv[3:], offset)
        force = cap(1350.0 * displacement + 6.0 * (point_velocity - cv[:3]), 30.0)
        return force, offset

    def advance(self, lever, lv, flapper, fv, clip, cv, trip, trip_rate, angle, rate):
        lever_axes = quat_xyzw_to_matrix(lever[3:])
        flapper_axes = quat_xyzw_to_matrix(flapper[3:])
        clip_axes = quat_xyzw_to_matrix(clip[3:])
        self.anchor = flapper[:3] + flapper_axes @ C.FLAPPER_ANCHOR
        self.end = clip[:3] + clip_axes @ C.CLIP_CHAIN
        chain_vector = self.end - self.anchor
        span = float(np.linalg.norm(chain_vector))
        self.chain_span = span
        line = chain_vector / max(span, 1e-8)
        relative_speed = (
            cv[:3]
            + np.cross(cv[3:], clip_axes @ C.CLIP_CHAIN)
            - fv[:3]
            - np.cross(fv[3:], flapper_axes @ C.FLAPPER_ANCHOR)
        )
        self.tension = (
            float(
                np.clip(
                    530.0 * (span - C.CHAIN_LENGTH)
                    + 1.1 * float(relative_speed @ line),
                    0,
                    20,
                )
            )
            if span > C.CHAIN_LENGTH
            else 0.0
        )
        chain_force = line * self.tension
        closing = -0.076 - 0.071 * angle - 0.022 * rate
        # A wet seated rubber disc feels a small hydrostatic hold-down. After
        # opening, trapped air in the cup keeps the flapper up until exposed.
        if self.litres > 1.0:
            closing += 0.143 if angle > 0.31 else -0.044 * max(1 - angle / 0.31, 0)
        chain_torque = float(np.cross(flapper_axes @ C.FLAPPER_ANCHOR, chain_force)[1])
        latch_force, lever_offset = self._clip_force(
            lever, lever_axes, lv, clip, clip_axes, cv
        )
        trip_torque = float(
            -0.028
            - 1.90 * trip
            - 0.06 * trip_rate
            - np.cross(lever_offset, latch_force)[0]
        )
        total_clip = latch_force - chain_force
        clip_torque = (
            np.cross(clip_axes @ (C.CLIP_CHAIN - C.CLIP_BODY_CENTER), -chain_force)
            - 0.0003 * cv[3:]
        )
        if self.latch is not None:
            clip_torque += 0.018 * np.cross(clip_axes[:, 2], lever_axes[:, 2])
            # Hook sides bear against the eye if the clip twists across its pin.
            clip_torque += 0.018 * np.cross(clip_axes[:, 1], lever_axes[:, 1])
        clip_torque = cap(clip_torque, 0.022)
        self.max_flush_angle = max(
            self.max_flush_angle, -trip if self.latch == C.CORRECT else 0.0
        )
        self.max_flapper_angle = max(self.max_flapper_angle, angle)
        if (
            self.latch == C.CORRECT
            and trip < -0.135
            and angle > 0.16
            and self.tension > 0.08
        ):
            self.flush_started = True
        # Effective annular opening area grows with native hinge angle.
        area_fraction = float(np.clip((angle - 0.032) / 0.43, 0, 1))
        head = max(0.0, self.litres / (1000 * TANK_AREA) - 0.009)
        self.flow = 1000.0 * 0.80 * DRAIN_AREA * area_fraction * sqrt(2 * 9.81 * head)
        discharged = min(self.litres, self.flow * TIMESTEP)
        self.litres -= discharged
        self.delivered += discharged
        if self.flush_started and self.latch == C.CORRECT:
            self.handle_discharge += discharged
        return (trip_torque, closing + chain_torque), total_clip, clip_torque

    def metrics(self):
        return {
            "engagement": self.latch,
            "clip_seated_error_mm": 1000 * self.lever_error,
            "chain_span_mm": 1000 * self.chain_span,
            "tension_n": self.tension,
            "remaining_l": self.litres,
            "discharged_l": self.delivered,
            "linked_discharge_l": self.handle_discharge,
            "flow_l_s": self.flow,
            "handle_witness": self.flush_started,
            "peak_linked_handle_deg": float(np.rad2deg(self.max_flush_angle)),
            "peak_flapper_deg": float(np.rad2deg(self.max_flapper_angle)),
        }
