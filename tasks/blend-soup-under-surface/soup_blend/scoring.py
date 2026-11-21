"""Measure the native guard and motor, and circulate conservative soup tracers."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def soup_markers():
    rings, bins = np.indices((C.RINGS, C.SECTORS))
    radius = (C.JUG_RADIUS - 0.002) * np.sqrt((rings.ravel() + 0.5) / C.RINGS)
    theta = (bins.ravel() + 0.5) * 2 * np.pi / C.SECTORS
    return np.column_stack((radius * np.cos(theta), radius * np.sin(theta)))


REGIONS = soup_markers()
CELLS = np.arange(len(REGIONS)).reshape(C.RINGS, C.SECTORS)
A = np.r_[CELLS.ravel(), CELLS[:-1].ravel()]
B = np.r_[np.roll(CELLS, -1, axis=1).ravel(), CELLS[1:].ravel()]
EDGES = (REGIONS[A] + REGIONS[B]) / 2
BANKS = 0.5 + 0.45 * np.tanh((REGIONS[:, 0] + 0.39 * REGIONS[:, 1]) / 0.015)


def tracer_color(concentration):
    dark = np.asarray([0.63, 0.135, 0.04])
    light = np.asarray([0.99, 0.74, 0.25])
    return tuple(float(v) for v in dark + float(concentration) * (light - dark))


class Scorer:
    def reset(self):
        self.concentration = BANKS.copy()
        self.unsafe = False
        self.power = False
        self.was_running = False
        self.powered_s = 0.0
        self.ready_to_park = False
        self.final_s = 0.0
        self.unsafe_at_s = None
        self.unsafe_depth_m = None
        self.unsafe_cause = None
        self.max_jug_drift = 0.0
        self.minimum_powered_immersion = None
        self.guard_clearance = None
        self.button_travel = 0.0
        self.rotor_speed = 0.0
        self.clock = 0.0
        self.guard_relative = np.zeros(3)

    def physics_sample(self, tool, jug, press, omega, dt):
        self.clock += dt
        self.button_travel = press
        self.rotor_speed = abs(omega)
        self.power = press >= (C.RELEASE_TRAVEL if self.power else C.RUN_TRAVEL)
        running = self.power or self.rotor_speed > 15.0
        frame = quat_xyzw_to_matrix(jug[3:])
        direction = frame.T @ quat_xyzw_to_matrix(tool[3:])[:, 2]
        center = frame.T @ (tool[:3] - jug[:3])
        self.guard_relative = center
        lateral_tip = C.GUARD_RADIUS * np.sqrt(max(0.0, 1.0 - direction[2] ** 2))
        guard_top = center[2] + max(0.0, C.GUARD_TOP * direction[2]) + lateral_tip
        footprint = (
            np.linalg.norm(center[:2])
            + C.GUARD_RADIUS
            + C.GUARD_TOP * np.linalg.norm(direction[:2])
        )
        inside = footprint <= C.JUG_RADIUS - 0.002
        self.guard_clearance = C.SURFACE_Z - guard_top
        submerged = inside and self.guard_clearance >= 0.004 and frame[2, 2] >= 0.990
        if running:
            if not submerged and not self.unsafe:
                self.unsafe = True
                self.unsafe_at_s = self.clock
                self.unsafe_depth_m = self.guard_clearance
                self.unsafe_cause = (
                    "exposed withdrawal" if self.was_running else "exposed start"
                )
            self.was_running = True
        else:
            self.was_running = False
        if self.power and submerged and self.rotor_speed > 35.0:
            self.powered_s += dt
            if self.minimum_powered_immersion is None:
                self.minimum_powered_immersion = self.guard_clearance
            else:
                self.minimum_powered_immersion = min(
                    self.minimum_powered_immersion, self.guard_clearance
                )
            strength = min(self.rotor_speed / C.ROTOR_RAD_S, 1.0)
            # Rotor entrainment is strongest near the actual guard and drives
            # weaker recirculation throughout the contained liquid.
            dist_sq = np.sum((EDGES - center[:2]) ** 2, axis=1)
            plume = np.exp(-dist_sq / (2 * 0.024**2))
            weights = (0.40 + 14.0 * plume) * dt * strength
            flux = weights * (self.concentration[A] - self.concentration[B])
            np.add.at(self.concentration, A, -flux)
            np.add.at(self.concentration, B, flux)
            self.max_jug_drift = max(
                self.max_jug_drift, float(np.linalg.norm(jug[:2] - C.JUG_RESET[:2]))
            )
        if self.mixed and not running and submerged:
            self.ready_to_park = True
        return self.power

    @property
    def deviation(self):
        return float(np.std(self.concentration) / np.mean(self.concentration))

    @property
    def mixed(self):
        return self.powered_s >= 4.0 and self.deviation <= C.MIX_GOAL

    def evaluate(self, state, elapsed):
        tool = state["objects"]["blender"]
        jug = state["objects"]["jug"]
        tpos = np.asarray(tool["pose"])
        jpos = np.asarray(jug["pose"])
        upright = quat_xyzw_to_matrix(tpos[3:])[2, 2] >= 0.988
        jug_flat = quat_xyzw_to_matrix(jpos[3:])[2, 2] >= 0.990
        near_rest = (
            np.linalg.norm(tpos[:2] - C.REST[:2]) < 0.010
            and abs(tpos[2] - C.REST[2]) < 0.008
        )
        tool_settled = np.linalg.norm(tool["velocity"][:3]) < 0.035
        hand_clear = all(
            np.linalg.norm(np.asarray(hand["points"][0]) - (tpos[:3] + C.GRIP_CENTER))
            >= 0.069
            for hand in state["hands"].values()
        )
        intact = (
            jug_flat
            and np.linalg.norm(jpos[:2] - C.JUG_RESET[:2]) < 0.030
            and self.max_jug_drift < 0.030
        )
        parked = bool(
            near_rest
            and upright
            and tool_settled
            and hand_clear
            and not self.power
            and self.rotor_speed < 15.0
        )
        complete = self.ready_to_park and parked and intact and not self.unsafe
        self.final_s = self.final_s + elapsed if complete else 0.0
        score = 0.0
        if not self.unsafe and intact and self.powered_s >= 1.0:
            score = 0.25
            if self.powered_s >= 2.0 and self.deviation <= 0.40:
                score = 0.50
            if self.ready_to_park:
                score = 0.75
            if self.final_s + 1e-8 >= 0.50:
                score = 1.0
        return {
            "score": score,
            "success": score == 1.0,
            "motor_powered": self.power,
            "button_travel_m": self.button_travel,
            "rotor_rad_s": self.rotor_speed,
            "guard_clearance_m": self.guard_clearance,
            "guard_relative": self.guard_relative.tolist(),
            "safe_powered_s": self.powered_s,
            "minimum_powered_immersion_m": self.minimum_powered_immersion,
            "soup_concentration_cv": self.deviation,
            "switched_off_submerged": self.ready_to_park,
            "jug_maximum_drift_m": self.max_jug_drift,
            "parked": parked,
            "unsafe_activation": self.unsafe,
            "unsafe_time_s": self.unsafe_at_s,
            "unsafe_depth_m": self.unsafe_depth_m,
            "unsafe_cause": self.unsafe_cause,
        }
