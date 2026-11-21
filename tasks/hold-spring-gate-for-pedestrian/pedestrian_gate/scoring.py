"""Score a complete through-passage and re-latching without striking the person."""

import math

import numpy as np

from . import config as C


def gap_from_leaf(theta, torso):
    """Distance in units of the padded cross-section of the clothed pedestrian."""
    pivot = C.HINGE[:2]
    tip = pivot + C.SPAN * np.array((math.sin(theta), -math.cos(theta)))
    radii = np.array((0.135, 0.248))
    a, b = (pivot - torso[:2]) / radii, (tip - torso[:2]) / radii
    delta = b - a
    fraction = np.clip(-np.dot(a, delta) / np.dot(delta, delta), 0.0, 1.0)
    return float(np.linalg.norm(a + fraction * delta))


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.entered = False
        self.passed = False
        self.unsafe = False
        self.stable_s = 0.0

    def sample(self, hinge, torso, moving, contact):
        x, y = torso[:2]
        in_opening = -0.425 + C.SHOULDER_WIDTH / 2 < y < 0.438 - C.SHOULDER_WIDTH / 2
        if moving and in_opening and C.HINGE[0] - 0.03 < x < C.HINGE[0] + 0.12:
            self.entered = True
        if self.entered and in_opening and x <= C.PASS_X:
            self.passed = True
        self.unsafe |= contact or (moving and gap_from_leaf(hinge, torso) < 1.0)

    def evaluate(self, state, elapsed):
        gate = state["objects"]["gate"]
        hinge, bolt = gate["joints"]["Swing"], gate["joints"]["LiftBolt"]
        hinge_speed = gate["joint_speeds"]["Swing"]
        torso = np.asarray(state["pedestrian"]["torso"])
        self.sample(
            hinge, torso, state["pedestrian"]["walking"], state["pedestrian"]["contact"]
        )
        latched = hinge < math.radians(1.7) and bolt < 0.005 and abs(hinge_speed) < 0.10
        hands_clear = all(
            np.linalg.norm(np.asarray(p) - C.BOLT_GRIP) > 0.077
            and np.linalg.norm(
                np.asarray(p)
                - (C.HINGE + [-C.HANDLE_OFFSET, -C.HOLD_RADIUS, C.RAIL_HEIGHT])
            )
            > 0.077
            for hand in state["hands"].values()
            for p in hand["points"]
        )
        valid = not self.unsafe and self.passed
        self.stable_s = (
            self.stable_s + elapsed if valid and latched and hands_clear else 0.0
        )
        success = self.stable_s >= 0.60
        return {
            "success": success,
            "score": 1.0 if success else (0.50 if valid else 0.0),
            "gate_angle_deg": round(math.degrees(hinge), 3),
            "latch_travel_mm": round(1000.0 * bolt, 2),
            "gate_speed_deg_s": round(math.degrees(hinge_speed), 3),
            "person_x_m": round(float(torso[0]), 4),
            "person_y_m": round(float(torso[1]), 4),
            "leaf_distance_in_body_radii": round(gap_from_leaf(hinge, torso), 4),
            "walker_t_s": round(state["pedestrian"]["elapsed"], 4),
            "person_contact": bool(state["pedestrian"]["contact"]),
            "first_contact": state["pedestrian"]["first_contact"],
            "passage_completed": self.passed,
            "unsafe": self.unsafe,
            "latched": latched,
            "hands_clear": hands_clear,
            "stable_s": round(self.stable_s, 3),
        }
