"""Require a safe withdrawal followed by physical lid closure."""

import math

import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.unsafe = False
        self.closed_steps = 0
        self.early_lid_motion = False

    def observe_safety(self, theta, occupied, worker_contact):
        self.early_lid_motion |= occupied and theta < C.MIN_OCCUPIED
        self.unsafe |= self.early_lid_motion or worker_contact

    def evaluate(self, state):
        theta = state["lid_rad"]
        self.observe_safety(theta, state["occupied"], state["worker_contact"])
        closed = theta <= C.CLOSED and abs(state["lid_speed_rad_s"]) < .035
        low = C.HINGE - (C.DEPTH, C.WIDTH / 2., 0.)
        high = C.HINGE + (0., C.WIDTH / 2., C.THICKNESS)
        hands_clear = all(
            np.linalg.norm(pos - np.clip(pos, low, high)) >= C.HAND_CLEARANCE
            for pos in state["hands"].values()
        )
        ready = not self.unsafe and not state["occupied"] and closed and hands_clear
        self.closed_steps = self.closed_steps + 1 if ready else 0
        success = self.closed_steps >= C.CLOSED_HOLD_STEPS
        return {
            "score": float(success),
            "success": success,
            "occupied": state["occupied"],
            "worker_contact": state["worker_contact"],
            "unsafe_lid_motion": self.early_lid_motion,
            "hands_clear": hands_clear,
            "opening_deg": math.degrees(theta),
            "closed_steps": self.closed_steps,
        }
