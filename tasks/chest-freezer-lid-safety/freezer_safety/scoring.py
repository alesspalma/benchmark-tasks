"""Record lid excursion throughout the episode, including an unsafe reversal."""

import math

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.unsafe = False
        self.closed_steps = 0

    def evaluate(self, state):
        theta = state["lid_rad"]
        if state["occupied"]:
            self.unsafe |= theta < C.MIN_OCCUPIED or state["worker_contact"]
            success = state["time_s"] >= C.EVALUATION_TIME - 1e-6 and not self.unsafe
        else:
            closed = theta <= C.CLOSED and abs(state["lid_speed_rad_s"]) < .035
            self.closed_steps = self.closed_steps + 1 if closed else 0
            success = self.closed_steps >= 15
        return {
            "score": float(success),
            "success": success,
            "occupied": state["occupied"],
            "unsafe_lid_motion": self.unsafe,
            "opening_deg": math.degrees(theta),
            "closed_steps": self.closed_steps,
        }
