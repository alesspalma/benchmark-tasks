"""Measure persistent coating and a settled dispenser by their physical state."""

import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.parked_steps = 0

    def evaluate(self, state):
        left, target, right = state["material"]["coverage"]
        overrun = state["material"]["overrun_mm2"]
        tape = state["objects"]["applicator"]
        board = state["objects"]["clipboard"]
        tool = np.asarray(tape["pose"])
        velocity = np.asarray(tape["velocity"])
        points = np.asarray([p for h in state["hands"].values() for p in h["points"]])
        clear = np.min(np.linalg.norm(points - tool[:3], axis=1)) > .065
        flat = 1. - 2. * float(tool[3]**2 + tool[4]**2) > .94
        park = (
            clear and flat and abs(tool[2] - C.TOOL_START[2]) < .012
            and np.linalg.norm(tool[:2] - C.TOOL_START[:2]) < .050
            and np.linalg.norm(velocity[:3]) < .025
        )
        self.parked_steps = self.parked_steps + 1 if park else 0
        adjacent = max(left, right)
        paper_intact = (
            abs(board["pose"][2] - C.BOARD_START[2]) < .015
            and np.linalg.norm(np.asarray(board["pose"][:2]) - C.BOARD_START[:2]) < .13
        )
        success = (
            paper_intact and adjacent <= .02 and overrun <= 16.
            and target >= .94 and self.parked_steps >= 8
        )
        score = (1.0 if success else
                 .5 if paper_intact and adjacent <= .02 and target >= .65 and overrun <= 16. else 0.)
        return {
            "score": score, "success": success, "target_coverage": target,
            "neighbor_coverage": adjacent, "overrun_mm2": overrun,
            "parked": park, "parked_steps": self.parked_steps,
            "tool_xyz": tool[:3].tolist(), "board_xyz": board["pose"][:3],
        }
