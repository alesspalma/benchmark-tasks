"""Count complete, separately supported deposits based on final tray evidence."""

from itertools import combinations

import numpy as np


class Scorer:
    def reset(self):
        self.settled_frames = 0

    def evaluate(self, state):
        material = state["material"]
        deposits = material["deposits"]
        valid = []
        for item in deposits:
            xy = np.asarray(item["xy"])
            if not (5.0 <= item["mass_g"] <= 6.3 and not item["distorted"]):
                continue
            if any(
                other is not item
                and other["mass_g"] >= 0.6
                and np.linalg.norm(xy - other["xy"]) < 0.041
                for other in deposits
            ):
                continue
            valid.append(item)
        complete_count = len(valid)
        press = state["objects"]["press"]
        press_xy = np.asarray(press["pose"][:2])
        clear = all(np.linalg.norm(press_xy - item["xy"]) > 0.047 for item in valid)
        count = min(3, complete_count)
        apart = all(
            np.linalg.norm(np.asarray(a["xy"]) - b["xy"]) >= 0.043
            for a, b in combinations(valid, 2)
        )
        large_deposits = sum(item["mass_g"] >= 0.6 for item in deposits)
        full = (
            count == 3
            and apart
            and clear
            and material["wasted_g"] < 0.75
            and large_deposits == 3
        )
        self.settled_frames = self.settled_frames + 1 if full else 0
        success = self.settled_frames >= 10
        score = 1.0 if success else [0.0, 0.25, 0.5, 0.75][count]
        return {
            "score": score,
            "success": success,
            "valid_cookies": complete_count,
            "steps_clear": self.settled_frames,
            "tool_clear": clear,
            **material,
        }
