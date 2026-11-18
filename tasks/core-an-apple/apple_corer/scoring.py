"""Judge a native extracted core and the apple remaining on the board."""

import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.hold = 0

    def evaluate(self, state):
        fruit = np.asarray(state["objects"]["apple"]["pose"])
        core = np.asarray(state["objects"]["core"]["pose"])
        tool = np.asarray(state["objects"]["corer"]["pose"])
        tissue = state["material"]
        upright = 1. - 2.*float(fruit[3]**2 + fruit[4]**2) >= .93
        board = upright and np.linalg.norm(fruit[:2]-C.APPLE[:2]) <= .045 and abs(
            fruit[2]-C.APPLE[2]
        ) < .009
        clear = not (
            np.linalg.norm((core[:3]-fruit[:3])[:2]) <= .024
            and -.020 < core[2]-fruit[2] < .094
        )
        extracted = tissue["separated"] and clear
        self.hold = self.hold+1 if extracted and board else 0
        success = self.hold >= 8
        score = 1. if success else .5 if tissue["separated"] and board else 0.
        return {"score":score, "success":success, "apple_on_board":bool(board),
                "core_clear":bool(clear), "hold_frames":self.hold,
                "tip_xyz":tool[:3].tolist(), **tissue}
