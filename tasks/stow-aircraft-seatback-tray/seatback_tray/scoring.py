"""Evaluate the passive hinge and keeper result after the grippers withdraw."""

import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.hold = 0.0

    def evaluate(self, state, elapsed):
        joints = state["objects"]["seatback"]["joints"]
        tray = float(np.degrees(joints["TrayPivot"]))
        latch = float(np.degrees(joints["LatchPivot"]))
        upper = tray >= C.STOW_ANGLE - 3.
        engaged = 83. <= latch <= 94.
        clear = all(point[0] < .386 or abs(point[1]) > .244 or point[2] > 1.153
                    for hand in state["hands"].values() for point in hand["points"])
        retained = upper and engaged and clear
        self.hold = self.hold + elapsed if retained else 0.
        score = 1.0 if self.hold >= .999 else .65 if upper and engaged else .25 if upper else 0.
        return dict(score=score, success=score == 1.0, tray_deg=tray, latch_deg=latch,
                    latch_engaged=engaged, hands_clear=clear, retain_s=self.hold)
