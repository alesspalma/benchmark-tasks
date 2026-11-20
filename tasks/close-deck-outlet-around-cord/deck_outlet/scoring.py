"""Score protected operation and record any earlier exposed energization."""
import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.hold = 0.
        self.unsafe = False
        self.on = False

    def evaluate(self, state, elapsed):
        joints = state["objects"]["outlet"]["joints"]
        lid, cord = [float(np.degrees(joints[name])) for name in ("CoverHinge", "CordFlex")]
        lid += C.LID_OPEN
        cord += C.CABLE_OPEN
        switch = float(state["objects"]["lamp_post"]["joints"]["LightSwitch"])
        exit_x = float(C.CABLE_PIVOT[0] - (C.OUTLET_EXIT_Y-C.CABLE_PIVOT[1])
                       * np.tan(np.radians(cord)))
        routed = abs(exit_x - C.ENDING_X) <= .011
        closed = abs(lid) <= 1.3
        protected = routed and closed
        pressed = switch >= .0045
        self.on |= pressed
        self.unsafe |= self.on and not protected
        on = self.on
        clear = all(p[0] <= .245 or p[2] >= .954 or abs(p[1]) >= .255
                    for h in state["hands"].values() for p in h["points"])
        self.hold = self.hold + elapsed if on and protected and clear and not self.unsafe else 0.
        score = 1. if self.hold >= .999 else .5 if protected and not self.unsafe else 0.
        return dict(score=score, success=score == 1., lid_deg=lid, cord_deg=cord,
                    exit_offset_mm=1000*(exit_x-C.ENDING_X), switch_mm=1000*switch, cord_routed=routed, lid_seated=closed, switch_on=on,
                    unsafe_energization=self.unsafe, hands_clear=clear, hold_s=self.hold)
