"""Functional released height and tension checks from a light and weighted gauge."""

import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.stable = 0

    def evaluate(self, state):
        force = C.tension(state["angle"])
        near = C.NEAR_HEIGHT + state["height"]
        heights = C.net_height(np.array([.3, C.POST_SPAN - .3]), near, C.HEIGHT_STOP, force)
        loaded = C.net_height(C.POST_SPAN / 2., near, C.HEIGHT_STOP, force, C.MIDPOINT_GAUGE_N)
        # The practical benchmark uses a 4 mm band around either gauge reading.
        height_ok = bool(np.all(np.abs(heights - .1525) <= .004))
        tension_ok = abs(float(loaded) - .1425) <= .004
        grip_sites = [C.POST_AXIS + [0., 0., state["height"]], C.SPINDLE]
        clear = all(np.linalg.norm(np.asarray(hand[:3]) - site) >= .064
                    for hand in state["hands"].values() for site in grip_sites)
        held = state["tooth"] >= 1 and abs(state["tooth"] * C.TOOTH - state["angle"]) <= .065
        still = abs(state["angular_speed"]) <= .18 and abs(state["guide_speed"]) <= .008
        self.stable = self.stable + 1 if height_ok and tension_ok and clear and held and still else 0
        complete = self.stable >= 15
        residual = min(float(force / 40.), 1.)
        partial = 0.5 if tension_ok and held and clear and still else 0.25 if residual >= .5 and held and clear else 0.
        return dict(score=1. if complete else partial, success=complete,
                    measured_tension_n=float(force), guide_change_mm=state["height"]*1000,
                    height_near_mm=float(heights[0]*1000), height_far_mm=float(heights[1]*1000),
                    loaded_midpoint_mm=float(loaded*1000), height_ok=height_ok, tension_ok=tension_ok,
                    engaged_tooth=state["tooth"], hands_clear=clear, settled_frames=self.stable)
