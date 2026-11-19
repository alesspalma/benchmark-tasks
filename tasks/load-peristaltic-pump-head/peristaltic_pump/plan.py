"""Support both sides of each flexible tube during its move to the pump."""

import numpy as np

from . import config as C
from .motion import Program, rotation

LEFT = rotation("X", np.pi - 0.12) @ rotation("Y", 0.13)
RIGHT = rotation("X", np.pi + 0.12) @ rotation("Y", 0.13)


def line(i, x, z, grip):
    y = C.HEAD[1] + (1 if i == 0 else -1) * (C.TUBE_GRIP * C.TUBE_L + 0.011)
    return (x - (0.003 if i == 0 else 0), y, z), (LEFT, RIGHT)[i], grip


def lever(angle, above=0.0, opening=0.03):
    p = C.HEAD + C.PIVOT + rotation("Y", angle) @ C.LEVER
    return p + [0.0, 0.0, above - 0.004], RIGHT, opening


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    staged = C.TABLE + 0.036
    loaded = C.TABLE + C.BED_Z + C.TUBE_R - 0.005
    raised = C.TABLE + 0.126
    for tube, stage_x, lane in zip(("orange", "blue"), C.STAGING, C.LANES):
        target = C.HEAD[0] + lane
        p.add(1.8, f"approach {tube} tube", left=line(0, stage_x, raised, 0.65), right=line(1, stage_x, raised, 0.65))
        p.add(1.65, f"straddle the ends of the {tube} tube", left=line(0, stage_x, staged, 0.65), right=line(1, stage_x, staged, 0.65))
        p.add(0.5, "settle above the tubing")
        p.add(1.15, f"pinch both sides of the {tube} tube", left=line(0, stage_x, staged, 0.03), right=line(1, stage_x, staged, 0.03))
        p.add(1.25, f"lift the {tube} tube with tension support", left=line(0, stage_x, raised, 0.03), right=line(1, stage_x, raised, 0.03))
        p.add(1.55, f"align the {tube} tube in its own lane", left=line(0, target, raised, 0.03), right=line(1, target, raised, 0.03))
        p.add(1.3, f"lower the {tube} tube against both retainers", left=line(0, target, loaded, 0.03), right=line(1, target, loaded, 0.03))
        p.add(0.85, "release the seated tube", left=line(0, target, loaded, 0.65), right=line(1, target, loaded, 0.65))
        p.add(0.8, "clear the tube exits", left=line(0, target, raised, 0.65), right=line(1, target, raised, 0.65))
    if mode == "open-head":
        return p.moves
    p.add(1.4, "approach the cam lever", right=lever(C.OPEN, above=0.040, opening=0.65))
    p.add(1.3, "straddle the lever grip", right=lever(C.OPEN, opening=0.65))
    p.add(1.1, "grasp the cam lever", right=lever(C.OPEN))
    for angle in np.linspace(C.OPEN, 0, 8)[1:]:
        p.add(0.44, "drive the cam over the loaded tubes", right=lever(angle))
    p.add(0.65, "release the cam lever", right=lever(0, opening=0.65))
    p.add(1.5, "withdraw for the water check", right=line(1, 0.31, C.TABLE + 0.19, 0.65), left=line(0, 0.33, C.TABLE + 0.19, 0.65))
    return p.moves
