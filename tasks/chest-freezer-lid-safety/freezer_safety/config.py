"""Midea 3.5 ft³ floor-standing chest freezer and full-scale stocker proxy."""

import math
import numpy as np

KEY = "freezer_safety"
INSTRUCTION = "Close the chest freezer."
DESCRIPTION = "A floor-standing chest freezer is open beside its stock bin."
PHYSICS_HZ = 240
MAX_STEPS = 240
DEPTH, WIDTH, HEIGHT = .4445, .5461, .8458
THICKNESS = .045
FLOOR = -.0125
BASES = {name: (0., side * .305, .835) for name, side in (("left", 1), ("right", -1))}
STOWED = {
    "left": (.433, .4314, .8856, -1.100, .3535, .2553),
    "right": (-.433, .4314, .8856, -1.100, -.3535, -.2553),
}
HINGE = np.array([.580, 0., FLOOR + HEIGHT - THICKNESS])
LID_MASS = 3.5
ANGLE_OPEN = math.radians(52.)
MIN_OCCUPIED = math.radians(48.)
CLOSED = math.radians(2.)
EVALUATION_TIME = 10.
