"""Kitchen station and ricer geometry, in metres."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "potato_ricer"
INSTRUCTION = (
    "Rice the cooked potato into the mixing bowl, then take your hands off the ricer."
)
DESCRIPTION = "A loose cooked potato portion, an open hand ricer and a mixing bowl."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
MAX_STEPS = 600
CHECK_DT = 0.05

TABLE_Z = 0.7475
BOWL_RADIUS = 0.112
BOWL_TOP = 0.8545
BOWL_FLOOR = TABLE_Z + 0.008
BOWL = np.array([0.444, -0.010, TABLE_Z])
RICER = np.array([0.444, -0.010, BOWL_TOP + 0.009])
HINGE = np.array([0.088, 0.0, 0.027])
PLATE_PIN = np.array([-0.085, 0.0, 0.0])
PLATE_DROP = 0.0795
OPEN = np.radians(72.0)
BASKET_INNER = 0.038
BASKET_FLOOR = -0.060
FIXED_GRIP = np.array([-0.209, 0.0, -0.002])
PRESS_GRIP = np.array([-0.163, 0.0, 0.007])

PORTION = np.array([0.359, 0.166, TABLE_Z + 0.007])
POTATO_G = 44.0
POTATO_RADIUS = 0.022
POTATO_HEIGHT = 0.0335
FOOD_FULL_TRAVEL = 0.0235
FLOOR_Z = BASKET_FLOOR + 0.0018

DIE_HOLES = tuple(
    (float(r * np.cos(theta)), float(r * np.sin(theta)))
    for r, n in ((0.0, 1), (0.012, 7), (0.023, 12), (0.031, 16))
    for theta in np.linspace(0, 2 * np.pi, n, endpoint=False)
)
