"""Full-size manual roller dimensions and fixed kitchen worktop layout."""

import math

import numpy as np

KEY = "pasta_sheet"
INSTRUCTION = "Feed the dough squarely into the pasta machine and turn the crank to roll one continuous, evenly thin sheet onto the board."
DESCRIPTION = "One arm guides the dough into the roller nip while the other supplies the rolling work with the hand crank."
MAX_STEPS = 600
PHYSICS_HZ = 480
BASES = {"left": (0.0, 0.305, 0.750), "right": (0.0, -0.305, 0.750)}
STOWED = {name: (0.0, 1.0, 1.0, -0.6, 0.0, 0.0) for name in BASES}
TABLE = 0.7475
NIP = np.array([0.445, 0.025, 0.920])
ROLL_RADIUS = 0.017
ROLL_WIDTH = 0.150
GAP = 0.0037
# Two native skins share this relaxed thickness and a compression joint.
STOCK_THICKNESS = 0.0048
STOCK_LENGTH = 0.092
STOCK_WIDTH = 0.086
STOCK = np.array([0.272, 0.112, 0.9154])
ROLL_NEAR = NIP + [-ROLL_RADIUS - GAP / 2, 0.0, 0.0]
ROLL_FAR = NIP + [ROLL_RADIUS + GAP / 2, 0.0, 0.0]
CRANK_SPAN = -0.147
CRANK_RADIUS = 0.072
CRANK_START = math.pi / 2
CRANK_KNOB_RADIUS = 0.011
