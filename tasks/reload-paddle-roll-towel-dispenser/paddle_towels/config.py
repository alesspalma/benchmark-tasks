"""Dispenser and paper geometry in metres and radians."""

import numpy as np
from yam_arena.constants import READY
from yam_arena.station import BASES

KEY = "paddle_towels"
INSTRUCTION = (
    "Reload the open paper towel dispenser with the new roll. Tuck the free edge "
    "between the feed rollers and work the paddle until a usable tail protrudes "
    "from the bottom slot. Close the cover securely, leaving the tail exposed."
)
DESCRIPTION = (
    "An empty wall-mounted push-paddle roll towel dispenser and a new unthreaded roll."
)
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
CHECK_DT = 0.05
MAX_STEPS = 800
TABLE_Z = 0.7475

ROLL_RADIUS = 0.092
ROLL_WIDTH = 0.200
CORE_RADIUS = 0.0254
STOCK = np.array([0.539, 0.234, 0.964])
STOCK_YAW = 0.0
SEAT = np.array([0.539, -0.029, 0.964])
PAPER_X = -0.093
PAPER_HINGE_Z = -0.086
PAPER_LENGTH = 0.103
PAPER_WIDTH = 0.199
NIP_X = 0.483
NIP_Z = 0.804
OUTLET_Z = 0.754
NIP_TO_OUTLET = 0.087
PADDLE = np.array([0.426, SEAT[1], 0.744])
PADDLE_TRAVEL = 0.044
COVER_HINGE = np.array([0.421, SEAT[1], 0.766])
COVER_OPEN = -1.32
COVER_HEIGHT = 0.356
