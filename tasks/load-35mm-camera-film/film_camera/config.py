"""Dimensions and visible assembly targets for a manual 35 mm camera."""

from yam_arena.constants import READY
from yam_arena.station import BASES

STOWED = dict.fromkeys(BASES, READY)

KEY = "load-35mm-camera-film"
INSTRUCTION = (
    "Pull the 35 mm film leader "
    "straight to the orange mark, close the back, and work the advance lever "
    "until the rewind knob turns."
)
DESCRIPTION = INSTRUCTION
PHYSICS_HZ = 120
MAX_STEPS = 660
CHECK_DT = 0.05
TABLE = 0.7475
X = 0.392
SEAT_Y = 0.044
SEAT_Z = 0.810
DECK = 0.816
FILM_Z = DECK + 0.0015
LEADER_DY = -0.033
PULL = 0.057
ADVANCE = 0.018
DOOR = (X, 0.063, DECK + 0.005)
DOOR_OPEN = 96
DOOR_REACH = 0.137
LEVER = (0.358, -0.099, 0.818)
LEVER_REACH = 0.031
