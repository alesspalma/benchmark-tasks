"""Cup station dimensions, in metres."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "spring_cups"
INSTRUCTION = (
    "Reload the spring-fed dispenser with the six nested cups. "
    "Then pull one cup out onto the blue serving mat and leave five in the dispenser."
)
DESCRIPTION = "An empty in-counter cup dispenser, a loose stack of six paper cups, and a serving mat."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
MAX_STEPS = 620
CHECK_DT = .05
TABLE_Z = .7475
DISPENSER = np.array([.442, .030])
STOCK = np.array([.313, -.110])
STOCK_ELEVATION = .036
MAT = np.array([.342, -.226])
NUM_CUPS = 6
CUP_HEIGHT = .093
CUP_BASE_R = .0254
CUP_WALL_R = .0388
CUP_RIM_R = .0406
CUP_PITCH = .011
CUP_MASS = .008
TUBE_LENGTH = .457
TUBE_RADIUS = .0535
CUTOUT_RADIUS = .062
FINGER_Z = TABLE_Z + .019
FOLLOWER_REST = TABLE_Z - .015
FOLLOWER_THICKNESS = .007
