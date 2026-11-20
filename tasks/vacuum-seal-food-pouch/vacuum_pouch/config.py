"""Fixed counter layout and full-size dimensions in metres."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "vacuum_pouch"
INSTRUCTION = "Vacuum-seal the filled food pouch, then take it out and leave it flat on the counter."
DESCRIPTION = "A countertop suction sealer with an open lid and a dry embossed food pouch on the counter in front."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
CHECK_DT = .05
MAX_STEPS = 1000
TABLE = .7475
LID = np.array([.571, 0., .797])
LID_OPEN = 42.
CAM = np.array([.402, -.219, .769])
CAM_TRAVEL = 24.
LATCH_ARM = np.array([-.027, 0., .008])
BUTTON = np.array([.479, .117, .833])
POUCH_START = np.array([.232, 0., TABLE+.0045])
MOUTH_TIP = .144
MOUTH_Y = .197 / 2
MOUTH_Z = .029
MOUTH_PANEL_START = .035
WING = .034
INITIAL_WING = 7.
GRASP_X = .054
GRASP_Y = .084
CHANNEL_HALF_WIDTH = .126
CHANNEL_X_RANGE = (.442, .490)
COUNTER_X = .379
CYCLE_SECONDS = 2.8
KEEPER_MIN = 22.
HEATER_X = .423
HEATER_TOP = .790
