"""Station geometry and physical thresholds in SI units."""

from math import radians

import numpy as np
from yam_arena.constants import READY
from yam_arena.station import BASES

KEY = "crimp-filled-pastry-turnover"
INSTRUCTION = (
    "Finish folding the press and squeeze it to seal the filled turnover. "
    "Reopen the press and leave the sealed turnover resting in the lower well."
)
DESCRIPTION = "A partly folded hinged pastry press holding a filled round wrapper."
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
MAX_STEPS = 720
CHECK_DT = 0.05
TABLE = 0.7475
PRESS = np.array([0.365, 0.0, 0.796])
LID_TAB = np.array([0.023, -0.078, -0.0075])
LID_GRIP = LID_TAB + [0.0, -0.010, -0.019]
BASE_TAB = np.array([-0.040, 0.070, -0.0075])
RADIUS = 0.051
PASTRY_Z = 0.006
WRAPPER_RIM = 0.001
READY_FOLD = radians(134.0)
READY_DOUGH = READY_FOLD + radians(13.0)
FOLDED = radians(176.5)
SEAM_RADIUS = 0.048
SECTOR_COUNT = 13
MIN_CRIMP_FORCE = 9.0
MIN_SEAL_DWELL = 0.33
CLOSE_STOP = radians(180.0)
