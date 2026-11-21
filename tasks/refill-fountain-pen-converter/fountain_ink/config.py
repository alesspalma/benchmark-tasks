"""Fixed writing-desk coordinates and push-button converter geometry, in SI units."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "fountain_ink"
INSTRUCTION = "Fill the fountain pen's converter with blue ink, then put the pen back on its rest."
DESCRIPTION = (
    "An uncapped fountain pen with its converter exposed stands in a pen rest. "
    "An open bottle of blue ink sits beside it."
)
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
MAX_STEPS = 580
PHYSICS_HZ = 240
CHECK_DT = 0.05
TABLE_Z = 0.7475

BOTTLE = np.array([0.377, -0.033, TABLE_Z])
REST = np.array([0.339, 0.119, TABLE_Z])
REST_NIB_Z = 0.80655
INK_Z = TABLE_Z + 0.049
FILL_NIB_Z = INK_Z - 0.020
MOUTH_R = 0.023
WELL_R = 0.0161
INSERT_R = 0.012
BOTTOM_Z = TABLE_Z + 0.008
FEED = np.array([0.0, 0.0, 0.011])
GRIP_Z = 0.039
BUTTON_Z = 0.110
STROKE = 0.0060
CAPACITY_ML = 1.10
BOTTLE_ML = 30.0
RESERVOIR_BOTTOM = 0.053
RESERVOIR_HEIGHT = 0.038
