"""Hose-accessory station dimensions, in metres."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "water_stop"
INSTRUCTION = (
    "Swap the gray spray head on the live garden hose for the blue fan nozzle. "
    "Put the gray head in the parts tray and leave the blue nozzle locked on "
    "the water-stop connector, spraying into the catch tray."
)
DESCRIPTION = (
    "The hose has live water. Its orange sleeve pulls toward the hose to release "
    "the spray head. The valve closes when the nozzle comes out."
)
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
MAX_STEPS = 620
CHECK_DT = 0.05
TABLE = 0.7475
SOCKET = np.array([0.359, 0.056, 0.817])
GRAY_READY = SOCKET - [0, 0.058, 0]
BLUE_READY = np.array([0.421, -0.177, 0.817])
PARK = np.array([0.220, -0.164, 0.817])
CATCH = (0.310, -0.062, 0.509, 0.121)  # center X, center Y, length X, length Y
COLLAR_STROKE = 0.011
POPPET_STROKE = 0.0075
FLOW_ML_S = 75.0
