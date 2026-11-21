"""Station dimensions, circuit parameters and task contract."""

import numpy as np
from yam_arena.constants import READY
from yam_arena.station import BASES

STOWED = dict.fromkeys(BASES, READY)
KEY = "verify-robot-vacuum-cup-seal"
INSTRUCTION = (
    "Open the vacuum supply and seat the aluminum test plate against the suction cup. "
    "Isolate the supply and let the cup hold the plate on its own for two seconds "
    "with at least 40 kilopascals of vacuum. Support the plate before pressing "
    "the vent and return it to its rest."
)
DESCRIPTION = (
    "A stationary cobot vacuum head has a downward bellows cup and a pressure "
    "gauge. The blue push-pull spool controls supply and the orange button "
    "vents the downstream test circuit. The free aluminum plate has a raised blue handle."
)
PHYSICS_HZ = 240
MAX_STEPS = 780
CHECK_DT = 0.05
TABLE = 0.7475
CUP_CENTER = np.array([0.405, 0.084, 0.920])
CUP_RADIUS = 0.026
INNER_RADIUS = 0.018
LIP_R = 0.023
CAPTURE_R = 0.0030
STIFFNESS = 8_000.0
LOAD_PRESSURE_KPA = 60.0
HOLD_PRESSURE_KPA = 40.0
PLATE_SIZE = np.array([0.107, 0.091, 0.007])
PLATE_MASS = 0.174
PICK = np.array([0.334, 0.217, TABLE + 0.0195])
HANDLE_LOCAL = np.array([-0.073, 0.000, 0.0155])
TARGET = CUP_CENTER + [0, 0, 0.0010]
SPOOL = np.array([0.383, -0.182, 0.867])
SPOOL_TRAVEL = 0.028
VENT = np.array([0.391, -0.284, 0.850])
VENT_TRAVEL = 0.010
GAUGE = np.array([0.495, -0.166, 1.001])
