"""Fixed bench positions and source-calibrated instrument dimensions in metres."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "refractometer"
INSTRUCTION = "Zero the handheld refractometer using the distilled-water dropper. Leave the prism cover closed so the reading can be checked."
DESCRIPTION = "A handheld analog Brix refractometer with an open daylight plate, a prepared dropper of distilled water, and a slotted micro-adjustment driver."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
MAX_STEPS = 660
PHYSICS_HZ = 240
CHECK_DT = .05
TABLE = .7475

INSTRUMENT = np.array([.366, .027, TABLE + .029])
HANDLE = np.array([0., .043, 0.])
HELD = INSTRUMENT + [0., 0., .046]
PRISM = np.array([0., -.047, .014])
PRISM_WIDTH = .023
PRISM_LENGTH = .032
HINGE = PRISM + [0., -.017, .003]
LID_OPEN_DEG = 112.
SCREW = np.array([0., -.009, .018])
SCREW_INIT = np.radians(38.)
BRIX_PER_RADIAN = 1.83
BRIX_TOLERANCE = .20

DROPPER = np.array([.348, -.160, TABLE + .101])
DROPPER_YAW = np.radians(-43.)
BULB_TRAVEL = .010
DROPPER_TIP = np.array([0., 0., -.080])
DROP_ML = .036
DROPS = 4
DROP_TRAVEL = .00175

DRIVER = np.array([.457, -.173, TABLE + .101])
DRIVER_TIP = np.array([0., 0., -.063])
DRIVER_REST_YAW = np.radians(22.)
