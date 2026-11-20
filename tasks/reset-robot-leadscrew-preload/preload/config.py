"""Dimensions and calibrated bench test loads for the cam-loaded screw stage."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "preload"
INSTRUCTION = "Reset the preload of the leadscrew nut. Leave the unloaded stage ready for its play and drag checks."
DESCRIPTION = "A robotics lead screw has a black nut case and a brass-coloured ramp cam on a floating bench mount."
BASES = station.BASES
STOWED = {name: READY for name in BASES}
MAX_STEPS = 420
PHYSICS_HZ = 240
TABLE = 0.7475
AXIS = np.array([.374, .003, TABLE + .109])
CASE_GRIP = np.array([.464, .003, TABLE + .109])
CAM_GRIP = AXIS.copy()
START = np.radians(57.)
RELIEF = np.radians(-26.)
RAMP_M_PER_RAD = .00125
UNLOADED_GAP_M = .00008
AXIAL_STIFFNESS = 16800.
AXIAL_PROBE_N = 5.0
THREAD_STIFFNESS = 180000.
PROBE_DRIVE_NM = .020
CHECK_DT = .05
