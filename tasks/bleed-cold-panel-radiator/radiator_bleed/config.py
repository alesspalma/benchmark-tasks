"""Domestic flat-panel radiator service rig dimensions in SI units."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "radiator_bleed"
INSTRUCTION = "Bleed the cold radiator, catch the water in the cup, and close the vent as soon as the water runs steadily."
DESCRIPTION = "A full-size radiator stands on a service bench. A brass key is seated in its upper vent and an empty graduated cup sits nearby."
MAX_STEPS = 620
PHYSICS_HZ = 240
BASES = station.BASES
STOWED = {name: READY for name in BASES}
TABLE = .7475
VENT = np.array([.352, -.034, TABLE + .284])
PORT = VENT + [-.011, .014, -.011]
KEY_HOME = VENT + [-.035, 0., 0.]
KEY_HANDLE = np.array([-.003, 0., 0.])
CUP_START = np.array([.31, .187, TABLE])
CUP_CATCH = PORT + [.005, .029, -.130]
CUP_HEIGHT = .075
CUP_RADIUS = .040
CUP_HANDLE = np.array([-.005, .046, .043])
CHECK_DT = .05
AIR_ML = 40.
