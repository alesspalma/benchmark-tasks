"""Dimensions in metres and the fixed outlet reset."""
import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "deck_outlet"
INSTRUCTION = "Turn on the patio light for the evening."
DESCRIPTION = "A patio light is connected to the open deck receptacle. Its wall switch is off."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
MAX_STEPS = 430
CHECK_DT = .05

FLOOR = .720
CENTER_X = .368
HINGE = np.array([.467, 0., .748])
LID_OPEN = 103.
CABLE_PIVOT = np.array([.385, -.022, .746])
CABLE_OPEN = -24.
CABLE_POINTS = (
    (0., 0., 0.), (0., .030, -.011), (0., .057, -.022),
    (0., .083, -.018), (0., .104, .001), (0., .129, -.008),
    (0., .169, -.010), (0., .207, -.010)
)
OUTLET_EXIT_Y = .068
ENDING_X = .385
SWITCH_PIVOT = np.array([.446, -.207, .884])
SWITCH_ON = 31.
