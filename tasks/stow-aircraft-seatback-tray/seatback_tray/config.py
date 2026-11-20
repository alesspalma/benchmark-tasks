"""Seatback tray dimensions in metres and the fixed 20 Hz YAM station."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "seatback_tray"
INSTRUCTION = "Stow the tray table securely against the seatback."
DESCRIPTION = "A full-size passenger seat with a deployed folding tray and a swivel latch."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
MAX_STEPS = 380
CHECK_DT = .05
DEPTH = .260
WIDTH = .418
THICK = .017
PIVOT = np.array([.501, 0., .802])
STOW_ANGLE = 83.
SEAT_LEAN = 90. - STOW_ANGLE
BACK_ORIGIN = np.array([.515, 0., .802])
LATCH_PIVOT = np.array([.454, 0., 1.086])
LATCH_LENGTH = .039
TRAY_MASS = .91
