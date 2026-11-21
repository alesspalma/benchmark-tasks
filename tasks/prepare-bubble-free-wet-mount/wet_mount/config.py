"""Station coordinates and physical dimensions in metres unless indicated."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "wet_mount"
INSTRUCTION = "Prepare a wet microscope mount of the printed letter using one drop of distilled water. Leave the coverslip centered over the letter with no large air bubbles."
DESCRIPTION = "A loose glass microscope slide and printed letter on a dark inspection pad, a distilled-water dropper, and a loose coverslip on two supports."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
MAX_STEPS = 590
PHYSICS_HZ = 240
CHECK_DT = 0.05
TABLE = 0.7475
STAGE_TOP = 0.7895
SLIDE_SIZE = np.array([0.075, 0.025, 0.001])
SLIDE = np.array([0.385, 0.012, STAGE_TOP + SLIDE_SIZE[2] / 2])
DOSE_OFFSET = np.array([0.0083, 0, SLIDE_SIZE[2] / 2])
SLIP_SIZE = 0.022
SLIP_THICK = 0.00022
SLIP_REST_TOP = 0.782
SLIP_REST = np.array([0.374, -0.105, SLIP_REST_TOP + SLIP_THICK / 2])
SLIP_MASS = 2500 * SLIP_SIZE**2 * SLIP_THICK
DROPPER = np.array([0.347, 0.147, TABLE + 0.101])
DROPPER_YAW = np.radians(-150)
DROPPER_TIP = np.array([0.0, 0.0, -0.080])
BULB_TRAVEL = 0.010
DROP_ML = 0.025
DROP_TRAVEL = 0.0022
DROP_RADIUS = 0.00295
DROP_HEIGHT = 0.00185
FILM_DEPTH = 0.000055
