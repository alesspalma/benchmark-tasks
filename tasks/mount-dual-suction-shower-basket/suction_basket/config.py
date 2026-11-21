"""Dimensions and reset layout for a full-size bathroom basket (metres)."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "suction_basket"
INSTRUCTION = (
    "Mount the shower basket on the glazed tile wall. Press both suction cups flat "
    "against the tile and close both levers. Put the blue shampoo bottle upright in "
    "the basket, then let go."
)
DESCRIPTION = (
    "A loose shower storage basket with two open suction levers and a blue shampoo "
    "bottle sit on a counter in front of a glazed tile wall."
)
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
CHECK_DT = 0.05
MAX_STEPS = 520
TABLE = 0.7475
WALL_X = 0.545
SEAM_Z = (0.788, 1.061)
SEAM_Y = (-0.30, 0.0, 0.30)
GROUT_WIDTH = 0.004

BASKET_START = np.array([0.293, 0.0, TABLE + 0.001])
BASKET_MOUNT = np.array([0.457, 0.0, 0.826])
RIM_X = -0.053
RIM_Z = 0.055
LEVER_ANGLE_OPEN = 56.0
LEVER_PIVOT = np.array([0.044, 0.0, 0.147])
CUP_Y = 0.092
CUP_Z = 0.095
CUP_X = 0.088
LIP_FORWARD = 0.004
CUP_RADIUS = 0.026
LIP_TRAVEL = 0.006
MIN_COMPRESSION = 0.001
BOTTLE_START = np.array([0.311, -0.240, TABLE + 0.001])
