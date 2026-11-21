"""Dimensions and fixed kitchen setup in metres."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "wire_cheese"
INSTRUCTION = "Cut an even 8 mm slice from the cheese block with the wire slicer and place the slice on the side plate."
DESCRIPTION = "A single-wire hinged cheese slicer, an intact block of cheese beside the cut line, and an empty side plate."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
MAX_STEPS = 660
CHECK_DT = 0.05
TABLE = 0.7475
BOARD_TOP = TABLE + 0.024
PIVOT = np.array([0.425, 0.076, BOARD_TOP])
WIRE_LENGTH = 0.185
WIRE_Z = -0.0017
OPEN = np.deg2rad(-52.0)
STOCK_SIZE = np.array([0.064, 0.054, 0.031])
SLICE_SIZE = np.array([0.008, STOCK_SIZE[1], STOCK_SIZE[2]])
GAP = 0.00045
BOND_LENGTH = (STOCK_SIZE[0] + SLICE_SIZE[0]) * 0.5 + GAP
STOCK_CUT = np.array(
    [PIVOT[0] - STOCK_SIZE[0] / 2 - GAP / 2, 0.0, BOARD_TOP + STOCK_SIZE[2] / 2]
)
STOCK_RESET = STOCK_CUT + [-0.025, 0.024, 0]
SLICE_RESET = STOCK_RESET + [BOND_LENGTH, 0, 0]
PLATE = np.array([0.345, 0.208, TABLE + 0.006])
PLATE_INNER = 0.065
HANDLE = np.array([0.0, -0.207, 0.005])
