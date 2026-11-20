"""Fixed reset and SI dimensions for the ECHO-pattern two-line head."""
import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "fixed_line"
INSTRUCTION = "Load the two fresh nylon strips into the empty string-trimmer head and leave both securely seated."
DESCRIPTION = "An inert string trimmer lies head-up with two empty side eyelets and spring gripping channels."
MAX_STEPS = 460
PHYSICS_HZ = 240
CHECK_DT = 0.05
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
TABLE = 0.7475
LINE_LENGTH = 0.2032
LINE_DIAMETER = 0.00241
HEAD = np.array([0.390, 0.0, 0.835])
ENTRY = 0.052
INNER_LIMIT = 0.015
THREAD_X = 0.007
GRIP_FROM_TIP = 0.083
STOCK_X = 0.279
STOCK_Y = 0.255
STOCK_Z = TABLE + 0.042


def sign(side):
    return 1 if side == "left" else -1


def line_center(side):
    return np.array([STOCK_X, sign(side) * STOCK_Y, STOCK_Z])


def hole(side):
    return HEAD + [sign(side) * THREAD_X, sign(side) * ENTRY, 0.0]
