"""Dimensions and fixed poses for the slotted battery cover."""

import math

import numpy as np
from yam_arena import station

KEY = "remote_cell"
INSTRUCTION = "Put the remote back on the toy shelf."
DESCRIPTION = "Return the handheld remote to the low open shelf beside the wooden toys."
MAX_STEPS = 720
PHYSICS_HZ = 480
BASES = station.BASES
TABLE = .7475

REMOTE = np.array([.336, -.009, TABLE + .0003])
SHELL_WIDTH = .040
SHELL_BACK = -.0715
SHELL_FRONT = .0915
SHELL_HEIGHT = .017
SEAT_CENTER = np.array([0., -.0515, 0.])
SEAT_HEIGHT = .0155
CAP_RADIUS = .0118
CAP_BOTTOM = -.0036
CAP_TOP = .0037
CAP_PAD = np.array([.341, -.118, TABLE])
CAP = CAP_PAD + [0., 0., .0055 - CAP_BOTTOM + .0002]
LOCK_ANGLE = -math.radians(68.0)
LOCKED_MIN = math.radians(52.0)
LOCKED_MAX = math.radians(90.0)

DRIVER = np.array([.312, -.229, TABLE + .0003])
DRIVER_GRIP_Z = .056
SHELF = np.array([.410, .205, TABLE])
SHELF_WIDTH = .184
SHELF_LENGTH = .210
SHELF_SURFACE = TABLE + .019
