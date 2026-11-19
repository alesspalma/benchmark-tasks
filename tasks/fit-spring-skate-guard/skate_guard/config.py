"""Standard bench geometry and dry-skate setup."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "skate_guard"
INSTRUCTION = "Put the skate in the gear bag."
DESCRIPTION = "One dry skate, an open gear bag, and loose two-piece spring runner protector are on the bench."
MAX_STEPS = 540
PHYSICS_HZ = 240
BASES = station.BASES
STOWED = {name: READY for name in BASES}
TABLE = 0.7475
SIDE = np.array([[0., 0., -1.], [0., 1., 0.], [1., 0., 0.]])
BOOT_START = np.array([0.414, 0.150, TABLE + .046])
GUARD_START = np.array([0.475, 0.150, TABLE + .046])
BAG = np.array([0.419, -0.151, TABLE])
BAG_SIZE = (0.193, 0.307, 0.065)
BAG_FLOOR = 0.006
RUNNER_TOE = 0.124
RUNNER_HEEL = -0.119
GUARD_TOE_INNER = 0.127
HEEL_AT_REST = -0.100
GUARD_SEAT_Z = 0.012
HEEL_BODY_Y = -0.068
GUARD_TRAVEL = 0.054
GRIP_BOOT = np.array([0.030, 0.111, 0.039])
GRIP_GUARD = np.array([0., HEEL_BODY_Y, 0.014])
SPRING_RATE = 105.0
SPRING_DAMPING = 1.8
SPRING_PRELOAD_EXTENSION = 0.012
