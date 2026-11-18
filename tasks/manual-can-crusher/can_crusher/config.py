"""Full-size can and pivot dimensions, metres and radians."""

import math

import numpy as np

KEY = "can_crusher"
INSTRUCTION = "Load the empty can in the wall-mounted crusher. Pull the handle to flatten it, release the handle, then put the crushed can in the bin."
DESCRIPTION = "An empty aluminum drink can stands beside a wall-mounted hand-lever can crusher and its collection bin."
PHYSICS_HZ = 480
MAX_STEPS = 920
BASES = {"left": (0., .305, .82), "right": (0., -.305, .82)}
STOWED = {name: (0., 1., 1., -.6, 0., 0.) for name in BASES}
FLOOR = -.0125
LOWER = np.array([.457, 0., .750])
DRIVE_RADIUS = .105
GUIDE_LENGTH = .187
PIVOT = LOWER + [0., 0., GUIDE_LENGTH - DRIVE_RADIUS]
LEVER_LENGTH = .340
OPEN = 0.
CLOSED = -math.radians(115.)
PLATE_HALF = .006
BUCKLING_BOSS_HEIGHT = .003
CAN_RADIUS = .033
CAN_HEIGHT = .122
CAN_START = np.array([.333, .198, .748])
CENTER_OFFSET = -.049
LOAD = LOWER + [CENTER_OFFSET, 0., PLATE_HALF + .0025]
CAN_BOTTOM_STOP = .023
AXIAL_STIFFNESS = 30000.
AXIAL_DAMPING = 2.5
AXIAL_FORCE_LIMIT = 520.
BUCKLE_FORCE = 300.
AXIAL_REBOUND = .004
BIN_CENTER = np.array([.342, 0., .22])
BIN_DEPTH = .301
BIN_WIDTH = .224
BIN_HEIGHT = .204
