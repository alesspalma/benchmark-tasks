"""Dimensions of the 12-inch bag and decorating tools in metres."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "piping_bag"
INSTRUCTION = (
    "Fill the piping bag with the icing on the spatula. Pinch the cuff around "
    "the blade and draw the spatula out cleanly, then leave the bag upright and "
    "return the spatula to its rest."
)
DESCRIPTION = "A cuffed decorating bag, ordinary tumbler, offset spatula and a portion of icing."
MAX_STEPS = 720
PHYSICS_HZ = 240
BASES = station.BASES
STOWED = {name: READY for name in BASES}

TABLE = .7475
BAG = np.array([.288, .022, TABLE + .185])
BAG_YAW = np.deg2rad(83.)
BAG_ROTATION = np.array([
    [np.cos(BAG_YAW), -np.sin(BAG_YAW), 0.],
    [np.sin(BAG_YAW), np.cos(BAG_YAW), 0.],
    [0., 0., 1.],
])
CUFF_HALF_LENGTH = .055
CUFF_HALF_GAP = .030
CUFF_THICKNESS = .0045
CUFF_HEIGHT = .035
CUP_HEIGHT = .142
TIP_Z = -.177
CUFF_K = 145.0
SPATULA = np.array([.205, -.251, TABLE + .022])
START_YAW = np.deg2rad(-110.)
TOOL_START_R = np.array([
    [np.cos(START_YAW), -np.sin(START_YAW), 0.],
    [np.sin(START_YAW), np.cos(START_YAW), 0.],
    [0., 0., 1.],
])
HANDLE_GRIP = np.array([-.024, 0., .023])
BLADE_X0 = .035
BLADE_X1 = .127
BLADE_WIDTH = .027
BLADE_Z = .009
FLEX_K = 3400.0

ICING_MASS_G = 30.0
SEGMENTS = 8
ICING_XS = np.linspace(.058, .113, SEGMENTS)
