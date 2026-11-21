"""Lifted leg, swivel and replacement dimensions in metres."""

import numpy as np
from yam_arena.constants import READY
from yam_arena.station import BASES

KEY = "chair_glide"
INSTRUCTION = (
    "Use the flat-blade tool to remove the worn felt cap from the chair foot. "
    "Snap on the new felt cap so it fits flush and the glide can still swivel."
)
DESCRIPTION = (
    "Replace the worn felt underside of a swivel chair foot without removing the foot."
)
STOWED = dict.fromkeys(BASES, READY)
TABLE = 0.7475
FOOT = np.array([0.335, -0.045, 1.118])
TOOL_HOME = np.array([0.285, -0.195, TABLE + 0.011])
FRESH_HOME = np.array([0.301, -0.286, TABLE + 0.024])
MALLET_HOME = np.array([0.383, -0.334, TABLE + 0.015])
CAP_HEIGHT = 0.008
CAP_RADIUS = 0.0151
GLIDE_RADIUS = 0.0128
SNAP_RADIUS = 0.0115
SWIVEL_LIMIT = 15.0
PHYSICS_HZ = 240
CHECK_DT = 0.05
MAX_STEPS = 760

SNAP_ANCHORS = np.array(
    [
        [SNAP_RADIUS, 0, 0],
        [0, SNAP_RADIUS, 0],
        [-SNAP_RADIUS, 0, 0],
        [0, -SNAP_RADIUS, 0],
    ]
)
