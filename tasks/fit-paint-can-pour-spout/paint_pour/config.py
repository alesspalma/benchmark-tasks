"""Bench geometry and episode constants, in metres unless indicated."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "paint_pour"
INSTRUCTION = (
    "Fit the orange pour lid to the paint can and pour paint into the roller "
    "tray until it reaches the blue fill line. Return the can upright with "
    "the lid attached."
)
DESCRIPTION = (
    "A gallon paint can with a small amount of water-based paint, a separate "
    "vented snap lid and a roller tray. Press the lid around the rim and use "
    "its raised pour spout to transfer the paint."
)
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
TABLE = station.TABLE_TOP
PHYSICS_HZ = 240
CHECK_DT = 0.05
MAX_STEPS = 740
HEIGHT, INNER_RADIUS, OUTER_RADIUS, FLOOR = 0.188, 0.080, 0.082, 0.004
CAN = np.array([0.315, -0.040, TABLE + 0.006])
LID = np.array([0.35, 0.244, TABLE + 0.015])
HANDLE = np.array([0.0, -0.149, 0.090])
AUX_HANDLE = np.array([0.0, 0.144, 0.077])
LID_GRIP = np.array([-0.060, 0.0, 0.028])
LID_SEAT = np.array([0.0, 0.0, HEIGHT + 0.0026])
SPOUT_MOUTH = np.array([-0.099, 0.0, 0.038])
SPOUT_INLET = np.array([-0.076, 0.0, HEIGHT])
SPOUT_AXIS = np.array([-0.8, 0.0, 0.6])
RIM_ANGLES = np.arange(16) * np.pi / 8
CLIP_ANGLES = np.arange(4) * np.pi / 2 + np.pi / 4
CLIP_POINTS = np.column_stack(
    (0.078 * np.cos(CLIP_ANGLES), 0.078 * np.sin(CLIP_ANGLES), np.zeros(4))
)
WELL_X, WELL_Y = (0.024, 0.222), (-0.192, 0.016)
WELL_FLOOR = TABLE + 0.007
WELL_RIM = TABLE + 0.041
VOLUME_ML = 410.0
TARGET_ML = (145.0, 210.0)
MAX_SPILL_ML = 8.0
PAINT = (0.66, 0.184, 0.48)
ORANGE = (0.91, 0.255, 0.065)
DOWN = np.diag([1.0, -1.0, -1.0])
SIDE_GRIP = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
