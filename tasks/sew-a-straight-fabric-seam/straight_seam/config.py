"""Metre-scale geometry and target tolerances for the straight seam."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "straight_seam"
INSTRUCTION = (
    "Sew the two swatches together with a straight lap seam, 6.5 mm from the orange "
    "edge. Lift the joined swatches by the orange one to check the seam holds."
)
DESCRIPTION = (
    "A domestic straight-stitch sewing machine with a raised presser foot, "
    "a start/stop button, two separate canvas swatches and a 6.5 mm seam guide."
)
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
TABLE = station.TABLE_TOP
BED = TABLE + 0.040
MAX_STEPS = 850
PHYSICS_HZ = 240
CHECK_DT = 0.05
LENGTH, WIDTH, THICK = 0.145, 0.140, 0.0016
MACHINE_X_SHIFT = -0.070
NEEDLE_XY = np.array([0.435 + MACHINE_X_SHIFT, 0.0])
NEEDLE_BASE = np.array([*NEEDLE_XY, BED - 0.0019])
FOOT_BASE = np.array([0.442 + MACHINE_X_SHIFT, 0.0, BED + 0.0032])
LEVER = np.array([0.356 + MACHINE_X_SHIFT, -0.131, BED + 0.102])
BUTTON = np.array([0.385 + MACHINE_X_SHIFT, -0.250, BED + 0.113])
UPPER = np.array([0.244, 0.154, BED + THICK / 2 + 0.001])
LOWER = np.array([0.239, -0.108, BED + THICK / 2 + 0.001])
UPPER_WORK = np.array([0.369 + MACHINE_X_SHIFT, WIDTH / 2 - 0.0065, BED + 1.5 * THICK])
LOWER_WORK = np.array([0.369 + MACHINE_X_SHIFT, -0.047, BED + THICK / 2])
UPPER_CLIP = np.array([-0.059, 0.060, 0.009])
LOWER_CLIP = np.array([-0.059, -0.060, 0.009])
FOOT_TRAVEL, LEVER_TRAVEL, NEEDLE_TRAVEL = 0.039, 0.019, 0.019
NEEDLE_RATE = 4.0
FEED_RATE = 0.016
SEAM_ALLOWANCE = 0.0065
SEAM_LENGTH = 0.073
TILT = np.deg2rad(-27)
FRONT_APPROACH = np.array(
    [
        [np.cos(TILT), 0, np.sin(TILT)],
        [0, 1, 0],
        [-np.sin(TILT), 0, np.cos(TILT)],
    ]
)
RIGHT_GRIP = FRONT_APPROACH @ np.diag([1.0, -1.0, -1.0])
LEFT_GRIP = RIGHT_GRIP @ np.diag([-1.0, -1.0, 1.0])

PROOF_ANGLE = np.deg2rad(-38)
PROOF_HAND = np.array(
    [
        [-1, 0, 0],
        [0, np.cos(PROOF_ANGLE), np.sin(PROOF_ANGLE)],
        [0, np.sin(PROOF_ANGLE), -np.cos(PROOF_ANGLE)],
    ]
)
