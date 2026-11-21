"""Fixture, physical target and camera geometry in SI units."""

import numpy as np
from yam_arena.constants import READY
from yam_arena.station import BASES

KEY = "stereo_calibration"
INSTRUCTION = (
    "Hold the checkerboard by its blank edges and show the whole grid to both "
    "stereo cameras. Collect eight steady paired views covering different parts of "
    "the image at different tilts and distances, then return the board to its stand."
)
DESCRIPTION = "A rigid checkerboard, a return stand and two synchronized cameras for a robot vision rig."
STOWED = dict.fromkeys(BASES, READY)
TABLE = .7475
MAX_STEPS = 760
PHYSICS_HZ = 240
CHECK_DT = .05
SIZE = (.008, .310, .190)
SQUARE = .026
GRID = (7, 5)
STAND_TOP = TABLE + .008
BOARD = np.array([.354, 0., STAND_TOP + SIZE[2]/2])
PATTERN_X = SIZE[0]/2 + .0003
CAMERA_X = .801
CAMERA_Z = .920
BASELINE = .108
CAMERAS = (
    (CAMERA_X, BASELINE/2, CAMERA_Z),
    (CAMERA_X, -BASELINE/2, CAMERA_Z),
)
IMAGE_SIZE = (640, 360)
FOCAL = 12.0
APERTURE = 20.955
ACCEPTED_PAIRS = 8
