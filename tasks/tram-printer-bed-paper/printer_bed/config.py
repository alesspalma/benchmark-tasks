"""Ender-3 scale geometry and cold paper-test parameters in SI units."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "printer_bed"
INSTRUCTION = (
    "Adjust the front-right wheel until the paper moves under the cold nozzle "
    "with light drag. Leave the paper and wheel released."
)
DESCRIPTION = "An open-frame filament printer with a spring-mounted build plate and a loose paper strip at the cold nozzle."
BASES = station.BASES
STOWED = {n: READY for n in BASES}
MAX_STEPS = 660
PHYSICS_HZ = 240
TABLE = .7475
BED_CENTER = np.array([.453, 0., .856])
BED_SIDE = .235
SUPPORTS = np.array([[-.085, -.085], [-.085, .085], [.085, -.085], [.085, .085]])
WHEEL = np.r_[BED_CENTER[:2] + SUPPORTS[0], .820]
WHEEL_RADIUS = .0295
WHEEL_THICKNESS = .010
NOZZLE = np.r_[WHEEL[:2], BED_CENTER[2] + .000083]
SCREW_LEAD = .0007
TARGET_WHEEL_DEG = -50.
RESET_ANGLE = np.deg2rad(48.)
SUPPORT_START = -SCREW_LEAD * RESET_ANGLE / (2 * np.pi)
PAPER_NEAR = np.array([.310, .044, BED_CENTER[2] - .000075])
PAPER_AXIS = np.r_[WHEEL[:2] - PAPER_NEAR[:2], 0.]
PAPER_AXIS /= np.linalg.norm(PAPER_AXIS)
PAPER_YAW = np.arctan2(PAPER_AXIS[1], PAPER_AXIS[0])
PAPER_SIZE = np.array([.179, .058, .000108])
PAPER_CENTER = PAPER_NEAR + .084 * PAPER_AXIS
PAPER_GRIP = PAPER_NEAR + np.array([0., 0., .0030])
NOZZLE_STIFFNESS = 5400.
CHECK_DT = .05


def support_plane(wheels):
    """Equilibrium plane of four equally spaced and equally stiff springs."""
    height = -SCREW_LEAD * np.asarray(wheels) / (2 * np.pi)
    height[0] += SUPPORT_START
    x, y = SUPPORTS.T
    return np.array([height.mean(), np.dot(y, height) / (y @ y), -np.dot(x, height) / (x @ x)])
