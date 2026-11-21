"""Robot-relative dimensions for the liner, replacement ring and installed rod."""

from math import radians

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "snap_liner"
INSTRUCTION = (
    "Reattach the loose corner of the shower liner to the rod using the open ring on the counter. "
    "Snap the ring shut and leave the corner hanging from it."
)
DESCRIPTION = "One shower liner corner hangs loose. A replacement snap ring lies open in a counter rest."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
CHECK_DT = 0.05
MAX_STEPS = 560
TABLE = 0.7475
ROD = np.array([0.399, 0.0, 1.039])
ROD_RADIUS = 0.0127
CORNER_PIVOT = np.array([0.404, 0.138, 0.992])
EYELET = np.array([0.0, -0.165, 0.0])
GRIP_HEM = np.array([-0.001, -0.105, 0.003])
LOOSE = 0.505
EYELET_RADIUS = 0.0108
EYELET_OUTER = 0.0162
RING_X = 0.039
RING_Z = 0.048
WIRE_RADIUS = 0.0045
GATE_OPEN = radians(-101.0)
GATE_TRANSFER = radians(-93.0)
PIVOT_ANGLE = 35.0
RING_START = np.array([0.268, -0.170, 0.761])
RING_FLAT_QUAT = (-np.sqrt(0.5), 0.0, 0.0, np.sqrt(0.5))
RING_MOUNT = np.array([0.420, -0.027, 1.014])
RING_GRIP = np.array([-0.041, 0.0, 0.010])


def ring_point(degrees):
    theta = radians(degrees)
    return np.array([RING_X * np.cos(theta), 0.0, RING_Z * np.sin(theta)])


GATE_PIVOT = ring_point(PIVOT_ANGLE)
GATE_TAB = ring_point(337.0) - GATE_PIVOT + [0.005, 0.0, -0.001]
