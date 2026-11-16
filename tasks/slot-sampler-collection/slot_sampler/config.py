"""Fixed real-scale laboratory sampler and free-flowing calibration granules."""

import numpy as np
from yam_arena import station

KEY = "slot_sampler"
TITLE = "Collect a granule sample with a bottom-discharge probe"
INSTRUCTION = "Use the closing sampling probe to collect at least 5 grams of granules from the tub into the sample jar. Leave the probe supported, the sample contained, and both hands released."
DESCRIPTION = "An upright 300 mm sampling probe has two independent spring-closed shutters. Pull the blue handle upward while the head is buried to admit granules through the side port; close it before withdrawal. Hold the closed probe above the jar and pull the lower orange handle horizontally toward the robot to slide the bottom plate aside. The sample falls vertically into the jar; the blue intake stays closed. Release the orange handle to close the floor, then return the probe to its cradle."
MAX_STEPS = 2800
TOP = station.TABLE_TOP
TUB = np.array([0.43, 0.02, TOP])
JAR = np.array([0.35, 0.15, TOP])
PROBE = np.array([0.43, 0.38, TOP + 0.006])
RADIUS = 0.0034
MASS = 0.00018
WINDOW = (0.004, 0.050)
TUB_INNER = (0.105, 0.105)
TUB_H = 0.145
JAR_R = 0.055
JAR_H = 0.095
PROBE_LENGTH = 0.300
SUPPORT_GRASP = np.array([-0.010, 0.070, 0.255])
GATE_GRASP = np.array([-0.050, -0.080, 0.220])
GATE_TRAVEL = 0.050
GATE_CLOSED = 0.002
GATE_OPEN = 0.012
GATE_SPRING = 25.0  # N/m; constant passive linear spring throughout the episode.
GATE_PRELOAD = 0.020  # Spring rest point lies beyond the closed mechanical stop.
INNER_BACK, INNER_FRONT, INNER_HALF_WIDTH = -0.024, 0.0275, 0.024
CHAMBER_TOP = 0.050
FLOOR_SLOPE = 0.0
FLOOR_MID_HEIGHT = 0.004

SAMPLING_BASE_HEIGHT = 0.030
SAMPLING_MAX_BASE_HEIGHT = 0.055

# Separate spring-return bottom shutter, translating toward world -X.
DUMP_GRASP = np.array([-0.105, -0.080, 0.130])
DUMP_TRAVEL = 0.058
DUMP_CLOSED = 0.002
DUMP_SPRING = 20.0
DUMP_PRELOAD = 0.050
DISCHARGE_HEIGHT = JAR_H + 0.018
