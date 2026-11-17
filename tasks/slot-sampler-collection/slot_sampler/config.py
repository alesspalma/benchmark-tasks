"""Fixed real-scale laboratory sampler and free-flowing calibration granules."""

import numpy as np
from yam_arena import station

KEY = "slot_sampler"
TITLE = "Collect granules with a sleeve-and-notched-rod sampler"
INSTRUCTION = "Collect at least 5 grams of granules from the tub into the sample jar using the sleeve-and-notched-rod sampler. Return the closed sampler upright to its holder and release both hands."
DESCRIPTION = "This is a sleeve-and-notched-rod granule sampler. Its operating sequence is documented by Buerkle for the powder sampler Mini (https://www.buerkle.de/en/powder-sampler-mini_p5346-3010). Hold the barrel with the left hand. Press the blue top button downward with the right hand to extend the collecting rod. Granules enter the exposed side notch above the rod's integral floor. Release the button to retract the rod into the sleeve and enclose the sample before withdrawing. Over the jar, tilt the closed sampler toward its side opening and press the button again to expose the notch and empty the sample by gravity. Release the button, return the closed sampler upright to its holder, and release both hands. The rectangular pocket and coarse clean granules are specified in the setup."
MAX_STEPS = 2800
TOP = station.TABLE_TOP
TUB = np.array([0.40, 0.02, TOP])
JAR = np.array([0.22, 0.10, TOP])
PROBE = np.array([0.28, -0.06, TOP + 0.006])
RADIUS = 0.0034
MASS = 0.00018
WINDOW = (0.004, 0.020)
TUB_INNER = (0.105, 0.105)
TUB_H = 0.145
JAR_R = 0.055
JAR_H = 0.095
PROBE_LENGTH = 0.300
SUPPORT_GRASP = np.array([0.0, 0.0, 0.180])
GATE_GRASP = np.array([0.0, 0.0, 0.326])
GATE_TRAVEL = 0.025
GATE_CLOSED = 0.002
GATE_SPRING = 20.0  # N/m; constant passive linear spring throughout the episode.
GATE_PRELOAD = 0.060  # Spring rest point lies beyond the closed mechanical stop.
INNER_BACK, INNER_FRONT, INNER_HALF_WIDTH = -0.017, 0.0205, 0.017
CHAMBER_TOP = 0.020
FLOOR_SLOPE = 0.0
FLOOR_MID_HEIGHT = 0.004

SAMPLING_BASE_HEIGHT = 0.050
SAMPLING_MAX_BASE_HEIGHT = 0.055

DISCHARGE_HEIGHT = 0.050
DISCHARGE_TILT_DEG = 20.0

PORT_SIGN = -1.0  # Sampling notch faces the operator and native overview camera.

PORT_AXIS = 1  # The notch faces the right wrist camera, along tool -Y.
