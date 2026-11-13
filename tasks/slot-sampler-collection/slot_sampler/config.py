"""Fixed real-scale laboratory sampler and free-flowing calibration granules."""

import numpy as np
from yam_arena import station

KEY = "slot_sampler"
TITLE = "Collect a granule sample with a closing slot probe"
INSTRUCTION = "Use the closing sampling probe to collect at least 5 grams of granules from the tub into the sample jar. Leave the probe supported, the sample contained, and both hands released."
DESCRIPTION = "A 300 mm two-sleeve sampling probe has a rotating window near its closed pointed end. A tub contains free-flowing granules. An empty sample jar and a probe stand are on the table."
MAX_STEPS = 2400
TOP = station.TABLE_TOP
TUB = np.array([0.43, 0.02, TOP])
JAR = np.array([0.35, 0.30, TOP])
PROBE = np.array([0.43, 0.38, TOP + 0.021])
RADIUS = 0.0034
MASS = 0.00018
WINDOW = (0.015, 0.085)
TUB_INNER = (0.105, 0.105)
TUB_H = 0.145
JAR_R = 0.040
JAR_H = 0.070
PROBE_LENGTH = 0.300
