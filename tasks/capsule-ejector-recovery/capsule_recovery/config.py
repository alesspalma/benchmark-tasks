"""Cold, unpowered capsule-loading/ejection assembly at household scale."""

import numpy as np
from yam_arena import station

KEY = "capsule_recovery"
TITLE = "Clear a backed-up capsule machine and reload its carrier"
INSTRUCTION = "Clear the used capsules into the recycling tub and restore the capsule machine with its collection bin fitted, the fresh capsule correctly seated, and its head fully closed. Release both hands when finished."
DESCRIPTION = "A capsule machine has a backed-up used-capsule bin. A fresh gold capsule rests on a narrow pedestal on the left. An empty recycling tub is on the left. The apparatus is cold and unpowered."
MAX_STEPS = 2400
TOP = station.TABLE_TOP
HINGE = np.array([0.55, 0.0, TOP + 0.195])
SLOT = np.array([0.44, 0.0, TOP + 0.1695])
BIN = np.array([0.465, 0.0, TOP + 0.010])
BIN_INNER = (0.160, 0.110)
BIN_H = 0.145
WASTE = np.array([0.30, 0.20, TOP])
WASTE_INNER = (0.22, 0.22)
WASTE_H = 0.055
FRESH = np.array([0.45, 0.15, TOP + 0.070])
OPEN_ANGLE = 0.0
USED_COUNT = 7
