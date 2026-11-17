"""Capsule coffee machine with passive loading/ejection and removable collector."""

import numpy as np
from yam_arena import station

KEY = "capsule_recovery"
TITLE = "Empty a capsule coffee machine's collector and load a fresh capsule"
INSTRUCTION = "Clear the used capsules into the recycling tub and restore the coffee machine with its collection bin fitted, the fresh capsule correctly seated, and its head fully closed. Release both hands when finished."
DESCRIPTION = "This is a capsule coffee machine with a rear water reservoir, front coffee outlet, drip grille, hinged loading head and removable used-capsule collector. One used capsule is in the loading carrier and six are in the collector. A fresh gold capsule rests on a narrow pedestal on the left, beside the green recycling tub. The machine is cold and unpowered for this mechanical clearing and reloading trial."
MAX_STEPS = 2400
TOP = station.TABLE_TOP
HEAD_RISE = 0.040
HINGE = np.array([0.55, 0.0, TOP + 0.195 + HEAD_RISE])
HEAD_HANDLE = np.array([0.38, -0.12, TOP + 0.240 + HEAD_RISE])
SLOT = np.array([0.44, 0.0, TOP + 0.1695 + HEAD_RISE])
BIN = np.array([0.485, 0.0, TOP + 0.010])
BIN_INNER = (0.120, 0.082)
BIN_H = 0.065
BIN_WALL = 0.002
BIN_MASS = 0.065
BIN_GRIP = np.array([-0.095, 0.0, 0.060])
BIN_GRIP_SIZE = (0.030, 0.055, 0.024)
WASTE = np.array([0.37, 0.27, TOP])
WASTE_INNER = (0.30, 0.30)
WASTE_H = 0.055
FRESH = np.array([0.56, 0.25, TOP + 0.070])
OPEN_ANGLE = 0.0
USED_COUNT = 7
CAPSULE_SLEEP_THRESHOLD = 0.0002  # Native mass-normalized kinetic energy, m^2/s^2.
