"""Dimensions and ordinary Shore A measurement conditions in SI units."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "shore_hardness"
INSTRUCTION = "Place the rubber coupon flat on the steel platen. Take five readings with the handheld Shore A durometer. Measure at least 12 mm from the edges and 6 mm apart, holding the foot flat and still for one second at each point."
DESCRIPTION = "A loose, nominal 60 Shore A rubber coupon, a firm steel platen and an upright handheld Shore A durometer."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
MAX_STEPS = 820
PHYSICS_HZ = 480
TABLE = .7475

PLATEN = np.array([.410, .018, TABLE + .010])
PLATEN_SIZE = (.140, .115, .010)
COUPON_SIZE = np.array([.100, .070, .010])
TRAY_HEIGHT = .012
START_COUPON = np.array([.303, .148, TABLE + TRAY_HEIGHT + COUPON_SIZE[2] / 2])
TEST_COUPON = PLATEN + [0., 0., COUPON_SIZE[2] / 2]
TOOL = np.array([.331, -.162, TABLE + .016])
TOOL_GRIP_HEIGHT = .058
FOOT_RADIUS = .016
STROKE = .0025
SPRING_RATE = 3000.
SPRING_PRELOAD = .55
RUBBER_RATE = 5050.
RUBBER_DAMPING = 2.5
SPACING = .006
EDGE_MARGIN = .012
TILT_DEG = 5.
DWELL = 1.0
MIN_READING = 55.
MAX_READING = 65.
SITES = ((-.015, -.009), (.015, -.009), (.015, .009), (0., 0.), (-.015, .009))
