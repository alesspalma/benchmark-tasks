"""Fixed working geometry and reduced-material parameters, in SI units."""

import numpy as np

KEY = "cookie_press"
DESCRIPTION = (
    "Hold the hand press flat and use its lever to deposit three shaped cookies."
)
INSTRUCTION = (
    "Use the cookie press to make three separate shaped cookies on the baking sheet."
)
PHYSICS_HZ = 240
MAX_STEPS = 600

TABLE = 0.7475
TRAY = np.array([0.385, 0.018, TABLE])
TRAY_SIZE = (0.251, 0.343)
TRAY_TOP = TABLE + 0.0105
START = TRAY + [-0.032, 0.100, 0.0105]
SPOTS = (
    START[:2],
    TRAY[:2] + [-0.065, 0.032],
    TRAY[:2] + [-0.012, -0.062],
)
BASE_RADIUS = 0.032
BARREL_RADIUS = 0.0245
BARREL_GRIP_Z = 0.093
PIVOT = np.array([0.0, -0.006, 0.169])
LEVER_VECTOR = np.array([0.0, -0.073, 0.038])
LEVER_PRESS = 0.53
LEVER_LIMIT = 0.55
DOSE_G = 5.7
DOUGH_G = 26.0
COOKIE_DIAMETER = 0.037
COOKIE_HEIGHT = 0.0085
SEAT_TOLERANCE = 0.0023
