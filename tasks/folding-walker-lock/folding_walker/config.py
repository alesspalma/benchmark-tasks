"""Walker dimensions in metres and angles in radians."""

import math

import numpy as np

KEY = "folding_walker"
INSTRUCTION = "Get the folding walker ready for the person in the chair."
DESCRIPTION = "A full-size two-button folding walker stands on the floor with its open side facing the seated person."
PHYSICS_HZ = 480
MAX_STEPS = 400
BASES = {"left": (0.0, 0.305, 0.75), "right": (0.0, -0.305, 0.75)}
STOWED = {side: (0.0, 1.0, 1.0, -0.6, 0.0, 0.0) for side in BASES}
SIDES = {"left": 1, "right": -1}

FLOOR = -0.0125
START = np.array([0.215, 0.0, FLOOR + 0.0015])
HINGE_HALF_WIDTH = 0.219
GRIP_HEIGHT = 0.825
GRASP_DISTANCE = 0.207
BACK_FOOT_DISTANCE = 0.395
BACK_FOOT_SPLAY = 0.022
FRONT_FOOT_RADIUS = 0.016
START_FOLD = {"left": math.radians(23), "right": math.radians(18)}
BUTTON_TRAVEL = 0.007
LATCH_SEAT = math.radians(1.25)
LATCH_CLEARANCE = math.radians(0.45)
LATCH_STIFFNESS = 285.0
LATCH_DAMPING = 3.0
LATCH_MAX_TORQUE = 14.0
PROBE_FORCE = 8.0
CHAIR_X = 0.990
