"""Thorlabs SM2-size lens tube dimensions and bench coordinates, in SI units."""

import math

import numpy as np

KEY = "optic_ring"
INSTRUCTION = (
    "Hold the lens tube steady. Use the two-prong spanner to seat the "
    "recessed retaining ring against the lens without touching its clear "
    "face. Withdraw the spanner and release the tube."
)
DESCRIPTION = (
    "On a white optical inspection pad, a 2-inch circular optic rests on a "
    "shoulder in a loose 2-inch lens tube. The upper retaining ring is only "
    "finger-started below the tube mouth. A two-prong spanner is upright in "
    "its bench rest. Its two teeth match the slots in the ring."
)
MAX_STEPS = 620
PHYSICS_HZ = 480
BASES = {"left": (0., .244, .75), "right": (0., -.244, .75)}
TABLE = .750
FIXTURE_TOP = TABLE + .018
TUBE = np.array([.316, .017, FIXTURE_TOP + .0002])
TOOL = np.array([.299, -.143, TABLE + .0002])
TUBE_HEIGHT = .0467
TUBE_OUTER = .028575
TUBE_BORE = .02606
LENS_RADIUS = .02540
LENS_THICKNESS = .0040
LENS_MASS = .024
LENS_SEAT = .02765
LENS_CENTER = LENS_SEAT + LENS_THICKNESS / 2
CLEAR_RADIUS = .02415
RING_THICKNESS = .00254
SLOT_BOTTOM = .00084
SLOT_HALF_WIDTH = .00170
TOOL_TEETH_RADIUS = .02492
TOOL_GRIP_Z = .075
TOOL_HEIGHT = .104

# Positive Z is toward the tube mouth. Clockwise drive advances downward.
PITCH_PER_RADIAN = .000635 / math.tau
RING_CLEARANCE = .000104
RING_BOTTOM = LENS_SEAT + LENS_THICKNESS + RING_CLEARANCE
THREAD_TRAVEL = .00042
TURN_FRICTION = .012
THREAD_FORCE_LIMIT = 15.0

STOWED = {side: (0., 1., 1., -.6, 0., 0.) for side in BASES}
