"""Dimensions, mass and passive loads for an axle-tension table-tennis net."""

import math

import numpy as np

KEY = "tennis_net"
INSTRUCTION = "Set the table-tennis net to playing height and tension, then let go."
DESCRIPTION = (
    "A full-size table-tennis net is fitted to both side posts. The near post is "
    "too high and the net cord is slack. A ratcheting knob winds the cord. Its "
    "near guide slides vertically against friction."
)
MAX_STEPS = 460
PHYSICS_HZ = 480
BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}
STOWED = {name: (0., 1., 1., -.6, 0., 0.) for name in BASES}

TABLE_HEIGHT = .760
TABLE_WIDTH = 1.525
TABLE_LENGTH = 2.740
TABLE_THICKNESS = .025
POST_X = .425
POST_SPAN = 1.830
NEAR_HEIGHT = .1665
POST_TRAVEL = .014
HEIGHT_STOP = NEAR_HEIGHT - POST_TRAVEL
POST_AXIS = np.array([POST_X, .030, TABLE_HEIGHT + NEAR_HEIGHT])
FAR_AXIS = np.array([POST_X + POST_SPAN, .030, TABLE_HEIGHT + HEIGHT_STOP])
TABLE_EDGE = POST_X + (POST_SPAN - TABLE_WIDTH) / 2.

SPINDLE = np.array([POST_X - .047, -.055, TABLE_HEIGHT + .077])
SPOOL_RADIUS = .008
CRANK_RADIUS = .032
CRANK_ANGLE = math.radians(-100.)
KNOB_HEIGHT = .026
KNOB_WIDTH = .021
TOOTH_COUNT = 21
TOOTH = math.tau / TOOTH_COUNT
ANGULAR_STOP = 4 * TOOTH + math.radians(4.)
CORD_SLACK = .00075
CORD_STIFFNESS = 6200.
CORD_LOAD_PER_METRE = .24
MIDPOINT_GAUGE_N = .100 * 9.81

def tension(angle):
    """Elastic axial reaction for the amount of cord wound onto the axle."""
    return max(0., CORD_STIFFNESS * (SPOOL_RADIUS * angle - CORD_SLACK))

def net_height(relative_x, near_height, far_height, force, weight=0.):
    """Small-sag cord profile and optional central measurement weight."""
    fraction = np.asarray(relative_x, dtype=float) / POST_SPAN
    horizontal_force = max(force, 1.)
    sag = CORD_LOAD_PER_METRE * POST_SPAN**2 / (2. * horizontal_force)
    unloaded = near_height + (far_height - near_height) * fraction - sag * fraction * (1. - fraction)
    return np.maximum(.002, unloaded - weight * POST_SPAN / (2. * horizontal_force) * np.minimum(fraction, 1. - fraction))
