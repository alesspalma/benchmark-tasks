"""Fixed, full-size two-inch gravity flush valve and service bench."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "flapper_chain"
INSTRUCTION = "Reconnect the flapper chain to the trip lever directly above the flapper with about half an inch of slack. Use the outside handle to test the flush and let the flapper reseat."
DESCRIPTION = "An open gravity toilet tank with a disconnected flapper-chain clip, a front trip handle, and a low clean-water test charge."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
MAX_STEPS = 640
PHYSICS_HZ = 240
CHECK_DT = 0.05
TABLE = 0.7475
FLOOR = 0.811
RIM = 1.018
WALL_LEFT = 0.208
WALL_FRONT = 0.350
WALL_BACK = 0.562
LEVER = np.array([0.345, 0.124, 0.996])
HANDLE = np.array([-0.040, 0.061, 0.0])
PINS = np.array([[0.076, -0.078, 0.0], [0.096, -0.150, 0.0], [0.107, -0.217, 0.0]])
CORRECT = 1
CLIP_HOME = np.array([0.465, -0.041, 0.9875])
CLIP_GRIP = np.array([0.013, 0.0, 0.027])
CLIP_BODY_CENTER = np.array([0.012, 0.0, 0.012])
HOOK_REST = np.array([0.0, 0.0, -0.005])
CLIP_CHAIN = np.array([0.007, 0.0, -0.006])
FLAPPER = np.array([0.487, -0.026, 0.834])
OUTLET = np.array([0.442, -0.026, 0.826])
FLAPPER_ANCHOR = np.array([-0.059, 0.0, 0.004])
CHAIN_LENGTH = (
    float(
        np.linalg.norm(
            LEVER + PINS[CORRECT] + HOOK_REST + CLIP_CHAIN - FLAPPER - FLAPPER_ANCHOR
        )
    )
    + 0.0127
)
CHAIN_LINK_COUNT = 21
CAPACITY_L = 5.20
WATER_FOOTPRINT = (0.194, 0.396)
CATCH_FOOTPRINT = (0.260, 0.378)
DRAIN_DIAMETER = 0.0508
