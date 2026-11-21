"""Immersion blender, jug, and bench dimensions in metres."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "soup_blend"
INSTRUCTION = "Blend the soup in the jug, then set the blender on the rest."
DESCRIPTION = "A tall jug of soup next to an immersion blender and a countertop rest."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
MAX_STEPS = 650
CHECK_DT = 0.05

TABLE_Z = 0.7475
JUG_RESET = np.array([0.368, 0.031, TABLE_Z])
JUG_RADIUS = 0.074
JUG_HEIGHT = 0.153
HANDLE = np.array([-0.012, 0.122, 0.101])
SURFACE_Z = 0.113
REST = np.array([0.374, -0.166, TABLE_Z + 0.009])
BLEND_Z = TABLE_Z + 0.020

GUARD_RADIUS = 0.029
GUARD_TOP = 0.045
GRIP_CENTER = np.array([-0.004, 0.0033, 0.236])
BUTTON = np.array([-0.005, 0.0245, 0.236])
BUTTON_TRAVEL = 0.008
RUN_TRAVEL = 0.0049
RELEASE_TRAVEL = 0.0039
ROTOR_RAD_S = 130.0

RINGS = 3
SECTORS = 16
MIX_GOAL = 0.19
