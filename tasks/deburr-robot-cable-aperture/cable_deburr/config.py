"""Bench coordinates and machining dimensions in SI units."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "cable_deburr"
INSTRUCTION = (
    "Deburr the upper edge all the way around the cable opening in the robot tool plate. "
    "Leave the opening clear and the plate undamaged, then put the plate and deburring tool back in their rests."
)
DESCRIPTION = "Two-handed finishing of the inside edge of a loose machined robot tool plate."
BASES = station.BASES
STOWED = {name: READY for name in BASES}
PHYSICS_HZ = 240
MAX_STEPS = 2200
TABLE = .7475
PLATE_X = .154
PLATE_Y = .168
PLATE_THICKNESS = .010
STOCK_SUPPORT = .038
PLATE_START = np.array([.373, -.089, TABLE + STOCK_SUPPORT + PLATE_THICKNESS / 2])

PLATE_GRASP = np.array([-.044, .072, 0.])
TOOL_START = np.array([.303, -.365, TABLE + .0025])
TOOL_GRASP = np.array([0., 0., .083])
# Reference height for constructing the cutting toe.
KNIFE = np.array([0., 0., .003])
# Cutting nose in the live swivel frame. The lower edge is at 1.5 mm.
TOE = np.array([.0040, 0., .0015])
FLEX_K = 4400.
FORCE_MIN = 2.4
FORCE_MAX = 23.
COVERAGE_LIMIT = .98
CAPSULE_RADIUS = .021
CAPSULE_OFFSET = .023
PORT_CENTER = np.array([0.0, -.040])
PARALLEL_OFFSET_X = .062
CELLS = 52

def outline():
    """Clockwise work coordinates with uniform arc-length samples."""
    r, c = CAPSULE_RADIUS, CAPSULE_OFFSET
    path = np.concatenate((
        np.column_stack((np.linspace(-c, c, 12, endpoint=False), np.full(12, r))),
        np.array([[c + r * np.sin(a), r * np.cos(a)]
                  for a in np.linspace(0, np.pi, 14, endpoint=False)]),
        np.column_stack((np.linspace(c, -c, 12, endpoint=False), np.full(12, -r))),
        np.array([[-c - r * np.sin(a), -r * np.cos(a)]
                  for a in np.linspace(0, np.pi, 14, endpoint=False)]),
    ))
    return path + PORT_CENTER
