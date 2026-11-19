"""Full-scale geometry and reset poses in metres."""

import math

import numpy as np

KEY = "lint_roller"
DESCRIPTION = "A household adhesive lint roller with one used outer sheet and a lint-covered cloth."
INSTRUCTION = "Remove the used outer sheet from the lint roller, then use the fresh layer to clean the lint off the blue cloth."
TABLE = 0.7475
PHYSICS_HZ = 240
MAX_STEPS = 480

RADIUS = .0254
WIDTH = .1016
SHEET_LENGTH = math.tau * RADIUS + .0032
TAB_Y = -.018
TAB_SIZE = (.008, .025, .004)
TAB_MASS = .004
REST = .030
ROLLER = np.array([.259, -.034, TABLE + REST + RADIUS])
SEAM = np.array([.006, TAB_Y, RADIUS])
TAB_OFFSET = np.array([.010, TAB_Y, RADIUS + .005])
TAB = ROLLER + TAB_OFFSET
HANDLE_Y = .124
CLOTH_SIZE = np.array([.220, .218, .004])
CLOTH = np.array([.426, .020, TABLE + CLOTH_SIZE[2] / 2])
PILE = .0016
DIRT = np.array([
    [x + shift, y + offset, .0035]
    for x, shift in ((-.063, .000), (-.030, .003), (.000, -.003), (.030, .002), (.063, -.001))
    for y, offset in ((-.031, .000), (-.012, .004), (.012, -.004), (.030, .001))
])
