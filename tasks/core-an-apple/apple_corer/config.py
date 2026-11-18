"""Fixed kitchen setup, tissue approximation parameters and scoring limits."""

import numpy as np

KEY = "apple_corer"
DESCRIPTION = "Use the tube blade to extract the central core of a whole apple."
INSTRUCTION = "Hold the apple steady and remove its core with the corer. Leave the cored apple upright on the board and lift the core clear."
PHYSICS_HZ = 240
MAX_STEPS = 680
TABLE = .7475
BOARD_SIZE = (.248, .172, .010)
BOARD = np.array([.385, .017, TABLE])
APPLE = np.array([.386, .029, TABLE + BOARD_SIZE[2] + .001])
FRUIT_HEIGHT = .066
FRUIT_RADIUS = .037
CUT_RADIUS = .0105
CORE_RADIUS = .0089
CORE_CENTER = .033
CORE_HALF_LENGTH = .030
GRIP = .149
TOOL_START = np.array([.289, -.218, TABLE + .0127])
TOOL_ORIENT = (0., 0.7071067811865475, 0., 0.7071067811865476)
