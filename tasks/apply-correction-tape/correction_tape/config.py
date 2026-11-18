"""Fixed tabletop layout in metres."""

import numpy as np

KEY = "correction_tape"
DESCRIPTION = "Apply a side-action dry correction strip to one entry on a clipboard."
INSTRUCTION = "Cover the marked HOLD entry with the correction tape. Keep PACK and SHIP readable. Set the dispenser back beside the clipboard."
PHYSICS_HZ = 240
MAX_STEPS = 480
TABLE = .7475
BOARD_START = np.array([.382, -.093, TABLE + .001])
BOARD_SIZE = (.232, .165, .006)
PAPER_SIZE = (.210, .148)
PAPER_Z = .008
ROW_Y = .007
ENTRY_CENTERS = (-.040, 0., .040)
ENTRY_NAMES = ("PACK", "HOLD", "SHIP")
TAPE_WIDTH = .004
TOOL_START = np.array([.302, -.270, TABLE + .0095])
TOOL_GRIP = np.array([.042, 0., .006])
TOOL_TOE = np.array([-.0019, 0., -.005])
NOSE_STIFFNESS = 3600.0
NOSE_LIMIT = .0025
