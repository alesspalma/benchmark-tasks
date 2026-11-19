"""Full-size inert cassette and roller dimensions in SI units."""

import numpy as np

KEY = "blot_cassette"
INSTRUCTION = "Hold the wet layers aligned, roll the trapped air out, then close the cassette."
DESCRIPTION = "A full-size inert training cassette has an open clear cover, a wet laminate with three visible voids, and a loose rubber brayer."
MAX_STEPS = 540
PHYSICS_HZ = 240
BASES = {"left": (0., .305, .75), "right": (0., -.305, .75)}
TABLE = .7475
ORIGIN = np.array([.365, .025, .826])
SHEET_LENGTH = .100
SHEET_WIDTH = .075
SHEET_Z = .017
ROLLER_R = .014
ROLLER_WIDTH = .106
ROLLER_START = np.array([ORIGIN[0], -.116, ORIGIN[2] + SHEET_Z + ROLLER_R])
GRIP_HEIGHT = .081
SHEAR_LIMIT = .012
SHEAR_STIFFNESS = 80.
CUSHION_STIFFNESS = 1800.
LID_TRAVEL = .127
GAPS = ((-.029, -.022, .007), (.012, -.002, .008), (.028, .021, .007))
