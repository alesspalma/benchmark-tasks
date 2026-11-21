"""Reset dimensions in metres for a tabletop crayon rubbing."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "crayon_rubbing"
INSTRUCTION = (
    "Make a clear rubbing of the raised fern with the broad side of the blue wax block. "
    "Steady the loose paper while rubbing. Leave the paper flat and put the wax block "
    "down beside it."
)
DESCRIPTION = "Transfer a classroom relief-plate pattern to loose A5 paper using a blue wax block."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
TABLE = .7475
PAPER_CENTER = np.array([.327, 0.])
PAPER_SIZE = np.array([.148, .210])
PAPER_MASS = .080 * np.prod(PAPER_SIZE)
PAPER_THICKNESS = .00065
PLATE_SIZE = (.126, .200)
PLATE_BASE = .003
RELIEF = .0012
RUBBER_FEET = .0015
PLATE_TOP = TABLE + RUBBER_FEET + PLATE_BASE + RELIEF
PAPER_Z = PLATE_TOP + PAPER_THICKNESS
CRAYON_SIZE = np.array([.041, .023, .012])
CRAYON_START = np.array([.370, -.174])
CRAYON_REST = .002
TCP_FROM_WAX_BASE = .016
CRAYON_COLOR = (.035, .109, .50)
PAPER_COLOR = (.94, .953, .94)

PHYSICS_HZ = 240
MAX_STEPS = 620
CHECK_DT = .05
