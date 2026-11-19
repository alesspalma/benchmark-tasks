"""Geometry and physical parameters for the water-only wet-lab task."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "pipette_transfer"
INSTRUCTION = "Use the three-valve pipette bulb to transfer 5.0 mL of water from the blue source beaker to the clear receiving beaker. Keep water out of the bulb and off the worktop."
DESCRIPTION = (
    "A is the air valve at the top of the bulb. S is the short suction branch below it. "
    "E is the delivery valve on the longer branch, farther from the robot bases. "
    "The dark graduation is at 5.0 mL."
)
MAX_STEPS = 700
PHYSICS_HZ = 240
BASES = station.BASES
STOWED = {name: READY for name in BASES}
TABLE = station.TABLE_TOP
SOURCE = np.array([.331, .070, TABLE])
RECEIVER = np.array([.331, -.088, TABLE])
# Local pipe coordinates are measured upward from the delivery tip.
PARKED = SOURCE + [0., .069, .079]
ASPIRATE = SOURCE + [0., 0., .019]
DELIVER = RECEIVER + [0., 0., .037]
CLEAR_RIM = .077
PIPETTE_LENGTH = .350
GRADUATED_BORE = 50e-6                   # m²; 5 mL corresponds to a 100 mm column.
TIP_RESERVOIR = .07                      # mL in the taper below the graduate bore.
USABLE_VOLUME = 10.0                     # mL
WATER_START = 62.0                       # mL
TARGET = 5.0
TOLERANCE = .20                          # mL
SPILL_LIMIT = .20                        # mL
CUP_RADIUS = .023
CUP_HEIGHT = .053
CUP_WALL = .0018
CUP_BASE = .0030
BODY_GRIP = np.array([0., 0., .226])
SLEEVE_YAW = float(np.arctan(.54))
HANGER_Z = .295
BULB = np.array([0., 0., .378])
A_VALVE = np.array([0., .009, .437])
S_VALVE = np.array([-.013, -.028, .323])
E_VALVE = np.array([.038, -.030, .322])
VALVES = {"Air": A_VALVE, "Suction": S_VALVE, "Delivery": E_VALVE}
BULB_HALF_WIDTH = .027
BULB_TRAVEL = .027
VALVE_TRAVEL = .004
ATMOSPHERIC_PA = 101325.
VOLUME_AT_REST = 50.0                    # mL
COMPRESSION_PER_METRE = 700.             # mL/m
