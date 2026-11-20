"""Dimensions and fixed reset for the dual spinning-pad service operation."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "robot_mop"
INSTRUCTION = (
    "Turn over the robot vacuum and remove both mop-pad assemblies. "
    "Peel off the cloth pads. Put the cloth pads in the blue tray and the plastic carriers in the green tray."
)
DESCRIPTION = "Service two magnetically held, hook-and-loop mop-pad assemblies on a full-size robot cleaner."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
MAX_STEPS = 1400
CHECK_DT = 0.05

PIVOT = np.array([0.390, 0.0, 0.974])
TABLE = 0.7475
PAD_X = -0.079
PAD_Y = 0.075
PLATE_Z = -0.059
CLOTH_Z = -0.069
CARRIER_RADIUS = 0.060
CLOTH_RADIUS = 0.074
GRIP_RADIUS = 0.089
GRIP_Y = 0.237
SORT_Y = 0.329
CLOTH_TRAY_X = 0.141
CLOTH_TRAY_WIDTH = 0.282
CLOTH_TRAY_LENGTH = 0.950
CARRIER_TRAY_X = 0.467
SORT_Z = TABLE + 0.008
SIDES = ("L", "R")
PARTS = tuple(
    name
    for side in SIDES
    for name in (f"{side}Carrier", f"{side}FabricFront", f"{side}FabricBack")
)
