"""Fixed microwave and container dimensions in metres."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "lunch_vent"
INSTRUCTION = "Reheat the lunch in the microwave."
DESCRIPTION = "An open countertop microwave is ready. A lidded lunch bowl is on the turntable."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
PHYSICS_HZ = 240
MAX_STEPS = 740
CHECK_DT = .05

TABLE_Z = .7475
BOWL_RESET = np.array([.549, 0., .795])
BOWL_RADIUS = .072
HANDLE_YAW = 24.
HANDLE_RADIUS = .089
HANDLE_HEIGHT = .071
VENT_PIVOT = np.array([-.031, -.021, .083])
VENT_OPEN_DEG = 13.5
VENT_PORT_NOSE = .028
VENT_CLEARANCE_M = .0045
VENT_TAIL = np.array([.017, 0., .025])
DOOR_PIVOT = np.array([.447, .158, .879])
DOOR_OPEN_DEG = 130.
DOOR_CLOSED_DEG = 2.
DOOR_RADIUS = .210
START_POSITION = np.array([.432, -.207, .863])
START_TRAVEL = .006
START_SWITCH = .004
TRAY_CENTER = np.array([.591, 0., .794])
TRAY_RADIUS = .133
