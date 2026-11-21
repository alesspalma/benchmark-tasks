"""Dimensions in metres for the manual die handling station."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "vacuum_die"
INSTRUCTION = (
    "Unload the silicon die nearest the blue corner of the vacuum-release tray. "
    "Seat the tray on the vacuum plate, evacuate the tray, and use the vacuum pen "
    "to set that die face up in the receiving pocket. Leave the other dies in place "
    "and vent the release tray when finished."
)
DESCRIPTION = "Manual vacuum release and contact pickup of an unpackaged silicon die."
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)

TABLE = .7475
TRAY_SIZE = .102
TRAY_HEIGHT = .010
TRAY_TOP = .005
TRAY_START = np.array([.286, .099, TABLE + TRAY_HEIGHT / 2 + .0004])
PLATE = np.array([.427, .052, TABLE + .017])
TRAY_ON_PLATE = PLATE + [0., 0., TRAY_HEIGHT / 2]
SEAL_RADIUS = .039
DIE_NAMES = ("selected", "neighbor1", "neighbor2")
DIE_OFFSETS = {
    "selected": np.array([-.024, -.024, TRAY_TOP]),
    "neighbor1": np.array([-.024, .024, TRAY_TOP]),
    "neighbor2": np.array([.024, .024, TRAY_TOP]),
}
DIE_WIDTH = .012
DIE_THICKNESS = .0008
DIE_MASS = 2330 * DIE_WIDTH ** 2 * DIE_THICKNESS
RECEIVER = np.array([.402, -.108, TABLE + .006])
POCKET_WIDTH = .022

PUMP = np.array([.360, .211, TABLE])
PIVOT = np.array([.010, 0., .069])
LEVER_END = np.array([-.083, 0., .000])
PUMP_STROKE = .51
VENT = np.array([.039, 0., .067])
VENT_TRAVEL = .006
VACUUM_INHG = 25.0

PEN = np.array([.319, -.203, TABLE + .089 + .0005])
PEN_TIP = np.array([0., 0., -.089])
PEN_RADIUS = .0032
BULB_TRAVEL = .010
BULB_PURGE = .0062
BULB_RELEASE = .0033

PHYSICS_HZ = 240
MAX_STEPS = 900
CHECK_DT = .05
