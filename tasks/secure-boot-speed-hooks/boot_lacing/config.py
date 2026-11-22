"""Dimensions for the full-size boot and loose lace run, in metres."""

import numpy as np

from yam_arena.constants import READY
from yam_arena.station import BASES

KEY = "boot_lacing"
DESCRIPTION = "Full-size hiking boot on a solid foot last with two locking lace hooks."
INSTRUCTION = "Tighten the forefoot laces, seat them in the two locking hooks, and release both ends. Leave the instep snug and the ankle free."
TABLE = .7475
BOOT_RISE = .082
MAX_STEPS = 740
PHYSICS_HZ = 480
CHECK_DT = .05
STOWED = {key: np.asarray(READY) for key in BASES}
SIDES = {"L": 1, "R": -1}
LACE_RADIUS = .0023
LINK_LENGTH = .0075
LINK_COUNT = 24
PANEL_ROOT = np.array([.388, 0., .830+BOOT_RISE])
OPEN = .013
ANCHOR = np.array([.454, .040, .877+BOOT_RISE])
LOCK = np.array([.410, .031, .889+BOOT_RISE])
THROAT_TOP = {"L": .002, "R": .006}
LOWER_GUIDES = ((.480, .047, .960), (.508, .043, .952))


def lock_center(side):
    return LOCK * [1., SIDES[side], 1.]


def lock_axes(sign):
    tangent = np.array([.55, -.85*sign, 0.])
    tangent /= np.linalg.norm(tangent)
    return np.column_stack((tangent, np.cross([0., 0., 1.], tangent), [0., 0., 1.]))
