"""Fixed reset geometry and the self-locking reel's constitutive parameters."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "tape_rewind"
INSTRUCTION = "Rewind the tape measure under control and leave the hook seated against its case."
DESCRIPTION = "The tape starts extended and locked. Press its release button and guide the return of the end hook."
MAX_STEPS = 380
PHYSICS_HZ = 240
BASES = station.BASES
STOWED = {name: READY for name in BASES}
TABLE = .7475
CASE = np.array([.377, .130, TABLE + .001])
CASE_SIZE = (.043, .076, .064)
APERTURE = np.array([0., -.039, .024])
BUTTON = np.array([0., -.004, .067])
BUTTON_HEIGHT = .008
BUTTON_TRAVEL = .007
UNLOCK_START = .0025
UNLOCK_END = .005
EXPOSED_AT_RESET = .250
BLADE_WIDTH = .0254
HOOK_WIDTH = .033
HOOK_HEIGHT = .018
SCORE_REWIND_PER_FRAME = .035
HOME_GAP = .004
RELEASE_GAP = .017
SPRING_PRELOAD = 1.40
SPRING_RATE = 1.5
REEL_DAMPING = .55
BRAKE_CAPACITY = 8.
