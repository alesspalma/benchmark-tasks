"""Bench geometry and mechanical proof loads in SI units."""

from yam_arena.constants import READY
from yam_arena.station import BASES

KEY = "joint_lost_motion"
INSTRUCTION = (
    "Measure the lost motion of the robot joint. Hold the gold input lock down, "
    "load the blue handle at both proof marks until both readings register, "
    "then release the handle and lock."
)
DESCRIPTION = (
    "The gold input lock is spring-returned. The blue load handle slides "
    "against a calibrated spring. The test display stores the two settled "
    "angle readings and their difference in arcminutes."
)
STOWED = dict.fromkeys(BASES, READY)
MAX_STEPS = 540
PHYSICS_HZ = 240
CHECK_DT = 0.05

CENTER_X = 0.422
CENTER_Y = 0.020
TABLE_Z = 0.7475
INPUT_Z = 0.821
OUTPUT_Z = 0.850
LOAD_RADIUS = 0.115
LOAD_Z = 0.890
LOCK_Y = CENTER_Y + 0.122
LOCK_Z = 0.901
LOCK_TRAVEL = 0.014
LOAD_TRAVEL = 0.020
SPRING_N_M = 300.0
PROOF_NM = 0.4
PROOF_MIN = 0.32
PROOF_MAX = 0.48
STABLE_S = 0.55
