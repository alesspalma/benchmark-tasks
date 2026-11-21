"""Dimensions, initial layout, and output regions in metres."""

from yam_arena.constants import READY
from yam_arena.station import BASES

KEY = "open-rolled-change-into-till"
INSTRUCTION = (
    "Open the blue nickel roll over the cash drawer edge. Empty all the nickels "
    "into the compartment marked 5¢ and put the paper in the scrap tray."
)
DESCRIPTION = (
    "A cash drawer with three coin compartments, a narrow steel opening edge, "
    "a full kraft-paper nickel roll, and a separate scrap tray. The nickel "
    "compartment has a blue 5¢ label."
)
STOWED = dict.fromkeys(BASES, READY)
TABLE = 0.7475
PHYSICS_HZ = 480
CHECK_DT = 0.05
MAX_STEPS = 800

COIN_RADIUS = 0.010605
COIN_THICKNESS = 0.00195
COIN_MASS = 0.005
COIN_PITCH = 0.002012
COINS = tuple(f"nickel_{i:02}" for i in range(40))
PAPER_RADIUS = 0.0122
PAPER_LENGTH = 0.0417
PAPER_NAMES = ("paper_left", "paper_right")
HALF_Y = PAPER_LENGTH / 2 + 0.00012
ROLL_X = 0.323
ROLL_Z = TABLE + 0.045
EDGE_X = 0.430
EDGE_Z = TABLE + 0.065

BIN_X = (0.382, 0.619)
BIN_Y = (-0.074, 0.074)
BIN_FLOOR = TABLE + 0.009
BIN_TOP = TABLE + 0.079
SCRAP_X = (0.209, 0.304)
SCRAP_Y = (-0.162, 0.162)
SCRAP_FLOOR = TABLE + 0.006
SCRAP_TOP = TABLE + 0.033
