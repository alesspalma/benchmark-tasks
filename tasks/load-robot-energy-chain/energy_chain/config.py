"""Guided single-axis station geometry in metres."""

from yam_arena.constants import READY
from yam_arena.station import BASES

KEY = "energy_chain"
INSTRUCTION = (
    "Lay the blue and orange cables into the robot axis's cable carrier. "
    "Snap both crossbars closed, then give the exposed end of the pair a "
    "gentle upward tug and leave the cables captured."
)
DESCRIPTION = (
    "The robot axis is isolated from power. Two neighboring snap crossbars "
    "stand open on the service run. The two loose cable ends are paired by a service tie."
)
STOWED = dict.fromkeys(BASES, READY)
MAX_STEPS = 570
PHYSICS_HZ = 240
CHECK_DT = 0.05

TABLE_Z = 0.7475
CHAIN_X = 0.415
CHAIN_FLOOR = 0.803
INNER_HALF_WIDTH = 0.037
BAR_Z = 0.841
HINGE_X = CHAIN_X - 0.043
GATE_Y = (-0.048, -0.103)

BAR_OPEN = 84.0
ANCHOR_Y = 0.180
FLEX_Z = 0.820
FIRST_SPAN = 0.076
SECOND_SPAN = 0.221
TAIL_SPAN = 0.066
SLEEVE_LOCAL_Y = -0.045
TAIL_LOCAL_Y = -0.044
SERVICE_REST = -27.0
PROXIMAL_REST = -15.0

CABLE_RADIUS = 0.0045
