"""Station measurements and physical scoring thresholds."""

from math import radians

from yam_arena.constants import READY
from yam_arena.station import BASES

STOWED = dict.fromkeys(BASES, READY)
KEY = "focus-and-lock-vision-lens"
INSTRUCTION = (
    "Adjust the focus ring until the inspection monitor shows sharp black-and-white bars. "
    "Hold that setting while tightening the focus locking screw, then take both hands away."
)
DESCRIPTION = INSTRUCTION
PHYSICS_HZ = 240
MAX_STEPS = 520
CHECK_DT = 0.05
TABLE_Z = 0.7475
FOCUS_ORIGIN = (0.445, 0.0, 0.879)
LOCK_ORIGIN = (0.433, -0.045, 0.890)
FOCUS_RADIUS = 0.023
FOCUS_ANGLE = radians(36)
LOCK_ANGLE = radians(64)
FOCUS_GRIP = (0.015, 0.041, 0.0)
LOCK_RADIUS = 0.009
SCREW_PITCH = 0.0007
FULL_CONTRAST = 0.64
PARTIAL_CONTRAST = 0.38
DISPLAY_POSITION = (0.651, -0.268, 0.962)
TARGET_DISTANCE = 0.500
CHART_SPAN = 0.022
FOCAL_LENGTH = 0.016
F_NUMBER = 2.8
SENSOR_TRAVEL_PER_RADIAN = 0.00034
