"""Full-scale gate, fixed YAM station and pedestrian path in metres."""

import numpy as np
from yam_arena.constants import READY
from yam_arena.station import BASES

KEY = "pedestrian_gate"
INSTRUCTION = "Let the person through the gate, then shut it behind them."
DESCRIPTION = "The two robot arms are on a path beside a spring-closing garden gate. A person is waiting on the far side."
PHYSICS_HZ = 240
MAX_STEPS = 640
CHECK_DT = 0.05
STOWED = dict.fromkeys(BASES, READY)

FLOOR = 0.15
HINGE = np.array([0.385, 0.438, 0.0])
SPAN = 0.84
GATE_BOTTOM = FLOOR + 0.055
GATE_TOP = FLOOR + 0.92
OPEN_STOP = 82.0
OPEN_TARGET = 77.0
PASS_ANGLE = 70.0
HOLD_RADIUS = 0.280
HANDLE_OFFSET = 0.063
RAIL_HEIGHT = 0.968
BOLT = np.array([0.0, -0.858, 0.922])
LIFT = 0.044
BOLT_GRIP = HINGE + BOLT + [-0.061, -0.004, 0.080]
START_X, FINISH_X = 1.455, -0.320
WALK_Y = -0.081
WALK_DURATION = 4.12
BODY_DEPTH = 0.23
SHOULDER_WIDTH = 0.43
PASS_X = HINGE[0] - BODY_DEPTH / 2 - 0.06
