"""Full-size football, needle and dual-action hand pump dimensions in SI units."""

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "football_pump"
INSTRUCTION = "Inflate the football to the green pressure band, then withdraw the needle straight and put the pump down."
DESCRIPTION = "A partly inflated football sits on a rubber support ring. An unseated standard inflation needle is connected by a slack hose to a dual-action pump with a pressure gauge."
MAX_STEPS = 940
PHYSICS_HZ = 240
BASES = station.BASES
STOWED = {name: READY for name in BASES}
TABLE = .7475
BALL_R = .109
RING_INNER = .0610
RING_TOP = TABLE + .024
BALL = np.array([.348, .157, RING_TOP + np.sqrt(BALL_R**2 - RING_INNER**2)])
PORT = BALL + [0., 0., BALL_R]
NEEDLE = np.array([.274, .282, TABLE + .082])
NEEDLE_TIP = .029
NEEDLE_R = .00148
NEEDLE_FLAT = .010
PUMP = np.array([.460, .056, TABLE + .051])
PUMP_AXIS = np.array([0., -1., 0.])
PUMP_QUAT = [np.sqrt(.5), 0., 0., np.sqrt(.5)]
PUMP_ROT = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
PUMP_TOP = .163
PISTON_HOME = .189
STROKE = .098
BARREL_GRIP = np.array([0., 0., .114])
GAUGE = np.array([.041, .015, .042])
GAUGE_FRAME = PUMP_ROT.T
HOSE_ORIGIN = np.array([.025, 0., .106])
HOSE_LENGTH = .530
CHECK_DT = .05
START_BAR = .755
LOW_BAR = .790
HIGH_BAR = .825
PUMP_BORE = .020
BALL_VOL = 4 * np.pi * BALL_R ** 3 / 3


def hose_points(start, end):
    center = (start + end) * .5
    center[0] += .160
    center[2] = max(TABLE + .015, min(start[2], end[2]) - .078)
    return [(1-t)**2*start + 2*t*(1-t)*center + t**2*end
            for t in np.linspace(0, 1, 13)]
