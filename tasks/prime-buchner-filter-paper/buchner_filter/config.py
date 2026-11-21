"""Laboratory station dimensions in metres and working volumes in millilitres."""

from dataclasses import dataclass

import numpy as np
from yam_arena import station
from yam_arena.constants import READY

KEY = "buchner_filter"
INSTRUCTION = (
    "Seat the filter paper in the Büchner funnel and wet it with the rinse water. "
    "Filter the bead suspension so the beads stay on the paper and the water "
    "collects in the flask. Return both beakers upright."
)
DESCRIPTION = (
    "An assembled, clamped vacuum-filtration station with loose circular filter "
    "paper, a small rinse-water beaker and a larger beaker of water and inert beads."
)
BASES = station.BASES
STOWED = dict.fromkeys(BASES, READY)
TABLE = station.TABLE_TOP
PHYSICS_HZ = 240
CHECK_DT = 0.05
MAX_STEPS = 1200
FUNNEL = np.array([0.345, 0.008])
PLATE = TABLE + 0.175
RIM = PLATE + 0.039
PAPER_RADIUS = 0.035
PAPER_THICK = 0.00065
PAPER = np.array([0.330, 0.193, TABLE + 0.0295 + PAPER_THICK / 2])
FUNNEL_INNER = 0.038
FUNNEL_MOUTH = 0.046
FLASK_INNER = 0.047
FLASK_BOTTOM = TABLE + 0.007


@dataclass(frozen=True)
class Vessel:
    name: str
    x: float
    y: float
    radius: float
    height: float
    handle: float
    fill_ml: float
    mass: float
    color: tuple[float, float, float]

    @property
    def position(self):
        return np.array([self.x, self.y, TABLE + 0.0004])

    @property
    def grip(self):
        return np.array([0.0, self.handle, self.height * 0.49])

    @property
    def outlet(self):
        return np.array([-self.radius - 0.010, 0.0, self.height])


RINSE = Vessel(
    "rinse", 0.320, 0.282, 0.028, 0.051, 0.052, 17.0, 0.070, (0.27, 0.63, 0.73)
)
SUSPENSION = Vessel(
    "suspension", 0.320, -0.282, 0.028, 0.051, -0.052, 55.0, 0.108, (0.23, 0.60, 0.75)
)
VESSELS = (RINSE, SUSPENSION)
