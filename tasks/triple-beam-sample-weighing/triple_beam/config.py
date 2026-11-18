"""Table-mounted scale geometry and physical calibration in SI units."""

from dataclasses import dataclass
import numpy as np

KEY = "triple_beam"
TITLE = "Weigh a steel specimen on a mechanical triple-beam balance"
INSTRUCTION = "Weigh the steel specimen with the three sliding weights. Leave the specimen on the pan and the pointer lined up with zero, with both hands clear."
DESCRIPTION = "A full-size mechanical triple-beam balance on a laboratory counter. A steel specimen is on its 152 mm pan. Three captive poises start at zero."
MAX_STEPS = 760
PHYSICS_HZ = 240
TOP = .7475
PIVOT = np.array([.353, .043, TOP + .106])
PAN_LEVER = .092
PAN_RADIUS = .076
PAN_MASS = .13
PAN_Z = .052
POINTER_Y = -.244
SPECIMEN_MASS = .337
SPECIMEN_RADIUS = .020
SPECIMEN_HEIGHT = .0342
BEAM_MASS = .72
LIMIT_DEG = 2.5


@dataclass(frozen=True)
class Rider:
    name: str
    x: float
    z: float
    start_y: float
    travel: float
    capacity_g: float
    tick_g: float
    size: tuple[float, float, float]
    goal_g: float

    @property
    def mass(self):
        return self.capacity_g / 1000 * PAN_LEVER / self.travel

    @property
    def target(self):
        return -self.travel * self.goal_g / self.capacity_g


RIDERS = (
    Rider("Hundreds", .003, .034, -.030, .184, 500., 100., (.021, .033, .032), 300.),
    Rider("Tens", .027, .021, -.036, .171, 100., 10., (.018, .022, .019), 30.),
    Rider("Grams", -.025, .012, -.040, .170, 10., .1, (.016, .011, .010), 7.),
)
