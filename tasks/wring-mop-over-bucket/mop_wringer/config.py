"""Fixed floor bucket and a full-size cut-end yacht mop."""
import numpy as np
from yam_arena.station import FLOOR_Z

KEY = "mop_wringer"
TITLE = "Wring a wet yacht mop in a cone bucket"
INSTRUCTION = "Wring out the wet mop using the bucket's cone wringer. Lift the mop slightly to drain over the bucket. Keep the water in the bucket."
DESCRIPTION = "A full-size rayon yacht mop sits lightly in the cone of a 15 L bucket on the floor. Its shaft is upright and the mop head still holds 200 g of added water."
MAX_STEPS = 400
PHYSICS_HZ = 240
BUCKET = np.array([.34, 0., FLOOR_Z + .001])
MOUTH = .293
CONE_TOP_R = .107
CONE_BOTTOM = .112
CONE_BOTTOM_R = .029
NECK_Z = .414
SHAFT_LENGTH = 1.2192
SHAFT_RADIUS = .0125
WATER_START_G = 200.
WRING_G = 100.
WATER_ALLOW_G = 8.
BUNDLE_R = .045
BUNDLE_LENGTH = .172
BUNDLE_STIFFNESS = 375.
BUNDLE_TRAVEL = .033
