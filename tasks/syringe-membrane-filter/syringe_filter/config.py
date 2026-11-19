"""Fixed full-size bench geometry and filtration parameters, in SI units."""

import numpy as np

KEY = "syringe_filter"
DESCRIPTION = (
    "Support the loaded syringe and press its plunger to collect membrane filtrate."
)
INSTRUCTION = "Filter the sample into the marked vial. Collect at least 4 mL of clear filtrate without spilling more than 0.2 mL."
PHYSICS_HZ = 240
MAX_STEPS = 500

TABLE = 0.7475
TRAY = np.array([0.365, 0.061, TABLE])
TRAY_SIZE = (0.240, 0.250)
SURFACE_Z = TABLE + 0.005
START = np.array([0.346, 0.127, SURFACE_Z + 0.061])
VIAL = np.array([0.343, -0.038, SURFACE_Z])
VIAL_INNER_R = 0.0125
VIAL_LIP = 0.046
VIAL_FLOOR = 0.003
WORK = VIAL + [0.004, -0.004, 0.084]

BORE_R = 0.00735
BARREL_R = 0.0096
PISTON_AREA = np.pi * BORE_R**2
BARREL_BOTTOM = 0.031
BARREL_FLANGE = 0.099
GRIP_Z = BARREL_FLANGE - 0.0085
PLUNGER_HEAD = 0.159
FILTER_R = 0.0125
OUTLET_INNER_R = 0.00185
FILL_ML = 7.0
STOPPER_FRICTION_N = 0.52
FLOW_DAMPING = 400.0
FLOW_TARGET_ML = 4.0
SPILL_LIMIT_ML = 0.2
PARTICLE_CONCENTRATION_MG_ML = 0.5
MAX_TRAVEL = FILL_ML / (PISTON_AREA * 1e6)
