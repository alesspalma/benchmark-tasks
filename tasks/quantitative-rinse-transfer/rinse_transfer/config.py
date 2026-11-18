"""Fixed dimensions and bench coordinates for quantitative rinse recovery."""

import numpy as np

KEY = "rinse_transfer"
TITLE = "Recover retained beaker sample with a wash-bottle rinse"
INSTRUCTION = "Rinse the blue residue from the inside of the beaker with the wash bottle and collect the rinsings in the flask. Leave the glassware upright without spilling the sample."
DESCRIPTION = "An analytical bench has a bent-nozzle water wash bottle, a small beaker with blue residue, and a funnel in a clean volumetric flask."
MAX_STEPS = 680
PHYSICS_HZ = 240
TOP = 0.7475
BEAKER = np.array([0.356, 0.184, TOP + 0.0006])
BOTTLE = np.array([0.356, 0.117, TOP + 0.0006])
FLASK = np.array([0.280, 0.290, TOP])
FUNNEL = FLASK + [0, 0, 0.173]
FUNNEL_RADIUS = 0.027
RADIUS = 0.019
HEIGHT = 0.054
FLOOR = 0.003
BEAKER_GRIP = 0.027
BOTTLE_GRIP = 0.042
BOTTLE_RADIUS = 0.02415
NOZZLE = np.array([0.0, 0.044, 0.087])
JET = np.array([0.0, 0.45, -0.893])
SPOUT = np.array([0.0, -0.0225, HEIGHT])
FILM_COLORS = np.array([[0.08, 0.30, 0.71], [0.055, 0.37, 0.86]])
FILM_HEIGHTS = (0.006, 0.013)
PANEL_TRAVEL = 0.008
PANEL_STIFFNESS = 1150.0
