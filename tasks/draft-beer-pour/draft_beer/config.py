"""Fixed full-size bench-mounted draft faucet and tapered pub glass, SI."""
import numpy as np

TOP = .7475
KEY = "draft_beer"
TITLE = "Pour a draft beer with a proper foam head"
INSTRUCTION = "Pour a draft beer into the glass with about an inch of foam. Close the tap and leave the glass on the coaster."
DESCRIPTION = "A full-size manual draft faucet, an empty clear pub glass and a coaster are on the bar counter. The faucet is closed and the glass starts empty."
MAX_STEPS = 800
PHYSICS_HZ = 240
HEIGHT, FLOOR = .1651, .011
R_BASE, R_LIP, WALL = .027, .0354, .0027
DRY_MASS = .278
DRY_COM = np.array([0., 0., .060])
DRY_INERTIA = np.diag([.00072, .00072, .00027])
GLASS = np.array([.46, -.165, TOP + .004])
COASTER = np.array([.40, -.245, TOP + .005])
SPOUT = np.array([.325, -.040, TOP + .249])
PIVOT = np.array([.357, -.040, TOP + .284])
TAP_ANGLE = np.deg2rad(26.)
TAP_GRASP_HEIGHT = .060
GLASS_GRASP_HEIGHT = .072
FULL_FLOW_ML_S = 63.09
FOAM_EXPANSION = 3.5
