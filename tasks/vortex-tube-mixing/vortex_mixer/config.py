"""Full-size touch mixer, one capped 50 mL skirted tube, and fixed bench."""
import numpy as np

KEY = "vortex_mixer"
TITLE = "Homogenize a capped sample on a touch vortex mixer"
INSTRUCTION = "Mix the liquid in the capped tube until its color is uniform using the touch vortex mixer. Stand the tube upright beside the mixer when you are done."
DESCRIPTION = "A capped 50 mL sample tube with visibly layered liquid and a touch-activated benchtop vortex mixer sit on a clean lab counter."
MAX_STEPS = 640
PHYSICS_HZ = 240
TOP = .7475
BODY_RADIUS = .015
TUBE_HEIGHT = .115
BODY_MASS = .051
FILL_LOW, FILL_HIGH = .010, .072
SAMPLE_SPLIT = .029
TUBE = np.array([.408, -.266, TOP + .0006])
MIXER = np.array([.363, -.112, TOP])
PAD_Z = .060
PAD_RADIUS = .023
PAD_ORBIT = .002
MOTOR_HZ = 2800 / 60
PRESS_STIFFNESS = 1800.
PRESS_ON = .0015
PRESS_TRAVEL = .007
GRIP_HEIGHT = .085
