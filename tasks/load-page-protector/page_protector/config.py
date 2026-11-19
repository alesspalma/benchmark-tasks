"""Measured-format office materials and coordinates in the calibrated YAM station."""

KEY = "page_protector"
INSTRUCTION = "Put the loose printed page all the way inside the clear sheet protector. Leave the page flat inside and take both grippers away."
DESCRIPTION = INSTRUCTION
PHYSICS_HZ = 480
MAX_STEPS = 700
TABLE = .7475
PAPER_LENGTH = .2794
PAPER_WIDTH = .2159
PAPER_MASS = .00483
PAPER_HALF = PAPER_LENGTH / 2
MOUTH = .485
FAR_EDGE = .774
SLEEVE_Y = .005
INNER_HALF_WIDTH = .110
UPPER_LIP = MOUTH - .009
LOWER_FACE = TABLE + .044
UPPER_FACE = LOWER_FACE + .0014
FILM_PROXY = .00065
PAPER_PROXY = .00065
STOCK_REAR = (.209, -.244, .901)
STOCK_YAW = 0.
STOCK_PITCH = 19.
STOCK_OVERHANG = .022
PAPER_PANELS = 3
PANEL_LENGTH = PAPER_LENGTH / PAPER_PANELS
INSET = .0025
