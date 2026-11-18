"""Dimensions and fixed starting configuration in metres."""

import numpy as np

KEY = "tear_mailer"
DESCRIPTION = "Open a corrugated postal mailer by pulling its integrated tear strip."
INSTRUCTION = "Pull the tear strip all the way off and lift the lid so the book is visible. Leave the book in the mailer."
PHYSICS_HZ = 240
MAX_STEPS = 600
TABLE = .7475
BOX = np.array([.337, .010, TABLE + .0005])
DEPTH, WIDTH, HEIGHT = .165, .252, .050
WALL = .0022
SEAM_Z = .028
SEAM_WIDTH = .010
FIRST = np.array([-.5 * DEPTH - .002, -.5 * WIDTH + .010, SEAM_Z])
TEAR_LENGTH = WIDTH - .020
TAB_SIZE = (.020, .023, .0036)
TAB_MASS = .008
TAB = BOX + FIRST + [-.013, -.030, 0.]
LEADER = float(np.linalg.norm(TAB - BOX - FIRST))
BOOK_SIZE = (.105, .148, .019)
BOOK = BOX + [.009, .005, WALL + BOOK_SIZE[2] / 2 + .001]
