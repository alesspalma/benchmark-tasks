"""First native contact trial of looping and elastic retention."""

import math

import numpy as np
from yam_arena import ik

from . import config as c
from .motion import Program, rotation


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    if mode not in ("demo", "one-loop"):
        raise ValueError(f"Unknown native mode {mode!r}")
    down = rotation("X", math.pi)
    brace = rotation("Z", math.radians(-20.)) @ rotation("Y", math.radians(-15.)) @ down
    across = rotation("Z", -math.pi / 2) @ down
    stock_band = np.array([.3332, -.327, .8027])
    stock_frame = ik.quat_xyzw_to_matrix(np.array([-.207, -.206, -.051, .955]))
    stock_grip = stock_band + stock_frame @ [0., 0., c.BAND_RADIUS - .002]
    stock_normal = stock_frame[:, 2]
    press = c.MAT_Z + c.MAT_RADIUS - .008

    def grip(x, y, z, g=.05):
        return (x, y, z), across, g

    def stock(g, offset=0.):
        return stock_grip + offset * stock_normal, stock_frame @ across, g

    def steady(y, z, g=.25):
        return (c.MAT_X - .007, y, z), brace, g

    p.add(2.8, "approach the rolled mat and right loop",
          left=steady(.072, press + .055), right=stock(.65, .053))
    p.add(1.5, "lower to the stock loop and compress the loose outer turn",
          left=steady(.072, press), right=stock(.65))
    p.add(.7, "pinch the upper webbing and brace the roll",
          right=stock(.05))
    p.add(1.15, "raise the right loop to the roll axis",
          right=grip(.33, -c.BAND_Y, c.MAT_Z + c.BAND_RADIUS - .002))
    p.add(1.5, "bring the right loop in line with the roll end",
          right=grip(c.MAT_X + .017, -c.BAND_Y, c.MAT_Z + c.BAND_RADIUS - .006))
    p.add(2.0, "slide the elastic loop over the compressed roll end",
          right=grip(c.MAT_X + .017, -c.SEATED_Y, c.MAT_Z + c.BAND_RADIUS - .006))
    p.add(.8, "release the right loop", right=grip(c.MAT_X + .017, -c.SEATED_Y, c.MAT_Z + c.BAND_RADIUS - .006, .75))
    p.add(1.0, "move clear of the loop and mat", right=grip(c.MAT_X + .017, -.25, c.MAT_Z + .094, .8),
          left=steady(.072, press + .065, .8))
    return p.moves
