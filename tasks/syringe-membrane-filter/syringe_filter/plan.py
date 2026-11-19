"""Bimanual reference through native fingertip contact alone."""

import numpy as np

from . import config as C
from .motion import Program, rotation

DIRECTION = np.array([0.60, -0.79, -0.12])
DIRECTION /= np.linalg.norm(DIRECTION)
CLOSING = np.array([DIRECTION[1], -DIRECTION[0], 0.0])
CLOSING /= np.linalg.norm(CLOSING)
LEFT = np.column_stack((CLOSING, np.cross(DIRECTION, CLOSING), DIRECTION))
RIGHT = rotation("X", np.pi + 0.45) @ rotation("Y", 0.17) @ rotation("Z", 0.336)


def grip(outlet, opening=0.02, back=0.0):
    return outlet + [0.0, 0.0, C.GRIP_Z] - back * DIRECTION, LEFT, opening


def press(outlet, above=0.0, travel=0.0):
    pad = outlet + [0.0, 0.0, C.PLUNGER_HEAD + 0.002 + above - travel]
    return pad - 0.0125 * RIGHT[:, 2], RIGHT, 0.02


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    if mode not in ("demo", "off-vial"):
        raise ValueError(f"Unknown policy: {mode}")
    work = C.WORK.copy()
    if mode == "off-vial":
        work[1] -= 0.050
    lift = [0.0, 0.0, 0.031]
    p.add(
        2.1,
        "approach the staged syringe",
        left=grip(C.START, opening=0.86, back=0.095),
        right=(np.array([0.335, -0.144, 1.03]), RIGHT, 0.02),
    )
    p.add(1.05, "straddle the upper barrel", left=grip(C.START, opening=0.86))
    p.add(0.75, "capture the barrel below the finger flange", left=grip(C.START))
    p.add(1.05, "lift the syringe out of its stand", left=grip(C.START + lift))
    p.add(1.50, "bring the loose filter over the vial", left=grip(work + lift))
    p.add(1.35, "aim the outlet over the vial mouth", left=grip(work))
    p.add(0.8, "steady the supported barrel")
    p.add(1.4, "approach above the plunger", right=press(work, above=0.021))
    p.add(1.2, "touch the plunger thumb pad", right=press(work, above=0.001))
    p.add(
        8.8, "press while supporting the finger flange", right=press(work, travel=0.052)
    )
    p.add(0.5, "hold the outlet over the vial", right=press(work, travel=0.052))
    p.add(1.0, "withdraw the right hand", right=press(work, above=0.021))
    return p.moves
