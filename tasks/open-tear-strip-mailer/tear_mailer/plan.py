"""Counterhold, tension the reinforced tape, and raise the carton cover."""

import numpy as np

from . import config as C
from .motion import Program, rotation

TOP = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
FRONT = rotation("Y", np.deg2rad(-82)) @ TOP @ rotation("Z", np.pi / 2)


def waypoints(mode="demo"):
    p = Program()
    if mode == "hold":
        return p.moves
    hold = C.BOX + [.004, .060, C.HEIGHT - .010]
    tab = C.TAB.copy()
    approach = tab + [-.006, -.040, .105]
    near = tab + [-.004, -.020, .045]
    p.add(2.3, "approach the mailer and pull tab",
          left=(hold + [0, 0, .075], TOP, .19), right=(approach, TOP, .35))
    p.add(1.4, "counterhold the cover and approach the tab",
          left=(hold, TOP, .19), right=(near, TOP, .35))
    p.add(1.0, "line up over the reinforced tab", right=(tab, TOP, .30))
    p.add(.85, "pinch the factory pull tab", right=(tab, TOP, .012))
    first_pull = tab + [-.028, .005, .056]
    p.add(1.15, "initiate the tear away from the front flap", right=(first_pull, TOP, .012))
    pull = tab + [-.043, -.099, .168]
    p.add(2.35, "pull progressively along the profiled strip", right=(pull, TOP, .012))
    p.add(.55, "finish separating the strip", right=(pull, TOP, .012))
    discard = np.array([.371, -.225, .845])
    p.add(1.5, "carry removed strip beside the mailer", right=(discard, TOP, .012))
    p.add(.5, "release the removed strip", right=(discard, TOP, .6))
    # Angle the two pads vertically to straddle the freed front flap.
    hinge = C.BOX + [C.DEPTH / 2, 0., C.HEIGHT]
    closed_lip = C.BOX + [-C.DEPTH / 2 - .002, .081, C.SEAM_Z + .011]
    springback = rotation("Y", np.deg2rad(15.))
    lip = hinge + springback @ (closed_lip - hinge)
    aligned = springback @ FRONT
    p.add(.9, "lift the holding hand off the freed cover",
          left=(hold + [0, 0, .101], TOP, .24))
    if mode == "tear-only":
        p.add(4., "leave the freed cover at rest")
        return p.moves
    p.add(3.6, "orient the fingers clear of the carton",
          left=(lip + [-.055, 0., .015], aligned, .24))
    p.add(.9, "settle the turned wrist")
    p.add(.9, "level the fingers with the freed flap",
          left=(lip + [-.055, 0., 0.], aligned, .24))
    p.add(1.25, "place a finger on each side of the flap",
          left=(lip, aligned, .24))
    p.add(.65, "settle against the lip")
    p.add(.9, "pinch the free front flap", left=(lip, aligned, 0.))
    for angle in (30, 47, 64):
        lift = rotation("Y", np.deg2rad(angle))
        p.add(1.28, "lift the lid on its cardboard crease",
              left=(hinge + lift @ (closed_lip - hinge), lift @ FRONT, 0.))
    p.add(1.0, "hold the book in view")
    return p.moves
