"""Counterhold, tension the reinforced tape, and raise the carton cover."""

import numpy as np

from . import config as C
from .motion import Program, rotation

TOP = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])
FRONT = rotation("Y", np.deg2rad(-85)) @ TOP


def waypoints(mode="demo"):
    p = Program()
    if mode == "hold":
        return p.moves
    hold = C.BOX + [.004, .060, C.HEIGHT + .011]
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
    pull = tab + [-.050, -.065, .151]
    p.add(2.35, "pull progressively along the profiled strip", right=(pull, TOP, .012))
    p.add(.55, "finish separating the strip", right=(pull, TOP, .012))
    discard = np.array([.371, -.225, .845])
    p.add(1.5, "carry removed strip beside the mailer", right=(discard, TOP, .012))
    p.add(.5, "release the removed strip", right=(discard, TOP, .6))
    # Approach below the freed front flap and use the upper jaw as a hook.
    lip = C.BOX + [-C.DEPTH / 2 - .009, -.039, C.SEAM_Z + .002]
    hook = FRONT
    p.add(1.5, "approach the freed lid edge", right=(lip + [-.055, 0., .023], hook, .35))
    p.add(.8, "guide one finger under the front flap", right=(lip, hook, .35))
    p.add(.7, "release counterpressure after tear", left=(hold + [.014, 0., .075], TOP, .19))
    p.add(1.1, "lift the lid on its cardboard crease",
          right=(lip + [.052, 0., .134], hook, .35))
    p.add(1.0, "hold the book in view")
    return p.moves
