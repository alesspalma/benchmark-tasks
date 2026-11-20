"""Cartesian reference for filling with cuff pressure and controlled withdrawal."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def pinch(lateral, approach):
    z = np.asarray(approach, dtype=float)
    z /= np.linalg.norm(z)
    x = np.asarray(lateral, dtype=float) - np.dot(lateral, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


CUFF_APPROACH = np.array([-.75, 0., -1.])
CUFF_PINCH = pinch(C.BAG_ROTATION @ [0., -1., 0.], C.BAG_ROTATION @ CUFF_APPROACH)
TOOL_PINCH = pinch(C.TOOL_START_R @ [0., 1., 0.], [.70, -.30, -1.])
FILL_ROTATION = C.BAG_ROTATION @ rotation("Z", np.deg2rad(248.)) @ rotation("Y", np.deg2rad(60.))


def cuff(h=0., grip=.91, lead=0.):
    return C.BAG + C.BAG_ROTATION @ [.035-lead, 0., .024+h], CUFF_PINCH, grip


def pickup(h=0., grip=.83):
    return C.SPATULA + C.TOOL_START_R @ C.HANDLE_GRIP + [0., 0., h-.029], TOOL_PINCH, grip


def at_mouth(z, grip=.035):
    direction = FILL_ROTATION[:, 0]
    tip = C.BAG + C.BAG_ROTATION @ [-.013, .005, 0.] + direction * z / direction[2]
    tcp = tip + FILL_ROTATION @ (C.HANDLE_GRIP - [C.BLADE_X1, 0., C.BLADE_Z])
    return tcp, FILL_ROTATION @ C.TOOL_START_R.T @ TOOL_PINCH, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    p.add(2.4, "Approach bag and spatula", left=cuff(.045, lead=-.066), right=pickup(.110))
    p.add(1.35, "Set open fingers around the tool handle", left=cuff(lead=-.066), right=pickup())
    p.add(1.4, "Set open fingers around the folded film", left=cuff(), right=pickup(grip=.041))
    support = .63 if mode != "open-cuff" else .91
    p.add(1.35, "Support the cuff while lifting the loaded spatula", left=cuff(grip=support), right=pickup(.115, .041))
    p.add(2.9, "Bring the spatula above the open bag", right=at_mouth(.065, .041))
    p.add(1.65, "Center the blade above the open cuff", right=at_mouth(.035, .041))
    depth = -.092 if mode != "no-insertion" else .039
    p.add(3.0, "Load the blade below the cuff", right=at_mouth(depth, .041))
    p.add(.65, "Settle the loaded blade in the bag")
    p.add(1.0, "Raise the blade while the cuff is supported",
          right=at_mouth(-.065 if mode != "no-insertion" else .039, .041))
    grip = .135 if mode != "open-cuff" else .91
    p.add(1.95, "Pinch the cuff around the spatula", left=cuff(grip=grip))
    p.add(4.5, "Withdraw the blade through the pinched cuff", right=at_mouth(.025, .041))
    p.add(.6, "Release the cuff", left=cuff(), right=at_mouth(.029, .041))
    p.add(1.0, "Clear the bag before turning the spatula",
          left=cuff(.035, lead=-.066),
          right=(at_mouth(.029, .041)[0] + [0., -.105, -.008], at_mouth(.029, .041)[1], .041))
    p.add(2.5, "Return the spatula over its rest", right=pickup(.105, .041))
    p.add(1.15, "Set the spatula in the rest", right=pickup(.002, .041))
    p.add(.65, "Release the supported tool", right=pickup(.002, .88))
    p.add(1.0, "Withdraw both hands", right=pickup(.132, .88))
    return p.moves
