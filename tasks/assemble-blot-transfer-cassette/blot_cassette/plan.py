"""Reference trajectories for bracing, rolling and closing the cassette."""

import numpy as np

from . import config as C
from .motion import Program


def approach(x, y, z):
    forward = np.array([x, y, z])
    forward /= np.linalg.norm(forward)
    close = np.cross(forward, [0., 0., 1.])
    close /= np.linalg.norm(close)
    return np.column_stack((close, np.cross(forward, close), forward))


RIGHT = approach(.72, .66, -.22)
LEFT = approach(.56, -.69, -.46)
CORNER = approach(.43, -.88, -.20)


def roller_grip(root, grip=.01, back=0):
    return root + [0., 0., C.GRIP_HEIGHT] - back * RIGHT[:, 2], RIGHT, grip


def brace(point, grip=0., back=0.):
    return np.asarray(point) - back * LEFT[:, 2], LEFT, grip


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    x, y, z = C.ORIGIN
    top = z + C.SHEET_Z
    stage = C.ROLLER_START
    corner = np.array([x+.047, y+.050, top-.002])
    hold = (corner, CORNER, 0.)
    first = np.array([x-.009, y+.030, top+C.ROLLER_R])
    last = np.array([x-.009, y-.029, top+C.ROLLER_R])
    p.add(2.1, "approach brayer", right=roller_grip(stage, 1., .067),
          left=(corner + [0., .044, .052], CORNER, 0.))
    p.add(1.4, "encircle handle and hold exposed laminate margin",
          right=roller_grip(stage, 1.), left=hold)
    p.add(.8, "secure brayer", right=roller_grip(stage))
    p.add(1.20, "lift out of cradle", right=roller_grip(stage + [0., 0., .061]))
    p.add(1.75, "align above far edge", right=roller_grip(first + [0., 0., .064]))
    offset = .014 if mode == "hover" else -.0055
    p.add(1.7, "set roller load", right=roller_grip(first + [0., 0., offset]))
    p.add(3.9, "draw the brayer across the laminate", right=roller_grip(last + [0., 0., offset]))
    p.add(.8, "lift brayer", right=roller_grip(last+[0., 0., .040]))
    dock = stage + [0.008, 0.012, 0.0]
    p.add(1.65, "return above the open saddle", right=roller_grip(dock+[0., 0., .047]),
          left=brace((x+.023, y+.066, top+.08)))
    p.add(1.4, "seat the brayer on both supports", right=roller_grip(dock))
    p.add(.55, "settle on the supports")
    p.add(.7, "release the parked brayer", right=roller_grip(dock, 1.))
    p.add(.9, "clear work area", right=roller_grip(dock+[0., 0., .025], 1., .07))
    cover = np.array([x+C.LID_TRAVEL-.058, y+.019, z+.044])
    p.add(1.4, "approach the raised lid lip", left=brace(cover, 1., .054))
    p.add(1.1, "straddle the lid lip", left=brace(cover, 1.))
    p.add(.55, "hold the lip", left=brace(cover))
    p.add(2.2, "draw the cover through its guides", left=brace(cover+[-C.LID_TRAVEL-.010, 0., 0.]))
    p.add(.55, "release the shut cover", left=brace(cover+[-C.LID_TRAVEL-.010, 0., 0.], 1.))
    p.add(.8, "withdraw from the cassette", left=brace(cover+[-C.LID_TRAVEL-.010, 0., .045], 1.))
    return p.moves
