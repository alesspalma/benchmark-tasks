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
    hold = (x-.024, y+.035, top+.005)
    first = np.array([x, y-.037, top+C.ROLLER_R])
    last = np.array([x, y+.029, top+C.ROLLER_R])
    p.add(2.1, "approach brayer", right=roller_grip(stage, 1., .067),
          left=brace((x-.024, y+.061, top+.043), back=.022))
    p.add(1.4, "encircle handle and hold exposed laminate margin",
          right=roller_grip(stage, 1.), left=brace(hold))
    p.add(.8, "secure brayer", right=roller_grip(stage))
    p.add(1.20, "lift out of cradle", right=roller_grip(stage + [0., 0., .061]))
    p.add(1., "align above near edge", right=roller_grip(first + [0., 0., .031]))
    offset = .009 if mode == "hover" else -.003
    p.add(1., "set roller load", right=roller_grip(first + [0., 0., offset]))
    p.add(3.5, "roll out all three air gaps", right=roller_grip(last + [0., 0., offset]))
    p.add(.65, "lift brayer", right=roller_grip(last+[0., 0., .040]))
    p.add(1.25, "return to cradle", right=roller_grip(stage+[0., 0., .039]),
          left=brace((x+.023, y+.066, top+.08)))
    p.add(.8, "seat brayer", right=roller_grip(stage))
    p.add(.70, "release brayer", right=roller_grip(stage, 1.))
    p.add(.9, "clear work area", right=roller_grip(stage, 1., .07))
    cover = np.array([x+C.LID_TRAVEL-.058, y+.019, z+.044])
    p.add(1.4, "approach the raised lid lip", left=brace(cover, 1., .054))
    p.add(1.1, "straddle the lid lip", left=brace(cover, 1.))
    p.add(.55, "hold the lip", left=brace(cover))
    p.add(2.2, "draw the cover through its guides", left=brace(cover+[-C.LID_TRAVEL-.010, 0., 0.]))
    p.add(.55, "release the shut cover", left=brace(cover+[-C.LID_TRAVEL-.010, 0., 0.], 1.))
    p.add(.8, "withdraw from the cassette", left=brace(cover+[-C.LID_TRAVEL-.010, 0., .045], 1.))
    return p.moves
