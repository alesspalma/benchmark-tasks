"""Move both sewn ends past the deck edges, then tuck and release."""

import numpy as np
from yam_arena.constants import DOWN_XYZW
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C
from .motion import Program, rotation

FINGERS_ALONG_BOARD = rotation("Z", np.pi) @ quat_xyzw_to_matrix(DOWN_XYZW)


def hem(sign, y, z, grip=1., x=0., tilt=30.):
    orientation = rotation("Y", -np.deg2rad(tilt)) @ FINGERS_ALONG_BOARD
    return np.array([C.BOARD_X + x, sign * y, C.BOARD_TOP + z]), orientation, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    p.add(2.8, "approach both loose cover ends", left=hem(1,.308,.095), right=hem(-1,.308,.095))
    p.add(1.5, "lower onto sewn elastic hems", left=hem(1,.308,.019,.46), right=hem(-1,.308,.019,.46))
    p.add(.75, "secure both elastic hems", left=hem(1,.308,.012,.035), right=hem(-1,.308,.012,.035))
    p.add(1.45, "stretch both ends against elastic tension", left=hem(1,.407,.017,.035), right=hem(-1,.407,.017,.035))
    p.add(.88, "pull nose hem below the deck", left=hem(1,.408,-.029,.035))
    if mode != "lifted":
        p.add(.88, "draw heel hem under the deck", right=hem(-1,.408,-.029,.035))
    heel_z = .017 if mode == "lifted" else -.031
    p.add(1.1, "angle fingertips below the rim without opening", left=hem(1,.416,-.031,.035,x=.005,tilt=78.),
          right=hem(-1,.420,heel_z,.035,x=.005,tilt=74.))
    p.add(1.1, "guide the captive hems inward under the rim", left=hem(1,.339,-.039,.035,x=.005,tilt=78.),
          right=hem(-1,.416 if mode == "lifted" else .339,heel_z if mode == "lifted" else -.039,.035,x=.005,tilt=74.))
    p.add(.45, "open fingers below the fitted hem", left=hem(1,.337,-.043,.7,tilt=77.),
          right=hem(-1,.416 if mode == "lifted" else .337,heel_z if mode == "lifted" else -.043,.7,tilt=77.))
    p.add(.85, "withdraw fingertips downward toward the mounts", left=hem(1,.408,-.055,.85,-.029,77.),
          right=hem(-1,.422 if mode == "lifted" else .408,-.055,.85,-.029,77.))
    p.add(.95, "clear the deck ends before lifting", left=hem(1,.428,-.057,1.,-.120,63.),
          right=hem(-1,.428,-.057,1.,-.120,63.))
    p.add(1., "clear both ends", left=hem(1,.428,.108,1.,-.130), right=hem(-1,.428,.108,1.,-.130))
    return p.moves
