"""Move both sewn ends past the deck edges, then tuck and release."""

import numpy as np
from yam_arena.constants import DOWN_XYZW
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C
from .motion import Program, rotation

DOWN = rotation("Y", -np.pi/6) @ rotation("Z", np.pi) @ quat_xyzw_to_matrix(DOWN_XYZW)


def hem(sign, y, z, grip=1., x=0.):
    return np.array([C.BOARD_X + x, sign * y, C.BOARD_TOP + z]), DOWN, grip


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
    p.add(.55, "release the elastic before moving the wrists", left=hem(1,.408,-.029,.58),
          right=hem(-1,.408,.017 if mode == "lifted" else -.029,.58))
    p.add(1.3, "withdraw across the hem ends", left=hem(1,.422,-.029,.58,-.144),
          right=hem(-1,.422,.017 if mode == "lifted" else -.029,.58,-.144))
    p.add(1., "clear both ends", left=hem(1,.428,.108,1.,-.130), right=hem(-1,.428,.108,1.,-.130))
    return p.moves
