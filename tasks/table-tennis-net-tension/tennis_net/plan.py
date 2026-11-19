"""Drive the reel through its knob and set the near guide by pressing its cap."""

import numpy as np

from . import config as C
from .motion import Program, rotation

REEL_HAND = rotation("Y", np.deg2rad(-15.)) @ rotation("X", np.deg2rad(205.))
POST_HAND = rotation("Y", np.deg2rad(-45.)) @ rotation("X", np.deg2rad(163.))


def reel_pose(angle, height=0., offset=.018):
    phase = angle + C.CRANK_ANGLE
    radius = np.array([np.cos(phase), np.sin(phase), 0.])
    tangent = np.array([-radius[1], radius[0], 0.])
    xyz = C.SPINDLE + C.CRANK_RADIUS * radius - offset * tangent + [0., 0., .032 + height]
    return xyz, REEL_HAND, 0.


def post_pose(travel, above=0.):
    cap = C.POST_AXIS + [0., 0., .004 - travel + above]
    return cap - .013 * POST_HAND[:, 2], POST_HAND, 0.


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    if mode != "height-only":
        p.add(2.4, "approach behind the winding knob", right=reel_pose(0., .105))
        p.add(1.3, "descend beside the winding knob", right=reel_pose(0.))
        end = C.ANGULAR_STOP + .375
        for angle in np.linspace(0., end, 17)[1:]:
            p.add(.34, "drive the knob against cord tension", right=reel_pose(angle))
        p.add(.65, "hold at the final tooth")
        p.add(.85, "clear the retained winding knob", right=reel_pose(end, .10))
        raised, orient, grip = reel_pose(end, .10)
        p.add(.95, "park the winding hand", right=(raised + [-.085, -.175, .025], orient, grip))
    if mode != "tension-only":
        p.add(1.5, "approach above the near guide", left=post_pose(0., .107))
        p.add(1.15, "align with the head of the guide", left=post_pose(0., .014))
        p.add(1.45, "press the guide to playing height", left=post_pose(C.POST_TRAVEL + .017))
        p.add(.60, "set the friction shoe under load")
        p.add(.70, "withdraw from the guide", left=post_pose(C.POST_TRAVEL, .095))
    return p.moves
