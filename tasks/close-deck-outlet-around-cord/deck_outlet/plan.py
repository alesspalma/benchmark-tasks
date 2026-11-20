"""Bring the cable into the recessed hook, seat the lid, then operate the switch."""
import numpy as np

from . import config as C
from .motion import Program, rotation

FRONT = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
LOW = rotation("Y", np.radians(54.)) @ FRONT
BACK_DOWN = LOW


def pull(theta, clearance=0.):
    p = C.CABLE_PIVOT + rotation("Z",np.radians(theta)) @ [.012,.158,-.003]
    p[2] += clearance
    return p,LOW,.07


def lid(theta, clearance=0.):
    r = rotation("Y", np.radians(theta))
    p = C.HINGE + r @ [-.154,-.056,.010+clearance]
    return p,BACK_DOWN,.07


def switch(angle, clearance=0.):
    p = C.SWITCH_PIVOT + [-.038 + angle*.00086 - clearance, 0., -.001]
    return p,LOW,.07


def stow(y):
    return ([.224, y,.965],LOW,.95)


def waypoints(mode):
    if mode == "hold":
        return {}
    program = Program()
    if mode != "premature":
        program.add(1.5,"Approach the cord and lift clear of the raised cover",
                    left=pull(C.CABLE_OPEN,.12),
                    right=([.449,-.172,.994],BACK_DOWN,.07))
        program.add(.9,"Come around the cover's free right edge",
                    left=pull(C.CABLE_OPEN),right=([.520,-.124,.914],BACK_DOWN,.07))
        program.add(.8,"Contact the free edge from its opening side",
                    right=lid(C.LID_OPEN,.021))
        program.add(.6,"Rest the right fingertips on the cover",
                    right=lid(C.LID_OPEN))
        for angle in np.linspace(C.CABLE_OPEN,0.,8)[1:]:
            program.add(.18, "Guide the cord through the outlet side port",left=pull(angle))
        for angle in np.linspace(C.LID_OPEN, 0.,16)[1:]:
            program.add(.23, "Seat the cover while holding the cord at the hook",right=lid(angle))
        program.add(.65,"Lift above the last part of the lid travel",
                    right=([.344,-.044,.861],LOW,.07))
        program.add(.65,"Shift above the closing face",
                    right=([.353,-.030,.840],LOW,.07))
        program.add(1.5,"Press the top face onto its weather seal",
                    right=([.353,-.030,.738],LOW,.07))
        program.add(.65,"Let the gasket and trapped cable settle")
        program.add(.9,"Withdraw and approach the wall switch",left=stow(.255),
                    right=switch(0.,.095))
    else:
        program.add(1.3,"Approach the switch with the cover still open",
                    left=stow(.255),right=switch(0.,.095))
    program.add(.85,"Touch below the wall switch lever",right=switch(0.))
    for angle in np.linspace(0., C.SWITCH_ON+4,8)[1:]:
        program.add(.19,"Turn on the patio light",right=switch(angle))
    program.add(.9,"Release the wall switch",right=stow(-.26))
    program.add(1.2,"Leave both hands clear")
    return program.moves
