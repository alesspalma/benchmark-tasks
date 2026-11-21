"""Cartesian reference for the two arms with ordinary grasps and tool contacts."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def down(yaw=0., tilt=0.):
    return rotation("Z", yaw) @ np.diag([1., -1., -1.]) @ rotation("X", tilt)


def aim(direction):
    forward = np.asarray(direction, dtype=float)
    forward /= np.linalg.norm(forward)
    jaw = np.array([1., 0., 0.]) - forward[0] * forward
    jaw /= np.linalg.norm(jaw)
    return np.column_stack((jaw, np.cross(forward, jaw), forward))


LEFT_R = down(np.radians(-180.), .53)
DROP_R = down(C.DROPPER_YAW, .61)
COVER_R = aim([.30, 1., -.48])
DRIVER_GRASP_YAW = np.radians(-49.)
DRIVER_R = down(DRIVER_GRASP_YAW, .60)
DRIVER_BLADE_OFFSET = C.DRIVER_REST_YAW - DRIVER_GRASP_YAW

def driver_at(screw_angle):
    return down(screw_angle - DRIVER_BLADE_OFFSET, .60)
OPEN = .66
LEFT_GRIP = .19
DROP_HOLD = .292
DROP_PRESS = .168
DRIVER_GRIP = .16

L_START = C.INSTRUMENT + C.HANDLE + [0., 0., .003]
L_HELD = L_START + (C.HELD - C.INSTRUMENT)
DROP_ABOVE = C.HELD + C.PRISM - C.DROPPER_TIP + [0., 0., .060]
DROP_TOUCH = DROP_ABOVE - [0., 0., .040]
DRIVER_ABOVE = C.HELD + C.SCREW - C.DRIVER_TIP + [0., 0., .036]
DRIVER_SEATED = DRIVER_ABOVE - [0., 0., .0384]


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    p.add(2., "approach the barrel and dropper",
          left=(L_START+[0., 0., .066], LEFT_R, OPEN),
          right=(C.DROPPER+[0., 0., .063], DROP_R, OPEN))
    p.add(.95, "lower open jaws around the overmold and bulb",
          left=(L_START, LEFT_R, OPEN), right=(C.DROPPER, DROP_R, OPEN))
    p.add(.95, "hold the optical barrel and dispensing bulb", left=(L_START, LEFT_R, LEFT_GRIP),
          right=(C.DROPPER, DROP_R, DROP_HOLD))
    p.add(1.05, "lift both items from their rests",
          left=(L_HELD, LEFT_R, LEFT_GRIP),
          right=(C.DROPPER+[0., 0., .104], DROP_R, DROP_HOLD))
    p.add(1., "bring the dropper above the exposed prism", right=(DROP_ABOVE, DROP_R, DROP_HOLD))
    p.add(.95, "align nozzle over prism", right=(DROP_TOUCH, DROP_R, DROP_HOLD))
    close = DROP_HOLD if mode == "dry" else DROP_PRESS
    p.add(1.10, "deliver distilled-water droplets", right=(DROP_TOUCH, DROP_R, close))
    p.add(.45, "stabilize the water blank", right=(DROP_TOUCH, DROP_R, DROP_HOLD))
    p.add(.75, "withdraw the dropper", right=(DROP_ABOVE, DROP_R, DROP_HOLD))
    p.add(1.1, "return toward the dropper stand",
          right=(C.DROPPER+[0., 0., .065], DROP_R, DROP_HOLD))
    p.add(.95, "seat the dropper in its holder", right=(C.DROPPER, DROP_R, DROP_HOLD))
    p.add(.5, "release the dropper", right=(C.DROPPER, DROP_R, OPEN))
    p.add(.8, "clear the dropper", right=(C.DROPPER+[0., 0., .075], DROP_R, OPEN))

    hinge = C.HELD + C.HINGE
    p.add(1.2, "approach the open daylight plate",
          right=(hinge+[.004, -.081, .049], COVER_R, .025))
    p.add(1.20, "push the daylight plate past vertical",
          right=(hinge+[.001, -.017, .027], COVER_R, .025))
    p.add(.75, "close the daylight plate against the prism",
          right=(hinge+[.001, .013, .007], COVER_R, .025))
    p.add(.8, "withdraw from the closed plate",
          right=(hinge+[.003, -.082, .095], COVER_R, OPEN))
    if mode != "unadjusted":
        p.add(1.25, "approach the calibration driver",
              right=(C.DRIVER+[0., 0., .073], DRIVER_R, OPEN))
        p.add(.85, "lower around the screwdriver handle", right=(C.DRIVER, DRIVER_R, OPEN))
        p.add(.75, "grip the calibration driver", right=(C.DRIVER, DRIVER_R, DRIVER_GRIP))
        p.add(1., "lift the driver clear of its rest",
              right=(C.DRIVER+[0., 0., .100], DRIVER_R, DRIVER_GRIP))
        p.add(1.15, "pose the blade over the slotted screw",
              right=(DRIVER_ABOVE, driver_at(C.SCREW_INIT), DRIVER_GRIP))
        p.add(.95, "center the blade just above its slot",
              right=(DRIVER_SEATED + [0., 0., .0075], driver_at(C.SCREW_INIT), DRIVER_GRIP))
        p.add(.55, "center the blade just above its slot")
        p.add(.75, "seat the screwdriver blade in the slot",
              right=(DRIVER_SEATED, driver_at(C.SCREW_INIT), DRIVER_GRIP))
        p.add(.45, "seat the screwdriver blade in the slot")
        p.add(1.70, "turn the seated calibration screw",
              right=(DRIVER_SEATED, driver_at(np.radians(-20.)), DRIVER_GRIP))
        p.add(.55, "turn the seated calibration screw")
        p.add(.85, "lift the driver out of the calibration slot",
              right=(DRIVER_SEATED + [0., 0., .022], driver_at(np.radians(-20.)), DRIVER_GRIP))
        p.add(.75, "draw the driver back across clear bench space")
    right_grip = OPEN if mode == "unadjusted" else DRIVER_GRIP
    p.add(1.30, "clear the tool and return the instrument to its rest",
          left=(L_START + [0., 0., .003], LEFT_R, LEFT_GRIP),
          right=(C.DRIVER+[0., -.016, .095], DRIVER_R, right_grip))
    p.add(.6, "release the supported instrument", left=(L_START + [0., 0., .003], LEFT_R, OPEN))
    p.add(1.1, "draw both hands clear of the prism",
          left=(L_START+[0., .052, .116], LEFT_R, OPEN),
          right=(C.DRIVER+[0., -.041, .092], DRIVER_R, right_grip))
    return p.moves
