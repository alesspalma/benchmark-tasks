"""Reference waypoints for sealing, pumping and contact pickup."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def frame(closing, direction):
    x = np.asarray(closing, dtype=float)
    z = np.asarray(direction, dtype=float)
    z /= np.linalg.norm(z)
    x -= x.dot(z)*z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


TRAY_ROT = rotation("Z", np.pi) @ np.diag([1., -1., -1.]) @ rotation("X", .47)
PUMP_ROT = frame([0., -1., 0.], [.42, 0., -1.])
PEN_ROT = np.diag([1., -1., -1.]) @ rotation("X", .48)
TRAY_GRIP = np.array([-.062, .001, .010])
PEN_GRIP = np.array([.002, .003, .013])
PICKUP_OFFSET = np.array([-.0038, -.0007, -.008])
POCKET_OFFSET = np.array([.000, .0035, -.009])
LOOSE = .285
SQUEEZE = .075


def tray_at(center, up, grip):
    return np.asarray(center)+TRAY_GRIP+[0., 0., up], TRAY_ROT, grip


def pen_at(origin, grip):
    return np.asarray(origin)+PEN_GRIP, PEN_ROT, grip


def pump_at(angle, lift):
    point = C.PUMP + C.PIVOT + rotation("Y", angle) @ C.LEVER_END
    return point+[0., 0., .005+lift], PUMP_ROT, .07


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    target = C.TRAY_ON_PLATE.copy()
    if mode == "bad-seal":
        target[0] -= .016
    carrier_target = target + [-.008, -.002, -.002]
    p.add(1.7, "Approach the carrier lip", left=tray_at(C.TRAY_START, .072, .34),
          right=pen_at(C.PEN+[0., 0., .065], .55))
    p.add(.80, "Settle above the carrier lip", left=tray_at(C.TRAY_START, .036, .42))
    p.add(1.05, "Straddle the carrier lip", left=tray_at(C.TRAY_START, 0., .42),
          right=pen_at(C.PEN+[0., 0., .032], .55))
    p.add(.45, "Hold the open fingers across the lip")
    p.add(.8, "Grip the carrier", left=tray_at(C.TRAY_START, 0., .019),
          right=pen_at(C.PEN, .55))
    p.add(1.15, "Lift the loaded carrier", left=tray_at(C.TRAY_START, .104, .019),
          right=pen_at(C.PEN, SQUEEZE))
    p.add(.35, "Clear the staging guides")
    p.add(1.65, "Align the carrier above the seal", left=tray_at(carrier_target, .096, .019),
          right=pen_at(C.PEN+[0., 0., .051], SQUEEZE))
    p.add(1.5, "Lower the carrier onto the plate", left=tray_at(carrier_target, -.001, .019))
    p.add(.65, "Release the seated carrier", left=tray_at(carrier_target, -.001, .32))
    p.add(.80, "Clear the plate", left=tray_at(carrier_target, .095, .35))
    p.add(.95, "Approach the hand pump", left=pump_at(0., .049))
    if mode != "no-vacuum":
        for _ in range(6):
            p.add(.85, "Push the pump handle", left=pump_at(-C.PUMP_STROKE, -.006))
            p.add(.78, "Let the pump return", left=pump_at(0., .027))
    else:
        p.add(9.78, "Leave the plate at atmospheric pressure")
    die = target + C.DIE_OFFSETS["selected"] + [0., 0., C.DIE_THICKNESS]
    nozzle_origin = die - C.PEN_TIP + PICKUP_OFFSET
    pick_clear = nozzle_origin + [0., 0., .075]
    p.add(1., "Raise the primed pen", right=pen_at(C.PEN+[0., 0., .118], SQUEEZE))
    p.add(2.5, "Bring the primed pen over the target", right=pen_at(pick_clear, SQUEEZE))
    p.add(.55, "Settle above the target")
    p.add(1.65, "Seat the cup squarely on the die", right=pen_at(nozzle_origin, SQUEEZE))
    p.add(.45, "Seat the cup squarely on the die", right=pen_at(nozzle_origin, SQUEEZE))
    p.add(.75, "Release the bulb to draw suction", right=pen_at(nozzle_origin, LOOSE))
    p.add(1.55, "Lift the selected die", right=pen_at(pick_clear, LOOSE))
    p.add(.45, "Clear the carrier")
    pocket_origin = C.RECEIVER + [0., 0., C.DIE_THICKNESS] - C.PEN_TIP + POCKET_OFFSET
    p.add(2.5, "Carry the die over the receiving pocket",
          right=pen_at(pocket_origin+[0., 0., .058], LOOSE))
    p.add(.45, "Settle above the receiving pocket")
    p.add(1.35, "Seat the die on the receiving pocket floor", right=pen_at(pocket_origin, LOOSE))
    p.add(.33, "Rest the die on the pocket floor")
    p.add(.58, "Squeeze the bulb to release the die", right=pen_at(pocket_origin, SQUEEZE))
    vent = C.PUMP + C.VENT
    p.add(1., "Withdraw the cup and approach the vent",
          left=(vent+[0., 0., .044], PUMP_ROT, .07),
          right=pen_at(pocket_origin+[0., 0., .07], SQUEEZE))
    p.add(.83, "Press the pump vent", left=(vent-[0., 0., .007], PUMP_ROT, .07),
          right=pen_at(C.PEN+[0., 0., .066], SQUEEZE))
    p.add(.50, "Vent the vacuum plate")
    p.add(.85, "Lower the pen into its stand",
          left=(vent+[0., 0., .085], PUMP_ROT, .10), right=pen_at(C.PEN+[0., 0., .003], SQUEEZE))
    p.add(.45, "Release the pen in its stand", right=pen_at(C.PEN+[0., 0., .003], .55))
    p.add(.95, "Clear the station", right=pen_at(C.PEN+[0., -.037, .079], .55))
    return p.moves
