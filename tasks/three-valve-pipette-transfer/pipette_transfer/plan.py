"""Reset-calibrated bimanual positions for charging, aspirating and delivering."""

import numpy as np

from . import config as C
from .motion import Program


def aim(x, y, z, flip=False):
    forward = np.array([x, y, z], dtype=float)
    forward /= np.linalg.norm(forward)
    jaw = np.array([1., 0., 0.]) - forward[0] * forward
    jaw /= np.linalg.norm(jaw)
    if flip:
        jaw *= -1
    return np.column_stack((jaw, np.cross(forward, jaw), forward))


AIR_ROT = aim(.44, -1., .11, flip=True)
BULB_ROT = aim(.30, 1., -.12)
HOLD_ROT = aim(.54, -1., .04, flip=True)
VALVE_ROT = aim(.41, 1., -.29)
DELIVERY_ROT = aim(.45, 1., -.08)
HOLD_GRIP = .036


def pipette_pose(tip, opening=HOLD_GRIP):
    return tip + C.BODY_GRIP + [0., -.004, 0.], HOLD_ROT, opening


def bulb_pose(opening, retraction=0., offset=0.):
    target = C.PARKED + C.BULB + [offset, 0., 0.]
    return target - retraction * BULB_ROT[:, 2], BULB_ROT, opening


def press(tip, valve, opening, retreat=0.):
    point = tip + C.VALVES[valve]
    axis = {"Air": AIR_ROT, "Suction": VALVE_ROT, "Delivery": DELIVERY_ROT}[valve]
    bias = [0., -.002, .005] if valve == "Delivery" else [-.002, 0., 0.]
    return point + bias - retreat * axis[:, 2], axis, opening


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    p.add(2.6, "reach the air valve and the bulb",
          left=press(C.PARKED, "Air", .64, .045), right=bulb_pose(.93, .073))
    p.add(1.1, "position fingers around the bulb",
          left=press(C.PARKED, "Air", .58), right=bulb_pose(.86))
    if mode != "no-charge":
        p.add(.82, "open air valve A",
              left=press(C.PARKED, "Air", .155))
    else:
        p.add(.82, "leave air valve A shut")
    p.add(1.15, "compress the bulb while it can vent", right=bulb_pose(.5935, offset=-.005))
    p.add(.5, "let the air escape")
    p.add(.60, "seal the air valve with the bulb still compressed",
          left=press(C.PARKED, "Air", .63, .057))
    p.add(.81, "let the bulb recover under vacuum", right=bulb_pose(.90))
    left_position = C.PARKED + C.BODY_GRIP
    p.add(1.1, "move to the rubber pipette adapter",
          left=(left_position - .064 * HOLD_ROT[:, 2], HOLD_ROT, .68),
          right=bulb_pose(.91, .15))
    p.add(1.10, "surround the adapter sleeve", left=pipette_pose(C.PARKED, .67))
    p.add(1.10, "grasp the adapter sleeve", left=pipette_pose(C.PARKED))
    raised = C.PARKED + [0., 0., .032]
    p.add(1.4, "lift the pipette off its support", left=pipette_pose(raised))
    p.add(1.65, "move the tip over the source",
          left=pipette_pose(C.SOURCE + [0., 0., C.CLEAR_RIM]))
    p.add(.50, "steady the pipette over the source")
    p.add(.85, "immerse the tip in the water",
          left=pipette_pose(C.ASPIRATE))
    p.add(.80, "approach the suction valve",
          right=press(C.ASPIRATE, "Suction", .58, .059))
    p.add(.85, "position the fingers at S",
          right=press(C.ASPIRATE, "Suction", .57))
    p.add(.85, "pinch suction valve S",
          right=press(C.ASPIRATE, "Suction", .145))
    p.add(1.65, "draw water up to the graduation")
    p.add(.65, "close S to trap the column",
          right=press(C.ASPIRATE, "Suction", .59, .066))
    p.add(1.10, "clear the transfer path before lifting the pipette",
          right=press(C.DELIVER, "Delivery", .52, .103))
    p.add(.75, "lift the measured charge clear of the source",
          left=pipette_pose(C.SOURCE + [0., 0., C.CLEAR_RIM]))
    p.add(1.65, "move the held pipette over the receiving beaker",
          left=pipette_pose(C.RECEIVER + [0., 0., C.CLEAR_RIM]))
    p.add(.90, "lower the outlet over the receiving vessel",
          left=pipette_pose(C.DELIVER))
    p.add(.45, "hold the pipette steady at the receiver")
    if mode == "no-delivery":
        return p.moves
    p.add(1.15, "position the fingers at E",
          right=press(C.DELIVER, "Delivery", .43))
    p.add(.85, "pinch delivery valve E",
          right=press(C.DELIVER, "Delivery", .14))
    p.add(4., "allow the measured volume to drain")
    p.add(.65, "withdraw from the delivery valve",
          right=press(C.DELIVER, "Delivery", .63, .072))
    return p.moves
