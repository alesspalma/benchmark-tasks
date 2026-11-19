"""Task-space reference waypoints and two non-paying score controls."""

import numpy as np

from . import config as C
from .motion import Program, rotation


DOWN = rotation("X", np.pi)
LEFT = rotation("X", -.22) @ DOWN
RIGHT = DOWN
LIFT = np.array([0, 0, .059])
R_CAP = np.array([0., 0., .0130])
R_DRIVER = np.array([0., 0., C.DRIVER_GRIP_Z])
L_REMOTE = np.array([0., .066, .0115])
LOCK_REST = .00055


def left_hand(remote, gap=.23, up=0.):
    return remote + L_REMOTE + [0., 0., up], LEFT, gap


def right_cap(origin, opening=.19):
    return origin + R_CAP, RIGHT, opening


def right_driver(origin, opening=.23, angle=0.):
    return origin + R_DRIVER, rotation("Z", angle) @ RIGHT, opening


def put_away(p):
    target = C.SHELF + [0., -.009, C.SHELF_SURFACE - C.TABLE]
    p.add(1.10, "lift the remote from the service table", left=left_hand(C.REMOTE, up=.061))
    p.add(2.30, "carry the remote over the toy shelf", left=left_hand(target, up=.061))
    p.add(1.25, "set the remote down", left=left_hand(target))
    p.add(.55, "release the remote onto the shelf", left=left_hand(target, .77))
    p.add(1.15, "leave the shelf clear", left=left_hand(target, .77, .115))
    p.add(.65, "observe the returned remote")


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    if mode not in ("demo", "unsafe"):
        raise ValueError(f"Unknown policy {mode!r}")
    l = left_hand(C.REMOTE, .80, .080)
    cover_seat = C.REMOTE + C.SEAT_CENTER + [0., 0., C.SEAT_HEIGHT]
    if mode == "unsafe":
        p.add(2.15, "approach the remote", left=l)
        p.add(1.14, "bring the pads around the body", left=left_hand(C.REMOTE, .80))
        p.add(.85, "grasp the remote", left=left_hand(C.REMOTE))
        put_away(p)
        return p.moves

    p.add(2.15, "approach the cover", right=right_cap(C.CAP + LIFT, .70))
    p.add(1.14, "bring the pads around the cover", right=right_cap(C.CAP, .70))
    p.add(.67, "grasp the cover", right=right_cap(C.CAP))
    p.add(.85, "lift the cap from the service mat", right=right_cap(C.CAP + LIFT))
    p.add(1.4, "line up the cap with the two keyways", right=right_cap(cover_seat + LIFT))
    p.add(2.3, "seat the cap between the keyways", right=right_cap(cover_seat + [0, 0, -.003]))
    p.add(.60, "settle the cap", right=right_cap(cover_seat + [0, 0, -.003]))
    p.add(.80, "release the seated but unsecured cover",
          right=right_cap(cover_seat + [0, 0, -.003], .75))
    p.add(.9, "clear the remote above the cap", right=right_cap(cover_seat + LIFT, .75))
    p.add(2.15, "approach the screwdriver and remote", left=l,
          right=right_driver(C.DRIVER + LIFT, .75))
    p.add(1.15, "bring the pads around the screwdriver",
          left=left_hand(C.REMOTE, .80), right=right_driver(C.DRIVER, .75))
    p.add(.85, "grasp the remote and screwdriver",
          left=left_hand(C.REMOTE), right=right_driver(C.DRIVER))
    p.add(.85, "lift the screwdriver", right=right_driver(C.DRIVER + LIFT))
    tip = cover_seat + [0, 0, LOCK_REST]
    p.add(1.65, "approach the unlocked slot", right=right_driver(tip + [0, 0, .025]))
    p.add(2.15, "engage the blade in the slot", right=right_driver(tip))
    p.add(.55, "engage the blade against the slot floor", right=right_driver(tip))
    p.add(1.85, "turn the blade clockwise under load",
          right=right_driver(tip, angle=C.LOCK_ANGLE))
    p.add(.65, "settle against the closed stop",
          right=right_driver(tip, angle=C.LOCK_ANGLE))
    p.add(.85, "lift the blade out", right=right_driver(tip + LIFT, angle=C.LOCK_ANGLE))
    p.add(1.50, "move the screwdriver out of the carrying path",
          right=right_driver(C.DRIVER + LIFT, angle=C.LOCK_ANGLE))
    put_away(p)
    return p.moves
