"""Native robot-only reference; the damaged device is never touched in demo."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def waypoints(mode="demo"):
    if mode == "hold":
        return {}
    p = Program()
    obj = C.BAD.copy() if mode in {"bad", "bad-unplug"} else C.GOOD.copy()
    # A prong bank is held by its ordinary body, with jaws descending from above.
    grasp = obj + [-0.008, 0, 0.012]
    side = "left" if mode in {"bad", "bad-unplug", "left", "alternate"} else "right"
    down = C.DOWN
    ori = rotation("Y", -np.deg2rad(35 if side == "left" else 20)) @ down
    def add(t, label, pos=None, grip=0.72):
        kw = {} if pos is None else {side: (np.asarray(pos), ori, grip)}
        p.add(t, label, **kw)
    add(3, "approach intact power bank", grasp + [0, 0, 0.10], 1)
    add(3, "lower jaws around ordinary enclosure", grasp, 1)
    add(2, "grasp power bank body", grasp, 0.72)
    add(3, "lift bank clear of ceramic pad", grasp + [0, 0, 0.12])
    target = np.array([
        C.SOCKET[0] - C.BODY_X / 2 - 0.0007,
        C.SOCKET[1] + C.OUTLET_YS[0],
        C.SOCKET[2],
    ])
    if mode in {"bad", "bad-unplug", "alternate", "left"}:
        target[1] = C.SOCKET[1] + C.OUTLET_YS[1]
    tcp = target + [-0.008, 0, 0.012]
    pre = tcp + [-0.035, 0, 0]
    add(4, "align real plug blades in front of outlet", pre + [0, 0, 0.03])
    add(3, "lower blade pair to socket height", pre)
    add(2, "settle alignment before insertion")
    if mode == "near":
        add(3, "stop short of outlet")
        return p.moves
    add(6, "insert both blades against passive socket contacts", tcp + [0.004, 0, 0])
    add(7, "confirm stable charging contact")
    if mode == "held":
        add(4, "keep holding connected bank")
        return p.moves
    if mode == "bad-unplug":
        add(3, "unplug after unsafe energization", pre)
        add(3, "return damaged bank to pad", grasp)
        add(2, "release damaged bank", grasp, 1)
        add(3, "withdraw after unsafe charging", grasp + [0, 0, 0.15], 1)
        return p.moves
    add(2, "release plugged bank", tcp, 1)
    add(3, "withdraw open gripper above connected bank", tcp + [-0.10, 0, 0.10], 1)
    add(4, "leave bank charging independently", np.array([0.28, 0.32 if side == "left" else -0.32, 0.94]), 1)
    add(5, "observe released charging indication")
    return p.moves
