"""Reference actions around the cold vent and catch cup."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def pinch(axis, approach):
    z = np.asarray(approach) / np.linalg.norm(approach)
    x = np.asarray(axis) - np.dot(axis, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


RIGHT = pinch([0., 0., -1.], [.98, .12, -.11])
LEFT = pinch([0., -1., 0.], [.39, -.14, -1.])


def cup(origin, opening=.10):
    return origin + C.CUP_HANDLE, LEFT, opening


def key(angle, opening=.07, extra=(0., 0., 0.)):
    frame = rotation("X", angle)
    return C.KEY_HOME + C.KEY_HANDLE + extra, frame @ RIGHT, opening


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    jug = C.CUP_START
    lift = jug + [0, 0, .129]
    catch = C.CUP_CATCH
    clear_key = key(0., .72, [-.072, -.027, .067])
    if mode == "demo":
        p.add(2.3, "approach the cup and bleed key", left=cup(lift, .66), right=clear_key)
        p.add(1.6, "lower fingers around cup grip", left=cup(jug, .63))
        p.add(.8, "hold the cup by its molded grip", left=cup(jug))
        p.add(1.25, "lift the empty cup", left=cup(lift))
        p.add(1.65, "place the cup beneath the side outlet", left=cup(catch))
    else:
        p.add(2.3, "approach the bleed key without a cup", right=clear_key)
    p.add(1.35, "approach the brass bleed key", right=key(0., .66))
    p.add(.8, "grasp the seated key", right=key(0.))
    p.add(1.5, "open the vent a quarter turn", right=key(-np.radians(88.)))
    p.add(1.55, "watch for a steady water stream")
    p.add(1.65, "turn the key to close the vent", right=key(np.radians(10.)))
    p.add(1.0, "finish seating the valve stem")
    p.add(.8, "release the closed vent key", right=key(np.radians(10.), .66))
    p.add(1.15, "withdraw the hand from the vent", right=clear_key)
    if mode == "demo":
        p.add(1.45, "return the cup to the bench", left=cup(lift))
        p.add(1.30, "lower the cup onto the worktop", left=cup(jug))
        p.add(.72, "release the filled cup", left=cup(jug, .68))
        p.add(1.1, "lift the open gripper clear", left=cup(lift, .68))
    return p.moves
