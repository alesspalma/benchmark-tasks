"""Native Cartesian routing paths for the two lace tails."""

import numpy as np
from yam_arena import ik

from . import config as c
from .lace import initial_lace, link_name
from .motion import Program

PICKUP_SECONDS = 3.35


def grip_frame(sign):
    approach = np.array([.40, -.85*sign, -1.])
    approach /= np.linalg.norm(approach)
    closing = np.array([-sign, 0., 0.])
    closing -= np.dot(closing, approach)*approach
    closing /= np.linalg.norm(closing)
    return np.column_stack((closing, np.cross(approach, closing), approach))


def pickup_frames(scene=None):
    poses = None if scene is None else scene["objects"]["boot"]["links"]
    points = {}
    for side, sign in c.SIDES.items():
        index = c.LINK_COUNT-(2 if side == "L" else 4)
        if poses is None:
            p = initial_lace(side)[0][index]
            tangent = np.array([0., 0., -1.])
        else:
            p, a, b = [np.asarray(poses[link_name(side, i)][:3])
                       for i in (index, index-1, index+1)]
            tangent = b-a
        approach = np.array([.43, -.85*sign, -.36])
        approach /= np.linalg.norm(approach)
        tangent -= np.dot(tangent, approach)*approach
        tangent /= np.linalg.norm(tangent)
        closing = np.cross(tangent, approach)
        if closing[0]*sign > 0:
            closing = -closing
        points[side] = (p, np.column_stack((closing, np.cross(approach, closing), approach)))
    return points


def waypoints(mode, pickups=None):
    if mode == "hold":
        return {}
    move = Program()
    pickups = pickup_frames() if pickups is None else pickups
    hands = (("left", "L"),) if mode == "one-side" else (("left", "L"), ("right", "R"))
    takes = {}
    for name, side in hands:
        tip, frame = pickups[side]
        takes[name] = (tip - .008*frame[:, 2], frame)
    move.add(2.1, "Approach both hanging ends", **{
        name: (tip-.074*frame[:, 2], frame, .26) for name, (tip, frame) in takes.items()
    })
    move.add(PICKUP_SECONDS-2.1, "Reach both hanging ends", **{
        name: (tip, frame, .26) for name, (tip, frame) in takes.items()
    })
    move.add(.60, "Take both ends of the same lace", **{
        name: (tip, frame, 0.) for name, (tip, frame) in takes.items()
    })
    for name, side in hands:
        direction = c.SIDES[side]
        _, down = takes[name]
        frame = grip_frame(direction)
        same = lambda point, grip: {name: (np.asarray(point)+[0., 0., c.BOOT_RISE]-.009*frame[:, 2], frame, grip)}
        move.add(1.25, "Lift hanging lace outside the boot "+side,
                 **{name: (np.array([.445, direction*.131, 1.010]), down, 0.)})
        move.add(1.65, "Turn wrist above the outer edge "+side,
                 **same([.445, .132*direction, .929], 0.))
        move.add(.8, "Bring lace over the forefoot "+side,
                 **same([.452, .061*direction, .930], 0.))
        move.add(2.4, "Cross in front of the nearest hook "+side,
                 **same([.448, -.029*direction, .932], 0.))
        above_hook = [.373, -.101, .917] if side == "L" else [.375, .098, .917]
        move.add(1.35, "Draw the strand over the opposing hook "+side,
                 **same(above_hook, 0.))
        seat = [.371, -.104, .884] if side == "L" else [.375, .101, .875]
        move.add(1.10, "Pull lace into the locking throat "+side, **same(seat, 0.))
        if side == "R":
            seat = [.373, .101, .875]
        move.add(.70, "Set tension at the clamping hook "+side, **same(seat, 0.))
        move.add(.70, "Open the gripper without moving the lace "+side, **same(seat, .56))
        clear = np.asarray(seat) + [0., 0., .092]
        move.add(.75, "Lift open fingers above the boot "+side, **same(clear, .62))
        move.add(1.10, "Cross high over the collar "+side,
                 **same([.354, direction*.117, .963], .76))
        parked = (*ik.tcp_pose(c.STOWED[name], c.BASES[name]), 1.)
        move.add(1.60, "Return to the outside lane "+side, **{name: parked})
    return move.moves
