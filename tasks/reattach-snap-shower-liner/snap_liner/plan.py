"""Fixed-reset Cartesian teaching program for the reference manipulation."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def attitude(approach, pinch):
    axis = np.array(approach, dtype=float)
    axis /= np.linalg.norm(axis)
    closing = np.array(pinch) - axis * np.dot(axis, pinch)
    closing /= np.linalg.norm(closing)
    return np.column_stack((closing, np.cross(axis, closing), axis))


RING_ROT = attitude((0.25, 0.70, -0.25), (1.0, 0.0, 0.0))
FLAT = rotation("X", -np.pi / 2)
HEM_ROT = attitude((1.60, -0.52, -0.21), (0.0, 0.0, 1.0))
CLASP_ROT = attitude((2.1, -0.85, -1.0), (0.0, -1.0, 0.0))


def ring(center, dz=0.0, openings=0.09, shape_rot=None):
    rot = np.eye(3) if shape_rot is None else shape_rot
    return (
        np.asarray(center) + rot @ C.RING_GRIP + [0.0, 0.0, dz],
        rot @ RING_ROT,
        openings,
    )


def hem(angle, dz=0.0, opening=0.18, dx=0.0, dy=0.0):
    p = C.CORNER_PIVOT + rotation("X", angle) @ C.GRIP_HEM + [0.002 + dx, dy, dz]
    return p, HEM_ROT, opening


def clasp(center, angle, delta, grip=0.13):
    position = np.asarray(center) + C.GATE_PIVOT + rotation("Y", angle) @ C.GATE_TAB
    return position + delta, CLASP_ROT, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    start, target = C.RING_START, C.RING_MOUNT
    high = target + [-0.029, 0.0, 0.037]
    before_thread = target + [-0.030, 0.0, -0.014]
    hold = hem(-0.006, 0.002, 0.18, -0.012, -0.010)
    h = mode in ("demo", "open-only")
    open_angle = C.GATE_OPEN
    p.add(
        1.8,
        "Approach the open replacement ring and the free liner hem",
        right=ring(start, 0.086, 0.52, FLAT),
        left=hem(C.LOOSE, 0.032, 0.62, -0.091) if h else None,
    )
    p.add(
        1.04,
        "Straddle the ring spine and the reinforced liner hem",
        right=ring(start, 0.0, 0.44, FLAT),
        left=hem(C.LOOSE, 0.0, 0.60) if h else None,
    )
    p.add(
        0.58,
        "Grip the two separate objects",
        right=ring(start, 0.0, 0.09, FLAT),
        left=hem(C.LOOSE) if h else None,
    )
    p.add(
        1.08,
        "Lift the ring and raise the liner corner",
        right=ring(start, 0.115, 0.09, FLAT),
        left=hem(0.28, 0.002, 0.18, -0.012, -0.009) if h else None,
    )
    p.add(1.05, "Turn the ring upright", right=ring(start, 0.140))
    p.add(
        2.1,
        "Raise the opening clear in front of the rod",
        right=ring(high + [-0.148, 0.0, 0.0]),
        left=hold if h else None,
    )
    p.add(1.65, "Slip the open ring around the installed rod", right=ring(high))
    p.add(
        0.88,
        "Lower the ring nose toward the reinforced eyelet",
        right=ring(before_thread),
    )
    target = target + [0.003, 0.0, 0.0]
    p.add(
        1.18, "Feed the open ring nose through the eyelet", right=ring(target, -0.014)
    )
    p.add(
        0.42,
        "Hold the spine while making room for the snap closure",
        left=(hold[0], hold[1], 0.58) if h else None,
    )
    p.add(
        0.84,
        "Back clear of the hem edge",
        left=hem(-0.005, 0.041, 0.52, -0.088) if h else None,
    )
    if mode == "open-only":
        p.add(2.0, "Keep the opened clasp free")
        p.add(
            0.66,
            "Let the installed ring bear the liner weight",
            right=ring(target, -0.014, 0.55),
        )
        p.add(
            1.1,
            "Withdraw both hands",
            right=ring(target + [-0.115, -0.121, 0.005], 0.0, 0.55),
        )
        return p.moves
    p.add(
        1.3,
        "Clear the metal rod before reaching across",
        left=clasp(target, open_angle, [-0.212, 0.126, 0.105], 0.16),
    )
    p.add(
        1.4,
        "Bring the closed finger pads above the snap clasp",
        left=clasp(target, open_angle, [-0.015, -0.006, 0.030], 0.035),
    )
    p.add(
        1.20,
        "Press the exposed thumb clasp",
        left=clasp(target, open_angle, [-0.012, -0.006, 0.005], 0.035),
    )
    p.add(
        1.1,
        "Rotate the clasp toward its catch",
        left=clasp(target, np.deg2rad(-74), [-0.011, -0.006, -0.002], 0.035),
    )
    p.add(
        1.05,
        "Push the clasp past its snap point",
        left=clasp(target, np.deg2rad(-43), [-0.010, -0.006, -0.010], 0.035),
    )
    p.add(
        0.90,
        "Withdraw the pusher from the clicked clasp",
        left=clasp(target, np.deg2rad(-43), [-0.106, 0.082, 0.072], 0.44),
    )
    p.add(2.0, "Let the clasp seat fully before giving it the load")
    p.add(
        0.66,
        "Let the installed ring bear the liner weight",
        right=ring(target, -0.014, 0.55),
    )
    p.add(
        1.1,
        "Withdraw both hands",
        right=ring(target + [-0.115, -0.121, 0.005], 0.0, 0.55),
    )
    return p.moves
