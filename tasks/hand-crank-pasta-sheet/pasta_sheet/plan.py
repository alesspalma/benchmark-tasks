"""Teach the stock pickup and crank path with normal YAM actions."""

import math

import numpy as np

from . import config as C
from .motion import Program, rotation


def aim(forward, jaw):
    forward = np.asarray(forward, dtype=float)
    forward /= np.linalg.norm(forward)
    jaw = np.asarray(jaw, dtype=float)
    jaw -= np.dot(jaw, forward) * forward
    jaw /= np.linalg.norm(jaw)
    return np.column_stack((jaw, np.cross(forward, jaw), forward))


PICK = aim((0.35, -1.0, 0.0), (0.0, 0.0, -1.0))
FEED = rotation("Y", math.pi / 2) @ PICK
CRANK = aim((0.60, 1.0, -0.10), (0.0, 0.0, -1.0))


def crank_pose(angle, opening, retreat=0.0):
    a = C.CRANK_START - 1.89 + angle
    offset = [C.CRANK_RADIUS * math.cos(a), C.CRANK_SPAN, -C.CRANK_RADIUS * math.sin(a)]
    return C.ROLL_NEAR + offset - retreat * CRANK[:, 2], CRANK, opening


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    pick = C.STOCK + [-0.021, 0.030, 0.0]
    start = C.NIP + [0.0, 0.055, 0.130]
    have_feed = mode != "crank-only"
    have_crank = mode != "feed-only"
    p.add(
        2.4,
        "approach the stock and handle",
        left=(pick + [-0.008, 0.063, 0.071], PICK, 0.50) if have_feed else None,
        right=crank_pose(0.0, 0.72, 0.078) if have_crank else None,
    )
    p.add(
        1.1,
        "get level with the overhang",
        left=(pick + [-0.015, 0.041, 0.0], PICK, 0.30) if have_feed else None,
        right=crank_pose(0.0, 0.66) if have_crank else None,
    )
    p.add(
        0.9,
        "surround the dough edge",
        left=(pick, PICK, 0.26) if have_feed else None,
        right=crank_pose(0.0, 0.20) if have_crank else None,
    )
    p.add(0.6, "pinch the loose stock", left=(pick, PICK, 0.0) if have_feed else None)
    p.add(
        1.2,
        "lift the loose sheet",
        left=(pick + [0.0, 0.0, 0.093], PICK, 0.0) if have_feed else None,
    )
    p.add(
        2.1,
        "turn the sheet toward the nip",
        left=(start, FEED, 0.0) if have_feed else None,
    )
    p.add(
        1.1,
        "offer the leading edge between the rolls",
        left=(C.NIP + [-0.002, 0.020, 0.055], FEED, 0.0) if have_feed else None,
    )
    for angle, push in zip(
        np.linspace(0.0, 2.75, 31)[1:], np.linspace(0.0, 0.046, 31)[1:], strict=True
    ):
        p.add(
            0.18,
            "crank while guiding the dough",
            left=(
                C.NIP + [-0.002, 0.020, 0.055 - push],
                FEED,
                0.030 if push > 0.020 else 0.0,
            )
            if have_feed
            else None,
            right=crank_pose(angle, 0.20) if have_crank else None,
        )
    p.add(
        0.35,
        "release the stock into the roller grip",
        left=(C.NIP + [-0.002, 0.020, 0.018], FEED, 0.20) if have_feed else None,
    )
    p.add(
        1.15,
        "withdraw from the incoming stock",
        left=(C.NIP + [-0.05, 0.055, 0.160], FEED, 0.7) if have_feed else None,
    )
    return p.moves
