"""Teach stock indexing, a loaded cut and a clean single-slice transfer."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def hand(approach, pinch):
    a = np.asarray(approach, dtype=float)
    a /= np.linalg.norm(a)
    b = np.asarray(pinch, dtype=float) - (a @ pinch) * a
    b /= np.linalg.norm(b)
    return np.column_stack((b, np.cross(a, b), a))


LEFT = hand([0.60, -0.25, -0.76], [0.0, -1.0, 0.0])
RIGHT = hand([0.50, 0.33, -0.80], [1.0, 0.0, 0.0])


def lever(angle, grip, offset=(0.0, 0.0, 0.008)):
    return C.PIVOT + rotation("X", angle) @ C.HANDLE + offset, RIGHT, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    at_stock = C.STOCK_RESET + [-0.014, 0.0, 0.009]
    at_cut = C.STOCK_CUT + [-0.012, 0.0, 0.009]
    at_slice = C.STOCK_CUT + [C.BOND_LENGTH + 0.003, 0.0, 0.005]
    upright = rotation("Z", np.deg2rad(6.0)) @ LEFT
    p.add(
        1.8,
        "Approach the intact block",
        left=(at_stock + [-0.048, 0.042, 0.084], LEFT, 0.92),
    )
    p.add(1.25, "Bring fingers to the two cheese faces", left=(at_stock, LEFT, 0.92))
    p.add(0.75, "Grasp the loose stock", left=(at_stock, LEFT, 0.43))
    p.add(
        0.8,
        "Lift the block just clear of the board",
        left=(at_stock + [0, 0, 0.020], LEFT, 0.43),
    )
    p.add(
        1.5,
        "Square the block to the eight millimetre mark",
        left=(at_cut + [0, 0, 0.018], upright, 0.43),
    )
    p.add(0.9, "Lower the block onto the board", left=(at_cut, upright, 0.43))
    p.add(
        1.4,
        "Approach the raised wire handle",
        right=lever(C.OPEN, 0.96, (-0.066, -0.058, 0.065)),
    )
    p.add(1.1, "Bring fingers around the handle", right=lever(C.OPEN, 0.96))
    p.add(0.65, "Grasp the wire handle", right=lever(C.OPEN, 0.16))
    stop_angle = -14 if mode == "short_stroke" else 0
    for degrees in (-45, -34, -24, -14, -8, -3, 0):
        if degrees > stop_angle:
            break
        p.add(
            0.40,
            "Drive the tensioned wire down through the cheese",
            right=lever(np.deg2rad(degrees), 0.17),
        )
    p.add(0.7, "Hold the cutting stroke against resistance")
    for degrees in (-12, -25, -40, -51):
        p.add(
            0.37,
            "Raise the wire clear of the cheese",
            right=lever(np.deg2rad(degrees), 0.17),
        )
    p.add(
        0.55,
        "Open both grippers at the raised wire",
        left=(at_cut, upright, 0.92),
        right=lever(C.OPEN, 0.95, (0, 0, 0.018)),
    )
    p.add(
        0.85,
        "Move the wire hand back and clear the stock",
        left=(at_cut + [0.001, 0.005, 0.086], upright, 0.92),
        right=lever(C.OPEN, 0.96, (-0.063, -0.096, 0.086)),
    )
    if mode == "short_stroke":
        return p.moves
    p.add(
        1.05,
        "Reach only for the newly separated serving",
        left=(at_slice + [0.012, 0.001, 0.063], LEFT, 0.92),
    )
    p.add(1.0, "Place the fingers at the serving edges", left=(at_slice, LEFT, 0.92))
    p.add(0.65, "Grip just the separated slice", left=(at_slice, LEFT, 0.46))
    p.add(
        0.9,
        "Slide the serving clear of the remaining block",
        left=(at_slice + [0.029, 0.002, 0.0], LEFT, 0.46),
    )
    p.add(
        0.95,
        "Lift the single serving free of the board",
        left=(at_slice + [0.029, 0.002, 0.090], LEFT, 0.46),
    )
    landing = C.PLATE + [0.0, 0.0, C.SLICE_SIZE[2] / 2 + 0.012]
    p.add(
        1.25,
        "Carry the serving over the side plate",
        left=(landing + [0, 0, 0.10], LEFT, 0.46),
    )
    p.add(0.95, "Lower the serving onto the plate", left=(landing, LEFT, 0.46))
    p.add(0.55, "Release the serving", left=(landing, LEFT, 0.94))
    p.add(
        0.95,
        "Withdraw from the plate",
        left=(landing + [-0.060, 0.023, 0.094], LEFT, 0.95),
    )
    p.add(0.7, "Leave the plated serving and opened slicer undisturbed")
    return p.moves
