"""Verify a cut serving remains separate, lifted and correctly plated."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.stable_s = 0.0
        self.lifted = False

    def evaluate(self, state, elapsed):
        objects = state["objects"]
        material = state["material"]
        stock = np.asarray(objects["stock"]["pose"])
        serving = np.asarray(objects["slice"]["pose"])
        angle = objects["slicer"]["joints"]["WirePivot"]
        stock_axis = quat_xyzw_to_matrix(stock[3:])
        slice_axis = quat_xyzw_to_matrix(serving[3:])
        separated = material["severed"]
        cut_line_error = abs(stock[0] + C.STOCK_SIZE[0] / 2 + C.GAP / 2 - C.PIVOT[0])
        aligned = (
            cut_line_error < 0.004 and abs(stock[1]) < 0.021 and stock_axis[0, 0] > 0.99
        )
        stock_on_board = (
            abs(stock[0] - C.STOCK_CUT[0]) < 0.035
            and abs(stock[1]) < 0.040
            and abs(stock[2] - C.STOCK_CUT[2]) < 0.006
            and stock_axis[2, 2] > 0.97
        )
        top_clearance = (
            serving[2] - (np.abs(slice_axis[2]) @ (C.SLICE_SIZE / 2)) - C.BOARD_TOP
        )
        self.lifted |= separated and stock_on_board and top_clearance > 0.025
        on_plate = (
            np.linalg.norm(serving[:2] - C.PLATE[:2]) < 0.035
            and abs(serving[2] - (C.PLATE[2] + C.SLICE_SIZE[2] / 2)) < 0.021
        )
        grippers_clear = all(
            min(
                np.linalg.norm(np.asarray(p) - location)
                for location in (serving[:3], stock[:3], C.PIVOT + [0, -0.125, 0.07])
            )
            > 0.047
            for hand in state["hands"].values()
            for p in hand["points"]
        )
        open_wire = angle < np.deg2rad(-32.0)
        still = all(
            np.linalg.norm(objects[n]["velocity"][:3]) < 0.02
            for n in ("stock", "slice")
        )
        full = (
            separated
            and self.lifted
            and on_plate
            and stock_on_board
            and open_wire
            and grippers_clear
            and still
        )
        self.stable_s = self.stable_s + elapsed if full else 0.0
        success = self.stable_s >= 0.50
        if success:
            score = 1.0
        elif separated:
            score = 0.65
        elif aligned:
            score = 0.25
        else:
            score = 0.0
        return {
            "success": bool(success),
            "score": score,
            **material,
            "aligned": bool(aligned),
            "cut_line_error_mm": round(1000 * cut_line_error, 3),
            "stock_on_board": bool(stock_on_board),
            "lifted_independently": bool(self.lifted),
            "on_plate": bool(on_plate),
            "wire_open": bool(open_wire),
            "hands_clear": bool(grippers_clear),
            "stable_s": round(self.stable_s, 3),
        }
