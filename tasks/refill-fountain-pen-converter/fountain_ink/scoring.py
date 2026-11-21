"""Conservative pump-volume transfer and a physical held-outcome rubric."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.ink_ml = 0.0
        self.dwell = 0.0
        self.stroke = 0.0
        self.wet_cycle = True
        self.loaded_cycles = 0
        self.dry_cycles = 0
        self.maximum_travel = 0.0
        self.nib_contact = False

    def evaluate(self, state, elapsed):
        pen = state["objects"]["pen"]
        bottle = state["objects"]["bottle"]
        center = np.asarray(pen["pose"][:3])
        neck_up = quat_xyzw_to_matrix(pen["pose"][3:])[:, 2]
        bottle_pos = np.asarray(bottle["pose"][:3])
        bottle_up = quat_xyzw_to_matrix(bottle["pose"][3:])[:, 2]
        feed = center + C.FEED[2] * neck_up
        ink_plane = (
            bottle_pos[2]
            + C.INK_Z
            - C.TABLE_Z
            - self.ink_ml * 1e-6 / (np.pi * C.WELL_R**2)
        )
        depth = ink_plane - feed[2]
        near_axis = np.linalg.norm(feed[:2] - bottle_pos[:2]) < C.WELL_R - 0.003
        base_clearance = center[2] - (bottle_pos[2] + C.BOTTOM_Z - C.TABLE_Z + 0.004)
        safe_immersion = bool(
            near_axis
            and depth >= 0.0015
            and base_clearance >= 0.003
            and neck_up[2] >= 0.955
            and bottle_up[2] >= 0.985
        )
        self.nib_contact |= bool(near_axis and base_clearance < 0.0015)
        travel = float(np.clip(-pen["joints"]["Stroke"], 0, C.STROKE))
        self.maximum_travel = max(self.maximum_travel, travel)
        if travel > 0.0009:
            self.stroke = max(self.stroke, travel)
            self.wet_cycle &= safe_immersion
        elif self.stroke:
            if self.stroke >= 0.0047:
                if self.wet_cycle and safe_immersion:
                    # Each complete passage through the measured pump stroke
                    # moves the swept dose from the bottle into the converter.
                    displacement = 0.191 * self.stroke / C.STROKE
                    self.ink_ml = min(C.CAPACITY_ML, self.ink_ml + displacement)
                    self.loaded_cycles += 1
                else:
                    self.dry_cycles += 1
            self.stroke = 0.0
            self.wet_cycle = True

        on_rest = bool(
            np.linalg.norm(center[:2] - C.REST[:2]) <= 0.009
            and abs(center[2] - C.REST_NIB_Z) <= 0.008
            and neck_up[2] >= 0.97
            and np.linalg.norm(pen["velocity"][:3]) <= 0.030
        )
        grip = center + C.GRIP_Z * neck_up
        clear = all(
            min(np.linalg.norm(np.asarray(p) - grip) for p in hand["points"][:2])
            >= 0.044
            for hand in state["hands"].values()
        )
        upright_bottle = bool(
            bottle_up[2] >= 0.985
            and np.linalg.norm(bottle_pos[:2] - C.BOTTLE[:2]) < 0.020
        )
        passed = (
            self.ink_ml >= 0.9
            and on_rest
            and clear
            and upright_bottle
            and not self.nib_contact
        )
        self.dwell = self.dwell + elapsed if passed else 0.0
        if self.nib_contact or not upright_bottle:
            score = 0.0
        elif passed and self.dwell >= 0.5 - 1e-8:
            score = 1.0
        elif self.ink_ml >= 0.9:
            score = 0.8
        elif self.ink_ml >= 0.50:
            score = 0.55
        elif self.ink_ml >= 0.12:
            score = 0.25
        else:
            score = 0.0
        return {
            "score": score,
            "success": score == 1.0,
            "ink_ml": self.ink_ml,
            "bottle_ink_ml": C.BOTTLE_ML - self.ink_ml,
            "button_travel_mm": 1000 * travel,
            "maximum_button_travel_mm": 1000 * self.maximum_travel,
            "nib_feed_depth_mm": 1000 * depth,
            "nib_floor_clearance_mm": 1000 * base_clearance,
            "wet_strokes": self.loaded_cycles,
            "dry_strokes": self.dry_cycles,
            "nib_contacted_floor": self.nib_contact,
            "upright_bottle": upright_bottle,
            "on_rest": on_rest,
            "hands_clear": clear,
            "stable_s": self.dwell,
        }
