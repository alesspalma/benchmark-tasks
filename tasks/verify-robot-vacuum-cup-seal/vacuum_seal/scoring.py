"""Score isolated pressure retention, supported release and plate return."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.hold = 0.0
        self.proved = False
        self.evacuated = False
        self.controlled_vent = False
        self.parked = 0.0
        self.min_pressure = None
        self.unsupported_vent = False

    def evaluate(self, state, circuit, elapsed):
        plate = state["objects"]["plate"]
        pose = np.asarray(plate["pose"])
        position = pose[:3]
        rotation = quat_xyzw_to_matrix(pose[3:])
        can_support = False
        clear = True
        for hand in state["hands"].values():
            points = np.asarray(hand["points"])
            relative = (points - position) @ rotation
            near_object = np.any(
                (np.abs(relative[:, 0]) <= 0.074)
                & (np.abs(relative[:, 1]) <= 0.063)
                & (np.abs(relative[:, 2]) <= 0.038)
            )
            # A jaw grasp at the handle or a supported lower plate face both
            # permit controlled venting, regardless of which arm does it.
            near_handle = (
                np.min(np.linalg.norm(relative - C.HANDLE_LOCAL, axis=1)) < 0.027
            )
            clear &= not (near_object or near_handle)
            under_face = np.any(
                (np.abs(relative[:, 0]) <= 0.056)
                & (np.abs(relative[:, 1]) <= 0.048)
                & (relative[:, 2] > -0.019)
                & (relative[:, 2] < 0.001)
            )
            can_support |= bool(
                (near_handle and hand["opening_m"] < 0.040) or under_face
            )
        pressure = circuit.pressure
        elevated = position[2] >= C.PICK[2] + 0.11
        carrying = elevated and circuit.sealed and pressure >= C.HOLD_PRESSURE_KPA
        self.evacuated |= bool(carrying and circuit.supply)
        if not self.proved:
            if carrying and clear and not circuit.supply and not circuit.vent:
                self.hold += elapsed
                self.min_pressure = (
                    pressure
                    if self.min_pressure is None
                    else min(self.min_pressure, pressure)
                )
            else:
                self.hold = 0.0
                self.min_pressure = None
            self.proved = self.hold + 1e-9 >= 2.0
        if self.proved and circuit.vent and elevated and pressure > 4.0:
            self.controlled_vent |= can_support
            self.unsupported_vent |= not can_support
        offset = position - C.PICK
        returned = (
            abs(offset[0]) <= 0.016
            and abs(offset[1]) <= 0.025
            and abs(offset[2]) <= 0.005
            and rotation[2, 2] >= 0.985
            and np.linalg.norm(plate["velocity"][:3]) < 0.015
        )
        complete = (
            self.proved
            and self.controlled_vent
            and not self.unsupported_vent
            and returned
            and clear
            and not circuit.supply
            and pressure < 3.0
        )
        self.parked = self.parked + elapsed if complete else 0.0
        success = self.parked + 1e-9 >= 0.50
        return {
            "success": bool(success),
            "score": 1.0
            if success
            else 0.75
            if self.proved
            else 0.35
            if self.evacuated
            else 0.0,
            "vacuum_kpa": round(pressure, 3),
            "minimum_proof_kpa": None
            if self.min_pressure is None
            else round(self.min_pressure, 3),
            "supply_open": bool(circuit.supply),
            "vent_pressed": bool(circuit.vent),
            "lip_sealed": circuit.sealed,
            "bellows_compression_mm": round(1000 * circuit.compression, 3),
            "maximum_lip_gap_mm": round(1000 * circuit.max_gap, 3),
            "plate_elevated": bool(elevated),
            "unsupported_hold_s": round(min(2.0, self.hold), 3),
            "pressure_proof": bool(self.proved),
            "supported_vent": bool(self.controlled_vent),
            "unsupported_vent": bool(self.unsupported_vent),
            "plate_on_rest": bool(returned),
            "grippers_clear": bool(clear),
        }
