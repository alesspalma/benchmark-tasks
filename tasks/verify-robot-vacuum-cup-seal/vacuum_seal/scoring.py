"""Score isolated pressure retention, supported release and plate return."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def supports_plate(forces):
    """Forces on the plate from the palm and the two finger bodies, in world N."""
    _, first, second = forces
    first_load = np.linalg.norm(first)
    second_load = np.linalg.norm(second)
    opposed = (
        first_load >= C.MIN_PINCH_FORCE_N
        and second_load >= C.MIN_PINCH_FORCE_N
        and first @ second <= -0.55 * first_load * second_load
    )
    return bool(opposed or forces[:, 2].sum() >= C.MIN_UPWARD_SUPPORT_N)


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
        self.releasing = False

    def observe_vent(self, *, vent, elevated, previous_pressure, supported):
        """Witness the opening and supported pressure release at native physics rate."""
        self.releasing = vent and elevated and (self.releasing or previous_pressure > 4.0)
        if self.releasing:
            self.unsupported_vent |= not supported
            self.controlled_vent |= self.proved and supported

    def observe_hold(self, circuit, *, elevated, hand_touch, dt):
        """Track uninterrupted, hands-off isolation at native physics rate."""
        if self.proved:
            return
        if (
            elevated and circuit.sealed and circuit.pressure >= C.HOLD_PRESSURE_KPA
            and not hand_touch and not circuit.supply and not circuit.vent
        ):
            self.hold += dt
            self.min_pressure = min(self.min_pressure or circuit.pressure, circuit.pressure)
        else:
            self.hold = 0.0
            self.min_pressure = None

    def evaluate(self, state, circuit, elapsed):
        plate = state["objects"]["plate"]
        pose = np.asarray(plate["pose"])
        position = pose[:3]
        rotation = quat_xyzw_to_matrix(pose[3:])
        contacts = state["plate_contacts"]
        can_support = contacts["supported"]
        clear = not contacts["hand_touch"]
        for hand in state["hands"].values():
            points = np.asarray(hand["points"])
            relative = (points - position) @ rotation
            near_object = np.any(
                (np.abs(relative[:, 0]) <= 0.074)
                & (np.abs(relative[:, 1]) <= 0.063)
                & (np.abs(relative[:, 2]) <= 0.038)
            )
            near_handle = (
                np.min(np.linalg.norm(relative - C.HANDLE_LOCAL, axis=1)) < 0.027
            )
            clear &= not (near_object or near_handle)
        pressure = circuit.pressure
        elevated = position[2] >= C.PICK[2] + 0.11
        carrying = elevated and circuit.sealed and pressure >= C.HOLD_PRESSURE_KPA
        self.evacuated |= bool(carrying and circuit.supply)
        if not self.proved:
            if not clear:
                self.hold = 0.0
                self.min_pressure = None
            self.proved = self.hold + 1e-9 >= 2.0
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
            "native_rim_contact": bool(circuit.rim_contact),
            "native_rim_load_n": round(circuit.rim_load, 3),
            "bellows_compression_mm": round(1000 * circuit.compression, 3),
            "maximum_lip_gap_mm": round(1000 * circuit.max_gap, 3),
            "plate_elevated": bool(elevated),
            "unsupported_hold_s": round(min(2.0, self.hold), 3),
            "pressure_proof": bool(self.proved),
            "supported_vent": bool(self.controlled_vent),
            "unsupported_vent": bool(self.unsupported_vent),
            "plate_supported_by_hand": bool(can_support),
            "plate_on_rest": bool(returned),
            "grippers_clear": bool(clear),
        }
