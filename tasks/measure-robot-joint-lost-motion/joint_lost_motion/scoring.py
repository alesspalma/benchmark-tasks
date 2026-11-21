"""Bidirectional angle readings from a positively retained input shaft."""

import math

import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.samples = {-1: [], 1: []}
        self.readings = {-1: None, 1: None}
        self.release_time = 0.0

    def evaluate(self, state, elapsed):
        joint = state["objects"]["fixture"]["joints"]
        lock_m = -joint["LockStroke"]
        applied_nm = C.LOAD_RADIUS * C.SPRING_N_M * joint["ProofSlide"]
        output_arcmin = math.degrees(joint["InputAngle"] + joint["Torsion"]) * 60.0
        relative_arcmin = math.degrees(joint["Torsion"]) * 60.0
        input_arcmin = abs(math.degrees(joint["InputAngle"]) * 60.0)
        locked = lock_m >= 0.009 and input_arcmin <= 16.0
        side = 1 if applied_nm > 0 else -1
        count = round(C.STABLE_S / C.CHECK_DT)
        for direction, samples in self.samples.items():
            if (
                locked
                and side == direction
                and C.PROOF_MIN <= abs(applied_nm) <= C.PROOF_MAX
                and side * relative_arcmin >= 2.0
            ):
                if elapsed:
                    samples.append(output_arcmin)
                    if len(samples) > count:
                        samples.pop(0)
                    if len(samples) == count and np.ptp(samples) <= 3.0:
                        self.readings[direction] = float(np.median(samples))
            else:
                samples.clear()

        negative, positive = self.readings[-1], self.readings[1]
        complete = negative is not None and positive is not None
        lost_motion = positive - negative if complete else None
        neutral = abs(applied_nm) <= 0.07 and lock_m <= 0.003
        hands_clear = all(
            min(
                np.linalg.norm(np.array(point) - target)
                for point in hand["points"][:2]
                for target in (
                    np.array([C.CENTER_X, C.LOCK_Y, C.LOCK_Z]),
                    np.array([C.CENTER_X, C.CENTER_Y - C.LOAD_RADIUS, C.LOAD_Z]),
                )
            )
            >= 0.055
            for hand in state["hands"].values()
        )
        valid_span = lost_motion is not None and 8.0 <= lost_motion <= 70.0
        if valid_span and neutral and hands_clear:
            self.release_time += elapsed
        else:
            self.release_time = 0.0
        success = self.release_time >= 0.45 - 1e-8
        present = int(negative is not None) + int(positive is not None)
        score = (
            1.0 if success else (0.40 if present == 1 else 0.80 if valid_span else 0.0)
        )
        return {
            "score": score,
            "success": success,
            "input_locked": locked,
            "lock_stroke_mm": 1000.0 * lock_m,
            "load_handle_mm": 1000.0 * joint["ProofSlide"],
            "proof_torque_nm": applied_nm,
            "input_angle_arcmin": input_arcmin,
            "output_angle_arcmin": output_arcmin,
            "relative_angle_arcmin": relative_arcmin,
            "negative_arcmin": negative,
            "positive_arcmin": positive,
            "lost_motion_arcmin": lost_motion,
            "release_dwell_s": self.release_time,
            "hands_clear": hands_clear,
        }
