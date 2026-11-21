"""Full score requires fitted slack, a linked flush, and a reseated valve."""

import numpy as np

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.stable = 0.0

    def evaluate(self, state, elapsed):
        material = state["material"]
        joints = state["objects"]["tank"]["joints"]
        clip_on = (
            material["engagement"] == C.CORRECT and material["clip_seated_error_mm"] < 9
        )
        returned = abs(joints["TripPivot"]) <= 0.12 and joints["FlapperPivot"] < 0.085
        slack = 1000 * C.CHAIN_LENGTH - material["chain_span_mm"]
        free_slack = bool(
            clip_on and returned and 6 <= slack <= 19 and material["tension_n"] < 0.25
        )
        points = [p for value in state["hands"].values() for p in value["points"]]
        clear = all(
            np.linalg.norm(np.asarray(p) - center) >= 0.050
            for p in points
            for center in (C.LEVER + C.HANDLE, C.LEVER + C.PINS[C.CORRECT])
        )
        verified_flow = (
            material["linked_discharge_l"] >= 3.7 and material["handle_witness"]
        )
        full = free_slack and verified_flow and clear and material["flow_l_s"] <= 0.026
        self.stable = self.stable + elapsed if full else 0.0
        if self.stable >= 0.6:
            score = 1.0
        elif verified_flow and clip_on:
            score = 0.80
        elif free_slack:
            score = 0.50
        elif clip_on:
            score = 0.25
        else:
            score = 0.0
        return {
            "success": score == 1.0,
            "score": score,
            "chain_connected": bool(clip_on),
            "slack_mm": round(slack, 2),
            "flapper_angle_deg": round(float(np.rad2deg(joints["FlapperPivot"])), 3),
            "trip_angle_deg": round(float(np.rad2deg(joints["TripPivot"])), 3),
            "reseated": bool(returned),
            "grippers_clear": clear,
            "stable_s": round(self.stable, 3),
            **{
                key: round(value, 4)
                if isinstance(value, (float, np.floating))
                else value
                for key, value in material.items()
            },
        }
