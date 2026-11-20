"""Count real piston displacement only through a straight, sufficiently seated needle."""

import math

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.pressure = C.START_BAR
        self.previous = None
        self.delivered_ml = 0.
        self.held = 0
        self.maximum_pressure = self.pressure

    def evaluate(self, state, elapsed):
        objects = state["objects"]
        ball = np.asarray(objects["ball"]["pose"])
        needle = np.asarray(objects["needle"]["pose"])
        pump = np.asarray(objects["pump"]["pose"])
        direction = quat_xyzw_to_matrix(ball[3:])[:, 2]
        needle_direction = quat_xyzw_to_matrix(needle[3:])[:, 2]
        port = ball[:3] + C.BALL_R * direction
        tip = needle[:3] - C.NEEDLE_TIP * needle_direction
        depth = float((port - tip) @ direction)
        radial = np.linalg.norm((tip - port) + depth * direction)
        collar_distance = float((needle[:3] - port) @ direction)
        angle = math.degrees(math.acos(np.clip(needle_direction @ direction, -1., 1.)))
        tube_start = pump[:3] + quat_xyzw_to_matrix(pump[3:]) @ C.HOSE_ORIGIN
        hose_end = needle[:3] + .027 * needle_direction
        hose_ok = np.linalg.norm(tube_start - hose_end) < C.HOSE_LENGTH - .012
        sealed = .012 <= depth <= .028 and radial <= .0024 and angle <= 8.0 and collar_distance >= -.001 and hose_ok
        stroke = objects["pump"]["joints"]["Stroke"]

        if self.previous is not None and self.previous[1] and sealed:
            travel = abs(stroke - self.previous[0])
            if travel > .0005:
                bore_area = np.pi * C.PUMP_BORE**2 / 4
                # Seal bypass and the rod side of a dual action chamber reduce
                # net free-air volume. The reservoir already has its full shape.
                delivered = .80 * bore_area * travel
                self.pressure += 1.01325 * delivered / C.BALL_VOL
                self.delivered_ml += delivered * 1e6
        self.previous = stroke, sealed
        # A poorly aligned insertion opens the valve without establishing the
        # seal. Merely resting the needle on the outside leaves the valve shut.
        if .004 < depth < .028 and radial < .006 and not sealed:
            self.pressure = max(.05, self.pressure - .012 * elapsed)
        self.maximum_pressure = max(self.maximum_pressure, self.pressure)

        extracted = np.linalg.norm(tip - port) >= .035 and not sealed
        ball_parked = (np.linalg.norm(ball[:2] - C.BALL[:2]) <= C.RING_INNER * .48
                       and abs(ball[2] - C.BALL[2]) < .022)
        pump_supported = (C.TABLE + .018 <= pump[2] <= C.TABLE + .080 and
                          abs(quat_xyzw_to_matrix(pump[3:])[2, 2]) < .15 and
                          np.linalg.norm(objects["pump"]["velocity"][:3]) < .025)
        clear = all(all(np.linalg.norm(np.asarray(point) - center) > distance
                        for point in hand["points"][:2]
                        for center, distance in ((needle[:3], .044), (pump[:3]+C.PUMP_ROT @ C.BARREL_GRIP, .055), (ball[:3], .122)))
                    for hand in state["hands"].values())
        band = C.LOW_BAR <= self.pressure <= C.HIGH_BAR
        safe_pressure = self.maximum_pressure <= .85
        complete = band and extracted and pump_supported and clear and ball_parked and safe_pressure
        self.held = self.held + 1 if complete else 0
        success = self.held >= 20
        partial = self.pressure >= C.START_BAR + .014 and safe_pressure
        score = 1. if success else .7 if band and safe_pressure else .35 if partial else 0.
        return {
            "score": score, "success": success, "bar": self.pressure,
            "gauge_bar": self.pressure if sealed else .70,
            "delivered_ml": self.delivered_ml, "sealed": bool(sealed),
            "needle_angle_deg": angle, "needle_depth_mm": depth*1000.,
            "needle_radial_mm": float(radial*1000.), "stroke_mm": stroke*1000.,
            "needle_out": bool(extracted), "pump_supported": bool(pump_supported),
            "ball_supported": bool(ball_parked), "hands_clear": bool(clear),
            "held_steps": self.held,
        }
