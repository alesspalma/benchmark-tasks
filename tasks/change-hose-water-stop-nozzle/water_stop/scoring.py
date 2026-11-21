"""Observe the actual valve, hydraulic interval and hands-off installation."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def tray_impact(position, frame, outlet_y, spread):
    outlet = position + frame @ [0.0, outlet_y, -0.023]
    direction = frame @ [0.0, 0.0, -1.0]
    distance = (C.TABLE + 0.0115 - outlet[2]) / min(direction[2], -0.01)
    impact = outlet + distance * direction
    if (
        direction[2] < -0.65
        and distance > 0.0
        and abs(impact[0] - C.CATCH[0]) < C.CATCH[2] / 2 - spread
        and abs(impact[1] - C.CATCH[1]) < C.CATCH[3] / 2 - 0.021
    ):
        return impact[:2].tolist()
    return None


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.shut_time = 0.0
        self.sleeve_opened = False
        self.shut_verified = False
        self.fan_time = 0.0
        self.fan_volume = 0.0

    def evaluate(self, world, elapsed):
        objects = world["objects"]
        coupling = objects["coupling"]
        joints = coupling["joints"]
        socket = np.asarray(coupling["links"]["Socket"])[:3]
        gray_pose = np.asarray(objects["gray"]["pose"])
        blue_pose = np.asarray(objects["blue"]["pose"])
        gray, blue = gray_pose[:3], blue_pose[:3]
        gray_frame = quat_xyzw_to_matrix(gray_pose[3:])
        blue_frame = quat_xyzw_to_matrix(blue_pose[3:])
        collar = joints["ReleaseTravel"]
        valve = max(0.0, joints["ValveTravel"])
        dogs = max(abs(joints["FrontDogTravel"]), abs(joints["RearDogTravel"]))

        # Reduced hydraulic circuit at a constant feed pressure. The open
        # annular area is proportional to axial poppet lift at small lifts.
        lift_ratio = max(0.0, min(1.0, (valve - 0.0010) / 0.0045))
        flow = C.FLOW_ML_S * lift_ratio

        def alignment(position, frame):
            groove = position + frame @ [0.0, 0.063, 0.0]
            shift = groove - socket - [0.0, 0.0068, 0.0]
            return np.linalg.norm(shift[[0, 2]]) < 0.003 and abs(shift[1]) < 0.004

        seated_gray = alignment(gray, gray_frame)
        seated_blue = alignment(blue, blue_frame)
        valve_closed = valve < 0.0007
        self.sleeve_opened |= collar >= 0.007
        if self.sleeve_opened and not seated_gray and not seated_blue and valve_closed:
            self.shut_time += elapsed
            self.shut_verified |= self.shut_time >= 0.60 - 1e-8
        elif not self.shut_verified:
            self.shut_time = 0.0

        gray_speed = np.linalg.norm(objects["gray"]["velocity"][:3])
        parked = (
            abs(gray[0] - C.PARK[0]) <= 0.032
            and abs(gray[1] - C.PARK[1]) <= 0.040
            and abs(gray[2] - C.PARK[2]) <= 0.018
            and gray_speed < 0.025
        )
        locked = seated_blue and collar < 0.002 and dogs < 0.0014 and valve > 0.0040
        points = [p for h in world["hands"].values() for p in h["points"]]
        clear = all(
            np.linalg.norm(np.asarray(p) - center) > radius
            for p in points
            for center, radius in ((gray, 0.051), (blue, 0.051), (socket, 0.045))
        )
        blue_speed = np.linalg.norm(objects["blue"]["velocity"][:3])
        gray_flowing = seated_gray and flow >= 5.0
        blue_flowing = seated_blue and flow >= 5.0
        blue_impact = tray_impact(blue, blue_frame, -0.078, 0.033)
        in_tray = blue_impact is not None
        impact = (
            blue_impact
            if blue_flowing
            else tray_impact(gray, gray_frame, -0.074, 0.025)
            if gray_flowing
            else None
        )
        if (
            self.shut_verified
            and parked
            and locked
            and in_tray
            and clear
            and blue_speed < 0.025
        ):
            self.fan_time += elapsed
            self.fan_volume += flow * elapsed
        else:
            self.fan_time = 0.0
            self.fan_volume = 0.0

        success = self.fan_time >= 2.0 - 1e-8 and self.fan_volume >= 100.0
        score = (
            1.0
            if success
            else 0.60
            if self.shut_verified and parked
            else 0.30
            if self.shut_verified
            else 0.0
        )
        return {
            "score": score,
            "success": success,
            "valve_lift_m": valve,
            "collar_retraction_m": collar,
            "dog_travel_m": dogs,
            "supply_flow_ml_s": flow,
            "gray_flowing": gray_flowing,
            "blue_flowing": blue_flowing,
            "tray_impact_xy": impact,
            "exchange_shutoff_s": self.shut_time,
            "exchange_shutoff_verified": self.shut_verified,
            "old_head_parked": bool(parked),
            "replacement_locked": bool(locked),
            "catch_aligned": bool(in_tray),
            "hands_clear": bool(clear),
            "hands_off_flow_s": self.fan_time,
            "catch_volume_ml": self.fan_volume,
        }
