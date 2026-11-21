"""Conservative water-blank model and physical zero checks independent of arm assignment."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation

from . import config as C


def pose(state, name):
    values = state["objects"][name]["pose"]
    return np.asarray(values[:3]), Rotation.from_quat(values[3:]).as_matrix()


def daylight_angle(state):
    return C.LID_OPEN_DEG + np.degrees(state["objects"]["instrument"]["joints"]["CoverAngle"])


@dataclass
class WetPrism:
    remaining: int = C.DROPS
    volume_ml: float = 0.
    spilled_ml: float = 0.
    previous_press: float = 0.
    stroke: float = 0.

    def update(self, state):
        press = -state["objects"]["dropper"]["joints"]["BulbTravel"]
        effective = max(0., press - .00385)
        increment = max(0., effective - self.previous_press)
        self.previous_press = effective
        tube, tube_R = pose(state, "instrument")
        open_deg = daylight_angle(state)
        if (tube_R[2, 2] < .89 and open_deg > 12.) or tube[2] < C.TABLE-.03:
            self.spilled_ml += self.volume_ml
            self.volume_ml = 0.
        dropper, dropper_R = pose(state, "dropper")
        if dropper_R[2, 2] < .82:
            return
        self.stroke += increment
        normal = tube_R[:, 2]
        prism = tube + tube_R @ C.PRISM
        tip = dropper + dropper_R @ C.DROPPER_TIP
        height = float(normal @ (tip - prism))
        if normal[2] > .85:
            point = tube_R.T @ (tip - [0., 0., height / normal[2]] - prism)
            lands = (.003 <= height <= .045 and open_deg >= 60.
                     and abs(point[0]) <= (C.PRISM_WIDTH - .004)/2
                     and abs(point[1]) <= (C.PRISM_LENGTH - .004)/2)
        else:
            lands = False
        # Each completed increment ejects one small drop. The measured bulb
        # contraction and geometric gravity trajectory decide its destination.
        # Releasing the bulb cannot create or recover an emitted droplet.
        while self.remaining and self.stroke >= C.DROP_TRAVEL:
            self.stroke -= C.DROP_TRAVEL
            self.remaining -= 1
            if lands:
                self.volume_ml += C.DROP_ML
            else:
                self.spilled_ml += C.DROP_ML


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.consecutive = 0.

    def evaluate(self, state, water, elapsed):
        cover_deg = daylight_angle(state)
        screw_rad = C.SCREW_INIT + state["objects"]["instrument"]["joints"]["ZeroAngle"]
        apparent_brix = float(C.BRIX_PER_RADIAN * screw_rad)
        tube, tube_R = pose(state, "instrument")
        wet = water.volume_ml >= .072 and water.spilled_ml <= .036001
        readable = bool(wet and -2. <= cover_deg <= 7. and tube_R[2, 2] >= .975)
        calibrated = bool(readable and abs(apparent_brix) <= C.BRIX_TOLERANCE)
        down = (np.all(np.abs(tube-C.INSTRUMENT) <= [.011, .015, .008])
                and tube_R[1, 1] >= .981 and tube_R[2, 2] >= .975)
        speed = np.linalg.norm(state["objects"]["instrument"]["velocity"][:3])
        free = min(np.linalg.norm(np.asarray(h["points"][0]) - tube)
                   for h in state["hands"].values()) > .127
        if calibrated and down and speed < .028 and free:
            self.consecutive += elapsed
        else:
            self.consecutive = 0.
        passed = self.consecutive >= .5
        score = 1. if passed else .65 if calibrated else .45 if readable else .25 if wet else 0.
        return {
            "score": score, "success": passed, "water_ml": water.volume_ml,
            "spilled_ml": water.spilled_ml, "drops_in_dropper": water.remaining,
            "cover_degrees": float(cover_deg), "screw_degrees_from_zero": float(np.degrees(screw_rad)),
            "reading_brix": apparent_brix if readable else None, "instrument_rested": bool(down),
            "hands_clear": bool(free), "held_result_s": self.consecutive,
        }
