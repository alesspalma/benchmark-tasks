"""Record the interlocked cycle and whether it heated a closed vent."""

import numpy as np
from scipy.spatial.transform import Rotation

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.active = False
        self.unsafe_start = False
        self.pressed = False
        self.hold = 0.

    def evaluate(self, state, elapsed):
        objects = state["objects"]
        oven = objects["microwave"]["joints"]
        bowl = objects["lunch_bowl"]
        vent_deg = float(np.degrees(bowl["joints"]["SteamVent"]))
        door_deg = float(C.DOOR_OPEN_DEG - np.degrees(oven["DoorHinge"]))
        bowl_pose = np.asarray(bowl["pose"])
        bowl_x, bowl_y, bowl_lower = bowl_pose[:3]
        upright = Rotation.from_quat(bowl_pose[3:]).apply([0., 0., 1.])[2] >= np.cos(np.radians(8.))
        on_tray = bool(
            upright and np.linalg.norm(bowl_pose[:2] - C.TRAY_CENTER[:2]) <= .052
            and abs(bowl_lower - C.TRAY_CENTER[2]) <= .008
        )
        vent_clearance = C.VENT_PORT_NOSE * np.sin(np.radians(max(0., vent_deg)))
        vented = vent_clearance >= C.VENT_CLEARANCE_M
        closed = abs(door_deg) <= C.DOOR_CLOSED_DEG
        pressed = oven["StartSwitch"] >= C.START_SWITCH
        if not closed:
            self.active = False
        if pressed and not self.pressed and closed and on_tray:
            self.active = True
        self.pressed = pressed
        self.unsafe_start |= self.active and on_tray and not vented
        hands_clear = all(
            point[0] < .366 or abs(point[1]) > .265
            for hand in state["hands"].values() for point in hand["points"]
        )
        ready = vented and on_tray and closed
        completed = ready and self.active and hands_clear and not pressed and not self.unsafe_start
        self.hold = self.hold + elapsed if completed else 0.
        score = 1. if self.hold >= .999 else .5 if ready and not self.unsafe_start else 0.
        return dict(
            score=score, success=score == 1., vent_deg=vent_deg, door_deg=door_deg,
            steam_gap_mm=float(1000. * vent_clearance),
            vent_open=vented, door_closed=closed, bowl_on_turntable=on_tray,
            start_mm=float(1000. * oven["StartSwitch"]), cycle_started=self.active,
            unsafe_start=self.unsafe_start, hands_clear=hands_clear, hold_s=self.hold,
            bowl_mm=[float(1000. * v) for v in (bowl_x, bowl_y, bowl_lower)],
        )
