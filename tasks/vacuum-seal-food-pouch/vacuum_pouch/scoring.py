"""Evaluate the whole-mouth interface, the vacuum cycle, and the removed pouch."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def world_point(pose, local):
    return np.asarray(pose[:3]) + quat_xyzw_to_matrix(pose[3:]) @ local


class Scorer:
    def reset(self):
        self.progress = 0.
        self.sealed = False
        self.started = False
        self.previous_press = False
        self.success_dwell = 0.

    def evaluate(self, state, elapsed):
        pouch = state["objects"]["pouch"]
        sealer = state["objects"]["sealer"]
        machine = sealer["joints"]
        mouth = []
        press_line = []
        neck_length = C.MOUTH_TIP - C.MOUTH_PANEL_START
        middle_width = C.MOUTH_Y - C.WING
        for link, ys in {"Neck": (-middle_width, 0., middle_width),
                         "Left": (C.WING,), "Right": (-C.WING,)}.items():
            for y in ys:
                pose = pouch["links"][link]
                near = world_point(pose, np.array([0., y, 0.]))
                tip = world_point(pose, np.array([neck_length, y, 0.]))
                mouth.append(tip)
                if near[0] <= C.HEATER_X <= tip[0]:
                    alpha = (C.HEATER_X - near[0]) / (tip[0] - near[0])
                    press_line.append(near + alpha*(tip-near))
        mouth = np.asarray(mouth)
        low, high = C.CHANNEL_X_RANGE
        fed = bool(
            np.all((low <= mouth[:, 0]) & (mouth[:, 0] <= high))
            and np.max(np.abs(mouth[:, 1])) <= C.CHANNEL_HALF_WIDTH
            and np.all((.778 <= mouth[:, 2]) & (mouth[:, 2] <= .805))
            and np.ptp(mouth[:, 2]) <= .009
        )
        lid_deg = np.degrees(machine["LidPivot"])
        lock_deg = np.degrees(machine["KeeperPivot"])
        lid = sealer["links"]["Lid"]
        back = world_point(lid, np.array([-.1435, 0., -.008]))
        front = world_point(lid, np.array([-.1525, 0., -.008]))
        lip_height = np.interp(C.HEATER_X, [front[0], back[0]], [front[2], back[2]])
        centre = np.asarray(press_line)[:, 2] if press_line else np.array([])
        calibrated_contact = (
            len(centre) == len(mouth)
            and np.ptp(centre) <= .0035
            and np.all(centre >= C.HEATER_TOP-.0015)
            and np.all(centre <= lip_height+.0015)
            and -.0005 < lip_height-C.HEATER_TOP <= .008
            and len(state["lid_film_force_n"]) == 3
            and min(state["lid_film_force_n"]) > .08
        )
        compressed = -3. <= lid_deg <= 3.5 and lock_deg >= C.KEEPER_MIN and calibrated_contact
        pressed = machine["CycleTravel"] <= -.0022
        if pressed and not self.previous_press and fed and compressed:
            self.started = True
        self.previous_press = pressed
        if self.started and not self.sealed:
            if fed and compressed:
                self.progress += elapsed
                if self.progress >= C.CYCLE_SECONDS:
                    self.sealed = True
            else:
                self.progress = 0.
                self.started = False

        pose = pouch["pose"]
        vertical = quat_xyzw_to_matrix(pose[3:])[2, 2]
        apart = float(mouth[:, 0].max()) < C.COUNTER_X
        resting = (
            abs(pose[2] - C.POUCH_START[2]) <= .010
            and vertical >= .968
            and np.linalg.norm(pouch["velocity"][:3]) < .025
        )
        hand_clear = all(
            np.min([np.linalg.norm(np.asarray(pt) - pose[:3]) for pt in hand["points"]]) > .145
            for hand in state["hands"].values()
        )
        if self.sealed and apart and resting and hand_clear:
            self.success_dwell += elapsed
        else:
            self.success_dwell = 0.
        success = self.success_dwell >= 1.
        score = 1. if success else (.7 if self.sealed else (.4 if fed and compressed else (.2 if fed else 0.)))
        return {"success": success, "score": score, "fed": fed, "compressed": bool(compressed),
                "button_pressed": bool(pressed), "sealed": self.sealed,
                "cycle_s": self.progress, "lid_deg": float(lid_deg), "keeper_deg": float(lock_deg),
                "mouth_xyz": mouth.tolist(), "pressure_line_z": centre.tolist(),
                "upper_bar_z": float(lip_height), "clamp_force_n": state["lid_film_force_n"], "apart": apart, "resting": bool(resting),
                "hands_clear": bool(hand_clear), "dwell_s": self.success_dwell}
