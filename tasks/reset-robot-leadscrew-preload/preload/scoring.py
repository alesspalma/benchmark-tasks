"""Quasi-static ramp preload and reversal play, plus a live drive-wheel test."""

import numpy as np

from . import config as C


def spring_preload(angle):
    rise = C.RAMP_M_PER_RAD * angle - C.UNLOADED_GAP_M
    return C.AXIAL_STIFFNESS * max(0., rise)


def reversal_play(angle):
    gap = max(0., C.UNLOADED_GAP_M - C.RAMP_M_PER_RAD * angle)
    separation = max(0., C.AXIAL_PROBE_N - spring_preload(angle))
    return gap + (C.AXIAL_PROBE_N + separation) / C.THREAD_STIFFNESS


def running_drag(angle):
    return .0095 + .00087 * spring_preload(angle)


class Scorer:
    def reset(self):
        self.probe_steps = 0
        self.initial_screw = 0.
        self.relief = False
        self.previous_angles = None

    def evaluate(self, state):
        joints = state["objects"]["drive"]["joints"]
        angle = joints["RampAngle"]
        screw = joints["ScrewAngle"]
        self.relief |= reversal_play(angle) >= .00030
        clear = all(
            all(np.linalg.norm(np.asarray(p)- q) > .060
                for p in h["points"][:2] for q in (C.CASE_GRIP, C.CAM_GRIP, C.AXIS + [-.164, 0., 0.]))
            for h in state["hands"].values()
        )
        play = reversal_play(angle)
        drag = running_drag(angle)
        acceptable_play = play <= .00010
        acceptable_drag = drag <= .0185
        angles = np.array([angle, joints["HousingAngle"]])
        steady = self.previous_angles is not None and max(
            abs(angles - self.previous_angles)
        ) <= np.radians(.8)
        self.previous_angles = angles
        if clear and steady and acceptable_play and acceptable_drag:
            if self.probe_steps == 0:
                self.initial_screw = screw
            self.probe_steps += 1
        else:
            self.probe_steps = 0
        travel = screw - self.initial_screw
        turned = travel >= np.radians(16.)
        success = acceptable_play and acceptable_drag and clear and turned and self.probe_steps >= 24
        score = 1. if success else .5 if acceptable_play and acceptable_drag else .25 if self.relief else 0.
        return dict(
            score=score, success=success, ramp_deg=float(np.degrees(angle)),
            housing_deg=float(np.degrees(joints["HousingAngle"])),
            play_mm=float(play * 1000), drag_nmm=float(drag * 1000),
            wheel_test_deg=float(np.degrees(travel)), clear=bool(clear), probe_steps=self.probe_steps,
        )
