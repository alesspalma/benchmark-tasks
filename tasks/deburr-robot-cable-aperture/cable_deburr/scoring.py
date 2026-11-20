"""Measure completed rim length and the returned parts."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def point_in_plate(plate, point):
    pose = np.asarray(plate["pose"])
    return quat_xyzw_to_matrix(pose[3:]).T @ (point - pose[:3])


class Scorer:
    def __init__(self):
        edge = C.outline()
        self.centres = .5 * (edge + np.roll(edge, -1, axis=0))
        diff = np.roll(edge, -1, axis=0) - edge
        self.length = np.linalg.norm(diff, axis=1)
        self.tangent = diff / self.length[:, None]
        self.reset()

    def reset(self):
        self.cleared = np.zeros(C.CELLS, dtype=bool)
        self.previous = None
        self.gouged = False
        self.since = None

    def _cut(self, plate, tool):
        blade = np.asarray(tool["links"]["Knife"])
        attitude = quat_xyzw_to_matrix(blade[3:])
        p = point_in_plate(plate, blade[:3] + attitude @ C.TOE)
        normal_load = max(0., tool["joints"]["Flex"] * C.FLEX_K)
        tool_up = attitude[:, 2] @ quat_xyzw_to_matrix(np.asarray(plate["pose"][3:]))[:, 2]
        d = np.linalg.norm(self.centres - p[:2], axis=1)
        seated = abs(p[2] - C.PLATE_THICKNESS / 2) < .004 and tool_up > .75
        previous = self.previous
        self.previous = (p, normal_load, seated)
        if previous is None or not seated or not previous[2]:
            return p, normal_load, []
        a, prior_force, _ = previous
        delta = p[:2] - a[:2]
        speed = np.linalg.norm(delta)
        valid_load = C.FORCE_MIN <= min(normal_load, prior_force)
        if valid_load and np.max((normal_load, prior_force)) > C.FORCE_MAX and d.min() < .004:
            self.gouged = True
        if not .00024 <= speed <= .007 or not valid_load or max(normal_load, prior_force) > C.FORCE_MAX:
            return p, normal_load, []
        q = self.centres - a[:2]
        fraction = q @ delta / speed ** 2
        nearest = a[:2] + np.clip(fraction, -.26, 1.26)[:, None] * delta
        swept = (
            (np.linalg.norm(self.centres - nearest, axis=1) < .0031)
            & (self.tangent @ delta > .47 * speed)
        )
        self.cleared |= swept
        return p, normal_load, np.flatnonzero(swept).tolist()

    def evaluate(self, state):
        plate = state["objects"]["plate"]
        tool = state["objects"]["tool"]
        tip, load, cut = self._cut(plate, tool)
        coverage = float(self.length @ self.cleared / self.length.sum())
        pp, tp = np.asarray(plate["pose"]), np.asarray(tool["pose"])
        pp_vertical = quat_xyzw_to_matrix(pp[3:])[2, 2]
        tp_vertical = quat_xyzw_to_matrix(tp[3:])[2, 2]
        plate_back = (np.linalg.norm(pp[:2] - C.PLATE_START[:2]) < .022
                      and abs(pp[2] - C.PLATE_START[2]) < .006
                      and pp_vertical > .97 and np.linalg.norm(plate["velocity"][:3]) < .018)
        tool_back = (np.linalg.norm(tp[:2] - C.TOOL_START[:2]) < .018
                     and abs(tp[2] - C.TOOL_START[2]) < .009
                     and tp_vertical > .93 and np.linalg.norm(tool["velocity"][:3]) < .020)
        clear = all(
            np.linalg.norm(np.asarray(point) - pp[:3]) > .083
            and np.linalg.norm(np.asarray(point) - (tp[:3] + C.TOOL_GRASP)) > .044
            for hand in state["hands"].values() for point in hand["points"]
        )
        done = not self.gouged and coverage >= C.COVERAGE_LIMIT and plate_back and tool_back and clear
        now = state["time_s"]
        self.since = (now if self.since is None else self.since) if done else None
        stable = 0. if self.since is None else now - self.since
        success = stable >= .749
        score = 1.0 if success else (.20 if self.gouged else .8) * coverage
        return dict(score=float(score), success=success, coverage=coverage, cells=int(self.cleared.sum()),
                    gouged=self.gouged, plate_back=bool(plate_back), tool_back=bool(tool_back),
                    clear=clear, dwell_s=stable, blade_load_n=load, blade_in_plate=tip.tolist(), cut=cut)
