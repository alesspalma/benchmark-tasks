"""Measure the actual native crossbar angles, cable location and pull."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def tie_point(state):
    pose = np.asarray(state["objects"]["harness"]["links"]["Tail"])
    return pose[:3] + quat_xyzw_to_matrix(pose[3:]) @ [0.0, C.TAIL_LOCAL_Y, 0.0]


def cable_sections(state):
    """Native cable centers sampled at both crossbars, under positive guidance."""
    poses = state["objects"]["harness"]["links"]
    pose = np.asarray(poses["Distal"])
    rotation = quat_xyzw_to_matrix(pose[3:])
    axis = rotation[:, 1]
    points = []
    for y in C.GATE_Y:
        centers = []
        for offset in (-0.013, 0.013):
            origin = pose[:3] + rotation[:, 0] * offset
            t = (y - origin[1]) / axis[1] if abs(axis[1]) > 0.20 else None
            centers.append(
                origin + t * axis
                if t is not None and -C.SECOND_SPAN <= t <= 0
                else None
            )
        points.append(centers)
    return points


def fits(point):
    return bool(
        point is not None
        and abs(point[0] - C.CHAIN_X) <= C.INNER_HALF_WIDTH - C.CABLE_RADIUS
        and C.CHAIN_FLOOR + C.CABLE_RADIUS - 0.003
        <= point[2]
        <= C.BAR_Z - 0.0045 - C.CABLE_RADIUS + 0.0015
    )


class Scorer:
    def reset(self):
        self.steady = 0.0
        self.pulled = False
        self.contained_time = 0.0
        self.low_tail = None
        self.peak_pull = 0.0

    def evaluate(self, state, elapsed):
        fixture = state["objects"]["carrier"]["joints"]
        angles = [abs(float(np.rad2deg(fixture[f"Latch{i}"]))) for i in range(2)]
        closed = [angle <= 5 for angle in angles]
        sections = cable_sections(state)
        housed = [all(fits(pt) for pt in segment) for segment in sections]
        tail = tie_point(state)
        tcp = [np.asarray(hand["points"][0]) for hand in state["hands"].values()]
        clear = all(
            not (
                0.328 <= hand[0] <= 0.487
                and -0.213 <= hand[1] <= 0.070
                and 0.765 <= hand[2] <= 0.955
            )
            for hand in tcp
        )
        valid = all(closed) and all(housed)
        if valid:
            self.contained_time += elapsed
            grasped = any(
                np.linalg.norm(np.asarray(hand["points"][0]) - tail) <= 0.021
                and hand["opening_m"] < 0.048
                for hand in state["hands"].values()
            )
            if grasped and self.contained_time >= 0.30:
                if self.low_tail is None:
                    self.low_tail = float(tail[2])
                lift = float(tail[2]) - self.low_tail
                self.peak_pull = max(self.peak_pull, lift)
                self.pulled |= lift >= 0.016
            else:
                self.low_tail = None
        else:
            self.low_tail = None
            self.contained_time = 0.0
            self.pulled = False
        self.steady = self.steady + elapsed if valid and self.pulled and clear else 0.0
        score = 0.0
        if all(housed):
            score = 0.30
        if any(c and h for c, h in zip(closed, housed)):
            score = 0.55
        if valid:
            score = 0.80
        if self.steady + 1e-8 >= 0.60:
            score = 1.0
        return {
            "score": score,
            "success": score == 1.0,
            "crossbar_degrees": angles,
            "crossbar_closed": closed,
            "cables_below_gate": housed,
            "cable_offsets_mm": [
                [
                    (1000 * (pt - [C.CHAIN_X, C.GATE_Y[i], C.BAR_Z])).tolist()
                    if pt is not None
                    else None
                    for pt in section
                ]
                for i, section in enumerate(sections)
            ],
            "tail_mm": (1000 * tail).tolist(),
            "proof_lift_mm": 1000 * self.peak_pull,
            "proof_completed": self.pulled,
            "hands_clear": clear,
            "stable_s": self.steady,
        }
