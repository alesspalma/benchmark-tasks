"""Assembly and real chain routing, measured from the current native state."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix
from .config import (
    PITCH,
    TABLE_TOP,
    CHAIN_Z,
    CONNECTOR_TARGET,
    SEAT_POINTS,
    SPROCKET_CENTERS,
)


def rotation(pose):
    return quat_xyzw_to_matrix(np.asarray(pose[3:]))


def chain_nodes(s):
    nodes = []
    for i in range(11):
        p = s["bodies"][f"Link{i:02}"]["pose"]
        r = rotation(p)
        if i == 0:
            nodes.append(np.asarray(p[:3]) + r @ np.array([-PITCH / 2, 0, 0]))
        nodes.append(np.asarray(p[:3]) + r @ np.array([PITCH / 2, 0, 0]))
    return np.array(nodes)


def cross(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def simple_loop(p):
    for i in range(len(p)):
        incoming = p[i] - p[i - 1]
        outgoing = p[(i + 1) % len(p)] - p[i]
        if abs(cross(incoming, outgoing)) < 1e-8 and np.dot(incoming, outgoing) < 0:
            return False
    for i in range(len(p)):
        a, b = p[i], p[(i + 1) % len(p)]
        if np.linalg.norm(a - b) < 0.001:
            return False
        for j in range(i + 1, len(p)):
            if j == i + 1 or (i == 0 and j == len(p) - 1):
                continue
            c, d = p[j], p[(j + 1) % len(p)]
            u, v, w, z = (
                cross(b - a, c - a),
                cross(b - a, d - a),
                cross(d - c, a - c),
                cross(d - c, b - c),
            )
            if u * v < -1e-12 and w * z < -1e-12:
                return False
            if max(abs(u), abs(v), abs(w), abs(z)) < 1e-8:
                axis = int(np.argmax(abs(b - a)))
                overlap = min(max(a[axis], b[axis]), max(c[axis], d[axis])) - max(
                    min(a[axis], b[axis]), min(c[axis], d[axis])
                )
                if overlap > 1e-5:
                    return False
    return True


def routing(s):
    nodes = chain_nodes(s)
    p = nodes[:, :2]
    simple = simple_loop(p)
    windings = []
    for center in SPROCKET_CENTERS:
        v = p - center
        w = np.roll(v, -1, axis=0)
        windings.append(
            float(
                np.arctan2(v[:, 0] * w[:, 1] - v[:, 1] * w[:, 0], (v * w).sum(1)).sum()
                / (2 * np.pi)
            )
        )
    seats = np.linalg.norm(
        SEAT_POINTS[:, :, None, :] - p[None, None, :, :], axis=-1
    ).min(-1)
    height = float(abs(nodes[:, 2] - CHAIN_Z).max())
    normals = all(rotation(v["pose"])[2, 2] > 0.995 for v in s["bodies"].values())
    ok = bool(
        simple
        and all(abs(abs(w) - 1) < 0.01 for w in windings)
        and (seats <= 0.005).all()
        and height <= 0.004
        and normals
    )
    return dict(
        chain_routed=ok,
        loop_simple=bool(simple),
        wheel_windings=windings,
        sprocket_seat_errors_m=seats.tolist(),
        chain_height_error_m=height,
    )


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.violations = set()
        self.physics_samples = 0
        self.hold_start = None

    def observe_physics(self, points):
        self.physics_samples += 1
        for p in points.values():
            if not np.isfinite(p).all():
                self.violations.add("invalid_state")
            elif (
                p[:, 2].min() < TABLE_TOP - 0.006
                or p[:, 0].min() < -0.19
                or p[:, 0].max() > 0.91
                or abs(p[:, 1]).max() > 0.65
            ):
                self.violations.add("part_left_work_area")

    def evaluate(self, s):
        parts = [s["connector"], s["keeper"], *s["bodies"].values()]
        finite = all(
            np.isfinite(p["pose"]).all() and np.isfinite(p["velocity"]).all()
            for p in parts
        )
        finite = (
            finite
            and np.isfinite(s["robot_clearance_m"])
            and all(np.isfinite(h["opening_m"]) for h in s["hands"].values())
        )
        if not finite:
            self.violations.add("invalid_state")
            self.hold_start = None
            return dict(
                score=0.0,
                success=False,
                violations=sorted(self.violations),
                ends_joined=False,
                plate_seated=False,
                retained=False,
                chain_routed=False,
                closed_loop=False,
                physics_samples=self.physics_samples,
            )
        c = s["connector"]["pose"]
        k = s["keeper"]["pose"]
        r = rotation(c)
        kr = rotation(k)
        ends = {}
        passed = []
        for arm, body, sign in [("right", "Link00", -1), ("left", "Link10", 1)]:
            p = s["bodies"][body]["pose"]
            point = np.asarray(p[:3]) + rotation(p) @ np.array([sign * PITCH / 2, 0, 0])
            local = r.T @ (point - c[:3])
            ends[arm] = local.tolist()
            passed.append(
                bool(
                    np.linalg.norm(local[:2] - [0, sign * PITCH / 2]) <= 0.010
                    and -0.022 < local[2] < -0.010
                    and np.dot(rotation(p)[:, 2], r[:, 2]) > 0.99
                )
            )
        joined = all(passed)
        local = r.T @ (np.asarray(k[:3]) - c[:3])
        axis = np.trace(r.T @ kr) >= 1 + 2 * np.cos(0.10)
        pins = [
            kr.T @ (np.asarray(c[:3]) + r @ np.array([0, y, -0.035]) - k[:3])
            for y in (-PITCH / 2, PITCH / 2)
        ]
        in_slots = [
            -0.0215 < p[0] < 0.035 and abs(p[1] - y) < 0.005
            for p, y in zip(pins, (-PITCH / 2, PITCH / 2))
        ]
        in_ends = [
            -0.0215 < p[0] < -0.008 and abs(p[1] - y) < 0.005
            for p, y in zip(pins, (-PITCH / 2, PITCH / 2))
        ]
        covered = bool(joined and -0.040 < local[2] < -0.030 and axis and all(in_slots))
        retained = bool(covered and all(in_ends))
        route = routing(s)
        parked = bool(
            np.linalg.norm(np.asarray(c[:3]) - CONNECTOR_TARGET) < 0.008
            and np.trace(r) >= 1 + 2 * np.cos(0.10)
        )
        settled = all(
            np.linalg.norm(p["velocity"][:3]) < 0.02
            and np.linalg.norm(p["velocity"][3:]) < 0.3
            for p in parts
        )
        clear = bool(
            s["robot_clearance_m"] >= 0.025
            and all(h["opening_m"] >= 0.070 for h in s["hands"].values())
        )
        full = bool(
            retained
            and route["chain_routed"]
            and parked
            and settled
            and clear
            and not self.violations
        )
        if full:
            if self.hold_start is None:
                self.hold_start = s["time_s"]
        else:
            self.hold_start = None
        success = bool(full and s["time_s"] - self.hold_start >= 1.0)
        score = (
            (1.0 if success else 0.7 if covered else 0.4 if joined else 0.0)
            if route["chain_routed"] and not self.violations
            else 0.0
        )
        return dict(
            score=score,
            success=success,
            violations=sorted(self.violations),
            ends_joined=joined,
            plate_seated=covered,
            retained=retained,
            closed_loop=bool(joined and retained and route["chain_routed"]),
            parked=parked,
            settled=bool(settled),
            hands_clear=clear,
            robot_clearance_m=s["robot_clearance_m"],
            end_points_local=ends,
            keeper_local=local.tolist(),
            pins_in_keeper=[p.tolist() for p in pins],
            physics_samples=self.physics_samples,
            **route,
        )
