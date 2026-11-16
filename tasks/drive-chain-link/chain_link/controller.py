"""Fixed-layout reference using elapsed time and ordinary joint observations."""

import hashlib
import json

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik, station
from yam_arena.constants import READY

from . import plan


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        self.mode = mode
        low = np.r_[ik.LOWER, 0.0, ik.LOWER, 0.0]
        high = np.r_[ik.UPPER, 1.0, ik.UPPER, 1.0]
        self.info = PolicyInfo(
            name="chain_link_scripted", action_space=action_box(low, high)
        )
        self.reset(None)

    def bind(self, info):
        if float(info.control_hz) != 20:
            raise ValueError("Reference requires the declared 20 Hz control rate")

    def reset(self, scene):
        self.moves = plan.waypoints(self.mode)
        self.program_sha256 = hashlib.sha256(
            json.dumps(self.moves, sort_keys=True, separators=(",", ":")).encode()
        ).hexdigest()
        self.q = {n: np.asarray(READY).copy() for n in ("left", "right")}
        self.starts = {
            n: (*ik.tcp_pose(self.q[n], station.BASES[n]), 1.0) for n in self.q
        }
        self.previous = np.r_[READY, 1.0, READY, 1.0]
        self.integral = np.zeros(14)
        self.duration = (
            max((sum(w["duration"] for w in v) for v in self.moves.values()), default=0)
            + 2
        )
        self.phase = "ready"
        self.program_time = 0.0
        self.last_time = 0.0
        self.precision_wait_s = 0.0
        self.nominal = self.previous.copy()
        self.boundaries = np.cumsum([m["duration"] for m in self.moves.get("left", [])])
        self.precision_tracking = {}
        self.precision_tolerances = {}
        for name, indices in (("left", range(6)), ("right", range(7, 13))):
            for b, move in zip(self.boundaries, self.moves.get(name, [])):
                if move.get("settle"):
                    self.precision_tracking.setdefault(float(b), []).extend(indices)
                    self.precision_tolerances[float(b)] = min(
                        self.precision_tolerances.get(float(b), 0.002),
                        move.get("tracking_tolerance", 0.002),
                    )
        self.precision_boundaries = np.array(sorted(self.precision_tracking))
        self.nominal_duration = self.duration
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def action_at(self, t, joint_pos=None):
        if joint_pos is not None:
            dt = max(0.0, float(t) - self.last_time)
            boundaries = self.precision_boundaries[
                abs(self.precision_boundaries - self.program_time) < 1e-8
            ]
            at_boundary = bool(len(boundaries))
            tracking = (
                np.max(
                    abs(self.nominal - np.asarray(joint_pos))[
                        self.precision_tracking[float(boundaries[0])]
                    ]
                )
                if at_boundary
                else 0.0
            )
            holding_precision = (
                at_boundary
                and tracking >= self.precision_tolerances[float(boundaries[0])]
            )
            self.precision_wait_s = (
                self.precision_wait_s + dt if holding_precision else 0.0
            )
            # Use a bounded ordinary settling pause. Native joint friction
            # leaves small encoder oscillations; physical assembly conditions
            # are enforced independently by the scorer.
            if holding_precision and self.precision_wait_s >= 3.0 - 1e-9:
                holding_precision = False
                self.precision_wait_s = 0.0
            if not holding_precision:
                future = self.boundaries[self.boundaries > self.program_time + 1e-8]
                self.program_time = (
                    min(self.program_time + dt, future[0])
                    if len(future)
                    else self.program_time + dt
                )
            self.last_time = float(t)
            self.duration = self.nominal_duration + float(t) - self.program_time
            t = self.program_time
        out = []
        labels = []
        for name in ("left", "right"):
            pos, rot, grip = self.starts[name]
            age = t
            for w in self.moves.get(name, []):
                goal = np.asarray(w["position"])
                r = np.asarray(w["rotation"])
                g = w.get("grip", grip)
                if age < w["duration"]:
                    s = ik.smoothstep(age / w["duration"])
                    pos = pos + s * (goal - pos)
                    rot = ik.slerp_matrix(rot, r, s)
                    grip += s * (g - grip)
                    labels.append(w.get("label", "move"))
                    break
                age -= w["duration"]
                pos, rot, grip = goal, r, g
            q, pe, re = ik.solve(
                pos,
                ik.matrix_to_quat_xyzw(rot),
                self.q[name],
                base_pos=station.BASES[name],
                iters=60,
            )
            if pe > 0.002 or re > 0.03:
                raise RuntimeError(
                    f"Unreachable {name} reference target: {pe:.6f} m, {re:.6f} rad"
                )
            self.q[name] = q
            out.extend([*q, grip])
            self.worst_ik_m = max(self.worst_ik_m, pe)
            self.worst_ik_rad = max(self.worst_ik_rad, re)
        target = np.asarray(out)
        self.nominal = target.copy()
        if joint_pos is not None:
            error = target - np.asarray(joint_pos)
            error[[6, 13]] = 0.0
            self.integral = np.clip(self.integral + 0.6 * 0.05 * error, -0.3, 0.3)
            target = target + self.integral
        target = np.clip(
            target, self.info.action_space.low, self.info.action_space.high
        )
        # Also pass Inspect's default delta guard after this taught command ramp.
        result = self.previous + np.clip(target - self.previous, -0.04, 0.04)
        self.previous = result
        self.phase = " / ".join(labels) or "settle"
        return result

    def act(self, observation):
        t = float(observation.state_time) + 0.05
        return ActionChunk(
            actions=[
                Action(
                    data=self.action_at(t, observation.state["joint_pos"]),
                    meta={"request_stop": True} if t > self.duration else {},
                )
            ],
            control_hz=20.0,
        )


def scripted_policy(**kwargs):
    return ScriptedPolicy(**kwargs)
