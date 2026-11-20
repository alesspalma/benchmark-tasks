"""Native reference using elapsed time and ordinary joint positions."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.constants import READY

from scipy.spatial.transform import Rotation

from . import config, plan
from .scoring import line_state


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        self.mode = mode
        low = np.r_[ik.LOWER, 0.0, ik.LOWER, 0.0]
        high = np.r_[ik.UPPER, 1.0, ik.UPPER, 1.0]
        self.info = PolicyInfo(
            name=config.KEY + "_scripted", action_space=action_box(low, high)
        )
        self.reset(None)

    def bind(self, info):
        pass

    def reset(self, scene):
        self.moves = plan.waypoints(self.mode)
        self.q = {n: np.asarray(READY).copy() for n in ("left", "right")}
        self.starts = {
            n: (*ik.tcp_pose(self.q[n], config.BASES[n]), 1.0) for n in self.q
        }
        self.previous = np.r_[READY, 1.0, READY, 1.0]
        self.integral = np.zeros(14)
        self.duration = (
            max((sum(w["duration"] for w in v) for v in self.moves.values()), default=0)
            + 2
        )
        if self.mode == "hold":
            self.duration = config.MAX_STEPS / 20
        self.line_calibration = {}
        self.phase = "ready"
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def action_at(self, t, joint_pos=None, observed=None):
        out = []
        labels = []
        for name in ("left", "right"):
            pos, rot, grip = self.starts[name]
            age = t
            label = ""
            for w in self.moves.get(name, []):
                goal = np.asarray(w["position"])
                r = np.asarray(w["rotation"])
                g = w.get("grip", grip)
                if age < w["duration"]:
                    s = ik.smoothstep(age / w["duration"])
                    pos = pos + s * (goal - pos)
                    rot = ik.slerp_matrix(rot, r, s)
                    grip += s * (g - grip)
                    label = w["label"]
                    labels.append(label)
                    break
                age -= w["duration"]
                pos, rot, grip = goal, r, g
            if observed is not None and self.mode != "hold" and 8.55 <= t < 15.4:
                j = 0 if name == "left" else 7
                _, current_rot = ik.tcp_pose(
                    np.asarray(joint_pos[j:j + 6]), config.BASES[name]
                )
                tcp = np.asarray(observed["hands"][name + "_robot"]["points"][0])
                center, axis = line_state(observed["objects"][name + "_line"])
                if axis[1] < 0:
                    axis = -axis
                tip = center - config.sign(name) * config.LINE_LENGTH / 2 * axis
                sample_offset = current_rot.T @ (tip - tcp)
                sample_axis = current_rot.T @ axis
                offset, axis = self.line_calibration.get(name, (sample_offset, sample_axis))
                # A fresh estimate handles a long strip rocking between two
                # small jaws as they lower into the trimmer's tapered eyelet.
                if t <= 15.02:
                    offset = .82 * offset + .18 * sample_offset
                if t <= 11.8:
                    axis = .82 * axis + .18 * sample_axis
                self.line_calibration[name] = (offset, axis)
                line_axis = rot @ axis
                goal_axis = np.array([0., 1., 0.])
                cross = np.cross(line_axis, goal_axis)
                sin_angle = np.linalg.norm(cross)
                blend = ik.smoothstep(min(1., (t - 8.55) / .8))
                if sin_angle > 1e-6:
                    angle = min(.12, np.arctan2(sin_angle, np.dot(line_axis, goal_axis)))
                    rot = Rotation.from_rotvec(blend * angle * cross / sin_angle).as_matrix() @ rot
                tip = pos - [0., config.sign(name) * config.GRIP_FROM_TIP, 0.]
                adjusted = tip - rot @ offset - pos
                travel = np.array([.009, .009, .018])
                pos += blend * np.clip(adjusted, -travel, travel)
            q, pe, re = ik.solve(
                pos,
                ik.matrix_to_quat_xyzw(rot),
                self.q[name],
                base_pos=config.BASES[name],
                iters=60,
            )
            if pe > 0.002 or re > 0.03:
                fallback = ik.solve(
                    pos, ik.matrix_to_quat_xyzw(rot), np.asarray(READY).copy(),
                    base_pos=config.BASES[name], iters=100,
                )
                if fallback[1] + 0.1 * fallback[2] < pe + 0.1 * re:
                    q, pe, re = fallback
            if pe > 0.004 or re > 0.045:
                raise RuntimeError(
                    f"Unreachable {name} reference target: {pe:.6f} m, {re:.6f} rad"
                )
            self.q[name] = q
            out.extend([*q, grip])
            self.worst_ik_m = max(self.worst_ik_m, pe)
            self.worst_ik_rad = max(self.worst_ik_rad, re)
        target = np.asarray(out)
        if joint_pos is not None:
            error = target - np.asarray(joint_pos)
            error[[6, 13]] = 0.0
            self.integral = np.clip(self.integral + 1.75 * 0.05 * error, -0.24, 0.24)
            target = target + self.integral + 0.40 * error
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


def policy(**kwargs):
    return ScriptedPolicy(**kwargs)
