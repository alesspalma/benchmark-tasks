"""Native reference with measured in-hand registration during the transfer."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik, station
from yam_arena.constants import READY
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config, plan


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        self.mode = mode
        self.native_feedback = None
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
            n: (*ik.tcp_pose(self.q[n], station.BASES[n]), 1.0) for n in self.q
        }
        self.previous = np.r_[READY, 1.0, READY, 1.0]
        self.integral = np.zeros(14)
        self.duration = (
            max((sum(w["duration"] for w in v) for v in self.moves.values()), default=0)
            + 2
        )
        self.phase = "ready"
        self.grasp_adjustment = np.zeros(3)
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def _register_nose(self, position, rotation, joints):
        # The demo tracks the held applicator. The model receives ordinary RGB
        # and joint observations. Corrections still use YAM joint commands.
        tool, board = self.native_feedback()
        hand_p, hand_r = ik.tcp_pose(joints[7:13], station.BASES["right"])
        tool_r = quat_xyzw_to_matrix(tool[3:7])
        measured_toe = tool[:3] + tool_r @ config.TOOL_TOE
        measured_offset = hand_r.T @ (measured_toe - hand_p)
        nominal_offset = plan.TOP.T @ (config.TOOL_TOE - config.TOOL_GRIP)
        adjustment = rotation @ (nominal_offset - measured_offset)
        adjustment += board[:3] - config.BOARD_START
        self.grasp_adjustment = .7 * self.grasp_adjustment + .3 * np.clip(adjustment, -.024, .024)
        return position + self.grasp_adjustment

    def action_at(self, t, joint_pos=None):
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
            if name == "right" and self.native_feedback and joint_pos is not None:
                if label.startswith(("pitch the", "press the", "stabilize", "draw the", "lift cleanly")):
                    pos = self._register_nose(pos, rot, np.asarray(joint_pos))
                else:
                    self.grasp_adjustment *= .82
                    pos = pos + self.grasp_adjustment
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
        if joint_pos is not None:
            error = target - np.asarray(joint_pos)
            error[[6, 13]] = 0.0
            self.integral = np.clip(self.integral + 0.6 * 0.05 * error, -0.15, 0.15)
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


def policy(**kwargs):
    return ScriptedPolicy(**kwargs)
