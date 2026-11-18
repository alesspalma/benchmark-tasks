"""Native reference using elapsed time and ordinary joint positions."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik, station
from yam_arena.constants import READY
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

from . import config, plan


CUT_PHASES = ("seat the rim", "settle the rim", "press", "finish the cut", "keep the core")
LIFT_PHASES = ("withdraw", "release the apple", "clear the apple", "hold the extracted")
REGISTERED_PHASES = ("align the tool", "brace the apple", *CUT_PHASES, *LIFT_PHASES)


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        self.mode = mode
        self.feedback = None
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
        if self.mode == "hold":
            self.duration = config.MAX_STEPS / 20
        self.line_trim = np.zeros(3)
        self.phase = "ready"
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def _register_tool(self, position, gripper_rotation, measured, phase):
        tool, fruit = self.feedback()
        tool_rotation = quat_xyzw_to_matrix(tool[3:])
        hand, hand_rotation = ik.tcp_pose(measured[7:13], station.BASES["right"])
        fruit_rotation = quat_xyzw_to_matrix(fruit[3:])
        nominal_tool = gripper_rotation @ plan.RELATIVE_HAND.T
        tip_from_fruit = position - config.GRIP * nominal_tool[:, 2] - config.APPLE
        desired_tip = fruit[:3] + fruit_rotation @ tip_from_fruit
        target_tool = fruit_rotation @ nominal_tool
        local_offset = tool_rotation.T @ (hand - tool[:3])
        registered = desired_tip + target_tool @ local_offset
        corrected = target_tool @ tool_rotation.T @ hand_rotation
        if phase.startswith(CUT_PHASES):
            error = np.clip(desired_tip - tool[:3], -.012, .012)
            self.line_trim = np.clip(self.line_trim + np.array([.08, .08, .045]) * error,
                                     [-.014, -.014, -.012], [.014, .014, .012])
            registered += self.line_trim + np.array([.60, .60, .30]) * error
            # The loaded wrist can bend even with a stable grasp. Correct
            # the shaft direction using an ordinary wrist orientation command.
            swing = np.cross(tool_rotation[:, 2], target_tool[:, 2])
            swing_size = np.linalg.norm(swing)
            if swing_size > .001:
                angle = min(.16, .75 * np.arcsin(np.clip(swing_size, 0., 1.)))
                corrected = axis_angle_matrix(swing / swing_size, angle) @ corrected
        elif phase.startswith(LIFT_PHASES):
            self.line_trim *= .86
            registered += self.line_trim
        return position + np.clip(registered - position, -.038, .038), ik.slerp_matrix(
            gripper_rotation, corrected, .84
        )

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
            if name == "right" and self.feedback and joint_pos is not None:
                if label.startswith(REGISTERED_PHASES):
                    pos, rot = self._register_tool(pos, rot, joint_pos, label)
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
