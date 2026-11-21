"""Command ordinary position actions from the taught station layout."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.constants import READY
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config, plan


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
        self.phase = "ready"
        self.park_delta = None
        self.fill_delta = None
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def contact_alignment(self, name, phase, position, state):
        if not state:
            return position
        pose = np.asarray(state["objects"]["pen"]["pose"])
        origin = pose[:3]
        if name == "right" and phase in (
            "Position the other gripper above the pump knob",
            "Compress the converter pump inside the bottle",
            "Let the pump spring return and draw ink",
        ):
            normal = quat_xyzw_to_matrix(pose[3:])[:, 2]
            button = origin + (config.BUTTON_Z + 0.004) * normal
            nib_height = config.FILL_NIB_Z + (0.034 if self.mode == "dry" else 0.0)
            expected = np.r_[config.BOTTLE[:2], nib_height + config.BUTTON_Z + 0.004]
            return (
                position
                + np.clip(button - expected, -0.035, 0.035)
                + [0.001, -0.003, 0.0]
            )
        if name == "left" and phase in (
            "Hold the pen at filling depth",
            "Position the other gripper above the pump knob",
            "Compress the converter pump inside the bottle",
            "Let the pump spring return and draw ink",
            "Move the pumping hand clear",
            "Lift the filled pen from the bottle",
        ):
            if self.fill_delta is None:
                hand_center = np.asarray(state["hands"]["left_robot"]["points"][0])
                self.fill_delta = np.r_[
                    np.clip(hand_center[:2] - origin[:2], -0.012, 0.012),
                    np.clip(hand_center[2] - origin[2] - 0.043, -0.005, 0.014),
                ]
            return position + self.fill_delta
        if name == "left" and phase in (
            "Move the filled pen over its rest",
            "Seat the pen on the protective shoulder rest",
            "Let go of the pen",
        ):
            if self.park_delta is None:
                hand_center = np.asarray(state["hands"]["left_robot"]["points"][0])
                self.park_delta = np.r_[
                    np.clip(hand_center[:2] - origin[:2], -0.016, 0.016), 0.0
                ]
            return position + self.park_delta
        return position

    def action_at(self, t, joint_pos=None, state=None):
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
            pos = self.contact_alignment(name, label, np.asarray(pos), state)
            q, pe, re = ik.solve(
                pos,
                ik.matrix_to_quat_xyzw(rot),
                self.q[name],
                base_pos=config.BASES[name],
                iters=60,
            )
            if pe > 0.010 or re > 0.09:
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
            self.integral = np.clip(self.integral + 0.9 * 0.05 * error, -0.085, 0.085)
            target = target + self.integral + 0.25 * error
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
