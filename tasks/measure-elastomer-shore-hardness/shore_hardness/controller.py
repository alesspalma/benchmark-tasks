"""Native reference using elapsed time and ordinary joint positions."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.constants import READY

from . import config, plan
from .motion import rotation as axis_rotation
from .scoring import pose

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
        self.tool_offset = None
        self.duration = (
            max((sum(w["duration"] for w in v) for v in self.moves.values()), default=0)
            + 2
        )
        if self.mode == "hold":
            self.duration = config.MAX_STEPS / 20
        self.phase = "ready"
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def _correct(self, name, label, position, rotation, joints, fixture):
        press_phases = ("position the gauge above site", "seat pressure foot on specimen",
                        "hold for the Shore A reading", "lift the pressure foot")
        if name != "right" or label not in press_phases or fixture is None:
            return position, rotation
        sample, sample_rotation = pose(fixture, "coupon")
        tool, tool_rotation = pose(fixture, "durometer")
        hand, hand_rotation = ik.tcp_pose(np.asarray(joints)[7:13], config.BASES[name])
        if self.tool_offset is None:
            self.tool_offset = hand_rotation.T @ (tool - hand)
        grip_orientation = hand_rotation.T @ tool_rotation
        ref = position - plan.GAUGE_TEST
        # The compliant wrist deflects a few degrees as the pressure foot bears
        # down. Compensate that loaded posture smoothly during the approach.
        load_fraction = np.clip((.004 - ref[2]) / .015, 0., 1.)
        deflection_correction = (
            axis_rotation("Y", .033 * load_fraction)
            @ axis_rotation("X", -.030 * load_fraction)
        )
        desired_rotation = sample_rotation @ deflection_correction @ grip_orientation.T
        rotation = ik.slerp_matrix(rotation, desired_rotation, .85)
        correct_foot = sample + sample_rotation @ (ref + [0., 0., config.COUPON_SIZE[2]/2])
        correction = correct_foot - rotation @ self.tool_offset - position
        return position + np.clip(correction, -.030, .030), rotation

    def action_at(self, t, joint_pos=None, fixture=None):
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
            pos, rot = self._correct(name, label, pos, rot, joint_pos, fixture)
            q, pe, re = ik.solve(
                pos,
                ik.matrix_to_quat_xyzw(rot),
                self.q[name],
                base_pos=config.BASES[name],
                iters=60,
            )
            if pe > .020 or re > .23:
                raise RuntimeError(
                    f"Unreachable {name} reference target at {label}: {pe:.6f} m, {re:.6f} rad"
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
