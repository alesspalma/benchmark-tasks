"""Command the arms by time, joint positions and measured insertion poses."""

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
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0
        self.nozzle_in_hand = None
        self.exit_pose = None

    def aligned_insertion(self, label, progress, pos, rot, state, joint_pos):
        """Place the carried nozzle coaxially from its measured in-hand pose."""
        if self.nozzle_in_hand is None:
            tcp, wrist = ik.tcp_pose(np.asarray(joint_pos)[7:13], config.BASES["right"])
            blue = np.asarray(state["objects"]["blue"]["pose"])
            self.nozzle_in_hand = (
                wrist.T @ (blue[:3] - tcp),
                wrist.T @ quat_xyzw_to_matrix(blue[3:]),
            )
        offset, nozzle_rotation = self.nozzle_in_hand
        wrist = nozzle_rotation.T
        hose = np.asarray(state["objects"]["coupling"]["links"]["Socket"])[:3]
        seated_depth = -0.077 if self.mode == "shallow_insert" else -0.046
        pull_depth = -0.087 if self.mode == "shallow_insert" else -0.061
        depth = {
            plan.ALIGN_PLUG: -0.090,
            plan.SEAT_PLUG: -0.090 + (seated_depth + 0.090) * progress,
            plan.LOCK: seated_depth,
            plan.LIFT_LEFT: seated_depth,
            plan.PULL_TEST: seated_depth + (pull_depth - seated_depth) * progress,
            plan.RELEASE: pull_depth,
        }[label]
        desired = hose + [0.0, depth, 0.0] - wrist @ offset
        if label == plan.ALIGN_PLUG:
            desired = pos * (1.0 - progress) + desired * progress
            wrist = ik.slerp_matrix(rot, wrist, progress)
        self.exit_pose = (desired, wrist)
        return desired, wrist

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
                    if name == "right" and state is not None:
                        if label in (
                            plan.ALIGN_PLUG,
                            plan.SEAT_PLUG,
                            plan.LOCK,
                            plan.LIFT_LEFT,
                            plan.PULL_TEST,
                            plan.RELEASE,
                        ):
                            pos, rot = self.aligned_insertion(
                                label, s, pos, rot, state, joint_pos
                            )
                        elif (
                            label.startswith("Move both hands clear")
                            and self.exit_pose is not None
                        ):
                            last_pos, last_rot = self.exit_pose
                            pos = last_pos + s * (goal - last_pos)
                            rot = ik.slerp_matrix(last_rot, r, s)
                    break
                age -= w["duration"]
                pos, rot, grip = goal, r, g
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
