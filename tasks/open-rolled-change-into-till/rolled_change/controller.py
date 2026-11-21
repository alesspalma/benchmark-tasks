"""Track the known drawer layout using ordinary robot joint position commands."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.constants import READY
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C
from . import plan


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        self.mode = mode
        low = np.r_[ik.LOWER, 0.0, ik.LOWER, 0.0]
        high = np.r_[ik.UPPER, 1.0, ik.UPPER, 1.0]
        self.info = PolicyInfo(
            name=C.KEY + "_scripted", action_space=action_box(low, high)
        )
        self.reset(None)

    def bind(self, info):
        pass

    def reset(self, scene):
        self.moves = plan.waypoints(self.mode)
        self.q = {name: np.asarray(READY).copy() for name in plan.SIGNS}
        self.starts = {
            name: (*ik.tcp_pose(self.q[name], C.BASES[name]), 1.0)
            for name in plan.SIGNS
        }
        self.previous = np.r_[READY, 1.0, READY, 1.0]
        self.integral = np.zeros(14)
        self.duration = (
            max((sum(w["duration"] for w in v) for v in self.moves.values()), default=0)
            + 2.0
        )
        if self.mode == "hold":
            self.duration = C.MAX_STEPS / 20
        self.phase = "ready"
        self.worst_ik_m = self.worst_ik_rad = 0.0
        self.grasps = {}

    @staticmethod
    def _world_goal(name, waypoint, grasp):
        center = np.asarray(waypoint["position"])
        wrapper_axes = np.asarray(waypoint["rotation"])
        if waypoint["label"].startswith(("Reach to", "Place the", "Grip both")):
            return center, wrapper_axes
        offset, orientation = grasp.get(
            name, (plan.GRIP_POINT[name], plan.NEUTRAL[name])
        )
        return center + wrapper_axes @ offset, wrapper_axes @ orientation

    def action_at(self, t, joint_pos=None, state=None):
        output, labels = [], []
        for i, name in enumerate(plan.SIGNS):
            pos, rot, grip = self.starts[name]
            age = t
            for w in self.moves[name]:
                label = w["label"]
                if (
                    label == "Lift the intact roll"
                    and age < w["duration"]
                    and name not in self.grasps
                    and state is not None
                    and joint_pos is not None
                ):
                    paper = C.PAPER_NAMES[i]
                    pose = np.asarray(state["objects"][paper]["pose"])
                    obj_rot = quat_xyzw_to_matrix(pose[3:])
                    arm_pos, arm_rot = ik.tcp_pose(
                        np.asarray(joint_pos)[7 * i : 7 * i + 6], C.BASES[name]
                    )
                    self.grasps[name] = (
                        obj_rot.T @ (arm_pos - pose[:3]),
                        obj_rot.T @ arm_rot,
                    )
                goal, r = self._world_goal(name, w, self.grasps)
                g = w["grip"]
                if age < w["duration"]:
                    u = ik.smoothstep(age / w["duration"])
                    pos = pos + u * (goal - pos)
                    rot = ik.slerp_matrix(rot, r, u)
                    grip += u * (g - grip)
                    labels.append(label)
                    break
                age -= w["duration"]
                pos, rot, grip = goal, r, g
            q, pe, re = ik.solve(
                pos,
                ik.matrix_to_quat_xyzw(rot),
                self.q[name],
                base_pos=C.BASES[name],
                iters=60,
            )
            self.worst_ik_m = max(self.worst_ik_m, pe)
            self.worst_ik_rad = max(self.worst_ik_rad, re)
            self.q[name] = q
            output.extend([*q, grip])
        target = np.asarray(output)
        if joint_pos is not None:
            error = target - np.asarray(joint_pos)
            error[[6, 13]] = 0.0
            self.integral = np.clip(self.integral + 0.045 * error, -0.085, 0.085)
            target = target + self.integral + 0.25 * error
        target = np.clip(
            target, self.info.action_space.low, self.info.action_space.high
        )
        delta = target - self.previous
        if any(
            label.startswith(("Tilt the half-roll", "Turn the empty paper"))
            for label in labels
        ):
            # Scale the whole increment during rotation so elbow and wrist
            # limits cannot independently bend the path into the drawer.
            scale = min(1.0, 0.04 / max(float(np.max(np.abs(delta))), 1e-9))
            self.previous = self.previous + scale * delta
        else:
            self.previous = self.previous + np.clip(delta, -0.04, 0.04)
        self.phase = " / ".join(labels) or "settle"
        return self.previous

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
