"""The reference moves the ordinary 14 joint targets, like a policy."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik

from . import config as C, plan


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        if mode not in ("demo", "hold", "repeat"):
            raise ValueError(mode)
        self.mode = mode
        self.info = PolicyInfo(
            name=C.KEY+"_scripted",
            action_space=action_box(np.r_[ik.LOWER, 0., ik.LOWER, 0.],
                                    np.r_[ik.UPPER, 1., ik.UPPER, 1.]),
        )
        self.moves = plan.waypoints(mode)
        self.duration = sum(w["duration"] for w in self.moves["left"]) + 1.
        self.reset()

    def reset(self, scene=None):
        self.q = {s: np.array(C.STOWED[s]) for s in C.BASES}
        self.starts = {s: (*ik.tcp_pose(self.q[s], C.BASES[s]), 1.) for s in self.q}
        self.previous = np.r_[self.q["left"], 1., self.q["right"], 1.]
        self.integral = np.zeros(14)
        self.phase = "reset"
        self.worst_ik_m = self.worst_ik_rad = 0.

    def action_at(self, t, joint_pos=None, scene_state=None):
        if self.mode == "hold":
            self.phase = "idle"
            return self.previous
        target = []
        for side in ("left", "right"):
            position, orientation, opening = self.starts[side]
            age = t
            self.phase = "settle"
            for waypoint in self.moves[side]:
                goal = np.asarray(waypoint["position"])
                goal_rot = np.asarray(waypoint["rotation"])
                goal_open = waypoint["grip"]
                duration = waypoint["duration"]
                if age < duration:
                    s = ik.smoothstep(age/duration)
                    position = position+s*(goal-position)
                    orientation = ik.slerp_matrix(orientation, goal_rot, s)
                    opening += s*(goal_open-opening)
                    self.phase = waypoint["label"]
                    break
                age -= duration
                position, orientation, opening = goal, goal_rot, goal_open
            q, position_error, angle_error = ik.solve(
                position, ik.matrix_to_quat_xyzw(orientation), self.q[side],
                base_pos=C.BASES[side], iters=60,
            )
            if position_error > .011 or angle_error > .12:
                raise RuntimeError(f"Unreachable {side} target in {self.phase}: {position_error:.4f} m and {angle_error:.3f} rad")
            self.q[side] = q
            self.worst_ik_m = max(self.worst_ik_m, position_error)
            self.worst_ik_rad = max(self.worst_ik_rad, angle_error)
            target.extend([*q, opening])
        target = np.asarray(target)
        if joint_pos is not None:
            error = target-np.asarray(joint_pos)
            error[[6, 13]] = 0.
            self.integral = np.clip(self.integral+.04*error, -.055, .055)
            target += .18*error+self.integral
        target = np.clip(target, self.info.action_space.low, self.info.action_space.high)
        self.previous += np.clip(target-self.previous, -.04, .04)
        return self.previous.copy()

    def act(self, observation):
        t = float(observation.state_time) + C.CHECK_DT
        return ActionChunk(actions=[
            Action(data=self.action_at(t, observation.state["joint_pos"]),
                   meta={"request_stop": t > self.duration} if self.mode == "demo" else {})
        ], control_hz=20.)


def policy(**kwargs):
    return ScriptedPolicy(**kwargs)
