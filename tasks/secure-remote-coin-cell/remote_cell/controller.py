"""Reference policy using the same gripper and joint commands as learned policies."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik, station
from yam_arena.constants import READY
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config, plan

DRIVER_ALIGNED = ("engage the blade", "turn the blade", "settle against")
CAP_ALIGNED = ("line up the cap", "seat the cap", "settle the cap", "release the seated")

class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        self.mode = mode
        self.feedback = None
        self.info = PolicyInfo(
            name=config.KEY + "_scripted",
            action_space=action_box(
                np.r_[ik.LOWER, 0.0, ik.LOWER, 0.0],
                np.r_[ik.UPPER, 1.0, ik.UPPER, 1.0],
            ),
        )
        self.reset(None)

    def bind(self, info):
        pass

    def reset(self, scene):
        self.moves = plan.waypoints(self.mode)
        self.q = {n: np.asarray(READY).copy() for n in self.moves}
        self.starts = {
            n: (*ik.tcp_pose(self.q[n], station.BASES[n]), 1.0) for n in self.q
        }
        self.previous = np.r_[READY, 1.0, READY, 1.0]
        self.integral = np.zeros(14)
        self.tool_trim = np.zeros(3)
        self.cap_trim = np.zeros(2)
        travel = max(
            (sum(w["duration"] for w in v) for v in self.moves.values()), default=0
        )
        self.duration = config.MAX_STEPS / 20 if self.mode == "hold" else travel + 1.0
        self.worst_ik_m = self.worst_ik_rad = 0.0
        self.phase = "ready"

    def register(self, p, r, measured, label):
        remote, cover, driver = self.feedback()
        body_axes = quat_xyzw_to_matrix(remote[3:])
        if body_axes[2, 2] < .993 or abs(remote[2] - config.REMOTE[2]) > .006:
            return p, r
        if label.startswith(CAP_ALIGNED):
            origin = remote[:3] + body_axes @ (p - plan.R_CAP - config.REMOTE)
            error = origin[:2] - cover[:2]
            if np.linalg.norm(error) < .009:
                self.cap_trim = np.clip(
                    self.cap_trim + .10 * np.clip(error, -.002, .002), -.006, .006
                )
            return p + np.r_[self.cap_trim, 0.0], r
        axes = quat_xyzw_to_matrix(driver[3:])
        nominal_axes = r @ plan.RIGHT.T
        offset = plan.R_DRIVER
        origin = remote[:3] + body_axes @ (p - offset - config.REMOTE)
        # Follow the actual loose driver's slot for engagement. The fastening
        # stroke then commands its clockwise angle relative to the remote.
        nominal_seat = remote[:3] + body_axes @ (
            config.SEAT_CENTER + [0., 0., config.SEAT_HEIGHT]
        )
        goal_axes = body_axes @ nominal_axes
        if np.linalg.norm(cover[:3] - nominal_seat) < .006:
            origin += np.clip(cover[:3] - nominal_seat, -.0035, .0035)
            if label.startswith("engage the blade"):
                goal_axes = quat_xyzw_to_matrix(cover[3:])
        hand, hand_axes = ik.tcp_pose(measured[7:13], station.BASES["right"])
        grasp = axes.T @ (hand - driver[:3])
        if np.linalg.norm(grasp - offset) > .030:
            return p, r
        err = np.clip(origin - driver[:3], -.003, .003)
        self.tool_trim = np.clip(.96 * self.tool_trim + .13 * err, -.003, .003)
        candidate = origin + goal_axes @ grasp + self.tool_trim + .3 * err
        measured_tool_to_hand = axes.T @ hand_axes @ plan.RIGHT.T
        yaw = np.clip(np.arctan2(measured_tool_to_hand[1, 0], measured_tool_to_hand[0, 0]), -.20, .20)
        desired_yaw = np.clip(
            np.arctan2(goal_axes[1, 0], goal_axes[0, 0])
            - np.arctan2(nominal_axes[1, 0], nominal_axes[0, 0]), -.18, .18,
        )
        return p + np.clip(candidate - p, -.009, .009), plan.rotation("Z", yaw + desired_yaw) @ r

    def action_at(self, t, joint_pos=None):
        controls = []
        phases = []
        for side in self.moves:
            p, r, grip = self.starts[side]
            age = t
            label = ""
            for entry in self.moves[side]:
                target_p = np.asarray(entry["position"])
                target_r = np.asarray(entry["rotation"])
                target_g = entry["grip"]
                span = entry["duration"]
                if age < span:
                    blend = ik.smoothstep(age / span)
                    p = p + blend * (target_p - p)
                    r = ik.slerp_matrix(r, target_r, blend)
                    grip += blend * (target_g - grip)
                    label = entry["label"]
                    break
                age -= span
                p, r, grip = target_p, target_r, target_g
            if (
                side == "right"
                and self.feedback
                and joint_pos is not None
                and label.startswith(CAP_ALIGNED + DRIVER_ALIGNED)
            ):
                p, r = self.register(p, r, joint_pos, label)
            q, pe, re = ik.solve(
                p,
                ik.matrix_to_quat_xyzw(r),
                self.q[side],
                base_pos=station.BASES[side],
                iters=60,
            )
            if pe > 0.0025 or re > 0.048:
                raise RuntimeError(
                    f"{side} pose failed in {label!r}: {pe:.5f} m, {re:.5f} rad"
                )
            self.q[side] = q
            controls.extend([*q, grip])
            self.worst_ik_m = max(self.worst_ik_m, pe)
            self.worst_ik_rad = max(self.worst_ik_rad, re)
            if label and label not in phases:
                phases.append(label)
        target = np.asarray(controls)
        if joint_pos is not None:
            error = target - joint_pos
            error[[6, 13]] = 0.0
            self.integral = np.clip(self.integral + 0.044 * error, -0.15, 0.15)
            target += self.integral
        target = np.clip(
            target, self.info.action_space.low, self.info.action_space.high
        )
        self.previous = self.previous + np.clip(target - self.previous, -0.04, 0.04)
        self.phase = " / ".join(phases) or "settle"
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


def policy(**kwargs):
    return ScriptedPolicy(**kwargs)
