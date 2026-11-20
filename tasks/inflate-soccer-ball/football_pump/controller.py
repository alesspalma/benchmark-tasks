"""Native reference with joint feedback and measured needle alignment."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.constants import READY
from yam_arena.urdf import axis_angle_matrix, quat_xyzw_to_matrix

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
        self.chuck_targets = {}
        self.refined = False
        self.aimed_return = False
        self.needle_trim = np.zeros(3)

    def aim_chuck(self, state, joints):
        ball = np.asarray(state["objects"]["ball"]["pose"])
        chuck = np.asarray(state["objects"]["needle"]["pose"])
        needle = quat_xyzw_to_matrix(chuck[3:])
        axis = quat_xyzw_to_matrix(ball[3:])[:, 2]
        port = ball[:3] + config.BALL_R * axis
        grip_position, grip_rotation = ik.tcp_pose(joints[:6], config.BASES["left"])
        local_center = grip_rotation.T @ (chuck[:3] - grip_position)
        self.chuck_in_grip = local_center
        turn = np.cross(needle[:, 2], axis)
        deviation = np.arctan2(np.linalg.norm(turn), needle[:, 2] @ axis)
        aligned = axis_angle_matrix(turn, deviation) @ grip_rotation if deviation > 1e-5 else grip_rotation
        # Rotation about the shaft does not change its path into the round bore.
        # Choose a reachable wrist pose while keeping the same physical grip.
        best_cost = np.inf
        grip_target = aligned
        for yaw in (0., -.13, .13, -.26, .26, -.39, .39, -.52, .52):
            candidate = axis_angle_matrix(axis, yaw) @ aligned
            target = port + .044 * axis - candidate @ local_center
            _, position_error, angle_error = ik.solve(
                target, ik.matrix_to_quat_xyzw(candidate), self.q["left"],
                base_pos=config.BASES["left"], iters=110,
            )
            cost = 55*position_error + angle_error + .10*abs(yaw)
            if cost < best_cost:
                best_cost = cost
                grip_target = candidate
            if best_cost < .006:
                break
        approaching = port + .044 * axis - grip_target @ local_center
        preseat = port + .034 * axis - grip_target @ local_center
        inserted = port + .007 * axis - grip_target @ local_center
        for phase in (
            "Hold the needle square above the valve", "Clear the seated needle",
            "Return to the needle and ball", "Withdraw the needle straight",
        ):
            self.chuck_targets[phase] = approaching, grip_target
        for phase in (
            "Seat the needle in the rubber valve", "Release the inserted chuck",
            "Grip near the valve again", "Close on the hose chuck",
        ):
            self.chuck_targets[phase] = inserted, grip_target
        self.chuck_targets["Center the needle over the valve opening"] = preseat, grip_target

    def aim_return(self, state):
        chuck = np.asarray(state["objects"]["needle"]["pose"])
        axis = quat_xyzw_to_matrix(chuck[3:])[:, 2]
        _, grip_rotation = self.chuck_targets["Seat the needle in the rubber valve"]
        grip = chuck[:3] - grip_rotation @ self.chuck_in_grip
        clear = grip + .036 * axis
        self.chuck_targets["Return to the needle and ball"] = clear, grip_rotation
        self.chuck_targets["Grip near the valve again"] = grip, grip_rotation
        self.chuck_targets["Center the open jaws on the chuck"] = grip, grip_rotation
        self.chuck_targets["Close on the hose chuck"] = grip, grip_rotation
        self.chuck_targets["Withdraw the needle straight"] = clear, grip_rotation

    def trim_alignment(self, state):
        ball = np.asarray(state["objects"]["ball"]["pose"])
        chuck = np.asarray(state["objects"]["needle"]["pose"])
        valve_axis = quat_xyzw_to_matrix(ball[3:])[:, 2]
        stem_axis = quat_xyzw_to_matrix(chuck[3:])[:, 2]
        tip = chuck[:3] - config.NEEDLE_TIP * stem_axis
        delta = tip - (ball[:3] + config.BALL_R * valve_axis)
        side_error = delta - (delta @ valve_axis) * valve_axis
        self.needle_trim = np.clip(self.needle_trim - .29*side_error, -.012, .012)

    def action_at(self, t, joint_pos=None, scene_state=None):
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
                if (
                    name == "left" and 0 <= age < w["duration"]
                    and scene_state is not None
                ):
                    if w["label"] == "Hold the needle square above the valve" and not self.chuck_targets:
                        self.aim_chuck(scene_state, joint_pos)
                    if w["label"] == "Seat the needle in the rubber valve" and not self.refined:
                        previous_approach = self.chuck_targets["Center the needle over the valve opening"]
                        self.aim_chuck(scene_state, joint_pos)
                        self.chuck_targets["Center the needle over the valve opening"] = previous_approach
                        self.needle_trim[:] = 0.
                        self.refined = True
                    if w["label"] in (
                        "Center the needle over the valve opening",
                        "Seat the needle in the rubber valve",
                    ) and age > .38:
                        self.trim_alignment(scene_state)
                    if w["label"] == "Return to the needle and ball" and not self.aimed_return:
                        self.aim_return(scene_state)
                        self.aimed_return = True
                    if w["label"] == "Center the open jaws on the chuck":
                        earlier_targets = {
                            name: self.chuck_targets[name]
                            for name in ("Return to the needle and ball", "Grip near the valve again")
                        }
                        self.aim_return(scene_state)
                        self.chuck_targets.update(earlier_targets)
                if name == "left" and w["label"] in self.chuck_targets:
                    goal, r = self.chuck_targets[w["label"]]
                    if w["label"] in (
                        "Center the needle over the valve opening",
                        "Seat the needle in the rubber valve",
                        "Release the inserted chuck",
                    ):
                        goal = goal + self.needle_trim
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
            q, pe, re = ik.solve(
                pos,
                ik.matrix_to_quat_xyzw(rot),
                self.q[name],
                base_pos=config.BASES[name],
                iters=60,
            )
            if pe > 0.025 or re > 0.23:
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
