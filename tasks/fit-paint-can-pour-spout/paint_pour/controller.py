"""Reference joint commands for ordinary bimanual contact motion."""

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
        self.lid_grasp = None
        self.lid_clear = None
        self.can_grasps = {}

    def _align_lid(self, label, alpha, pos, rot, state, joints):
        center = "Center the pour lid over the can rim"
        if self.lid_grasp is None:
            tcp, wrist = ik.tcp_pose(np.asarray(joints)[:6], config.BASES["left"])
            lid = np.asarray(state["objects"]["lid"]["pose"])
            self.lid_grasp = (
                wrist.T @ (lid[:3] - tcp),
                wrist.T @ quat_xyzw_to_matrix(lid[3:]),
            )
        offset, grasp = self.lid_grasp
        can = np.asarray(state["objects"]["can"]["pose"])
        tin = quat_xyzw_to_matrix(can[3:])
        yaw = np.deg2rad(-18)
        spout_heading = np.array(
            [
                [np.cos(yaw), -np.sin(yaw), 0.0],
                [np.sin(yaw), np.cos(yaw), 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        wrist = tin @ spout_heading @ grasp.T
        gap = {
            center: 0.018,
            "Seat the flexible lid while holding the can": 0.018 - 0.028 * alpha,
            "Press the skirt past the rolled bead": -0.010,
            "Open the lid hand once the seal is seated": -0.010 + 0.010 * alpha,
        }[label]
        target = can[:3] + tin @ (config.LID_SEAT + [0.0, 0.0, gap]) - wrist @ offset
        if label == center:
            reference = (
                config.CAN + config.LID_SEAT + config.LID_GRIP + [0.0, 0.0, 0.018]
            )
            target = pos + alpha * (target - reference)
            wrist = ik.slerp_matrix(rot, wrist, alpha)
        return target, wrist

    def _carry_can(self, name, pos, rot, state, joints):
        can = np.asarray(state["objects"]["can"]["pose"])
        tin = quat_xyzw_to_matrix(can[3:])
        if name not in self.can_grasps:
            start = 0 if name == "left" else 7
            tcp, wrist = ik.tcp_pose(
                np.asarray(joints)[start : start + 6], config.BASES[name]
            )
            self.can_grasps[name] = (tin.T @ (tcp - can[:3]), tin.T @ wrist)
        offset, grip = self.can_grasps[name]
        if name == "left":
            reference = config.DOWN @ np.diag([-1.0, -1.0, 1.0])
            bail = config.AUX_HANDLE + [0.0, 0.0, 0.012]
        else:
            reference = config.DOWN
            bail = config.HANDLE + [0.0, 0.0, 0.012]
        desired_can = rot @ reference.T
        return pos + desired_can @ (offset - bail), desired_can @ grip

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
                    if (
                        name == "left"
                        and state is not None
                        and label
                        in (
                            "Center the pour lid over the can rim",
                            "Seat the flexible lid while holding the can",
                            "Press the skirt past the rolled bead",
                            "Open the lid hand once the seal is seated",
                        )
                    ):
                        pos, rot = self._align_lid(label, s, pos, rot, state, joint_pos)
                    if (
                        name == "left"
                        and state is not None
                        and label == "Lift the open lid hand clear"
                    ):
                        if self.lid_clear is None:
                            self.lid_clear = ik.tcp_pose(
                                np.asarray(joint_pos)[:6], config.BASES["left"]
                            )
                        cp, cr = self.lid_clear
                        pos = cp + s * (goal - cp)
                        rot = ik.slerp_matrix(cr, r, s)
                    if state is not None and label in (
                        "Lift the can with both hands supporting it",
                        "Tilt the spout toward the roller tray",
                        "Begin a measured pour",
                        "Pour through the seated lid",
                        "Return the can toward upright to stop the flow",
                        "Hold the can vertically",
                        "Set the can upright on its mat",
                        "Release the can with the lid still secured",
                    ):
                        pos, rot = self._carry_can(name, pos, rot, state, joint_pos)
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
            if pe > 0.018 or re > 0.12:
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
