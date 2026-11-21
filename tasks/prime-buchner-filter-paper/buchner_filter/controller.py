"""Command ordinary position actions from the taught station layout."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.constants import READY

from . import config, plan
from .filtration import pose_parts


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
        self.duration = (
            max((sum(w["duration"] for w in v) for v in self.moves.values()), default=0)
            + 2
        )
        if self.mode == "hold":
            self.duration = config.MAX_STEPS / 20
        self.phase = "ready"
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0
        self.grasps = {}
        self.paper_grasp = None
        self.aim_offsets = {name: np.zeros(3) for name in self.starts}

    def _paper_target(self, pos, rot, state, joint_pos, alpha):
        if self.paper_grasp is None:
            center, axes = pose_parts(state["objects"]["paper"]["pose"])
            wrist_pos, wrist_axes = ik.tcp_pose(
                np.asarray(joint_pos)[:6], config.BASES["left"]
            )
            yaw = np.arctan2(axes[1, 0], axes[0, 0])
            c, s = np.cos(yaw), np.sin(yaw)
            flat = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
            self.paper_grasp = (
                flat @ axes.T @ (wrist_pos - center),
                flat @ axes.T @ wrist_axes,
            )
        offset, alignment = self.paper_grasp
        return pos + alpha * offset, ik.slerp_matrix(rot, alignment, alpha)

    def _vessel_target(self, name, pos, rot, state, joint_pos, alpha, aim):
        vessel, neutral, start = (
            (config.RINSE, plan.LEFT_CUP, 0)
            if name == "left"
            else (config.SUSPENSION, plan.RIGHT_CUP, 7)
        )
        center, axes = pose_parts(state["objects"][vessel.name]["pose"])
        if name not in self.grasps:
            wrist_pos, wrist_axes = ik.tcp_pose(
                np.asarray(joint_pos)[start : start + 6], config.BASES[name]
            )
            self.grasps[name] = (axes.T @ (wrist_pos - center), axes.T @ wrist_axes)
        offset, grasp = self.grasps[name]
        planned_axes = rot @ neutral.T
        adjusted_pos = pos + planned_axes @ (offset - vessel.grip)
        adjusted_rot = planned_axes @ grasp
        offset = self.aim_offsets[name]
        if aim:
            actual_lip = center + axes @ vessel.outlet
            goal_lip = pos + planned_axes @ (vessel.outlet - vessel.grip)
            error = goal_lip - actual_lip
            error[2] = 0.0
            # Aim from native beaker-lip and jug poses. Bounded PI correction
            # lets the wrist track the geometry without any simulator edits.
            offset[:] = np.clip(
                offset + 0.055 * error, [-0.046, -0.047, -0.018], [0.063, 0.047, 0.038]
            )
            adjusted_pos += offset + np.clip(
                0.35 * error, [-0.019, -0.018, -0.008], [0.020, 0.018, 0.021]
            )
        else:
            offset *= 0.91
        return pos + alpha * (adjusted_pos - pos), ik.slerp_matrix(
            rot, adjusted_rot, alpha
        )

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
                        and joint_pos is not None
                        and label
                        in (
                            "Center the paper over the funnel plate",
                            "Lower the horizontal disk above the porcelain wall",
                            "Steady the paper at the mouth",
                            "Release the paper onto the perforated plate",
                            "Clear the paper and funnel",
                        )
                    ):
                        alpha = (
                            s
                            if label.startswith("Center")
                            else 1.0 - s
                            if label.startswith("Clear")
                            else 1.0
                        )
                        pos, rot = self._paper_target(pos, rot, state, joint_pos, alpha)
                    if (
                        state is not None
                        and joint_pos is not None
                        and label
                        in (
                            "Bring the beaker spout over the funnel",
                            "Steady the spout over the funnel",
                            "Align the tilted spout before liquid flows",
                            "Pour slowly into the funnel",
                            "Let the measured liquid drain",
                            "Restore the beaker upright",
                            "Let the upright beaker settle before transport",
                            "Return the upright beaker to its place",
                            "Lower the beaker to the bench",
                            "Release the beaker on the bench",
                        )
                    ):
                        # Each phase appears in both lists. Only the arm actively
                        # carrying a beaker receives the registered offset.
                        active_vessel = (
                            config.RINSE if name == "left" else config.SUSPENSION
                        )
                        obj_z = state["objects"][active_vessel.name]["pose"][2]
                        if obj_z > config.TABLE + 0.045 or (
                            name in self.grasps and g < 0.12
                        ):
                            alpha = s if label.startswith("Bring") else 1.0
                            pos, rot = self._vessel_target(
                                name,
                                pos,
                                rot,
                                state,
                                joint_pos,
                                alpha,
                                label.startswith(
                                    (
                                        "Steady the spout",
                                        "Align the tilted",
                                        "Pour",
                                        "Let the measured",
                                    )
                                ),
                            )
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
            if pe > 0.065 or re > 0.34:
                raise RuntimeError(
                    f"Unreachable {name} reference target: {pe:.6f} m, {re:.6f} rad"
                )
            self.q[name] = q
            out.extend([*q, grip])
            self.worst_ik_m = max(self.worst_ik_m, pe)
            self.worst_ik_rad = max(self.worst_ik_rad, re)
        target = np.asarray(out)
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
