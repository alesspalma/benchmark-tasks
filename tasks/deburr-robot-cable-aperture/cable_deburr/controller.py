"""Native reference tracking the loaded cutting nose in the workpiece frame."""

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
        self.pressure_bias = 0.0
        self.radial_bias = 0.0
        self.nose_to_hand = None
        self.plate_release_offset = np.zeros(3)
        self.duration = (
            max((sum(w["duration"] for w in v) for v in self.moves.values()), default=0)
            + 2
        )
        if self.mode == "hold":
            self.duration = config.MAX_STEPS / 20
        self.phase = "ready"
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

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
            if name == "right" and scene_state is not None and label.startswith((
                    "Contour:", "Return the tool to its rest", "Square the plate", "Seat the deburring tool")):
                pos, rot = self._track_tool(pos, rot, joint_pos, scene_state, label)
            if name == "left" and scene_state is not None:
                pos = self._track_plate_return(pos, scene_state, label)
            q, pe, re = ik.solve(
                pos,
                ik.matrix_to_quat_xyzw(rot),
                self.q[name],
                base_pos=config.BASES[name],
                iters=60,
            )
            if pe > 0.006 or re > 0.06:
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
            if any(label.startswith("Contour:") for label in labels):
                self.integral[7:13] = np.clip(self.integral[7:13], -.035, .035)
            target = target + self.integral + 0.40 * error
        target = np.clip(
            target, self.info.action_space.low, self.info.action_space.high
        )
        # Also pass Inspect's default delta guard after this taught command ramp.
        result = self.previous + np.clip(target - self.previous, -0.04, 0.04)
        self.previous = result
        self.phase = " / ".join(labels) or "settle"
        return result

    def _track_plate_return(self, position, scene, label):
        if label == "Square the plate on both parallels":
            plate_xy = np.asarray(scene["objects"]["plate"]["pose"][:2])
            error = config.PLATE_START[:2] - plate_xy
            offset = self.plate_release_offset[:2] + np.clip(.065 * error, -.0015, .0015)
            limit = np.array([.022, .037])
            self.plate_release_offset[:2] = np.clip(offset, -limit, limit)
        return position + self.plate_release_offset

    def _track_tool(self, position, rotation, joints, scene, label):
        state = scene["objects"]["tool"]
        tool_goal = rotation @ plan.TOOL_PINCH.T
        hand, hand_rotation = ik.tcp_pose(joints[7:13], config.BASES["right"])
        if label.startswith("Contour:"):
            work = np.asarray(scene["objects"]["plate"]["pose"])
            wp = quat_xyzw_to_matrix(work[3:])
            tip = position - tool_goal @ (config.TOOL_GRASP - config.KNIFE)
            local = plan.PART_YAW.T @ (tip - config.PLATE_START)
            target = work[:3] + wp @ local
            blade = np.asarray(state["links"]["Knife"])
            nose = blade[:3] + quat_xyzw_to_matrix(blade[3:]) @ config.TOE
            # Filter the live nose-to-hand translation because the blade swivels
            # and the tool settles slightly in the nylon grip under load.
            measured_offset = hand - nose
            if self.nose_to_hand is None:
                self.nose_to_hand = measured_offset
            else:
                self.nose_to_hand += .22 * (measured_offset - self.nose_to_hand)
            if label.startswith(("Contour: engage", "Contour: draw")) and self.mode != "air-pass":
                flex = state["joints"]["Flex"]
                load = max(flex, 0.0) * config.FLEX_K
                offset = wp.T @ (nose - work[:3])
                relative_xy = offset[:2] - config.PORT_CENTER
                measured_radius = np.hypot(
                    max(abs(relative_xy[0]) - config.CAPSULE_OFFSET, 0.0), relative_xy[1]
                )
                target_xy = local[:2] - config.PORT_CENTER
                outward = np.array([
                    np.sign(target_xy[0]) * max(abs(target_xy[0]) - config.CAPSULE_OFFSET, 0.0),
                    target_xy[1],
                ])
                outward /= np.linalg.norm(outward)
                lateral_error = np.linalg.norm(offset[:2] - local[:2])
                height_error = (nose - target) @ wp[:, 2]
                near_lip = lateral_error < .005
                seated = abs(height_error) < .006
                if lateral_error < .010 and seated:
                    error = config.CAPSULE_RADIUS + .0014 - measured_radius
                    self.radial_bias = np.clip(self.radial_bias + .12 * error, -.005, .005)
                target += wp[:, :2] @ outward * self.radial_bias
                if near_lip and seated:
                    self.pressure_bias = np.clip(
                        self.pressure_bias + .00011 * (load - 4.6), -.010, .005
                    )
                elif label.startswith("Contour: engage"):
                    self.pressure_bias *= .94
                normal = self.pressure_bias - .0020
                # The finishing cuts approach from above. Reseat a dropped toe
                # only when it is below the physical lip, after the approach.
                lip_error = offset[2] - config.PLATE_THICKNESS / 2
                dropped = lip_error if label.endswith("finishing cut") else height_error
                if dropped < -.0025 and load < 1.0:
                    normal = .008
                    self.pressure_bias *= .86
                target += wp[:, 2] * normal
            elif label.startswith("Contour: withdraw"):
                self.radial_bias = 0.0
                self.pressure_bias = 0.0
            return target + self.nose_to_hand, rotation
        tool = np.asarray(state["pose"])
        adjustment = tool_goal @ quat_xyzw_to_matrix(tool[3:]).T
        angle = np.arccos(np.clip((np.trace(adjustment) - 1.0) / 2.0, -1.0, 1.0))
        if angle > .055:
            adjustment = ik.slerp_matrix(np.eye(3), adjustment, .055 / angle)
        target = position - tool_goal @ config.TOOL_GRASP
        desired = target + adjustment @ (hand - tool[:3])
        return hand + np.clip(desired - hand, -.025, .025), adjustment @ hand_rotation

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
