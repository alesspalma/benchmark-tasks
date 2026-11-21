"""Command ordinary position actions from the taught station layout."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.constants import READY
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config, plan
from .detent import SEAT_IN_SWIVEL


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
        self.blade_start = None
        self.cap_grasp = None
        self.mallet_grasp = None
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    @staticmethod
    def _pose(data):
        pose = np.asarray(data["pose"])
        return pose[:3], quat_xyzw_to_matrix(pose[3:])

    def _steer(self, label, progress, pos, rot, state):
        if not state:
            return pos, rot
        parts = state["objects"]
        hand = state["hands"]["right_robot"]
        axes = np.asarray(hand["axes"])
        tcp = np.asarray(hand["points"][0])
        inserting = label == "Seat the blade in the notch beneath the worn cap"
        prying = label == "Pry upward against the glide flange"
        levering = label == "Continue the leverage until the worn cap releases"
        if inserting or prying or levering:
            tool_p, tool_r = self._pose(parts["pry"])
            tip = tool_p + tool_r @ [0, 0.065, -0.002]
            if self.blade_start is None:
                old_p, old_r = self._pose(parts["old_cap"])
                self.blade_start = old_p + old_r @ [0, -0.0100, -0.0012]
            local_tip = axes.T @ (tip - tcp)
            actual_goal = self.blade_start.copy()
            if prying:
                actual_goal += [0, 0.003 * progress, 0.011 * progress]
            elif levering:
                actual_goal += [0, 0.003 + 0.002 * progress, 0.011 + 0.020 * progress]
            correction = np.clip(actual_goal - (pos + rot @ local_tip), -0.028, 0.028)
            return pos + (progress if inserting else 1.0) * correction, rot

        aligning = label == "Align the new cap around the swivel flange"
        seating = label == "Seat the cap with a light press"
        releasing = label == "Release the aligned cap for seating"
        if aligning or seating or releasing:
            cap_p, cap_r = self._pose(parts["new_cap"])
            if self.cap_grasp is None:
                self.cap_grasp = (axes.T @ (cap_p - tcp), axes.T @ cap_r)
            local_centre, local_axes = self.cap_grasp
            swivel = np.asarray(parts["chair"]["links"]["Swivel"])
            swivel_r = quat_xyzw_to_matrix(swivel[3:])
            # Keep the cap's current azimuth. Its circular skirt only needs
            # its normal aligned to the face of the freely pivoting glide.
            up = np.array([0.0, 0.0, 1.0])
            forward = (rot @ local_axes)[:, 0].copy()
            forward -= forward.dot(up) * up
            forward /= np.linalg.norm(forward)
            face = np.column_stack((forward, np.cross(up, forward), up))
            hand_goal = face @ local_axes.T
            high = (
                0.015 * (1.0 - progress) - 0.004
                if seating
                else (-0.004 + 0.012 * progress if releasing else 0.011)
            )
            side = -0.014 if self.mode == "loose-cap" else 0.0
            centre_goal = swivel[:3] + swivel_r @ (SEAT_IN_SWIVEL + [0, side, high])
            tcp_goal = centre_goal - hand_goal @ local_centre
            influence = progress if aligning else 1.0
            pos += influence * np.clip(tcp_goal - pos, -0.024, 0.024)
            return pos, ik.slerp_matrix(rot, hand_goal, influence)

        above = label == "Bring the rubber face over the new cap"
        press = label == "Press the rubber face against the cap until the rim clicks"
        lift = label == "Lift the mallet from the retained cap"
        if above or press or lift:
            mallet_p, mallet_r = self._pose(parts["mallet"])
            face_p = mallet_p + mallet_r @ [0, 0.061, -0.026]
            if self.mallet_grasp is None:
                self.mallet_grasp = (axes.T @ (face_p - tcp), axes.T @ mallet_r)
            local_face, local_axes = self.mallet_grasp
            up = np.array([0.0, 0.0, 1.0])
            handle = (rot @ local_axes)[:, 1].copy()
            handle -= handle.dot(up) * up
            handle /= np.linalg.norm(handle)
            mallet_axes = np.column_stack((np.cross(handle, up), handle, up))
            hand_goal = mallet_axes @ local_axes.T
            influence = progress if above else 1.0
            z_gap = 0.029
            if press:
                z_gap -= 0.027 * progress
            elif lift:
                z_gap -= 0.027 * (1.0 - progress)
            # The desired face is centred over the support, measured from the
            # housing and not from a candidate cap pose that could be off-axis.
            goal = config.FOOT + [0, 0, config.CAP_HEIGHT + 0.006 + z_gap]
            tcp_goal = goal - hand_goal @ local_face
            return (
                pos + influence * np.clip(tcp_goal - pos, -0.035, 0.035),
                ik.slerp_matrix(rot, hand_goal, influence),
            )
        return pos, rot

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
                    if name == "right":
                        pos, rot = self._steer(label, s, pos, rot, state)
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
            if pe > 0.018 or re > 0.14:
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
            self.integral = np.clip(self.integral + 0.60 * 0.05 * error, -0.055, 0.055)
            target = target + self.integral + 0.10 * error
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
