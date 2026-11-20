"""Native reference using elapsed time and ordinary joint positions."""

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
        self.cuff_pose = None
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def action_at(self, t, joint_pos=None, scene_state=None):
        out = []
        labels = []
        for name in ("left", "right"):
            pos, rot, grip = self.starts[name]
            age = t
            label = ""
            s = 1.0
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
            if name == "left" and scene_state is not None:
                if label.startswith("Set open fingers around the folded"):
                    pos, rot = self._track_cuff(pos, rot, scene_state)
                    self.cuff_pose = (pos.copy(), rot.copy())
                elif self.cuff_pose is not None and label.startswith((
                    "Support the cuff", "Bring the spatula", "Center the blade",
                    "Load the blade", "Settle the loaded blade", "Raise the blade", "Pinch the cuff",
                    "Withdraw the blade", "Release the cuff",
                )):
                    pos, rot = self.cuff_pose
            nominal = pos
            if name == "right" and scene_state is not None and self.mode != "no-insertion":
                depth = None
                if label.startswith("Center the blade"):
                    depth = .065 - .030*s
                elif label.startswith("Load the blade"):
                    depth = .035 - .127*s
                elif label.startswith("Settle the loaded blade"):
                    depth = -.092
                elif label.startswith("Raise the blade"):
                    depth = -.092 + .027*s
                elif label.startswith("Pinch the cuff"):
                    depth = -.065
                elif label.startswith("Withdraw the blade"):
                    depth = -.065 + .090*s
                if depth is not None and joint_pos is not None:
                    adjusted = self._guide_blade(scene_state, joint_pos[7:13], rot, depth)
                    weight = s if label.startswith("Center the blade") else 1.0
                    shift = np.clip(adjusted - pos, -.052, .052)
                    pos = pos + weight * shift
            orientation = ik.matrix_to_quat_xyzw(rot)
            fractions = (1.0, .8, .6, .4, 0.0) if pos is not nominal else (1.0,)
            for fraction in fractions:
                target = nominal + fraction * (pos - nominal)
                q, pe, re = ik.solve(
                    target, orientation, self.q[name], base_pos=config.BASES[name], iters=60,
                )
                if pe <= .006 and re <= .06:
                    break
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
            target = target + self.integral + 0.40 * error
        target = np.clip(
            target, self.info.action_space.low, self.info.action_space.high
        )
        # Also pass Inspect's default delta guard after this taught command ramp.
        result = self.previous + np.clip(target - self.previous, -0.04, 0.04)
        self.previous = result
        self.phase = " / ".join(labels) or "settle"
        return result

    @staticmethod
    def _cuff_frame(scene):
        links = scene["objects"]["bag"]["links"]
        mouth = (np.asarray(links["Near"][:3]) + np.asarray(links["Far"][:3])) / 2
        bag_rotation = quat_xyzw_to_matrix(links["Bagshell"][3:])
        return mouth, bag_rotation

    @staticmethod
    def _guide_blade(scene, measured_q, target_rotation, depth):
        mouth, bag_rotation = ScriptedPolicy._cuff_frame(scene)
        blade = scene["objects"]["spatula"]["links"]["Blade"]
        blade_origin, blade_rotation = np.asarray(blade[:3]), quat_xyzw_to_matrix(blade[3:])
        tcp_pos, tcp_rotation = ik.tcp_pose(measured_q, config.BASES["right"])
        change = target_rotation @ tcp_rotation.T
        axis = change @ blade_rotation[:, 0]
        into_bag = np.clip(axis @ bag_rotation[:, 2], -1., -.75)
        tip = mouth + bag_rotation @ [-.013, -.003, .012] + axis * ((depth-.012)/into_bag)
        actual_tip = blade_origin + blade_rotation @ [config.BLADE_X1, 0., config.BLADE_Z]
        return tip + change @ (tcp_pos - actual_tip)

    @staticmethod
    def _track_cuff(position, rotation, scene):
        links = scene["objects"]["bag"]["links"]
        near = np.asarray(links["Near"][:3])
        far = np.asarray(links["Far"][:3])
        shell_rotation = quat_xyzw_to_matrix(np.asarray(links["Bagshell"][3:]))
        nominal = config.BAG_ROTATION.T @ (position - config.BAG)
        height = float(np.clip(nominal[2], .024, .051))
        offset_x = max(float(nominal[0]), .035)
        target = (near+far) / 2 + shell_rotation @ [offset_x, 0., height-.008]
        taken = target - position
        distance = np.linalg.norm(taken)
        if distance > .052:
            taken *= .052 / distance
        physical_orientation = plan.pinch(-shell_rotation[:, 1], shell_rotation @ plan.CUFF_APPROACH)
        # Limit the transient wrist correction while the tool is adjacent.
        return position + taken, ik.slerp_matrix(rotation, physical_orientation, .60)

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
