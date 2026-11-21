"""Native reference using elapsed time and ordinary joint positions."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.constants import READY

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
        self.grasp_offset = None
        self.pressure_adjustment = 0.
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def _adjust_wax_height(self, label, position, joints, state):
        paper = np.asarray(state["objects"]["paper"]["pose"])
        wax = np.asarray(state["objects"]["crayon"]["pose"])
        if self.grasp_offset is None:
            point, _ = ik.tcp_pose(joints[7:13], config.BASES["right"])
            if np.linalg.norm(point - wax[:3]) < .034:
                self.grasp_offset = np.clip(point - wax[:3], [-.012, -.012, .005],
                                            [.012, .012, .019])
        desired = position.copy()
        desired[2] += np.clip(paper[2] - (config.PAPER_Z - config.PAPER_THICKNESS / 2),
                              -.007, .007)
        if self.grasp_offset is not None:
            desired[:2] += self.grasp_offset[:2]
            desired[2] += self.grasp_offset[2] - (config.TCP_FROM_WAX_BASE - config.CRAYON_SIZE[2] / 2)
        if self.mode != "air-pass":
            if (label.startswith("Rub across")
                    or label.startswith("Lower the broad") and position[2] < plan.WAX_STROKE_Z + .003):
                error = np.clip(state["wax_load_n"] - .96, -.80, 1.65)
                self.pressure_adjustment = float(np.clip(
                    self.pressure_adjustment + .00063 * error, -.006, .009,
                ))
            desired[2] += self.pressure_adjustment
        return desired

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
            if (name == "right" and scene_state is not None and joint_pos is not None
                    and label.startswith(("Lower the broad", "Rub across", "Lift the wax"))):
                pos = self._adjust_wax_height(label, pos, joint_pos, scene_state)
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
            if labels and labels[0].startswith("Unpin and lift"):
                self.integral[:6] = 0.
            error = target - np.asarray(joint_pos)
            error[[6, 13]] = 0.0
            self.integral = np.clip(self.integral + .75 * .05 * error, -.07, .07)
            target = target + self.integral + .24 * error
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
