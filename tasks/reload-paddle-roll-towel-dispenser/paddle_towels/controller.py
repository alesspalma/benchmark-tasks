"""Physical reference commands using time, joint positions."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.constants import READY

from . import config, plan
from .scoring import tip_from


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
        self.cover_release = None
        self.leader_offsets = None
        self.leader_release = None
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def action_at(self, t, joint_pos=None, state=None):
        if (
            self.mode == "demo"
            and self.cover_release is None
            and state is not None
            and state["objects"]["mechanism"]["joints"]["Hinge"] >= -0.704
        ):
            # Withdraw as soon as the cover passes the real spring cam.
            start = self.q["right"].copy()
            park = np.array([0.245, -0.249, 0.940])
            end, _, _ = ik.solve(
                park,
                ik.matrix_to_quat_xyzw(plan.PRESS),
                start,
                base_pos=config.BASES["right"],
                iters=100,
            )
            self.cover_release = (t, start, end)
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
            # Maintain contact on the visible end face while the roll moves
            # off the cart. Keep the advance shallow rather than sweeping
            # diagonally off the round front edge if the roll initially sticks.
            if (
                name == "left"
                and state is not None
                and label
                == "Slide the roll off the service cart and into the empty cradle"
            ):
                centre = np.asarray(state["objects"]["roll"]["pose"][:3])
                pos[0] = centre[0] - 0.034
                pos[1] = max(pos[1], centre[1] + 0.037)
                pos[2] = centre[2] - 0.010
            if name == "left" and state is not None:
                if (
                    self.leader_offsets is None
                    and label == "Tuck the free paper edge into the roller mouth"
                ):
                    roll = state["objects"]["roll"]
                    edge = tip_from(roll)
                    pivot = np.asarray(roll["links"]["PayOut"][:3])
                    depth = float(np.clip(config.NIP_X - pivot[0], 0.026, 0.086))
                    in_mouth = pivot[2] - np.sqrt(config.PAPER_LENGTH**2 - depth**2)
                    entry = np.array(
                        [edge[0] - 0.010, 0.020, max(0.820, edge[2] + 0.010)]
                    )
                    finish = np.array(
                        [config.NIP_X - 0.010, 0.020, max(0.813, in_mouth - 0.011)]
                    )
                    self.leader_offsets = (
                        entry - plan.TUCK_ENTRY,
                        finish - plan.TUCK_FINISH,
                    )
                if self.leader_offsets is not None:
                    entry, finish = self.leader_offsets
                    if label == "Tuck the free paper edge into the roller mouth":
                        pos += s * entry
                    elif label == "Hold the tucked paper while it engages":
                        pos += (1.0 - s) * entry + s * finish
                    elif label in (
                        "Push the paddle through its feed stroke",
                        "Let the rollers advance the new paper tail",
                        "Release the spring-returned paddle",
                    ):
                        pos += finish
                    elif label == "Withdraw the paper hand after threading":
                        pos += (1.0 - s) * finish
            if (
                name == "left"
                and state is not None
                and label
                in (
                    "Hold the tucked paper while it engages",
                    "Push the paddle through its feed stroke",
                )
                and not state["engaged"]
            ):
                edge = tip_from(state["objects"]["roll"])
                if edge[0] > config.NIP_X - 0.006:
                    hand_x = state["hands"]["left_robot"]["points"][0][0]
                    pos[0] = min(
                        pos[0], hand_x + np.clip(config.NIP_X - edge[0], -0.007, 0.003)
                    )
            if (
                name == "left"
                and state is not None
                and state["engaged"]
                and self.leader_release is None
            ):
                tcp = np.asarray(state["hands"]["left_robot"]["points"][0])
                self.leader_release = (t, tcp)
            if name == "left" and self.leader_release is not None:
                onset, tcp = self.leader_release
                gap = tcp - np.array([0.104, 0.0, 0.0])
                if t - onset < 0.80:
                    progress = ik.smoothstep((t - onset) / 0.80)
                    pos = tcp + progress * (gap - tcp)
                    grip = 0.20
                else:
                    progress = ik.smoothstep(
                        float(np.clip((t - onset - 0.80) / 1.17, 0.0, 1.0))
                    )
                    clear = np.array([0.224, 0.275, 1.066])
                    pos = gap + progress * (clear - gap)
                    grip = 0.23
                rot = plan.GUIDE
                labels.append("Withdraw straight from the captured paper")
            if name == "right" and self.cover_release is not None:
                started, start, end = self.cover_release
                fraction = ik.smoothstep(
                    float(np.clip((t - started - 0.93) / 1.32, 0.0, 1.0))
                )
                q = start + fraction * (end - start)
                self.q[name] = q
                out.extend([*q, 0.91])
                labels.append("Release the cover after passing the spring cam")
                continue
            q, pe, re = ik.solve(
                pos,
                ik.matrix_to_quat_xyzw(rot),
                self.q[name],
                base_pos=config.BASES[name],
                iters=60,
            )
            if pe > 0.025 or re > 0.15:
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
