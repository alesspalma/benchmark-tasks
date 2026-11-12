"""Fixed waypoints converted to ordinary fourteen-dimensional YAM targets only."""

from __future__ import annotations
import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik, station
from yam_arena.constants import READY
from yam_arena.demo import grip_rotation
from .config import (
    CONNECTOR_POSITION,
    CONNECTOR_TARGET,
    KEEPER_POSITION,
    CONNECTOR_GRASP,
    KEEPER_GRASP,
    KEEPER_INSERTION,
)


def demo_plan(mode="demo", initial_state=None):
    if mode == "hold":
        return [dict(phase="hold reset", seconds=5.0)]
    if mode not in ("demo", "ends", "unlocked"):
        raise ValueError("Unknown mode")
    down = grip_rotation()
    plan = []

    def add(phase, seconds, **targets):
        stage = dict(phase=phase, seconds=seconds)
        for arm, (p, g) in targets.items():
            stage[arm] = dict(
                position=np.asarray(p).tolist(),
                rotation_xyzw=ik.matrix_to_quat_xyzw(down).tolist(),
                grip=g,
            )
        plan.append(stage)

    start = CONNECTOR_POSITION.copy()
    blue = KEEPER_POSITION.copy()
    if initial_state is not None:
        start = np.array(initial_state["connector"]["pose"][:3])
        blue = np.array(initial_state["keeper"]["pose"][:3])
    pick = start + CONNECTOR_GRASP
    seat = CONNECTOR_TARGET + CONNECTOR_GRASP
    add("approach connecting link", 2.0, right=(pick + [0, 0, 0.045], 0.85))
    add("lower to orange handle", 1.5, right=(pick, 0.85))
    add("grasp connecting link", 0.8, right=(pick, 0.20))
    add("lift connecting link", 1.8, right=(pick + [0, 0, 0.057], 0.20))
    add("carry connector above chain ends", 3.0, right=(seat + [0, 0, 0.057], 0.20))
    add("lower connector through both holes", 2.5, right=(seat, 0.20))
    add("hold connecting link", 1.0)
    add("release connecting link", 0.8, right=(seat, 0.85))
    add("withdraw right hand", 1.0, right=(seat + [0, 0, 0.045], 0.85))
    add("park right hand", 1.5, right=([0.15, -0.30, 0.95], 1.0))
    if mode == "ends":
        return plan + [dict(phase="hold connector check", seconds=3.0)]
    new = blue + KEEPER_GRASP
    add("approach retaining plate", 2.5, left=(new + [0, 0, 0.030], 0.85))
    add("lower to blue handle", 1.5, left=(new, 0.85))
    add("grasp retaining plate", 0.8, left=(new, 0.20))
    add("lift retaining plate", 1.8, left=(new + [0, 0, 0.030], 0.20))
    outside = new.copy()
    outside[0] = CONNECTOR_TARGET[0] - 0.075
    outside[2] += 0.030
    add("route keeper outside sprocket", 1.5, left=(outside, 0.20))
    entry = CONNECTOR_TARGET + [-0.075, 0, KEEPER_INSERTION[2]] + KEEPER_GRASP
    add("carry keeper beside joint", 3.0, left=(entry + [0, 0, 0.040], 0.20))
    add("lower keeper beside pins", 2.5, left=(entry, 0.20))
    add("settle keeper approach", 0.7)
    locked = CONNECTOR_TARGET + KEEPER_INSERTION + KEEPER_GRASP
    if mode != "unlocked":
        add("slide keeper into joint", 4.0, left=(locked, 0.20))
        add("settle retained keeper", 1.0)
    end = entry if mode == "unlocked" else locked
    add("release retaining plate", 0.8, left=(end, 0.85))
    add("withdraw left hand", 1.0, left=(end + [0, 0, 0.040], 0.85))
    add("park left hand", 1.5, left=([0.15, 0.30, 0.95], 1.0))
    add("hold closed routed loop", 3.0)
    return plan


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo", initial_state=None):
        self.mode = mode
        self.initial_state = initial_state
        self.plan = demo_plan(mode, initial_state)
        self.environment_id = None
        low = np.r_[ik.LOWER, 0.0, ik.LOWER, 0.0]
        high = np.r_[ik.UPPER, 1.0, ik.UPPER, 1.0]
        self.info = PolicyInfo(
            name="chain_link_scripted", action_space=action_box(low, high)
        )
        self.dt = 0.05
        self.phase = "reset"
        self.worst_position_error = 0.0
        self.worst_rotation_error = 0.0

    def bind(self, embodiment_info):
        self.dt = 1.0 / float(embodiment_info.control_hz)
        self.environment_id = embodiment_info.environment_id

    def reset(self, scene):
        self._scene = scene
        self.q = np.r_[READY, 1.0, READY, 1.0].astype(float)
        self.worst_position_error = self.worst_rotation_error = 0.0
        self.keeper_integral = np.zeros(3)
        self.release_positions = {}
        initial = {}
        for arm, offset in (("left", 0), ("right", 7)):
            pos, rot = ik.tcp_pose(self.q[offset : offset + 6], station.BASES[arm])
            initial[arm] = (pos, rot, 1.0)
        self.stages = []
        time = 0.0
        for stage in self.plan:
            start = {arm: (p.copy(), r.copy(), g) for arm, (p, r, g) in initial.items()}
            for arm in ("left", "right"):
                if arm in stage:
                    goal = stage[arm]
                    from yam_arena.urdf import quat_xyzw_to_matrix

                    initial[arm] = (
                        np.array(goal["position"]),
                        quat_xyzw_to_matrix(np.array(goal["rotation_xyzw"])),
                        goal["grip"],
                    )
            self.stages.append(
                (time, time + stage["seconds"], stage["phase"], start, initial.copy())
            )
            time += stage["seconds"]
        self.duration = time

    def targets(self, time):
        for begin, end, phase, starts, goals in self.stages:
            if time <= end:
                f = ik.smoothstep(np.clip((time - begin) / (end - begin), 0, 1))
                result = {}
                for arm in ("left", "right"):
                    p0, r0, g0 = starts[arm]
                    p1, r1, g1 = goals[arm]
                    result[arm] = (
                        p0 + f * (p1 - p0),
                        ik.slerp_matrix(r0, r1, f),
                        g0 + f * (g1 - g0),
                    )
                return phase, result
        return "finished", self.stages[-1][4]

    def action_at(self, time, state=None):
        self.phase, targets = self.targets(time)
        if state is not None and self.initial_state is not None:
            from yam_arena.urdf import quat_xyzw_to_matrix

            connector_phases = {
                "lift connecting link",
                "carry connector above chain ends",
                "lower connector through both holes",
                "hold connecting link",
            }
            keeper_phases = {
                "lift retaining plate",
                "route keeper outside sprocket",
                "carry keeper beside joint",
                "lower keeper beside pins",
                "settle keeper approach",
                "slide keeper into joint",
                "settle retained keeper",
            }
            for arm, name, grasp, active in [
                ("right", "connector", CONNECTOR_GRASP, self.phase in connector_phases),
                ("left", "keeper", KEEPER_GRASP, self.phase in keeper_phases),
            ]:
                if not active:
                    continue
                pos, rot, grip = targets[arm]
                hand = state["hands"][arm + "_robot"]
                tcp = np.array(hand["tcp"][:3])
                pose = state[name]["pose"]
                body = np.array(pose[:3])
                body_r = quat_xyzw_to_matrix(np.array(pose[3:]))
                gripper = hand["poses"][hand["names"].index("gripper")]
                hand_r = quat_xyzw_to_matrix(np.array(gripper[3:]))
                desired = pos - grasp
                desired_r = np.eye(3)
                align_keeper = {
                    "carry keeper beside joint",
                    "lower keeper beside pins",
                    "settle keeper approach",
                    "slide keeper into joint",
                    "settle retained keeper",
                    "release connecting link",
                    "withdraw right hand",
                    "park right hand",
                }
                if name == "keeper" and self.phase in align_keeper:
                    cp = state["connector"]["pose"]
                    desired_r = quat_xyzw_to_matrix(np.array(cp[3:]))
                    desired = np.array(cp[:3]) + desired_r @ (
                        desired - CONNECTOR_TARGET
                    )
                relative_tcp = body_r.T @ (tcp - body)
                targets[arm] = (
                    desired + desired_r @ relative_tcp,
                    desired_r @ body_r.T @ hand_r,
                    grip,
                )
            for phase, arm in [
                ("release connecting link", "right"),
                ("release retaining plate", "left"),
            ]:
                if self.phase == phase:
                    if arm not in self.release_positions:
                        self.release_positions[arm] = np.array(
                            state["hands"][arm + "_robot"]["tcp"][:3]
                        )
                    _, rot, grip = targets[arm]
                    targets[arm] = (self.release_positions[arm], rot, grip)
        for arm, offset in (("left", 0), ("right", 7)):
            pos, rot, grip = targets[arm]
            q, pe, re = ik.solve(
                pos,
                ik.matrix_to_quat_xyzw(rot),
                self.q[offset : offset + 6],
                base_pos=station.BASES[arm],
                iters=100,
            )
            self.worst_position_error = max(self.worst_position_error, pe)
            self.worst_rotation_error = max(self.worst_rotation_error, re)
            if pe > 0.003 or re > 0.04:
                raise RuntimeError(
                    f"Unreachable {arm} target during {self.phase}: {pe:.5f} m, {re:.5f} rad"
                )
            self.q[offset : offset + 6], self.q[offset + 6] = q, grip
        return self.q.copy()

    def act(self, observation):
        from .environment import LIVE_ENVIRONMENTS

        reference = LIVE_ENVIRONMENTS.get(self.environment_id)
        state = reference.diagnostics() if reference is not None else None
        if self.initial_state is None and state is not None:
            self.initial_state = state
            self.plan = demo_plan(self.mode, state)
            self.reset(self._scene)
        time = float(observation.state_time) + self.dt
        action = self.action_at(time, state=state)
        meta = {"request_stop": True} if time > self.duration + self.dt else {}
        return ActionChunk(
            actions=[Action(data=action, meta=meta)], control_hz=1 / self.dt
        )


def scripted_policy(**kwargs):
    return ScriptedPolicy(**kwargs)
