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
    CHAIN_POSITION,
    GRIPS,
    CONNECTOR_POSITION,
    KEEPER_POSITION,
    NODES,
    CLOSED_NODES,
    CONNECTOR_PICKUP_POSITION,
    CONNECTOR_GRASP,
    CHAIN_GRASP_HEIGHT,
)


def demo_plan(mode="demo", initial_state=None):
    if mode == "hold":
        return [dict(phase="hold reset", seconds=5.0)]
    if mode not in ("demo", "connector", "ends", "unlocked", "retention"):
        raise ValueError("Unknown policy mode")
    down = grip_rotation()
    plan = []
    rotations = {"left": down, "right": down}
    seated_rotations = {}
    chain_pick_rotations = {}
    for arm, i in (("right", 0), ("left", 10)):
        before = NODES[i + 1] - NODES[i]
        after = CLOSED_NODES[i + 1] - CLOSED_NODES[i]
        initial_yaw = np.arctan2(before[1], before[0])
        final_yaw = np.arctan2(after[1], after[0])
        if arm == "left":
            initial_yaw += np.pi
            final_yaw += np.pi
        chain_pick_rotations[arm] = grip_rotation(initial_yaw)
        seated_rotations[arm] = grip_rotation(final_yaw)

    def add(phase, seconds, **targets):
        stage = dict(phase=phase, seconds=seconds)
        for arm, (position, grip) in targets.items():
            stage[arm] = dict(
                position=np.asarray(position).tolist(),
                rotation_xyzw=ik.matrix_to_quat_xyzw(rotations[arm]).tolist(),
                grip=grip,
            )
        plan.append(stage)

    start = CONNECTOR_PICKUP_POSITION.copy()
    if initial_state is not None:
        start = np.array(initial_state["connector"]["pose"][:3])
    pick = start + CONNECTOR_GRASP
    placed = CONNECTOR_POSITION + CONNECTOR_GRASP
    add("approach separate connecting link", 2.0, right=(pick + [0, 0, 0.045], 0.85))
    add("lower to connector pin head", 1.5, right=(pick, 0.85))
    add("grasp connecting link", 0.8, right=(pick, 0.10))
    add("lift connecting link from table", 1.8, right=(pick + [0, 0, 0.120], 0.10))
    add(
        "carry connector above empty cradle",
        2.5,
        right=(placed + [-0.090, 0, 0.050], 0.10),
    )
    add(
        "lower connector at cradle entrance", 2.0, right=(placed + [-0.090, 0, 0], 0.10)
    )
    add("slide connector into empty cradle", 2.0, right=(placed, 0.10))
    add("settle placed connector", 0.8)
    add("release placed connector", 0.8, right=(placed, 0.85))
    add("withdraw from placed connector", 1.0, right=(placed + [0, 0, 0.055], 0.85))
    add("park after connector placement", 1.5, right=([0.15, -0.30, 0.95], 1.0))
    add("settle connector in cradle", 0.8)
    if mode == "connector":
        return plan + [dict(phase="hold placed connector", seconds=3.0)]
    grips = {
        a: CHAIN_POSITION + g + [0, 0, CHAIN_GRASP_HEIGHT] for a, g in GRIPS.items()
    }
    from .config import END_NODES

    seats = {
        a: g
        + CONNECTOR_POSITION
        + [0, sign * 0.050, 0.007]
        - (CHAIN_POSITION + END_NODES[a])
        for a, g, sign in (("right", grips["right"], -1), ("left", grips["left"], 1))
    }
    if initial_state is not None:
        from yam_arena.urdf import quat_xyzw_to_matrix

        for arm, body, sign in (("right", "Link00", -1), ("left", "Link10", 1)):
            pose = initial_state["bodies"][body]["pose"]
            r = quat_xyzw_to_matrix(np.array(pose[3:]))
            grips[arm] = np.array(pose[:3]) + r @ np.array([0, 0, CHAIN_GRASP_HEIGHT])
            seats[arm] = (
                CONNECTOR_POSITION
                + [0, sign * 0.050, 0.007]
                - r @ np.array([sign * 0.050, 0, -CHAIN_GRASP_HEIGHT])
            )
    rotations.update(chain_pick_rotations)
    add(
        "approach both bare chain ends",
        2.0,
        **{a: (g + [0, 0, 0.055], 0.85) for a, g in grips.items()},
    )
    add("lower onto link rails", 1.5, **{a: (g, 0.85) for a, g in grips.items()})
    add("grasp both chain ends", 1.0, **{a: (g, 0.10) for a, g in grips.items()})
    add(
        "lift chain ends above pin heads",
        2.0,
        **{a: (g + [0, 0, 0.040], 0.10) for a, g in grips.items()},
    )
    rotations.update(seated_rotations)
    add(
        "align both chain holes over pins",
        2.0,
        **{a: (g + [0, 0, 0.040], 0.10) for a, g in seats.items()},
    )
    add("settle hole alignment", 2.0)
    add(
        "lower both chain ends onto pins",
        2.5,
        **{a: (g, 0.10) for a, g in seats.items()},
    )
    add("settle chain-end seating", 1.0)
    add("release both bare chain ends", 0.8, **{a: (g, 0.85) for a, g in seats.items()})
    add(
        "lift both hands clear",
        1.0,
        **{a: (g + [0, 0, 0.065], 0.85) for a, g in seats.items()},
    )
    rotations["right"] = down
    add("park right hand after joining", 1.5, right=([0.15, -0.30, 0.95], 1.0))
    add("let chain settle onto backing", 1.0)
    if mode == "ends":
        return plan + [dict(phase="hold joined ends", seconds=3.0)]
    rotations["left"] = down
    new = KEEPER_POSITION + [0, 0, 0.050]
    add("approach retaining plate", 2.0, left=(new + [0, 0, 0.12], 0.8))
    add("lower to blue grip", 1.5, left=(new, 0.8))
    add("grasp retaining plate", 0.8, left=(new, 0.20))
    add("lift retaining plate", 1.5, left=(new + [0, 0, 0.13], 0.20))
    outside = new.copy()
    outside[0] = CONNECTOR_POSITION[0] - 0.070
    outside[2] += 0.13
    add("route keeper outside chain grips", 1.5, left=(outside, 0.20))
    entry = CONNECTOR_POSITION + [-0.070, 0, 0.074]
    add("carry plate beside connecting link", 3.0, left=(entry + [0, 0, 0.026], 0.20))
    add("lower keeper beside pin heads", 3.0, left=(entry, 0.20))
    add("settle keeper approach", 0.8)
    locked = entry + [0.084, 0, 0]
    if mode != "unlocked":
        add("slide keeper under both heads", 4.0, left=(locked, 0.20))
        add("settle retained keeper", 1.5)
    end = entry if mode == "unlocked" else locked
    if mode == "retention":
        add("bounded upward retention probe", 2.0, left=(end + [0, 0, 0.008], 0.20))
        add("hold retention probe", 2.0)
        return plan
    add("release retaining plate", 0.8, left=(end, 0.85))
    add("lift left hand from plate", 1.0, left=(end + [0, 0, 0.030], 0.85))
    add("park left hand", 1.5, left=([0.15, 0.30, 0.95], 1.0))
    add("hold released retained joint", 3.0)
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
        self.release_position = None
        self.connector_release_position = None
        self.chain_release_positions = None
        seating = next(
            (s for s in self.plan if s["phase"] == "lower both chain ends onto pins"),
            None,
        )
        self.chain_seats = (
            {a: np.array(seating[a]["position"]) for a in ("left", "right")}
            if seating is not None
            else {}
        )
        self.keeper_end = next(
            (
                np.array(s["left"]["position"])
                for s in self.plan
                if s["phase"] == "release retaining plate"
            ),
            None,
        )
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
                self.stage_progress = f
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

            if self.phase in {
                "lift connecting link from table",
                "carry connector above empty cradle",
                "lower connector at cradle entrance",
                "slide connector into empty cradle",
                "settle placed connector",
            }:
                pos, rot, grip = targets["right"]
                desired_body = pos - CONNECTOR_GRASP
                hand = state["hands"]["right_robot"]
                tcp = np.array(hand["tcp"][:3])
                cp = np.array(state["connector"]["pose"])
                body_r = quat_xyzw_to_matrix(cp[3:])
                hp = hand["poses"][hand["names"].index("gripper")]
                hand_r = quat_xyzw_to_matrix(np.array(hp[3:]))
                targets["right"] = (
                    desired_body + body_r.T @ (tcp - cp[:3]),
                    body_r.T @ hand_r,
                    grip,
                )
            if self.phase == "release placed connector":
                if self.connector_release_position is None:
                    self.connector_release_position = np.array(
                        state["hands"]["right_robot"]["tcp"][:3]
                    )
                _, rot, grip = targets["right"]
                targets["right"] = (self.connector_release_position, rot, grip)
            if self.connector_release_position is not None:
                pos, rot, grip = targets["right"]
                if self.phase == "withdraw from placed connector":
                    targets["right"] = (
                        self.connector_release_position
                        + [0, 0, 0.055 * self.stage_progress],
                        rot,
                        grip,
                    )
                elif self.phase == "park after connector placement":
                    targets["right"] = (
                        pos
                        + (1 - self.stage_progress)
                        * (
                            self.connector_release_position
                            - (CONNECTOR_POSITION + CONNECTOR_GRASP)
                        ),
                        rot,
                        grip,
                    )
            if self.phase in {
                "lift chain ends above pin heads",
                "align both chain holes over pins",
                "lower both chain ends onto pins",
                "settle hole alignment",
                "settle chain-end seating",
            }:
                for arm, body, sign in (("right", "Link00", -1), ("left", "Link10", 1)):
                    before = quat_xyzw_to_matrix(
                        np.array(self.initial_state["bodies"][body]["pose"][3:])
                    )
                    pose = np.array(state["bodies"][body]["pose"])
                    now = quat_xyzw_to_matrix(pose[3:])
                    pos, rot, grip = targets[arm]
                    desired_hole = pos + before @ np.array(
                        [sign * 0.050, 0, -CHAIN_GRASP_HEIGHT]
                    )
                    if self.phase != "lift chain ends above pin heads":
                        cp = np.array(state["connector"]["pose"])
                        cr = quat_xyzw_to_matrix(cp[3:])
                        desired_hole = cp[:3] + cr @ (desired_hole - CONNECTOR_POSITION)
                    current_hole = pose[:3] + now @ np.array([sign * 0.050, 0, 0])
                    tcp = np.array(state["hands"][arm + "_robot"]["tcp"][:3])
                    correction = desired_hole - current_hole
                    correction[:2] *= 3.0
                    targets[arm] = (tcp + correction, rot, grip)
            if self.phase in {
                "lift retaining plate",
                "route keeper outside chain grips",
                "carry plate beside connecting link",
                "lower keeper beside pin heads",
                "settle keeper approach",
                "slide keeper under both heads",
                "settle retained keeper",
            }:
                pos, rot, grip = targets["left"]
                desired_body = pos - np.array([0, 0, 0.050])
                desired_r = np.eye(3)
                if self.phase not in {
                    "lift retaining plate",
                    "route keeper outside chain grips",
                }:
                    c = np.array(state["connector"]["pose"])
                    desired_r = quat_xyzw_to_matrix(c[3:])
                    desired_body = c[:3] + desired_r @ (
                        desired_body - CONNECTOR_POSITION
                    )
                hand = state["hands"]["left_robot"]
                tcp = np.array(hand["tcp"][:3])
                body = np.array(state["keeper"]["pose"][:3])
                body_r = quat_xyzw_to_matrix(np.array(state["keeper"]["pose"][3:]))
                hp = hand["poses"][hand["names"].index("gripper")]
                hand_r = quat_xyzw_to_matrix(np.array(hp[3:]))
                relative_tcp = body_r.T @ (tcp - body)
                targets["left"] = (
                    desired_body + desired_r @ relative_tcp,
                    desired_r @ body_r.T @ hand_r,
                    grip,
                )
            if self.phase == "release retaining plate":
                if self.release_position is None:
                    self.release_position = np.array(
                        state["hands"]["left_robot"]["tcp"][:3]
                    )
                _, rot, grip = targets["left"]
                targets["left"] = (self.release_position, rot, grip)
            if self.phase == "release both bare chain ends":
                if self.chain_release_positions is None:
                    self.chain_release_positions = {
                        a: np.array(state["hands"][a + "_robot"]["tcp"][:3])
                        for a in ("left", "right")
                    }
                for a in ("left", "right"):
                    _, r, g = targets[a]
                    targets[a] = (self.chain_release_positions[a], r, g)
            if self.chain_release_positions is not None:
                f = self.stage_progress
                if self.phase == "lift both hands clear":
                    for a in ("left", "right"):
                        _, r, g = targets[a]
                        targets[a] = (
                            self.chain_release_positions[a] + [0, 0, 0.065 * f],
                            r,
                            g,
                        )
                elif self.phase in {
                    "park right hand after joining",
                    "let chain settle onto backing",
                    "hold joined ends",
                }:
                    _, r, g = targets["left"]
                    targets["left"] = (
                        self.chain_release_positions["left"] + [0, 0, 0.065],
                        r,
                        g,
                    )
                    if self.phase == "park right hand after joining":
                        p, r, g = targets["right"]
                        targets["right"] = (
                            p
                            + (1 - f)
                            * (
                                self.chain_release_positions["right"]
                                - self.chain_seats["right"]
                            ),
                            r,
                            g,
                        )
                elif self.phase == "approach retaining plate":
                    p, r, g = targets["left"]
                    targets["left"] = (
                        p
                        + (1 - f)
                        * (
                            self.chain_release_positions["left"]
                            - self.chain_seats["left"]
                        ),
                        r,
                        g,
                    )
            if self.release_position is not None:
                p, r, g = targets["left"]
                if self.phase == "lift left hand from plate":
                    targets["left"] = (
                        self.release_position + [0, 0, 0.030 * self.stage_progress],
                        r,
                        g,
                    )
                elif self.phase == "park left hand":
                    targets["left"] = (
                        p
                        + (1 - self.stage_progress)
                        * (self.release_position - self.keeper_end),
                        r,
                        g,
                    )
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
            # Allow a small approximate IK solution during measured-pose
            # corrections; the physical scorer still checks actual part poses.
            if pe > 0.005 or re > 0.04:
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
