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
    CRANK_RADIUS, CRANK_GRASP_Z, SPROCKET_CENTERS,
)


def side_grip_rotation():
    t=np.pi/18
    rz=np.array([[np.cos(t),-np.sin(t),0],[np.sin(t),np.cos(t),0],[0,0,1]])
    return rz@np.array([[0,0,1],[0,-1,0],[1,0,0.]])


def crank_rotation():
    t=np.deg2rad(20)
    return np.array([[1,0,0],[0,np.cos(t),-np.sin(t)],[0,np.sin(t),np.cos(t)]])@grip_rotation()


def demo_plan(mode="demo", initial_state=None):
    if mode == "hold":
        return [dict(phase="hold reset",seconds=5.0)]
    if mode not in ("demo","assembly","ends","unlocked"):
        raise ValueError("Unknown mode")
    down, side = grip_rotation(),side_grip_rotation()
    plan=[]

    def add(phase,seconds,rotations=None,**targets):
        stage=dict(phase=phase,seconds=seconds)
        for arm,(p,g) in targets.items():
            rot=(rotations or {}).get(arm,down)
            stage[arm]=dict(position=np.asarray(p).tolist(),
                           rotation_xyzw=ik.matrix_to_quat_xyzw(rot).tolist(),grip=g)
        plan.append(stage)

    start,blue=CONNECTOR_POSITION.copy(),KEEPER_POSITION.copy()
    if initial_state is not None:
        start=np.array(initial_state["connector"]["pose"][:3])
        blue=np.array(initial_state["keeper"]["pose"][:3])
    pick,seat=start+CONNECTOR_GRASP,CONNECTOR_TARGET+CONNECTOR_GRASP
    rside={"right":side}
    approach=pick+[-0.050*np.cos(np.pi/18),-0.050*np.sin(np.pi/18),0.020]
    add("approach connecting link",2.0,rotations=rside,right=(approach,0.20))
    add("align with lower plate",1.0,rotations=rside,right=(approach-[0,0,.020],0.20))
    add("enter connector grip",1.5,rotations=rside,right=(pick,0.20))
    add("grasp connecting link",0.8,rotations=rside,right=(pick,0.02))
    add("lift connecting link",1.5,rotations=rside,right=(pick+[0,0,.040],0.02))
    beside=seat+[-.065,0,.065]
    add("carry connector beside chain",2.5,rotations=rside,right=(beside,0.02))
    under=seat+[0,0,-.040]
    add("lower connector beside chain",2.0,rotations=rside,right=(under+[-.065,0,0],0.02))
    add("carry connector under holes",2.0,rotations=rside,right=(under,0.02))
    add("raise connector through both holes",2.5,rotations=rside,right=(seat,0.02))
    add("hold connecting link",0.8)
    if mode == "ends":
        return plan+[dict(phase="hold connector check",seconds=3)]
    new=blue+KEEPER_GRASP
    add("approach retaining plate",2.0,left=(new+[0,0,.025],.85))
    add("lower to blue tab",1.0,left=(new,.85))
    add("grasp retaining plate",0.8,left=(new,.20))
    add("lift retaining plate",1.2,left=(new+[0,0,.025],.20))
    entry=CONNECTOR_TARGET+[-.065,0,KEEPER_INSERTION[2]]+KEEPER_GRASP
    add("carry keeper beside joint",2.0,left=(entry+[0,0,.025],.20))
    add("lower keeper beside pins",1.5,left=(entry,.20))
    locked=CONNECTOR_TARGET+KEEPER_INSERTION+KEEPER_GRASP
    if mode != "unlocked":
        add("slide keeper into joint",3.0,left=(locked,.20))
        add("settle retained keeper",0.8)
    end=entry if mode == "unlocked" else locked
    add("release connecting link",.6,rotations=rside,right=(seat,.07))
    add("withdraw right hand",1.2,rotations=rside,right=(seat+[-.065,-.012,0],.07))
    add("park right hand",1.5,right=([.15,-.30,.95],1.0))
    add("release retaining plate",.8,left=(end,.85))
    add("withdraw left hand",1.0,left=(end+[-.010,.025,.020],.85))
    add("park left hand",1.5,left=([.15,.30,.95],1.0))
    add("hold closed routed loop",1.5)
    if mode != "demo":
        return plan
    crank=np.r_[SPROCKET_CENTERS[0]+[CRANK_RADIUS,0],CRANK_GRASP_Z]
    angle=4*np.pi+.15
    final=np.r_[SPROCKET_CENTERS[0]+CRANK_RADIUS*np.array([np.cos(angle),np.sin(angle)]),CRANK_GRASP_Z]
    add("approach hand crank",1.5,rotations={"right":crank_rotation()},right=(crank+[0,0,.012],.85))
    add("lower to hand crank",1.2,rotations={"right":crank_rotation()},right=(crank,.85))
    add("grasp hand crank",.8,rotations={"right":crank_rotation()},right=(crank,.20))
    add("turn hand crank",30.0,rotations={"right":crank_rotation()},right=(final,.20))
    add("hold completed circuit",1.0,rotations={"right":crank_rotation()},right=(final,.20))
    add("release hand crank",.8,rotations={"right":crank_rotation()},right=(final,.85))
    add("withdraw from hand crank",1.0,rotations={"right":crank_rotation()},right=(final+[0,-.030,.012],.85))
    add("park after test",1.5,right=([.15,-.30,.95],1.0))
    add("hold tested repair",3.0)
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
        self.crank_start = None
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
                if phase == "turn hand crank":
                    theta=(4*np.pi+.15)*f
                    result["right"]=(np.r_[SPROCKET_CENTERS[0]+CRANK_RADIUS*np.array([np.cos(theta),np.sin(theta)]),CRANK_GRASP_Z],crank_rotation(),.20)
                return phase, result
        return "finished", self.stages[-1][4]

    def action_at(self, time, state=None):
        self.phase, targets = self.targets(time)
        if state is not None and self.initial_state is not None:
            from yam_arena.urdf import quat_xyzw_to_matrix

            connector_phases = {
                "lift connecting link","carry connector beside chain",
                "lower connector beside chain","carry connector under holes",
                "raise connector through both holes","hold connecting link",
                "approach retaining plate","lower to blue tab","grasp retaining plate",
                "lift retaining plate","carry keeper beside joint","lower keeper beside pins",
                "slide keeper into joint","settle retained keeper",
            }
            keeper_phases = {"lift retaining plate","carry keeper beside joint",
                "lower keeper beside pins","slide keeper into joint","settle retained keeper"}
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
                align_keeper = {"carry keeper beside joint","lower keeper beside pins",
                                "slide keeper into joint","settle retained keeper"}
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
                    if phase not in self.release_positions:
                        self.release_positions[phase] = np.array(
                            state["hands"][arm + "_robot"]["tcp"][:3]
                        )
                    _, rot, grip = targets[arm]
                    targets[arm] = (self.release_positions[phase], rot, grip)
            crank_phases={"approach hand crank","lower to hand crank","grasp hand crank"}
            if self.phase in crank_phases:
                p,r,g=targets["right"]
                knob=np.asarray(state["wheels"]["wheel0"]["Knob"]["pose"][:3])
                nominal=np.r_[SPROCKET_CENTERS[0]+[CRANK_RADIUS,0],CRANK_GRASP_Z]
                targets["right"]=(p+knob-nominal,r,g)
            if self.phase == "turn hand crank":
                stage=next(v for v in self.stages if v[2]==self.phase)
                f=ik.smoothstep(np.clip((time-stage[0])/(stage[1]-stage[0]),0,1))
                if self.crank_start is None:
                    self.crank_start=np.asarray(state["hands"]["right_robot"]["tcp"][:3])
                theta=(4*np.pi+.15)*f
                rz=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
                xy=SPROCKET_CENTERS[0]+rz@(self.crank_start[:2]-SPROCKET_CENTERS[0])
                targets["right"]=(np.r_[xy,self.crank_start[2]],crank_rotation(),.20)
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
