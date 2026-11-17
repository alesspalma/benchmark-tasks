"""Robot-contact reference with read-only cup/crown pose feedback."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik, station
from yam_arena.constants import READY

from . import config, plan


class ScriptedPolicy(PolicyBase):
    def __init__(self, mode="demo"):
        self.mode = mode
        self.native_feedback=None
        low = np.r_[ik.LOWER, 0.0, ik.LOWER, 0.0]
        high = np.r_[ik.UPPER, 1.0, ik.UPPER, 1.0]
        self.info = PolicyInfo(
            name=config.KEY + "_scripted", action_space=action_box(low, high)
        )
        self.reset(None)

    def bind(self, info):
        from .environment import ACTIVE
        self.native_feedback=ACTIVE

    def reset(self, scene):
        self.moves = plan.waypoints(self.mode)
        self.q = {n: np.asarray(READY).copy() for n in ("left", "right")}
        self.starts = {
            n: (*ik.tcp_pose(self.q[n], station.BASES[n]), 1.0) for n in self.q
        }
        self.previous = np.r_[READY, 1.0, READY, 1.0]
        self.integral = np.zeros(14)
        self.duration = (
            max((sum(w["duration"] for w in v) for v in self.moves.values()), default=0)
            + 2
        )
        self.base_duration=self.duration
        self.delay=0.
        self.last_time=0.
        self.fit_since=None
        self.crown_since=None
        self.fitting=None
        self.fitting_started=None
        self.phase = "ready"
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def action_at(self, t, joint_pos=None):
        if self.fitting is not None:
            result=self.fitting.action_at(float(t)-self.fitting_started,joint_pos)
            self.previous=result
            self.phase=self.fitting.phase
            self.worst_ik_m=max(self.worst_ik_m,self.fitting.worst_ik_m)
            self.worst_ik_rad=max(self.worst_ik_rad,self.fitting.worst_ik_rad)
            return result
        real_time=float(t)
        dt=max(0.,min(.05,real_time-self.last_time))
        self.last_time=real_time
        t=real_time-self.delay
        native_state=None
        crown_pose=None
        if self.native_feedback is not None:
            native_state=self.native_feedback.state(light=True)
            links=native_state['objects']['protector']['links']
            from .scoring import fitted_side
            fits=all(fitted_side(links,sign,label)['fit'] for sign,label in ((1,'Left'),(-1,'Right')))
            if fits:
                if self.fit_since is None:self.fit_since=real_time
            else:self.fit_since=None
            seated=bool(self.native_feedback.metrics.get('crown_seated'))
            if seated:
                if self.crown_since is None:self.crown_since=real_time
            else:self.crown_since=None
            if self.mode in ('demo','held') and self.delay<8.:
                open_level=all(abs(np.asarray(links[label+'Cup'])[2]-(config.HEAD[2]+config.EAR_HEIGHT))<.006 for label in ('Left','Right'))
                if 27.9<=t<=28.1 and not open_level:
                    self.delay=real_time-27.9
                    t=27.9
                elif 37.9<=t<=38.1 and (not fits or self.fit_since is None or real_time-self.fit_since<.5):
                    self.delay=real_time-37.9
                    t=37.9
            self.duration=self.base_duration+self.delay
            if self.mode in ('demo','held') and t>=47.:
                from .fit_stage import FittingSequence
                self.fitting=FittingSequence(self.native_feedback,self.info,joint_pos,self.previous,held=self.mode=='held')
                self.fitting_started=real_time
                self.duration=real_time+self.fitting.duration
                return self.action_at(real_time,joint_pos)
        out = []
        labels = []
        for name in ("left", "right"):
            pos, rot, grip = self.starts[name]
            age = t
            for w in self.moves.get(name, []):
                goal = np.asarray(w["position"])
                r = np.asarray(w["rotation"])
                g = w.get("grip", grip)
                if age < w["duration"]:
                    s = ik.smoothstep(age / w["duration"])
                    pos = pos + s * (goal - pos)
                    rot = ik.slerp_matrix(rot, r, s)
                    grip += s * (g - grip)
                    labels.append(w.get("label", "move"))
                    break
                age -= w["duration"]
                pos, rot, grip = goal, r, g
            if native_state is not None and 24. <= t <= 38.:
                cup=np.asarray(native_state['objects']['protector']['links'][name.title()+'Cup'])[:3]
                pos=np.asarray(pos).copy()
                pos[0]+=np.clip(pos[0]-cup[0],-.010,.010)
                pos[2]+=np.clip(pos[2]-.002-cup[2],-.035,.035)
            q, pe, re = ik.solve(
                pos,
                ik.matrix_to_quat_xyzw(rot),
                self.q[name],
                base_pos=station.BASES[name],
                iters=60,
            )
            if pe > 0.002 or re > 0.03:
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
            limit=.30 if 24. <= t <= 64. else .15
            self.integral = np.clip(self.integral + 0.6 * 0.05 * error, -limit, limit)
            target = target + self.integral
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
