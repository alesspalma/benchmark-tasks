"""Native reference using elapsed time and ordinary joint positions."""

import numpy as np
from inspect_robots.policy import PolicyBase, PolicyInfo
from inspect_robots.types import Action, ActionChunk
from inspect_robots_yam.config import action_box
from yam_arena import ik
from yam_arena.constants import READY

from . import config, plan
from .scoring import pose
from .motion import rotation

def heading(matrix):
    return np.arctan2(matrix[1, 0], matrix[0, 0])


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
        self.driver_offset = None
        self.driver_bias = np.zeros(3)
        self.turn_result_rotation = None
        self.retreat_start = None
        self.duration = (
            max((sum(w["duration"] for w in v) for v in self.moves.values()), default=0)
            + 2
        )
        if self.mode == "hold":
            self.duration = config.MAX_STEPS / 20
        self.phase = "ready"
        self.worst_ik_m = 0.0
        self.worst_ik_rad = 0.0

    def _correct(self, name, label, position, fixture):
        if name != "right" or fixture is None:
            return position
        tube, tube_rotation = pose(fixture, "instrument")
        tcp = np.asarray(fixture["hands"]["right_robot"]["points"][0])
        if label in ("align nozzle over prism", "deliver distilled-water droplets",
                     "stabilize the water blank", "withdraw the dropper"):
            dropper, drop_rotation = pose(fixture, "dropper")
            tip = dropper + drop_rotation @ config.DROPPER_TIP
            target = tube + tube_rotation @ config.PRISM + position - plan.DROP_TOUCH + [0., 0., .020]
            return position + np.clip(target - position - (tip-tcp), -.024, .024)
        if label in ("return toward the dropper stand", "seat the dropper in its holder"):
            dropper, _ = pose(fixture, "dropper")
            return position - np.clip(dropper-tcp, -.018, .018)
        if label in ("center the blade just above its slot", "seat the screwdriver blade in the slot",
                     "turn the seated calibration screw",
                     "lift the driver out of the calibration slot"):
            driver, driver_rotation = pose(fixture, "driver")
            tip = driver + driver_rotation @ config.DRIVER_TIP
            target = tube + tube_rotation @ (config.SCREW - [0., 0., .0016])
            target += position - plan.DRIVER_SEATED
            offset = tip - tcp
            if self.driver_offset is None:
                self.driver_offset = offset
            self.driver_offset += .17 * (offset - self.driver_offset)
            if label == "center the blade just above its slot":
                error = np.clip((target-tip)[:2], -.004, .004)
                self.driver_bias[:2] = np.clip(self.driver_bias[:2] + .09 * error, -.009, .009)
            return position + np.clip(target - position - self.driver_offset + self.driver_bias,
                                      -.026, .026)
        if label in ("approach the open daylight plate", "push the daylight plate past vertical",
                     "close the daylight plate against the prism", "withdraw from the closed plate"):
            return position + np.clip(tube + tube_rotation @ config.HINGE
                                      - config.HELD - config.HINGE, -.019, .019)
        return position

    def _orient(self, name, label, r, joints, fixture):
        if name != "right" or fixture is None or joints is None:
            return r
        _, wrist_rotation = ik.tcp_pose(np.asarray(joints)[7:13], config.BASES["right"])
        returning = {
            "return toward the dropper stand": ("dropper", config.DROPPER_YAW),
            "seat the dropper in its holder": ("dropper", config.DROPPER_YAW),
        }
        if label in returning:
            key, yaw = returning[label]
            location, tool = pose(fixture, key)
            tcp = fixture["hands"]["right_robot"]["points"][0]
            if np.linalg.norm(location - tcp) > .027:
                return r
            grasp = wrist_rotation.T @ tool
            goal = rotation("Z", yaw) @ grasp.T
            angle = np.arccos(np.clip((np.trace(r.T @ goal)-1.)/2, -1., 1.))
            return ik.slerp_matrix(r, goal, min(.66, .115 / max(.001, angle)))
        if label not in ("pose the blade over the slotted screw", "center the blade just above its slot",
                         "seat the screwdriver blade in the slot",
                         "turn the seated calibration screw", "lift the driver out of the calibration slot"):
            return r
        if label == "turn the seated calibration screw" and self.turn_result_rotation is None:
            angle = config.SCREW_INIT + fixture["objects"]["instrument"]["joints"]["ZeroAngle"]
            if abs(angle * config.BRIX_PER_RADIAN) <= .128:
                self.turn_result_rotation = wrist_rotation.copy()
        if self.turn_result_rotation is not None and label in (
            "turn the seated calibration screw", "lift the driver out of the calibration slot",
        ):
            return self.turn_result_rotation
        _, tube_rotation = pose(fixture, "instrument")
        _, driver_rotation = pose(fixture, "driver")
        driver_in_hand = wrist_rotation.T @ driver_rotation
        goal = tube_rotation @ rotation("Z", heading(r) + plan.DRIVER_BLADE_OFFSET)
        return goal @ driver_in_hand.T

    def action_at(self, t, joint_pos=None, fixture=None):
        out = []
        labels = []
        for name in ("left", "right"):
            pos, rot, grip = self.starts[name]
            age = t
            label = ""
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
            if name == "right" and label == "draw the driver back across clear bench space" and self.retreat_start is None:
                joints = self.q[name] if joint_pos is None else np.asarray(joint_pos)[7:13]
                self.retreat_start = (t, joints.copy())
            if name == "right" and self.retreat_start is not None:
                started, joints = self.retreat_start
                weight = ik.smoothstep(np.clip((t-started)/1.8, 0., 1.))
                self.q[name] = joints + weight * (np.asarray(READY)-joints)
                out.extend([*self.q[name], grip])
                continue
            pos = self._correct(name, label, pos, fixture)
            rot = self._orient(name, label, rot, joint_pos, fixture)
            q, pe, re = ik.solve(
                pos,
                ik.matrix_to_quat_xyzw(rot),
                self.q[name],
                base_pos=config.BASES[name],
                iters=60,
            )
            clearance = label in ("lift the driver out of the calibration slot",
                                  "draw the driver back across clear bench space",
                                  "clear the tool and return the instrument to its rest",
                                  "draw both hands clear of the prism")
            position_tolerance = .011 if clearance else .003
            angle_tolerance = .11 if clearance else .077
            if label == "turn the seated calibration screw":
                # The attainable wrist can be slightly behind the requested turn
                # at its mechanical limit; the final optical check uses the real shaft.
                position_tolerance, angle_tolerance = .013, .17
            if pe > position_tolerance or re > angle_tolerance:
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
