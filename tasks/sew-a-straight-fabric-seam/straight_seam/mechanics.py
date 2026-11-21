"""Measured sewing motion and force-limited feed and thread constraints."""

from math import cos, pi

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def frame(pose):
    return pose[:3], quat_xyzw_to_matrix(pose[3:])


def local_at_needle(pose):
    p, r = frame(pose)
    local = r.T @ (np.array([*C.NEEDLE_XY, p[2]]) - p)
    local[2] = 0.0
    return local


def under_needle(point):
    return abs(point[0]) < C.LENGTH / 2 - 0.001 and abs(point[1]) < C.WIDTH / 2 - 0.001


def holds(poses, hands):
    supported = []
    clearance = []
    for pose, clip in zip(poses, (C.UPPER_CLIP, C.LOWER_CLIP)):
        origin, rotation = frame(pose)
        half = np.array([C.LENGTH, C.WIDTH, C.THICK]) / 2
        clearance.append(origin[2] - np.abs(rotation[2]) @ half - C.BED)
        grasp = False
        for hand in hands.values():
            points = np.asarray(hand["points"][:2])
            local = (points - origin) @ rotation
            near_clip = np.linalg.norm(local - clip, axis=1).min() < 0.025
            near_fabric = (
                np.linalg.norm(np.maximum(np.abs(local) - half, 0), axis=1).min()
                < 0.010
            )
            grasp |= hand["opening_m"] < 0.035 and (near_clip or near_fabric)
        supported.append(grasp)
    return supported, clearance


def oriented_on_bed(poses):
    p, r = frame(poses[0])
    q, s = frame(poses[1])
    return (
        abs(p[2] - C.UPPER_WORK[2]) < 0.009
        and abs(q[2] - C.LOWER_WORK[2]) < 0.008
        and r[2, 2] > 0.97
        and s[2, 2] > 0.97
    )


class Sewing:
    def __init__(self):
        self.switch_held = False
        self.foot_selected_down = False
        self.running = False
        self.motor_age = 0.0
        self.tip_low = False
        self.samples = []
        self.proof_s = 0.0

    def controls(self, foot, lever, needle, button, dt):
        if lever < 0.006:
            self.foot_selected_down = True
        elif lever > 0.013:
            self.foot_selected_down = False
        if button < -0.005 and not self.switch_held:
            self.running = not self.running if self.foot_selected_down else False
            self.switch_held = True
        elif button > -0.002:
            self.switch_held = False
        if not self.foot_selected_down:
            self.running = False
        self.motor_age = self.motor_age + dt if self.running else 0.0
        # The measured shaft height, rather than the target, gates contact and feeding.
        crossed = needle < 0.00265 and not self.tip_low
        if crossed:
            self.tip_low = True
        elif needle > 0.0040:
            self.tip_low = False
        targets = {
            "FootJoint": 0.0 if self.foot_selected_down else C.FOOT_TRAVEL,
            "LeverJoint": 0.0 if self.foot_selected_down else C.LEVER_TRAVEL,
            "NeedleJoint": C.NEEDLE_TRAVEL
            * (0.5 + 0.5 * cos(2 * pi * C.NEEDLE_RATE * self.motor_age)),
        }
        clamped = self.running and foot < 0.0042
        return targets, clamped and crossed, clamped and needle > 0.0039

    def forces(self, poses, velocities, stitch, advance):
        force = np.zeros((2, 3))
        torque = np.zeros((2, 3))
        local = [local_at_needle(p) for p in poses]
        cover = [under_needle(v) for v in local]
        target = oriented_on_bed(poses)
        if (
            stitch
            and target
            and all(cover)
            and (
                not self.samples
                or np.linalg.norm(local[0] - self.samples[-1][0]) >= 0.0013
            )
        ):
            self.samples.append(tuple(local))
        if advance and target:
            for i in range(2):
                if cover[i]:
                    force[i, 0] = np.clip(
                        (C.FEED_RATE - velocities[i, 0]) * 29, 0, 0.58
                    )
                    force[i, 1] = np.clip(-velocities[i, 1] * 0.6, -0.17, 0.17)
                    contact = np.array([*C.NEEDLE_XY, C.BED]) - poses[i][:3]
                    torque[i] = np.cross(contact, force[i])
                    torque[i, 2] -= np.clip(velocities[i, 5] * 0.018, -0.018, 0.018)
        if self.samples:
            frames = [frame(p) for p in poses]
            count = min(5, len(self.samples))
            for i in np.linspace(0, len(self.samples) - 1, count, dtype=int):
                p, r = frames[0]
                q, s = frames[1]
                a = r @ self.samples[i][0]
                b = s @ self.samples[i][1]
                desired_gap = 0.5 * C.THICK * (r[:, 2] + s[:, 2])
                speed_a = velocities[0, :3] + np.cross(velocities[0, 3:], a)
                speed_b = velocities[1, :3] + np.cross(velocities[1, 3:], b)
                f = -(p + a - q - b - desired_gap) * (155.0 / count) - (
                    speed_a - speed_b
                ) * (1.8 / count)
                n = np.linalg.norm(f)
                f *= min(1.0, 0.85 / (count * max(n, 1e-7)))
                force[0] += f
                force[1] -= f
                torque[0] += np.cross(a - r[:, 2] * 0.002, f)
                torque[1] -= np.cross(b - s[:, 2] * 0.002, f)
        return force, torque

    def quality(self):
        lines = np.asarray([p[0][:2] for p in self.samples]).reshape(-1, 2)
        if len(lines) > 1:
            pitch = np.linalg.norm(np.diff(lines, axis=0), axis=1)
            length = float(np.ptp(lines[:, 0]))
            gap = float(np.max(pitch))
        else:
            length = gap = 0.0
        deviation = (
            float(np.max(np.abs(lines[:, 1] + C.WIDTH / 2 - C.SEAM_ALLOWANCE)))
            if len(lines)
            else 0.0
        )
        return length, gap, deviation

    def evaluate(self, state, dt):
        upper = np.asarray(state["objects"]["upper"]["pose"])
        lower = np.asarray(state["objects"]["lower"]["pose"])
        joints = state["objects"]["machine"]["joints"]
        (top_supported, bottom_supported), clearance = holds(
            (upper, lower), state["hands"]
        )
        open_foot = joints["FootJoint"] >= 0.018
        length, gap, deviation = self.quality()
        joined = length >= C.SEAM_LENGTH and gap <= 0.009 and deviation <= 0.0045
        lifted = (
            open_foot
            and not self.running
            and upper[2] >= C.BED + 0.024
            and lower[2] >= C.BED + 0.021
            and top_supported
            and not bottom_supported
            and min(clearance) > 0.002
        )
        self.proof_s = self.proof_s + dt if lifted and joined else 0.0
        score = 0.0
        if self.samples:
            score = 0.25
        if length >= 0.034 and gap <= 0.012:
            score = 0.50
        if joined:
            score = 0.80
        if self.proof_s >= 0.40 - 1e-7:
            score = 1.0
        return {
            "score": score,
            "success": score == 1.0,
            "motor_running": self.running,
            "foot_height_mm": float(joints["FootJoint"] * 1000),
            "needle_height_mm": float(joints["NeedleJoint"] * 1000),
            "button_travel_mm": float(-joints["ButtonJoint"] * 1000),
            "lifter_height_mm": float(joints["LeverJoint"] * 1000),
            "stitches": len(self.samples),
            "seam_length_mm": 1000 * length,
            "max_gap_mm": 1000 * gap,
            "max_allowance_error_mm": 1000 * deviation,
            "orange_height_mm": float(1000 * (upper[2] - C.BED)),
            "teal_height_mm": float(1000 * (lower[2] - C.BED)),
            "proof_hold_s": self.proof_s,
            "teal_unsupported": not bottom_supported,
            "orange_grasped": top_supported,
            "orange_clearance_mm": 1000 * clearance[0],
            "teal_clearance_mm": 1000 * clearance[1],
        }
