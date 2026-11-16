"""Current physical insertion and positive spring-clip retention, never proximity latches."""

import math

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def rotation(pose):
    return quat_xyzw_to_matrix(np.array(pose[3:]))


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.violations = set()
        self.workspace_samples = 0
        self.hold_start = None
        self.last_time = None

    def observe_workspace(self, points):
        self.workspace_samples += 1
        for p in points.values():
            if not np.isfinite(p).all():
                self.violations.add("invalid_state")
            elif (
                p[:, 2].min() < C.TABLE_TOP - 0.006
                or p[:, 0].min() < -0.19
                or p[:, 0].max() > 0.91
                or abs(p[:, 1]).max() > 0.65
            ):
                self.violations.add("part_left_work_area")

    def evaluate(self, s):
        try:
            parts = (
                [s[n] for n in ("connector", "keeper", "tool")]
                + list(s["bodies"].values())
                + list(s["clips"].values())
            )
            valid = bool(
                set(s["bodies"]) == {f"Link{i:02}" for i in range(11)}
                and set(s["clips"]) == {"clip0", "clip1"}
                and len(s["hands"]) == 2
                and np.isfinite(s["time_s"])
                and s["time_s"] >= 0
                and np.isfinite(s["robot_clearance_m"])
                and all(
                    len(p["pose"]) == 7
                    and len(p["velocity"]) == 6
                    and np.isfinite(p["pose"]).all()
                    and np.isfinite(p["velocity"]).all()
                    and abs(np.linalg.norm(p["pose"][3:]) - 1) < 0.001
                    for p in parts
                )
                and all(
                    len(p["joints"]) == 2
                    and np.isfinite(list(p["joints"].values())).all()
                    for p in s["clips"].values()
                )
                and all(np.isfinite(h["opening_m"]) for h in s["hands"].values())
            )
        except (KeyError, TypeError, ValueError):
            valid = False
        if not valid:
            self.violations.add("invalid_state")
            self.hold_start = None
            return {
                "score": 0.0,
                "success": False,
                "violations": sorted(self.violations),
                "ends_joined": False,
                "plate_seated": False,
                "retained": False,
                "parked": False,
                "settled": False,
                "hands_clear": False,
                "guide_seated": False,
                "workspace_samples": self.workspace_samples,
            }
        time = float(s["time_s"])
        if self.last_time is not None:
            if time < self.last_time:
                self.violations.add("invalid_state")
            if time - self.last_time > 1.01 / C.CONTROL_HZ:
                self.hold_start = None
        self.last_time = time
        c = np.array(s["connector"]["pose"])
        r = rotation(c)
        ends = {}
        checks = []
        end_data = []
        for arm, body, sign in [("right", "Link00", -1), ("left", "Link10", 1)]:
            pose = np.array(s["bodies"][body]["pose"])
            br = rotation(pose)
            center = r.T @ (
                pose[:3] + br @ np.array([sign * C.PITCH / 2, 0, 0]) - c[:3]
            )
            axis = r.T @ br[:, 2]
            end_data.append((arm, center, axis))
        assignments = ((-C.PITCH / 2, C.PITCH / 2), (C.PITCH / 2, -C.PITCH / 2))
        assigned = min(
            assignments,
            key=lambda ys: sum(
                np.linalg.norm(center[:2] - [0, y])
                for (_, center, _), y in zip(end_data, ys)
            ),
        )
        for (arm, center, axis), pin_y in zip(end_data, assigned):
            errors = [
                float(np.linalg.norm((center + z * axis)[:2] - [0, pin_y]))
                for z in (-0.019, 0.019)
            ]
            ends[arm] = {"center_m": center.tolist(), "face_radial_errors_m": errors}
            checks.append(
                max(errors) <= 0.00015
                and abs(center[2] - C.CHAIN_Z) < 0.0008
                and abs(axis[2]) > 0.9999
            )
        joined = bool(all(checks))
        k = np.array(s["keeper"]["pose"])
        kr = rotation(k)
        center = r.T @ (k[:3] - c[:3])
        cover_errors = []
        cover_axial = []
        hole_centers = [
            r.T @ (k[:3] + kr @ np.array([0, y, 0]) - c[:3])
            for y in (-C.PITCH / 2, C.PITCH / 2)
        ]
        assigned = min(
            assignments,
            key=lambda ys: sum(
                np.linalg.norm(p[:2] - [0, y]) for p, y in zip(hole_centers, ys)
            ),
        )
        for y, pin_y in zip((-C.PITCH / 2, C.PITCH / 2), assigned):
            hole = r.T @ (k[:3] + kr @ np.array([0, y, 0]) - c[:3])
            cover_axial.append(float(abs(hole[2] - C.COVER_Z)))
            for z in (-0.0025, 0.0025):
                p = r.T @ (k[:3] + kr @ np.array([0, y, z]) - c[:3])
                cover_errors.append(float(np.linalg.norm(p[:2] - [0, pin_y])))
        covered = bool(
            joined and max(cover_errors) <= 0.00015 and max(cover_axial) < 0.0008
        )
        clips = {}
        names = ("clip0", "clip1")
        local_centers = [
            r.T @ (np.array(s["clips"][n]["pose"][:3]) - c[:3]) for n in names
        ]
        assigned = min(
            assignments,
            key=lambda ys: sum(
                np.linalg.norm(p[:2] - [0, y]) for p, y in zip(local_centers, ys)
            ),
        )
        for n, y in zip(names, assigned):
            clip = s["clips"][n]
            p = np.array(clip["pose"])
            local = r.T @ (p[:3] - c[:3])
            angles = list(clip["joints"].values())
            engaged = bool(
                np.linalg.norm(local[:2] - [0, y]) < 0.0004
                and abs(local[2] - C.GROOVE_Z) <= 0.0003
                and abs((r.T @ rotation(p))[2, 2]) > 0.999
                and max(abs(a) for a in angles) < math.radians(2)
            )
            clips[n] = {
                "local_m": local.tolist(),
                "pin_y_m": y,
                "spring_angles_rad": angles,
                "engaged": engaged,
            }
        retained = bool(covered and all(v["engaged"] for v in clips.values()))
        parked = bool(
            np.linalg.norm(c[:3] - C.CONNECTOR_POSITION) < 0.003
            and max(
                np.trace(target.T @ r)
                for target in (
                    C.CONNECTOR_ROTATION,
                    C.CONNECTOR_ROTATION @ np.diag([-1.0, -1.0, 1.0]),
                )
            )
            > 1 + 2 * math.cos(math.radians(1))
        )
        quiet = bool(
            all(
                np.linalg.norm(p["velocity"][:3]) < 0.02
                and np.linalg.norm(p["velocity"][3:]) < 0.3
                for p in [s[n] for n in ("connector", "keeper", "tool")]
            )
        )
        # Retainers can rotate around circular grooves without losing capture.
        # Their measured groove occupancy, normal and closed leaves must still
        # persist for the full hold. Require the applicator to be withdrawn so
        # its loading channels cannot substitute for independent retention.
        applicator_clear = bool(
            np.linalg.norm(np.asarray(s["tool"]["pose"][:2]) - c[:2]) >= 0.025
        )
        clear = bool(
            len(s["hands"]) == 2
            and s["robot_clearance_m"] >= 0.025
            and all(h["opening_m"] >= 0.070 for h in s["hands"].values())
        )
        guide = {}
        for i in range(2, 9):
            pose = np.array(s["bodies"][f"Link{i:02}"]["pose"])
            target = C.CHAIN_POSITION + (C.NODES[i] + C.NODES[i + 1]) / 2
            guide[f"Link{i:02}"] = float(np.linalg.norm(pose[:2] - target[:2]))
        seated = bool(
            all(v < 0.015 for v in guide.values())
            and all(
                abs(s["bodies"][f"Link{i:02}"]["pose"][2] - C.CHAIN_POSITION[2]) < 0.003
                and rotation(s["bodies"][f"Link{i:02}"]["pose"])[2, 2] > 0.995
                for i in range(2, 9)
            )
        )
        full = bool(
            retained
            and parked
            and quiet
            and clear
            and seated
            and applicator_clear
            and not self.violations
        )
        if full and self.hold_start is None:
            self.hold_start = time
        elif not full:
            self.hold_start = None
        success = bool(full and time - self.hold_start >= 1.0 - 1e-9)
        score = (
            0.0
            if self.violations
            else 1.0
            if success
            else 0.85
            if retained
            else 0.65
            if covered
            else 0.35
            if joined
            else 0.0
        )
        return {
            "score": score,
            "success": success,
            "violations": sorted(self.violations),
            "ends_joined": joined,
            "plate_seated": covered,
            "retained": retained,
            "parked": parked,
            "settled": quiet,
            "hands_clear": clear,
            "guide_seated": seated,
            "applicator_clear": applicator_clear,
            "guide_link_errors_m": guide,
            "robot_clearance_m": s["robot_clearance_m"],
            "end_points_local": ends,
            "keeper_local": center.tolist(),
            "cover_face_radial_errors_m": cover_errors,
            "cover_hole_axial_errors_m": cover_axial,
            "clips": clips,
            "workspace_samples": self.workspace_samples,
        }
