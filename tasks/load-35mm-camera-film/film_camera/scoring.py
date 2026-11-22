"""Measure cartridge seating, cover closure, payout and knob movement."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def seated(pose):
    x, y, z, qx, qy, qz, _ = pose
    return (
        abs(x - C.X) < 0.007
        and abs(y - C.SEAT_Y) < 0.006
        and abs(z - C.SEAT_Z) < 0.005
        and max(abs(qx), abs(qy), abs(qz)) < 0.12
    )


def aligned_leader(pose, payout):
    if not C.PULL - 0.007 <= payout <= C.PULL + 0.006:
        return False
    pose = np.asarray(pose)
    axes = quat_xyzw_to_matrix(pose[3:])
    # Both ends of the centerline must sit between the rail margins.
    centerline_x = pose[0] + np.array([-0.015, 0.015]) * axes[0, 1]
    return bool(
        np.all(np.abs(centerline_x - C.X) <= 0.004)
        and axes[1, 1] >= np.cos(np.deg2rad(10))
        and abs(pose[2] - C.FILM_Z) <= 0.003
    )


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.stable_s = 0.0

    def evaluate(self, state, elapsed):
        cassette = state["objects"]["film_cartridge"]
        camera = state["objects"]["camera"]
        joints = camera["joints"]
        is_seated = seated(cassette["pose"])
        payout = -cassette["joints"]["FilmPayout"]
        capture = state["transport"]["engaged"]
        leader = cassette["links"]["FilmLeader"]
        leader_axes = quat_xyzw_to_matrix(np.asarray(leader[3:]))
        skew = np.arctan2(leader_axes[1, 0], leader_axes[0, 0])
        aligned = capture or aligned_leader(leader, payout)
        captured_payout = state["transport"]["payout_at_capture_m"]
        advanced_by = max(0.0, payout - captured_payout) if capture else 0.0
        shut = abs(joints["BackHinge"]) < np.deg2rad(4)
        turning = joints["RewindPivot"] >= 0.70
        advance = advanced_by >= 0.013
        lever_angle = joints["AdvancePivot"]
        end = np.asarray(camera["links"]["AdvanceLever"][:3]) + np.array(
            [
                C.LEVER_REACH * np.sin(lever_angle),
                -C.LEVER_REACH * np.cos(lever_angle),
                0.008,
            ]
        )
        keepout = (
            (np.array([C.X, -0.022, 0.825]), 0.105),
            (end, 0.045),
            (np.asarray(camera["links"]["RewindKnob"][:3]), 0.040),
        )
        clear = all(
            np.linalg.norm(np.asarray(point) - center) > margin
            for arm in state["hands"].values()
            for point in arm["points"]
            for center, margin in keepout
        )
        lever_returned = abs(lever_angle) < np.deg2rad(5)
        finished = (
            is_seated
            and capture
            and shut
            and advance
            and turning
            and clear
            and lever_returned
        )
        self.stable_s = self.stable_s + elapsed if finished else 0.0
        success = self.stable_s >= 0.30
        if success:
            score = 1.0
        elif is_seated and capture and shut:
            score = 0.75
        elif is_seated and aligned:
            score = 0.50
        else:
            score = 0.0
        return {
            "success": bool(success),
            "score": score,
            "cassette_seated": bool(is_seated),
            "leader_aligned": bool(aligned),
            "transport_engaged": bool(capture),
            "back_latched": bool(shut),
            "film_payout_m": round(payout, 5),
            "film_advance_after_capture_m": round(advanced_by, 5),
            "leader_center_offset_m": round(leader[0] - C.X, 5),
            "leader_skew_deg": round(float(np.rad2deg(skew)), 2),
            "advance_lever_deg": round(float(np.rad2deg(joints["AdvancePivot"])), 2),
            "rewind_knob_deg": round(float(np.rad2deg(joints["RewindPivot"])), 2),
            "both_wrists_clear": bool(clear),
            "advance_lever_returned": bool(lever_returned),
            "stable_s": round(self.stable_s, 3),
        }
