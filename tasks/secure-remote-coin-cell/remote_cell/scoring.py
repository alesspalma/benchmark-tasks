"""Secure the bayonet before placing the remote within reach."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.unsafe_handoff = False
        self.settled_frames = 0
        self.turned_under_tool = False

    def evaluate(self, state):
        remote = state["objects"]["remote"]
        cover = state["objects"]["cover"]
        driver = state["objects"]["driver"]
        body_pos = np.asarray(remote["pose"][:3])
        body_axes = quat_xyzw_to_matrix(remote["pose"][3:])
        cover_pos = np.asarray(cover["pose"][:3])
        cover_axes = quat_xyzw_to_matrix(cover["pose"][3:])
        local = body_axes.T @ (cover_pos - body_pos) - C.SEAT_CENTER
        cap_axes = body_axes.T @ cover_axes
        angle = float(np.arctan2(cap_axes[1, 0], cap_axes[0, 0]))
        # Either set of ears can enter first. Both have the same clockwise stop.
        yaw = float((angle + np.pi / 2) % np.pi - np.pi / 2)
        radial = float(np.linalg.norm(local[:2]))
        axial = float(local[2] - C.SEAT_HEIGHT)
        seated = radial <= .0016 and -.0012 <= axial <= .0011 and cap_axes[2, 2] > .970
        locked = bool(seated and -C.LOCKED_MAX <= yaw <= -C.LOCKED_MIN)
        shelf_offset = body_pos - C.SHELF
        top = C.SEAT_HEIGHT + C.CAP_TOP if locked else C.SHELL_HEIGHT
        body_bottom = body_pos[2] + min(0., body_axes[2, 2] * top)
        footprint = np.stack([
            body_pos + body_axes @ [dx, dy, 0.]
            for dx in (-C.SHELL_WIDTH / 2, C.SHELL_WIDTH / 2)
            for dy in (C.SHELL_BACK, C.SHELL_FRONT)
        ]) - C.SHELF
        over_shelf = bool(
            np.all(np.abs(footprint[:, 0]) <= C.SHELF_WIDTH / 2 + .001)
            and np.all(np.abs(footprint[:, 1]) <= C.SHELF_LENGTH / 2 + .001)
        )
        accessible = bool(
            abs(shelf_offset[0]) <= C.SHELF_WIDTH / 2
            and abs(shelf_offset[1]) <= C.SHELF_LENGTH / 2
            and body_bottom < C.SHELF_SURFACE + .025
        )
        self.unsafe_handoff |= accessible and not locked
        in_shelf = bool(over_shelf and abs(body_bottom - C.SHELF_SURFACE) < .004
                        and abs(body_axes[2, 2]) > .985)
        stable = bool(np.linalg.norm(remote["velocity"][:3]) <= .025 and
                      np.linalg.norm(cover["velocity"][:3]) <= .025)
        low = np.array([-C.SHELL_WIDTH / 2, C.SHELL_BACK, 0.])
        high = np.array([C.SHELL_WIDTH / 2, C.SHELL_FRONT, top])
        hands_clear = True
        for hand in state["hands"].values():
            fingertips = body_axes.T @ (np.asarray(hand["points"][1]) - body_pos)
            nearest = np.clip(fingertips, low, high)
            hands_clear &= bool(np.linalg.norm(fingertips - nearest) >= .035)
        tool_distance = float(np.linalg.norm(np.asarray(driver["pose"][:3]) - cover_pos))
        self.turned_under_tool |= locked and tool_distance < .008
        good = locked and in_shelf and stable and hands_clear and not self.unsafe_handoff
        self.settled_frames = self.settled_frames + 1 if good else 0
        score = 0.0 if self.unsafe_handoff else (
            1.0 if self.settled_frames >= 10 else .5 if locked and stable else 0.0
        )
        return {
            "score": score, "success": score == 1.0,
            "cover_seated": bool(seated), "cover_locked": locked,
            "door_turn_degrees": float(np.degrees(yaw)),
            "seat_gap_mm": axial * 1000, "centre_error_mm": radial * 1000,
            "screwdriver_tip_distance_mm": tool_distance * 1000,
            "tool_present_at_lock": bool(self.turned_under_tool),
            "remote_in_shelf": in_shelf, "hands_clear": hands_clear,
            "unsafe_handoff": bool(self.unsafe_handoff), "settled_frames": self.settled_frames,
        }
