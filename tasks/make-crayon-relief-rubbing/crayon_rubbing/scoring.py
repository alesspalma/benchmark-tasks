"""Wax transfer from a sliding broad face and the finished paper overlay."""

from collections import deque

import numpy as np
from scipy.spatial.transform import Rotation

from . import config as C
from .plate import relief_at

SPACING = .002
CELLS = np.stack(np.meshgrid(
    np.arange(-C.PAPER_SIZE[0] / 2 + SPACING / 2, C.PAPER_SIZE[0] / 2, SPACING),
    np.arange(-C.PAPER_SIZE[1] / 2 + SPACING / 2, C.PAPER_SIZE[1] / 2, SPACING),
    indexing="ij",
), axis=-1).reshape(-1, 2)
RELIEF_CELLS = relief_at(CELLS)
INK_THRESHOLD = .34


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.wax = np.zeros(len(CELLS))
        self.force_history = deque(maxlen=6)
        self.previous = None
        self.load = 0.
        self.peak_load = 0.
        self.pin_load = {"left": 0., "right": 0.}
        self.peak_pin_load = 0.
        self.sliding_mm = 0.
        self.contact_frames = 0
        self.peak_shift = 0.
        self.hold = 0.

    def trace_contact(self, forces, points, paper, crayon):
        """Advance the paper's physical wax record by one PhysX step."""
        paper_rot = Rotation.from_quat(paper[3:]).as_matrix()
        crayon_rot = Rotation.from_quat(crayon[3:]).as_matrix()
        relative = (crayon[:3] - paper[:3]) @ paper_rot
        delta = 0. if self.previous is None else np.linalg.norm(relative[:2] - self.previous[:2])
        self.previous = relative

        # The contact points must be part of the block's broad wax face.
        normal = paper_rot[:, 2] @ crayon_rot[:, 2]
        face_side = -1. if normal >= 0. else 1.
        on_face = np.abs(((points - crayon[:3]) @ crayon_rot)[:, 2]
                         - face_side * C.CRAYON_SIZE[2] / 2) <= .0022
        force = float(np.abs(forces[on_face]).sum()) if abs(normal) >= .94 else 0.
        self.force_history.append(force)
        self.load = sum(self.force_history) / len(self.force_history)
        self.peak_load = max(self.peak_load, self.load)
        if force <= 0. or self.load < .35 or delta < .000018:
            return

        candidates = np.flatnonzero(np.all(np.abs(CELLS - relative[:2]) <= .026, axis=1))
        top = np.full(len(candidates), C.PAPER_THICKNESS / 2)
        world = np.column_stack((CELLS[candidates], top)) @ paper_rot.T + paper[:3]
        wax_local = (world - crayon[:3]) @ crayon_rot
        face = np.all(np.abs(wax_local[:, :2]) <= C.CRAYON_SIZE[:2] / 2 - .00065, axis=1)
        face &= np.abs(wax_local[:, 2] - face_side * C.CRAYON_SIZE[2] / 2) <= .0019
        indices = candidates[face]
        if not len(indices):
            return

        # Peaks concentrate the load under flexible paper. Repeated or heavy
        # passes across a recess can darken the background as well.
        gain = np.where(relief_at(world[face, :2] - C.PAPER_CENTER, include_margins=True), 1., .07)
        amount = .048 * min(self.load, 9.) * min(delta * 1000., 3.)
        self.wax[indices] = np.minimum(self.wax[indices] + amount * gain, 1.)
        self.sliding_mm += delta * 1000.
        self.contact_frames += 1

    def evaluate(self, state):
        paper = np.asarray(state["objects"]["paper"]["pose"])
        crayon = np.asarray(state["objects"]["crayon"]["pose"])
        orientation = Rotation.from_quat(paper[3:]).as_matrix()
        position = float(np.linalg.norm(paper[:2] - C.PAPER_CENTER))
        yaw = float(np.arctan2(orientation[1, 0], orientation[0, 0]))
        self.peak_shift = max(self.peak_shift, position)
        marked = self.wax >= INK_THRESHOLD
        transfer = float(marked[RELIEF_CELLS].mean())
        # Compare every strongly colored cell on the sheet to the physical
        # reference so marking an extra area cannot improve the score.
        background = float(np.count_nonzero(marked & ~RELIEF_CELLS) / np.count_nonzero(RELIEF_CELLS))
        placed = (position <= .016 and abs(yaw) <= np.deg2rad(8)
                  and orientation[2, 2] > .985 and abs(paper[2] - C.PLATE_TOP) < .010)
        crayon_flat = abs(Rotation.from_quat(crayon[3:]).as_matrix()[2, 2]) > .97
        crayon_clear = bool(
            (np.any(np.abs(crayon[:2] - paper[:2]) > C.PAPER_SIZE / 2 + .026)
             and C.TABLE + .003 <= crayon[2] <= C.TABLE + .015 and crayon_flat)
        )
        hands_clear = all(
            p[2] > C.PAPER_Z + .027
            or np.any(np.abs(np.asarray(p[:2]) - paper[:2]) > C.PAPER_SIZE / 2 + .025)
            for side in state["hands"].values() for p in side["points"][:2]
        )
        done = placed and crayon_clear and hands_clear and transfer >= .86 and background <= .15
        self.hold = self.hold + C.CHECK_DT if done else 0.
        if self.hold >= .45:
            score = 1.
        elif not placed or background > .30 or transfer < .23:
            score = 0.
        else:
            score = .75 if transfer >= .70 and background <= .15 else .50 if transfer >= .47 else .25
        return {
            "score": score, "success": score == 1., "fern_coverage": transfer,
            "extra_dark_area": background, "paper_flat": bool(placed),
            "wax_set_down": crayon_clear, "hands_clear": hands_clear,
            "paper_shift_mm": 1000. * position, "max_shift_mm": 1000. * self.peak_shift,
            "paper_yaw_deg": float(np.rad2deg(yaw)), "wax_load_n": self.load,
            "peak_wax_load_n": self.peak_load, "sliding_contact_mm": self.sliding_mm,
            "paper_gripper_load_n": self.pin_load, "peak_gripper_load_n": self.peak_pin_load,
            "wax_contact_samples": self.contact_frames, "finish_hold_s": self.hold,
        }
