"""Transfer a narrow pigment film along a physically loaded applicator stroke."""

import math

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C, paper

PITCH_RANGE = np.deg2rad((26., 60.))
# Covers adjacent ink more accurately than testing only word bounding rectangles.
GRID = .00025


def sample_centers(rectangles):
    return np.asarray([
        [x + w * a, y + h * b]
        for x, y, w, h in rectangles
        for a in (.25, .75)
        for b in (.25, .75)
    ])


class Tape:
    def __init__(self):
        self.ink_samples = [sample_centers(v) for v in paper.entries()]
        self.reset()

    def reset(self):
        self.covered = [np.zeros(len(samples), dtype=bool) for samples in self.ink_samples]
        self.previous = None
        self.stroke_m = 0.
        self.overrun_m2 = 0.
        self.max_compression = 0.
        self.current_compression = 0.
        self.nose = [0., 0., 0.]
        self.pitch_deg = 0.
        self.faces = []
        self.rendered_faces = 0

    def advance(self, dt, applicator, compression, clipboard):
        board_rot = quat_xyzw_to_matrix(clipboard[3:7])
        tool_rot = quat_xyzw_to_matrix(applicator[3:7])
        relative = board_rot.T @ tool_rot
        toe = C.TOOL_TOE + [0., 0., compression]
        pos = board_rot.T @ (applicator[:3] + tool_rot @ toe - clipboard[:3])
        axis = relative[:, 0]
        across = relative[:2, 1]
        across /= max(np.linalg.norm(across), 1e-8)
        direction = axis[:2] / max(np.linalg.norm(axis[:2]), 1e-8)
        pitch = math.asin(np.clip(axis[2], -1., 1.))
        self.nose = pos.tolist()
        self.pitch_deg = math.degrees(pitch)
        self.current_compression = compression
        self.max_compression = max(self.max_compression, compression)
        in_paper = np.all(np.abs(pos[:2]) < np.asarray(C.PAPER_SIZE) / 2 - .001)
        seated = (
            in_paper and abs(pos[2] - C.PAPER_Z) < .0015
            and .00022 < compression < C.NOSE_LIMIT + .0001
            and PITCH_RANGE[0] <= pitch <= PITCH_RANGE[1]
            and abs(relative[2, 1]) < .22
        )
        previous = self.previous
        self.previous = (pos[:2].copy(), across.copy()) if seated else None
        if not seated or previous is None:
            return
        center_before, across_before = previous
        displacement = pos[:2] - center_before
        length = float(np.linalg.norm(displacement))
        if not .0004 < length / dt < .145 or float(displacement @ direction) < .80 * length:
            return
        self.stroke_m += length
        half = C.TAPE_WIDTH / 2
        polygon = np.asarray([
            center_before - half * across_before,
            center_before + half * across_before,
            pos[:2] + half * across,
            pos[:2] - half * across,
        ])
        # Connect all correctly loaded edge poses into the actual surface strip.
        self.faces.append(polygon)
        delta = np.roll(polygon, -1, axis=0) - polygon
        for candidate, mask in zip(self.ink_samples, self.covered):
            rel = candidate[:, None, :] - polygon[None, :, :]
            side = delta[None, :, 0] * rel[:, :, 1] - delta[None, :, 1] * rel[:, :, 0]
            mask |= (side >= -1e-9).all(axis=1) | (side <= 1e-9).all(axis=1)
        if abs(pos[0]) > .013 or abs(pos[1] - C.ROW_Y) > .0035:
            self.overrun_m2 += C.TAPE_WIDTH * length

    def render(self, stage):
        if len(self.faces) == self.rendered_faces:
            return
        from pxr import Gf, UsdGeom

        film = UsdGeom.Mesh.Get(stage, "/World/envs/env_0/clipboard/Board/TransferredFilm")
        points = [Gf.Vec3f(float(x), float(y), C.PAPER_Z + .00012)
                  for face in self.faces for x, y in face]
        film.GetPointsAttr().Set(points)
        film.GetFaceVertexCountsAttr().Set([4] * len(self.faces))
        film.GetFaceVertexIndicesAttr().Set(list(range(len(points))))
        self.rendered_faces = len(self.faces)

    def snapshot(self):
        return {
            "coverage": [float(area.mean()) for area in self.covered],
            "laid_mm": 1000 * self.stroke_m,
            "overrun_mm2": 1e6 * self.overrun_m2,
            "peak_compression_mm": 1000 * self.max_compression,
            "compression_mm": 1000 * self.current_compression,
            "nose_in_page_m": self.nose,
            "pitch_deg": self.pitch_deg,
        }
