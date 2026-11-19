"""Physical coverage, under-film order, flatness and release scoring."""

import itertools

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as c

LOCAL = np.array(list(itertools.product(
    (-c.PAPER_HALF, 0., c.PAPER_HALF),
    (-c.PAPER_WIDTH / 2, 0., c.PAPER_WIDTH / 2),
    (0.,),
)))


class Scorer:
    def reset(self):
        self.since = None

    def evaluate(self, state):
        paper = state["objects"]["page"]
        pose = np.asarray(paper["pose"])
        rotation = quat_xyzw_to_matrix(pose[3:])
        samples = LOCAL @ rotation.T + pose[:3]
        x, y, z = samples.T
        upper = state["objects"]["sleeve"]["links"]["UpperFilm"]
        upper_rotation = quat_xyzw_to_matrix(upper[3:])
        under = (samples - upper[:3]) @ upper_rotation[:, 2] <= -.00025
        across = np.abs(y - c.SLEEVE_Y) <= c.INNER_HALF_WIDTH + .0005
        within = (x >= c.MOUTH) & (x <= c.FAR_EDGE) & across
        flat = float(np.ptp(z)) < .006 and abs(float(rotation[2, 2])) > .985
        low = float(np.min(z)) >= c.LOWER_FACE - .0008
        still = float(np.linalg.norm(paper["velocity"])) < .09
        closed = abs(state["objects"]["sleeve"]["joints"]["FilmFlex"]) <= .018
        released = all(
            all(min(float(np.linalg.norm(np.asarray(point) - sample))
                    for sample in samples) >= .035
                for point in hand["points"][:2])
            for hand in state["hands"].values()
        )
        complete = bool(np.all(within & under) and flat and low and still and closed and released)
        t = state["time_s"]
        self.since = (t if self.since is None else self.since) if complete else None
        success = self.since is not None and t - self.since >= .45
        coverage = float(np.sum(within & under)) / len(samples)
        partial = .5 if released and flat and low and 0.55 <= coverage < 1.0 else 0.
        return {
            "score": 1.0 if success else partial,
            "success": success,
            "inside_samples": int(np.sum(within & under)),
            "paper_lowest_m": float(np.min(z)),
            "paper_highest_m": float(np.max(z)),
            "trailing_edge_m": float(np.min(x) - c.MOUTH),
            "furthest_edge_m": float(np.max(x) - c.MOUTH),
            "lip_angle_rad": float(state["objects"]["sleeve"]["joints"]["FilmFlex"]),
            "sheet_flat": flat,
            "cover_closed": closed,
            "hands_clear": released,
        }
