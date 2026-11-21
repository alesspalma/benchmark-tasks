"""Arm-neutral measurements of closure, linkage and support."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C

BODY_ARC = np.array([C.ring_point(a) for a in np.linspace(C.PIVOT_ANGLE, 270.0, 81)])
GATE_ARC = np.array(
    [C.ring_point(a) - C.GATE_PIVOT for a in np.linspace(270.4, 394.5, 48)]
)


def transform(points, pose):
    return np.asarray(points) @ quat_xyzw_to_matrix(pose[3:]).T + np.asarray(pose[:3])


def intersections(polyline):
    """Intersections with the plane x=0, expressed on its local y,z axes."""
    a, b = polyline[:-1], polyline[1:]
    crossing = (a[:, 0] >= 0) != (b[:, 0] >= 0)
    a, b = a[crossing], b[crossing]
    blend = -a[:, 0] / (b[:, 0] - a[:, 0])
    return a[:, 1:] + blend[:, None] * (b[:, 1:] - a[:, 1:])


class Scorer:
    def __init__(self):
        self.stable = 0.0
        self.start_height = None

    def evaluate(self, state, elapsed):
        ring = state["objects"]["ring"]
        liner = state["objects"]["liner"]
        ring_pose = np.asarray(ring["links"]["Ring"])
        gate_pose = np.asarray(ring["links"]["Clasp"])
        corner_pose = np.asarray(liner["links"]["Corner"])
        center = transform(C.EYELET, corner_pose)
        if not np.isfinite(np.r_[ring_pose, gate_pose, corner_pose]).all():
            raise RuntimeError("Invalid native rod, ring or liner pose")
        if self.start_height is None:
            self.start_height = float(center[2])
        plane = quat_xyzw_to_matrix(corner_pose[3:])
        ring_frame = quat_xyzw_to_matrix(ring_pose[3:])
        body = transform(BODY_ARC, ring_pose)
        clasp = transform(GATE_ARC, gate_pose)
        crossings = np.concatenate(
            [intersections((arc - center) @ plane) for arc in (body, clasp)]
        )
        holes = np.linalg.norm(crossings, axis=1)
        throat = float(np.min(holes)) if len(holes) else None
        threaded = throat is not None and throat <= C.EYELET_RADIUS - 0.0025
        seam_gap = float(np.linalg.norm(body[-1] - clasp[0]))
        close = abs(ring["joints"]["Snap"]) <= np.deg2rad(4.0) and seam_gap <= 0.00475
        rod_point = np.array([C.ROD[0], ring_pose[1], C.ROD[2]])
        local_rod = (rod_point - ring_pose[:3]) @ ring_frame
        surrounded = (
            (local_rod[0] / 0.040) ** 2 + (local_rod[2] / 0.048) ** 2 <= 0.93
            and abs(ring_frame[1, 1]) >= 0.88
            and -0.37 <= ring_pose[1] <= 1.08
        )
        hands_clear = all(
            min(
                np.linalg.norm(np.asarray(hand["points"]) - p, axis=1).min()
                for p in (ring_pose[:3], center, transform(C.GATE_TAB, gate_pose))
            )
            >= 0.047
            for hand in state["hands"].values()
        )
        rise = float(center[2]) - self.start_height
        supported = rise >= 0.035
        stationary = np.linalg.norm(ring["velocity"][:3]) < 0.035
        passed = (
            threaded
            and surrounded
            and close
            and supported
            and hands_clear
            and stationary
        )
        self.stable = self.stable + elapsed if passed else 0.0
        score = 0.0
        if surrounded:
            score = 0.32
        if surrounded and (close or threaded):
            score = 0.56
        if surrounded and close and threaded:
            score = 0.82
        if self.stable >= 0.70 - 1e-9:
            score = 1.0
        return {
            "score": score,
            "success": score == 1.0,
            "closed": bool(close),
            "rod_inside": bool(surrounded),
            "corner_interlinked": bool(threaded),
            "throat_distance_mm": None if throat is None else round(1000 * throat, 3),
            "ring_center": ring_pose[:3].tolist(),
            "eyelet_center": center.tolist(),
            "clasp_seam_mm": round(1000 * seam_gap, 3),
            "snap_angle_deg": float(np.rad2deg(ring["joints"]["Snap"])),
            "fold_angle_deg": float(np.rad2deg(liner["joints"]["Fold"])),
            "supported": supported,
            "rise_mm": round(1000 * rise, 3),
            "hands_clear": hands_clear,
            "stable_s": self.stable,
        }
