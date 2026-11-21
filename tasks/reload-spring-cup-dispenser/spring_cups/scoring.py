"""Score visible nesting, a held ready lip and one cup on the serving mat."""

import numpy as np

from . import config as C


def upright(pose):
    x, y, _, _ = pose[3:]
    return 1 - 2*(x*x + y*y) > .965


class Scorer:
    def reset(self):
        self.loaded = False
        self.release_s = 0.
        self.final_s = 0.
        self.stack_ids = ()
        self.stack_reference = np.empty((0, 3))

    def evaluate(self, state, elapsed):
        cups = [state["objects"][f"cup{i}"] for i in range(C.NUM_CUPS)]
        poses = np.asarray([cup["pose"] for cup in cups])
        speed = np.linalg.norm([c["velocity"][:3] for c in cups], axis=1)
        seated = (
            (np.linalg.norm(poses[:, :2] - C.DISPENSER, axis=1) <= .009)
            & np.apply_along_axis(upright, 1, poses)
            & (poses[:, 2] >= C.TABLE_Z - C.TUBE_LENGTH + .012)
            & (poses[:, 2] < C.FINGER_Z + .013)
        )
        ids = np.flatnonzero(seated)
        order = ids[np.argsort(poses[ids, 2])]
        stack = poses[order]
        height = float(stack[-1, 2]) if len(stack) else None
        ready = bool(len(stack) >= 2 and np.all(np.abs(np.diff(stack[:, 2])-C.CUP_PITCH) <= .0045)
                     and C.FINGER_Z-.032 <= height <= C.FINGER_Z+.003)
        stable = (tuple(order) == self.stack_ids and np.all(
            np.linalg.norm(stack[:, :3] - self.stack_reference, axis=1) <= .003
        ))
        if not stable or not ready:
            self.stack_ids = tuple(order)
            self.stack_reference = stack[:, :3].copy()
        supported = ready and stable
        placed = (
            (np.max(np.abs(poses[:, :2] - C.MAT), axis=1) <= .024)
            & np.apply_along_axis(upright, 1, poses)
            & (np.abs(poses[:, 2] - C.TABLE_Z - .002) <= .006)
            & (speed < .035)
        )
        points = np.array([p for hand in state["hands"].values() for p in hand["points"][:2]])
        clear_dispenser = self.clear_hands(points, C.DISPENSER, C.FINGER_Z + C.CUP_HEIGHT + .031)
        clear_mat = self.clear_hands(points, C.MAT, C.TABLE_Z + C.CUP_HEIGHT + .038)
        hands_clear = clear_dispenser and clear_mat
        self.release_s = self.release_s + elapsed if supported and len(stack) == C.NUM_CUPS and clear_dispenser else 0.
        self.loaded |= self.release_s >= .5 - 1e-8
        finished = self.loaded and supported and len(stack) == C.NUM_CUPS-1 and placed.sum() == 1 and hands_clear
        self.final_s = self.final_s + elapsed if finished else 0.
        score = 0.
        if supported and clear_dispenser:
            score = .25
        if self.release_s >= .5 - 1e-8:
            score = .65
        if self.final_s >= .5 - 1e-8:
            score = 1.
        return {
            "score": score,
            "success": score == 1.,
            "seated_cups": len(stack),
            "cups_on_mat": int(placed.sum()),
            "top_lip_mm": None if height is None else (height - C.TABLE_Z)*1000,
            "follower_compression_mm": -1000*state["objects"]["follower"]["joints"]["FollowerTravel"],
            "retainer_deflection_mm": [1000*state["objects"][f"retainer{i}"]["joints"]["RetainerTravel"] for i in range(3)],
            "nested_ready": supported,
            "hands_clear": hands_clear,
            "verified_load": self.loaded,
            "hands_off_final_s": self.final_s,
        }

    @staticmethod
    def clear_hands(points, center, safe_z):
        near = np.linalg.norm(points[:, :2] - center, axis=1) <= .082
        return not bool(np.any(near & (points[:, 2] < safe_z)))
