"""Measure the actual printed planes, spring-pad seating and hands-off hold."""

import numpy as np
from scipy.spatial.transform import Rotation

from . import config as C


def transform(physics_pose):
    return np.asarray(physics_pose[:3]), Rotation.from_quat(physics_pose[3:])


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.hold = {"left": 0., "right": 0.}

    def evaluate(self, state, elapsed):
        book = state["objects"]["book"]
        stand = state["objects"]["stand"]
        stand_root, stand_rot = transform(stand["pose"])
        stand_origin = stand_root + stand_rot.apply(C.STAND_ORIGIN)
        frame = stand_rot * Rotation.from_matrix(C.FRAME)
        normal = frame.apply([1., 0., 0.])

        local = frame.inv().apply(np.asarray(book["pose"][:3]) - stand_origin)
        seated = bool(abs(local[0] + .004) <= .015 and abs(local[1]) <= .020
                      and -.006 <= local[2] <= .012)

        def clear(point):
            x, y, z = frame.inv().apply(np.asarray(point) - stand_origin)
            return x < -.105 or abs(y) > .200 or z > .257

        free = all(clear(point) for hand in state["hands"].values() for point in hand["points"])
        pivot_deg, page_deg, pad_gap_mm, flex_mm, held = {}, {}, {}, {}, {}
        for i, side in enumerate(self.hold):
            page_position, page_rot = transform(book["links"][f"Leaves{i}"])
            shoe_position, shoe_rot = transform(stand["links"][f"Shoe{i}"])
            shoe_center = shoe_position + shoe_rot.apply([.0065, 0., 0.])
            pad = page_rot.inv().apply(shoe_center - page_position)
            pad_axes = (page_rot.inv() * shoe_rot).as_matrix()
            pad_reach = abs(pad_axes[0]) @ np.array([.0035, .0065, .006])
            gap = -.01375 - pad[0] - pad_reach
            signed_y = C.sign(side) * pad[1]

            pivot_deg[side] = C.CLIP_START - C.sign(side) * np.degrees(stand["joints"][f"Pivot{i}"])
            page_deg[side] = float(np.degrees(np.arccos(np.clip(normal @ page_rot.apply([1., 0., 0.]), -1., 1.))))
            flex_mm[side] = 1000. * stand["joints"][f"Spring{i}"]
            pad_gap_mm[side] = float(1000. * gap)
            held[side] = bool(
                seated and free and 72. <= pivot_deg[side] <= 105.
                and page_deg[side] <= 10.
                and .020 <= signed_y <= .140 and .018 <= pad[2] <= .110
                and abs(gap) <= .0015 and flex_mm[side] <= 2.2
            )
            self.hold[side] = self.hold[side] + elapsed if held[side] else 0.

        score = sum(t >= .999 for t in self.hold.values()) * .5
        return dict(score=score, success=score == 1., seated=seated, hands_clear=free,
                    retained=held, hold_s=self.hold.copy(), pivot_deg=pivot_deg,
                    page_deg=page_deg, pad_gap_mm=pad_gap_mm, flex_mm=flex_mm,
                    spine_mm=(local * 1000.).tolist())
