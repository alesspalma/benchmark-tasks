"""Score the released geometric result at the eyelets, not the motion history."""
import numpy as np
from scipy.spatial.transform import Rotation

from . import config as C


def line_state(body):
    pose = np.asarray(body["pose"])
    return pose[:3], Rotation.from_quat(pose[3:]).as_matrix()[:, 1]


def check(body, side):
    center, axis = line_state(body)
    q = C.sign(side)
    axis *= np.sign(axis[1]) * q
    inner = center - axis * (C.LINE_LENGTH / 2)
    outer = center + axis * (C.LINE_LENGTH / 2)
    hole = C.hole(side)
    longitudinal = axis[1]
    crossing = (center + axis * (hole[1] - center[1]) / longitudinal
                if abs(longitudinal) > 0.1 else center)
    depth = q * (hole[1] - inner[1])
    exposed = q * (outer[1] - hole[1])
    center_error = np.linalg.norm((crossing - hole)[[0, 2]])
    aligned = abs(longitudinal) > .990 and center_error < .0024
    step = {
        "depth_mm": round(depth * 1000, 2),
        "exposed_mm": round(exposed * 1000, 2),
        "eyelet_error_mm": round(center_error * 1000, 2),
        "tip_position": inner.tolist(),
    }
    return step, bool(aligned and .031 <= depth <= .043 and .158 <= exposed <= .174)


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.duration = dict.fromkeys(("left", "right"), 0.)
        self.anchor = dict.fromkeys(self.duration)

    def evaluate(self, state, elapsed):
        objects = state["objects"]
        shoulders = np.asarray([point for hand in state["hands"].values()
                                for point in hand["points"]])
        lines = [objects["left_line"], objects["right_line"]]
        metrics = {}
        for side in ("left", "right"):
            readings = [(*check(obj, side), obj) for obj in lines]
            measurement, seated, obj = min(
                readings,
                key=lambda x: (not x[1], x[0]["eyelet_error_mm"]),
            )
            center, axis = line_state(obj)
            offsets = shoulders - center
            along = np.clip(offsets @ axis, -C.LINE_LENGTH / 2, C.LINE_LENGTH / 2)
            distance = np.linalg.norm(offsets - along[:, None] * axis, axis=1).min()
            joint_index = 0 if side == "left" else 1
            tooth = C.sign(side) * objects["attachment"]["joints"][f"GripJoint{joint_index}"]
            gripping = tooth < np.radians(2.0)
            held = seated and gripping and distance > .025
            endpoints = np.stack((center + C.LINE_LENGTH / 2 * axis,
                                  center - C.LINE_LENGTH / 2 * axis))
            anchor = self.anchor[side]
            held_motion = (0. if anchor is None else
                           np.max(np.linalg.norm(endpoints - anchor, axis=1)))
            if not held:
                self.anchor[side] = None
                self.duration[side] = 0.
            elif anchor is None or held_motion > .002:
                self.anchor[side] = endpoints
                self.duration[side] = elapsed
            else:
                self.duration[side] += elapsed
            metrics[side] = {
                **measurement,
                "released": bool(distance > .025),
                "seated": seated,
                "retainer_engaged": bool(gripping),
                "dwell_tip_displacement_mm": round(held_motion * 1000, 3),
                "seated_seconds": round(self.duration[side], 3),
            }
        passed = sum(value >= 1. - 1e-8 for value in self.duration.values())
        metrics.update(score=passed / 2, success=passed == 2)
        return metrics
