"""Contact-dependent vacuum retention and loaded outcome rubric."""

import numpy as np

from . import config as C

LOCK_ANGLE = np.deg2rad(13.0)
LIP_MIN_NORMAL = np.cos(np.deg2rad(8.0))
MAX_NORMAL = 30.0
MAX_SHEAR = 9.0


def pose_rotation(pose):
    x, y, z, w = pose[3:7]
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ]
    )


def smooth_tile(face):
    """A suction pad must fit wholly on continuous glazed tile."""
    margin = C.CUP_RADIUS + C.GROUT_WIDTH / 2
    return (
        -0.365 + margin < face[1] < 0.365 - margin
        and 0.732 + margin < face[2] < 1.156 - margin
        and min(abs(face[1] - j) for j in C.SEAM_Y) > margin
        and min(abs(face[2] - j) for j in C.SEAM_Z) > margin
    )


class SuctionCups:
    def __init__(self):
        self.anchors = [None, None]
        self.near_contact = [False, False]
        self.compression = [0.0, 0.0]
        self.angles = [np.deg2rad(C.LEVER_ANGLE_OPEN)] * 2
        self.forces = np.zeros((2, 3))
        self.seated_while_open = [False, False]

    def step(self, obj, body_com):
        angles = np.array([obj["joints"][f"Cam{i}"] for i in range(2)])
        self.angles = angles.tolist()
        root = np.asarray(obj["pose"][:3]) + pose_rotation(obj["pose"]) @ body_com
        vel = np.asarray(obj["velocity"])
        force = np.zeros(3)
        torque = np.zeros(3)
        detents = []
        for i in range(2):
            pose = obj["links"][f"Lip{i}"]
            r = pose_rotation(pose)
            face = np.asarray(pose[:3]) + C.LIP_FORWARD * r[:, 0]
            compress = -obj["joints"][f"Compress{i}"]
            self.compression[i] = float(compress)
            clearance = C.WALL_X - face[0]
            allowable = smooth_tile(face) and r[0, 0] > LIP_MIN_NORMAL
            contact = (
                allowable and abs(clearance) < 0.0015 and compress >= C.MIN_COMPRESSION
            )
            self.near_contact[i] = bool(contact)
            locked = angles[i] < LOCK_ANGLE
            anchor = self.anchors[i]
            self.forces[i] = 0.0
            if not locked:
                self.seated_while_open[i] = contact
            elif anchor is None:
                # The cam can create vacuum only when its closing stroke
                # crosses the lock point with the pad compressed on tile.
                if contact and self.seated_while_open[i]:
                    anchor = face.copy()
                    anchor[0] = C.WALL_X
                    self.anchors[i] = anchor
                self.seated_while_open[i] = False
            if anchor is not None:
                error = anchor - face
                displacement = face - root
                point_vel = vel[:3] + np.cross(vel[3:], displacement)
                # The sealed contact has finite normal pull and finite shear.
                pull = 8.5 + 2100.0 * error[0] - 12.0 * point_vel[0]
                shear = 950.0 * error[1:] - 8.0 * point_vel[1:]
                if (
                    not locked
                    or not allowable
                    or clearance > 0.004
                    or np.linalg.norm(error[1:]) > 0.012
                    or pull > MAX_NORMAL
                    or np.linalg.norm(shear) > MAX_SHEAR
                ):
                    self.anchors[i] = None
                else:
                    f = np.r_[max(0.0, pull), shear]
                    self.forces[i] = f
                    force += f
                    torque += np.cross(displacement, f)
            # The lever's over-centre spring detent reverses after the cam crest.
            # Actual finger contact must carry it over the centre.
            axis = pose_rotation(obj["links"][f"Lever{i}"])[:, 1]
            snap = 0.033 if angles[i] > np.deg2rad(30.0) else -0.041
            detents.append(snap * axis)
            torque -= detents[-1]
        return force, torque, detents


class Scorer:
    def __init__(self):
        self.unassisted_s = 0.0

    def evaluate(self, state, cups, elapsed):
        basket = state["objects"]["basket"]
        bottle = state["objects"]["bottle"]
        rot = pose_rotation(basket["pose"])
        d = rot.T @ (np.asarray(bottle["pose"][:3]) - basket["pose"][:3])
        bottle_up = pose_rotation(bottle["pose"])[:, 2]
        mounted = (
            0.807 < basket["pose"][2] < 0.882
            and rot[0, 0] > LIP_MIN_NORMAL
            and rot[2, 2] > 0.987
        )
        sealed = [anchor is not None for anchor in cups.anchors]
        bottle_in = (
            abs(d[0]) < 0.035
            and abs(d[1]) < 0.099
            and 0.001 < d[2] < 0.020
            and np.dot(bottle_up, rot[:, 2]) > 0.97
        )
        # Grippers are clear of the basket, bottle and both cam tabs.
        clear = True
        for values in state["hands"].values():
            for point in values["points"]:
                q = rot.T @ (np.asarray(point) - basket["pose"][:3])
                if (
                    -0.087 < q[0] < 0.105
                    and abs(q[1]) < 0.174
                    and -0.020 < q[2] < 0.162
                ):
                    clear = False
        steady = all(
            np.linalg.norm(obj["velocity"][:3]) < 0.021
            and np.linalg.norm(obj["velocity"][3:]) < 0.22
            for obj in (basket, bottle)
        )
        if all(sealed) and mounted and bottle_in and clear and steady:
            self.unassisted_s += elapsed
        else:
            self.unassisted_s = 0.0

        score = 0.25 * sum(sealed) if mounted else 0.0
        if score == 0.50 and bottle_in:
            score = 0.75
        if self.unassisted_s >= 0.70:
            score = 1.0
        return {
            "score": score,
            "success": score == 1.0,
            "closed_levers": [q < LOCK_ANGLE for q in cups.angles],
            "sealed_cups": sealed,
            "lip_compression_m": cups.compression.copy(),
            "smooth_lip_contact": cups.near_contact.copy(),
            "force_n": cups.forces.tolist(),
            "bottle_in_basket": bool(bottle_in),
            "hands_clear": clear,
            "unassisted_s": round(self.unassisted_s, 3),
        }
