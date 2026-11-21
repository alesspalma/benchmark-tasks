"""Check-valved pressure and bounded contact loads for the gel and suction cup."""

from math import exp

import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


def surface(obj):
    pose = np.asarray(obj["pose"])
    r = quat_xyzw_to_matrix(pose[3:])
    return pose[:3], r, np.asarray(obj["velocity"])


def bounded(vector, limit):
    magnitude = np.linalg.norm(vector)
    return vector * min(1., limit / max(magnitude, 1e-12))


def couple(part, carrier, local, relative_rotation, force_limit, stiffness=8.0):
    p, r, velocity = part
    parent, parent_r, parent_v = carrier
    arm = parent_r @ local
    position = parent + arm
    linear = parent_v[:3] + np.cross(parent_v[3:], arm)
    force = stiffness*(position-p) + .061*(linear-velocity[:3])
    turn = Rotation.from_matrix(parent_r @ relative_rotation @ r.T).as_rotvec()
    torque = .000067*turn + .00000054*(parent_v[3:]-velocity[3:])
    return bounded(force, force_limit), bounded(torque, force_limit*.003), position


class Vacuum:
    def __init__(self):
        self.pressure = 0.0
        self.peak = 0.0
        self.seated = False
        self.handle = 0.0
        self.swept_strokes = 0.0
        self.vented = False
        self.bulb = 0.0
        self.purged = False
        self.seal_time = dict.fromkeys(C.DIE_NAMES, 0.)
        self.tacked = set(C.DIE_NAMES)
        self.held = None
        self.lifted = set()
        self.last_hold = None
        self.retained = 0.0
        self.max_neighbor_shift = 0.0
        self.target_pickup_vacuum = None

    def sample_pressure(self, objects, tray, dt):
        p, r, _ = tray
        pump = objects["pump"]["joints"]
        travel = np.clip(-pump["HandleAngle"] / C.PUMP_STROKE, 0., 1.)
        increment = max(0., travel-self.handle)
        self.handle = float(travel)
        self.seated = bool(
            np.linalg.norm((p-C.TRAY_ON_PLATE)[:2]) <= .0038
            and abs(p[2]-C.TRAY_ON_PLATE[2]) <= .0028
            and r[2, 2] >= .997
            and r[0, 0] >= .988
        )
        vent = -pump["VentTravel"] >= .0035
        if not self.seated:
            self.pressure *= exp(-dt/.022)
        elif vent:
            self.vented |= self.target_pickup_vacuum is not None
            self.pressure *= exp(-dt/.018)
        elif increment:
            self.swept_strokes += increment
            self.pressure = min(25.2, 29.92 - (29.92 - self.pressure)*exp(-.43*increment))
        self.peak = max(self.peak, self.pressure)

    def sample_bulb(self, objects, pen, parts, dt):
        travel = float(-objects["pen"]["joints"]["BulbTravel"])
        pen_p, pen_r, _ = pen
        tip = pen_p + pen_r @ C.PEN_TIP
        sealed = []
        for name, (p, r, _) in parts.items():
            # The lip must fit the polished face and stay square long enough for
            # the cup to seal. The height interval models lip compression.
            local = r.T @ (tip-p)
            gap = local[2] - C.DIE_THICKNESS/2
            square = (
                np.max(np.abs(local[:2])) <= C.DIE_WIDTH/2-C.PEN_RADIUS
                and -.0010 <= gap <= .0014
                and pen_r[:, 2] @ r[:, 2] >= .981
            )
            self.seal_time[name] = self.seal_time[name]+dt if square else 0.
            if self.seal_time[name] >= .018:
                sealed.append((abs(gap), name, p, r))
        if travel >= C.BULB_PURGE:
            self.purged = True
            self.held = None
        elif self.purged and self.bulb > C.BULB_RELEASE >= travel:
            self.purged = False
            if sealed:
                _, name, p, r = min(sealed, key=lambda item: item[0])
                relative = pen_r.T @ (p-pen_p)
                relative[2] = C.PEN_TIP[2] - C.DIE_THICKNESS/2
                self.held = (name, relative, pen_r.T @ r)
        self.bulb = travel

    def step(self, objects, dt):
        tray = surface(objects["tray"])
        pen = surface(objects["pen"])
        parts = {name: surface(objects[name]) for name in C.DIE_NAMES}
        self.sample_pressure(objects, tray, dt)
        self.sample_bulb(objects, pen, parts, dt)
        forces = []
        tray_pos, tray_rot, _ = tray
        releasing = self.seated and self.pressure >= C.VACUUM_INHG
        for name, part in parts.items():
            p, r, _ = part
            local = tray_rot.T @ (p-tray_pos)
            expected = C.DIE_OFFSETS[name] + [0., 0., C.DIE_THICKNESS/2]
            difference = local - expected
            if name != "selected":
                self.max_neighbor_shift = max(self.max_neighbor_shift, np.linalg.norm(difference[:2]))
            if name in self.tacked:
                if (np.linalg.norm(difference) > (.0028 if releasing else .007)
                        or r[:, 2] @ tray_rot[:, 2] < .89):
                    self.tacked.remove(name)
                else:
                    force, torque, position = couple(
                        part, tray, expected, np.eye(3), .0018 if releasing else .071
                    )
                    forces.append((name, force, torque, p))
                    forces.append(("tray", -force, -torque, position))
            if self.held is not None and name == self.held[0]:
                _, offset, orientation = self.held
                position = pen[0] + pen[1] @ offset
                if np.linalg.norm(position-p) > .008 or r[:, 2] @ pen[1][:, 2] < .90:
                    self.held = None
                    continue
                force, torque, position = couple(part, pen, offset, orientation, .024, stiffness=10.)
                forces.append((name, force, torque, p))
                forces.append(("pen", -force, -torque, position))
                if releasing and difference[2] >= .014 and name not in self.tacked:
                    self.lifted.add(name)
                    if name == "selected" and self.target_pickup_vacuum is None:
                        self.target_pickup_vacuum = self.pressure
        self.last_hold = None if self.held is None else self.held[0]
        self.retained = min(self.retained + dt, .6) if releasing else 0.
        return forces


class Scorer:
    def reset(self):
        self.settled = 0.

    def __init__(self):
        self.reset()

    def evaluate(self, state, vacuum, elapsed):
        objects = state["objects"]
        tray, tray_r, _ = surface(objects["tray"])
        p, r, speed = surface(objects["selected"])
        relative = p-C.RECEIVER
        done = (
            np.max(np.abs(relative[:2])) <= (C.POCKET_WIDTH-C.DIE_WIDTH)/2
            and abs(relative[2]-C.DIE_THICKNESS/2) <= .002
            and r[2, 2] >= .985
            and np.linalg.norm(speed[:3]) < .015
        )
        neighbors = []
        for name in C.DIE_NAMES[1:]:
            p_other, r_other, _ = surface(objects[name])
            error = tray_r.T@(p_other-tray)-C.DIE_OFFSETS[name]
            neighbors.append(
                np.linalg.norm(error[:2]) <= .0025 and abs(error[2]-.0004) <= .0015
                and r_other[:, 2] @ tray_r[:, 2] >= .985
            )
        clean = all(neighbors) and vacuum.lifted.issubset({"selected"}) and vacuum.max_neighbor_shift <= .003
        selected = vacuum.target_pickup_vacuum is not None
        pen_p, pen_r, _ = surface(objects["pen"])
        tip = pen_p + pen_r @ C.PEN_TIP
        finished = (
            clean and selected and done and vacuum.last_hold is None and vacuum.seated
            and vacuum.pressure < 2.0 and vacuum.vented and np.linalg.norm(tip-p) >= .035
        )
        self.settled = self.settled+elapsed if finished else 0.
        score = 0.
        if clean:
            if vacuum.retained >= .20 and vacuum.seated and vacuum.tacked == set(C.DIE_NAMES):
                score = .25
            if selected and vacuum.seated:
                score = .75 if done and vacuum.last_hold is None else .50
            if self.settled >= .45:
                score = 1.
        return {
            "score": score, "success": score == 1., "vacuum_inhg": round(vacuum.pressure, 3),
            "peak_vacuum_inhg": round(vacuum.peak, 3), "plate_seated": vacuum.seated,
            "swept_strokes": round(vacuum.swept_strokes, 3), "pen_squeeze_mm": round(vacuum.bulb*1000., 3),
            "pen_holding": vacuum.last_hold, "pickup_vacuum_inhg": vacuum.target_pickup_vacuum,
            "selected_in_pocket": bool(done), "neighbors_intact": bool(clean),
            "neighbor_shift_mm": round(vacuum.max_neighbor_shift*1000., 4),
            "vented_after_pickup": vacuum.vented, "settled_s": self.settled,
        }
