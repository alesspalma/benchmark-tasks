"""Assembly and real chain routing, measured from the current native state."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix
from .config import (
    PITCH,
    TABLE_TOP,
    CHAIN_Z,
    SPROCKET_CENTERS,
)


def rotation(pose):
    return quat_xyzw_to_matrix(np.asarray(pose[3:]))


def chain_nodes(s):
    nodes = []
    for i in range(11):
        p = s["bodies"][f"Link{i:02}"]["pose"]
        r = rotation(p)
        if i == 0:
            nodes.append(np.asarray(p[:3]) + r @ np.array([-PITCH / 2, 0, 0]))
        nodes.append(np.asarray(p[:3]) + r @ np.array([PITCH / 2, 0, 0]))
    return np.array(nodes)


def cross(a, b):
    return float(a[0] * b[1] - a[1] * b[0])


def simple_loop(p):
    for i in range(len(p)):
        incoming = p[i] - p[i - 1]
        outgoing = p[(i + 1) % len(p)] - p[i]
        if abs(cross(incoming, outgoing)) < 1e-8 and np.dot(incoming, outgoing) < 0:
            return False
    for i in range(len(p)):
        a, b = p[i], p[(i + 1) % len(p)]
        if np.linalg.norm(a - b) < 0.001:
            return False
        for j in range(i + 1, len(p)):
            if j == i + 1 or (i == 0 and j == len(p) - 1):
                continue
            c, d = p[j], p[(j + 1) % len(p)]
            u, v, w, z = (
                cross(b - a, c - a),
                cross(b - a, d - a),
                cross(d - c, a - c),
                cross(d - c, b - c),
            )
            if u * v < -1e-12 and w * z < -1e-12:
                return False
            if max(abs(u), abs(v), abs(w), abs(z)) < 1e-8:
                axis = int(np.argmax(abs(b - a)))
                overlap = min(max(a[axis], b[axis]), max(c[axis], d[axis])) - max(
                    min(a[axis], b[axis]), min(c[axis], d[axis])
                )
                if overlap > 1e-5:
                    return False
    return True


def routing(s):
    nodes = chain_nodes(s)
    p = nodes[:, :2]
    simple = simple_loop(p)
    windings = []
    for center in SPROCKET_CENTERS:
        v = p - center
        w = np.roll(v, -1, axis=0)
        windings.append(
            float(
                np.arctan2(v[:, 0] * w[:, 1] - v[:, 1] * w[:, 0], (v * w).sum(1)).sum()
                / (2 * np.pi)
            )
        )
    seats = []
    for i, center in enumerate(SPROCKET_CENTERS):
        angle = float(s.get("wheel_angles", {}).get(f"wheel{i}", 0))
        phases = np.deg2rad(np.arange(30, 360, 60)) + angle
        outer = np.cos(phases) <= 1e-8 if i == 0 else np.cos(phases) >= -1e-8
        points = center + 0.100 * np.c_[np.cos(phases[outer]), np.sin(phases[outer])]
        seats.append(np.linalg.norm(points[:, None] - p[None], axis=-1).min(-1))
    height = float(abs(nodes[:, 2] - CHAIN_Z).max())
    normals = all(rotation(v["pose"])[2, 2] > 0.995 for v in s["bodies"].values())
    ok = bool(
        simple
        and all(abs(abs(w) - 1) < 0.01 for w in windings)
        and all((v <= 0.010).all() for v in seats)
        and height <= 0.006
        and normals
    )
    return dict(
        chain_routed=ok,
        loop_simple=bool(simple),
        wheel_windings=windings,
        sprocket_seat_errors_m=[v.tolist() for v in seats],
        chain_height_error_m=height,
    )


def assembly_metrics(s):
    c, k = np.asarray(s["connector"]["pose"]), np.asarray(s["keeper"]["pose"])
    r, kr = rotation(c), rotation(k)
    ends, passed = {}, []
    for name, sign in (("Link00",-1),("Link10",1)):
        p = np.asarray(s["bodies"][name]["pose"])
        local = r.T @ (p[:3] + rotation(p) @ np.array([sign*PITCH/2,0,0]) - c[:3])
        ends[name] = local.tolist()
        passed.append(np.linalg.norm(local[:2]-[0,sign*PITCH/2]) <= 0.007
                      and 0.010 < local[2] < 0.022
                      and np.dot(rotation(p)[:,2], r[:,2]) > 0.99)
    joined = bool(all(passed))
    local = r.T @ (k[:3]-c[:3])
    aligned = np.trace(r.T@kr) >= 1+2*np.cos(0.10)
    pins = [kr.T@(c[:3]+r@np.array([0,y,0.032])-k[:3]) for y in (-PITCH/2,PITCH/2)]
    slots = all(-0.0105 < p[0] < 0.022 and abs(p[1]-y) < 0.002
                for p,y in zip(pins,(-PITCH/2,PITCH/2)))
    ends_ok = all(-0.0105 < p[0] < -0.006 and abs(p[1]-y) < 0.002
                  for p,y in zip(pins,(-PITCH/2,PITCH/2)))
    covered = bool(joined and 0.030 < local[2] < 0.034 and aligned and slots)
    return dict(ends_joined=joined,plate_seated=covered,retained=bool(covered and ends_ok),
                end_points_local=ends,keeper_local=local.tolist(),
                pins_in_keeper=[p.tolist() for p in pins])


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.violations = set()
        self.physics_samples = 0
        self.hold_start = None
        self._clear_motion()

    def _clear_motion(self):
        self.motion_previous = None
        self.axle_travel = np.zeros(2)
        self.link_travel = 0.0
        self.max_transmission_error = 0.0
        self.motion_samples = 0

    def observe_physics(self, points):
        self.physics_samples += 1
        for p in points.values():
            if not np.isfinite(p).all():
                self.violations.add("invalid_state")
            elif (p[:,2].min() < TABLE_TOP-0.006 or p[:,0].min() < -0.19
                  or p[:,0].max() > 0.91 or abs(p[:,1]).max() > 0.65):
                self.violations.add("part_left_work_area")

    def observe_motion(self, s):
        # Measured each native physics step. A broken/derailed connection discards
        # the entire proof; a later reassembly must complete a new circuit.
        parts = [s["connector"],s["keeper"],*s["bodies"].values()]
        if not all(np.isfinite(p["pose"]).all() for p in parts):
            self.violations.add("invalid_state")
            self._clear_motion()
            return
        a, route = assembly_metrics(s), routing(s)
        if self.violations or not a["retained"] or not route["chain_routed"]:
            self._clear_motion()
            return
        wheels = np.array([s["wheel_angles"][f"wheel{i}"] for i in range(2)])
        p = np.asarray(s["connector"]["pose"][:2]) - SPROCKET_CENTERS.mean(0)
        phase = np.arctan2(p[1],p[0])
        current = np.r_[wheels,phase]
        if not np.isfinite(current).all():
            self.violations.add("invalid_state")
            self._clear_motion()
            return
        if self.motion_previous is not None:
            delta = (current-self.motion_previous+np.pi)%(2*np.pi)-np.pi
            if abs(delta).max() > 0.020:
                self._clear_motion()
            else:
                self.axle_travel += delta[:2]
                self.link_travel += delta[2]
                self.max_transmission_error = max(self.max_transmission_error,
                    abs(self.axle_travel[0]-self.axle_travel[1]))
                self.motion_samples += 1
        self.motion_previous = current

    def evaluate(self,s):
        parts = [s["connector"],s["keeper"],*s["bodies"].values()]
        parts += [p for v in s.get("wheels",{}).values() for p in v.values()]
        finite = all(np.isfinite(p["pose"]).all() and np.isfinite(p["velocity"]).all() for p in parts)
        finite = finite and np.isfinite(s["robot_clearance_m"]) and all(
            np.isfinite(h["opening_m"]) for h in s["hands"].values())
        if not finite:
            self.violations.add("invalid_state")
            self.hold_start = None
            self._clear_motion()
            return dict(score=0.0,success=False,violations=sorted(self.violations),
                        ends_joined=False,plate_seated=False,retained=False,
                        chain_routed=False,closed_loop=False,physics_samples=self.physics_samples)
        a, route = assembly_metrics(s), routing(s)
        closed = bool(a["retained"] and route["chain_routed"])
        if not closed:
            self._clear_motion()
        drive_test = bool(closed and self.motion_samples >= 240
            and abs(self.axle_travel[0]) >= 4*np.pi-0.12
            and abs(self.axle_travel[1]) >= 4*np.pi-0.20
            and self.axle_travel.prod() > 0
            and self.max_transmission_error <= 0.15
            and abs(self.link_travel) >= 2*np.pi-0.10
            and self.link_travel*self.axle_travel[0] > 0)
        settled = all(np.linalg.norm(p["velocity"][:3]) < 0.02
                      and np.linalg.norm(p["velocity"][3:]) < 0.3 for p in parts)
        clear = bool(s["robot_clearance_m"] >= 0.025
                     and all(h["opening_m"] >= 0.070 for h in s["hands"].values()))
        full = bool(drive_test and settled and clear and not self.violations)
        if full:
            if self.hold_start is None:
                self.hold_start=s["time_s"]
        else:
            self.hold_start=None
        success=bool(full and s["time_s"]-self.hold_start >= 1)
        score = ((1.0 if success else 0.7 if a["plate_seated"] else 0.4 if a["ends_joined"] else 0.0)
                 if route["chain_routed"] and not self.violations else 0.0)
        return dict(score=score,success=success,violations=sorted(self.violations),
            closed_loop=closed,drive_test_passed=drive_test,settled=bool(settled),
            hands_clear=clear,robot_clearance_m=s["robot_clearance_m"],
            axle_travel_rad=self.axle_travel.tolist(),link_circuit_rad=float(self.link_travel),
            max_transmission_error_rad=float(self.max_transmission_error),
            motion_samples=self.motion_samples,physics_samples=self.physics_samples,**a,**route)
