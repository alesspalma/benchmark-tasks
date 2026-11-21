"""Mass-conserving water flow and solid retention at the perforated plate."""

from dataclasses import dataclass
from math import pi

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C

BEAD_COUNT = 36
# Small volumes prime a thin Grade 1 cellulose disc on the running vacuum.
PRIME_ML = 6.0
GRAVITY = np.array([0.0, 0.0, -9.81])


def pose_parts(pose):
    return np.asarray(pose[:3]), quat_xyzw_to_matrix(np.asarray(pose[3:]))


def paper_seated(state):
    center, axes = pose_parts(state["objects"]["paper"]["pose"])
    return bool(
        np.linalg.norm(center[:2] - C.FUNNEL) <= 0.0045
        and abs(center[2] - C.PLATE - C.PAPER_THICK / 2) <= 0.0016
        and axes[2, 2] >= 0.996
    )


@dataclass
class Packet:
    source: str
    ml: float
    position: np.ndarray
    velocity: np.ndarray


class Source:
    """A cylindrical jug with a horizontal free surface and lowered lip."""

    def __init__(self, vessel):
        self.vessel = vessel
        self.ml = vessel.fill_ml
        sample, weights = np.polynomial.legendre.leggauss(15)
        height = vessel.height - 0.004
        self.z = 0.004 + (sample + 1) * height / 2
        self.weights = weights * height / 2
        self.radius = vessel.radius - 0.003
        self.level = 0.004 + self.ml * 1e-6 / (pi * self.radius**2)

    def capacity(self, axes, level):
        slope = np.linalg.norm(axes[2, :2])
        z_axis = axes[2, 2]
        r = self.radius
        if slope < 0.000001:
            area = pi * r * r
            if z_axis > 0:
                return float(
                    area * np.clip(level - 0.004, 0, self.vessel.height - 0.004) * 1e6
                )
            return float(
                area
                * np.clip(self.vessel.height + level, 0, self.vessel.height - 0.004)
                * 1e6
            )
        cut = np.clip((level - self.z * z_axis) / (r * slope), -1.0, 1.0)
        area = r * r * (np.arccos(-cut) + cut * np.sqrt(1 - cut * cut))
        return float(self.weights @ area * 1e6)

    def update(self, pose, velocity, dt):
        center, axes = pose_parts(pose)
        rim_local = self.vessel.outlet
        rim_height = axes[2] @ rim_local
        horizontal = axes[2, :2]
        side_height = axes[2, 2] * self.vessel.height - self.radius * np.linalg.norm(
            horizontal
        )
        if side_height + 0.001 < rim_height:
            lowest_xy = (
                -horizontal * self.radius / max(1e-8, np.linalg.norm(horizontal))
            )
            rim_local = np.r_[lowest_xy, self.vessel.height]
            rim_height = side_height
        overflow = max(0.0, self.ml - self.capacity(axes, rim_height))
        amount = min(self.ml, overflow * min(1.0, 4.9 * dt), 43.0 * dt)
        if amount <= 0.000005:
            amount = 0.0
        self.ml -= amount
        if axes[2, 2] > 0.99999:
            self.level = 0.004 + self.ml * 1e-6 / (pi * self.radius**2)
        else:
            low, high = -0.14, 0.14
            for _ in range(13):
                level = (low + high) / 2
                if self.capacity(axes, level) < self.ml:
                    low = level
                else:
                    high = level
            self.level = (low + high) / 2
        if not amount:
            return None
        lip = center + axes @ rim_local
        flow_velocity = np.asarray(velocity[:3]) + 0.16 * np.r_[axes[:2, 2], 0]
        return Packet(self.vessel.name, amount, lip, flow_velocity)


class Filtration:
    def __init__(self):
        self.sources = {v.name: Source(v) for v in C.VESSELS}
        self.packets = []
        self.paths = []
        self.funnel = {v.name: 0.0 for v in C.VESSELS}
        self.flask = {v.name: 0.0 for v in C.VESSELS}
        self.spill = {v.name: 0.0 for v in C.VESSELS}
        self.wet_ml = 0.0
        self.retained = 0.0
        self.bypassed = 0.0
        self.bead_spill = 0.0
        self.unsealed_suspension_ml = 0.0

    def ingress(self, packet, seat):
        # A visibly seated disc must develop a wet edge seal before capturing
        # the suspension. A dry or lifted edge passes suspension around it.
        if seat:
            previous_wetting = self.wet_ml
            self.wet_ml = min(PRIME_ML, self.wet_ml + packet.ml)
            unsealed = min(packet.ml, PRIME_ML - previous_wetting)
        else:
            unsealed = packet.ml
        self.funnel[packet.source] += packet.ml
        if packet.source == C.SUSPENSION.name:
            concentration = BEAD_COUNT / C.SUSPENSION.fill_ml
            self.unsealed_suspension_ml += unsealed
            self.retained += concentration * (packet.ml - unsealed)
            self.bypassed += concentration * unsealed

    def update(self, state, dt):
        seated = paper_seated(state)
        if not seated:
            self.wet_ml = 0.0
        new_packets = []
        for name, source in self.sources.items():
            obj = state["objects"][name]
            packet = source.update(obj["pose"], obj["velocity"], dt)
            if packet:
                new_packets.append(packet)
        self.paths = []
        # Substep the liquid tracks so a packet cannot jump across the rim.
        for part in range(5):
            if part == 0:
                self.packets.extend(new_packets)
            next_packets = []
            small = dt / 5
            for packet in self.packets:
                old = packet.position
                new = old + packet.velocity * small + GRAVITY * (small * small / 2)
                packet.velocity += GRAVITY * small
                if old[2] >= C.RIM > new[2]:
                    x = (old[2] - C.RIM) / (old[2] - new[2])
                    intersection = old + x * (new - old)
                    if (
                        np.linalg.norm(intersection[:2] - C.FUNNEL)
                        < C.FUNNEL_MOUTH - 0.001
                    ):
                        self.ingress(packet, seated)
                        if len(self.paths) < 15:
                            self.paths.append((old, intersection))
                        continue
                if new[2] <= C.TABLE:
                    self.spill[packet.source] += packet.ml
                    if packet.source == C.SUSPENSION.name:
                        self.bead_spill += packet.ml / C.SUSPENSION.fill_ml * BEAD_COUNT
                else:
                    packet.position = new
                    next_packets.append(packet)
                    if len(self.paths) < 15:
                        self.paths.append((old, new))
            self.packets = next_packets
        # Vacuum-driven drainage through the wet or dry open perforations.
        total = sum(self.funnel.values())
        drainage = min(total, 42.0 * dt)
        if total > 0:
            for source in self.funnel:
                draw = self.funnel[source] * drainage / total
                self.funnel[source] -= draw
                self.flask[source] += draw


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.stable = 0.0

    def evaluate(self, state, material, elapsed):
        seated = paper_seated(state)
        primed = seated and material.wet_ml >= PRIME_ML - 0.0001
        collected = material.flask[C.SUSPENSION.name]
        spill = sum(material.spill.values())
        cups_parked = all(
            np.linalg.norm(position[:2] - v.position[:2]) <= 0.05
            and abs(position[2] - C.TABLE) <= 0.011
            and axes[2, 2] >= 0.982
            for v in C.VESSELS
            for position, axes in [pose_parts(state["objects"][v.name]["pose"])]
        )
        zones = [(np.r_[C.FUNNEL, C.RIM], 0.107)]
        zones.extend(
            (
                np.asarray(state["objects"][v.name]["pose"][:3]) + [0, 0, v.height / 2],
                0.079,
            )
            for v in C.VESSELS
        )
        clear = all(
            np.linalg.norm(np.asarray(point) - center) >= distance
            for hand in state["hands"].values()
            for point in hand["points"]
            for center, distance in zones
        )
        finish = (
            primed
            and collected >= 50.0
            and material.retained >= 33.5
            and material.bypassed < 0.5
            and spill <= 3.0
            and cups_parked
            and clear
        )
        self.stable = self.stable + elapsed if finish else 0.0
        success = self.stable >= 0.50
        if success:
            score = 1.0
        elif primed and material.bypassed < 0.5 and spill <= 3.0:
            score = 0.50
        elif seated:
            score = 0.25
        else:
            score = 0.0
        accounted = (
            sum(s.ml for s in material.sources.values())
            + sum(packet.ml for packet in material.packets)
            + sum(material.flask.values())
            + sum(material.funnel.values())
            + spill
        )
        return {
            "success": success,
            "score": score,
            "paper_seated": seated,
            "paper_primed": bool(primed),
            "wetting_ml": round(material.wet_ml, 3),
            "filtrate_ml": round(sum(material.flask.values()), 3),
            "rinse_collected_ml": round(material.flask[C.RINSE.name], 3),
            "suspension_collected_ml": round(collected, 3),
            "retained_beads": round(material.retained, 3),
            "passed_beads": round(material.bypassed, 3),
            "spilled_beads": round(material.bead_spill, 3),
            "bench_spill_ml": round(spill, 3),
            "water_conservation_error_ml": round(
                sum(v.fill_ml for v in C.VESSELS) - accounted, 7
            ),
            "rinse_remaining_ml": round(material.sources[C.RINSE.name].ml, 3),
            "suspension_remaining_ml": round(material.sources[C.SUSPENSION.name].ml, 3),
            "beakers_parked": bool(cups_parked),
            "hands_clear": bool(clear),
            "stable_s": round(self.stable, 3),
        }
