"""Score potato placement, yield through the die, and collection by gravity."""

from dataclasses import dataclass

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C

YIELD_START = 0.002
YIELD_WORK = 0.020


def transform(pose):
    return np.asarray(pose[:3]), quat_xyzw_to_matrix(np.asarray(pose[3:]))


@dataclass
class Packet:
    grams: float
    center: np.ndarray
    age: float = 0.0


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.loaded = False
        self.set_depth = 0.0
        self.delivered = 0.0
        self.spilled = 0.0
        self.emitted = 0.0
        self.peak_force = 0.0
        self.depth = 0.0
        self.stable = 0.0
        self.pending_grams = 0.0
        self.pending = []
        self.contact = False

    def physics_sample(self, ricer_pose, ram_pose, potato_pose, travel, dt):
        body, axes = transform(ricer_pose)
        ram, ram_axes = transform(ram_pose)
        lump, lump_axes = transform(potato_pose)
        relative = axes.T @ (lump - body)
        cap = lump + lump_axes[:, 2] * (C.POTATO_HEIGHT + travel)
        face = ram - ram_axes[:, 2] * (C.PLATE_DROP + 0.0025)
        delta_z = (face - cap) @ axes[:, 2]
        self.loaded = bool(
            np.linalg.norm(relative[:2]) <= 0.019
            and abs(relative[2] - C.FLOOR_Z) <= 0.006
            and lump_axes[:, 2] @ axes[:, 2] > 0.92
        )
        self.depth = max(0.0, -float(travel))
        aligned = ram_axes[:, 2] @ axes[:, 2] > 0.96
        self.contact = bool(self.loaded and aligned and -0.006 < delta_z < 0.0024)
        increase = 0.0
        if self.contact:
            # Same spring that reacts against the native upper potato surface.
            load = 350.0 * max(0.0, self.depth + 0.003 - self.set_depth)
            self.peak_force = max(self.peak_force, load)
            if load > 1.6 and self.depth >= YIELD_START:
                demand = min(
                    0.96 * C.POTATO_G,
                    C.POTATO_G * (self.depth - YIELD_START) / YIELD_WORK,
                )
                # The flow cap limits discharge near a rapid squeeze. The material
                # sets at its attained stroke, so opening the lever cannot refill it.
                increase = min(max(0.0, demand - self.emitted), 115.0 * dt)
                self.emitted += increase
                self.pending_grams += increase
                self.set_depth = min(
                    0.56 * YIELD_WORK, 0.56 * YIELD_WORK * self.emitted / C.POTATO_G
                )
        if self.pending_grams >= 0.2 or (self.pending_grams > 0 and increase <= 1e-8):
            outlet = body + axes[:, 2] * (C.BASKET_FLOOR - 0.002)
            self.pending.append(Packet(self.pending_grams, outlet))
            self.pending_grams = 0.0
        flying = []
        for packet in self.pending:
            packet.age += dt
            dz = -4.905 * packet.age**2
            if packet.center[2] + dz > C.BOWL_FLOOR:
                flying.append(packet)
            elif np.linalg.norm(packet.center[:2] - C.BOWL[:2]) <= 0.062:
                self.delivered += packet.grams
            else:
                self.spilled += packet.grams
        self.pending = flying

    def evaluate(self, state, elapsed):
        objects = state["objects"]
        ricer = objects["ricer"]
        b, axes = transform(ricer["pose"])
        grip_points = np.array([b + axes @ C.FIXED_GRIP])
        lever, l_axes = transform(ricer["links"]["Lever"])
        grip_points = np.vstack([grip_points, lever + l_axes @ C.PRESS_GRIP])
        clear = all(
            np.min(np.linalg.norm(grip_points - np.asarray(h["points"][0]), axis=1))
            >= 0.055
            for h in state["hands"].values()
        )
        seated = bool(
            np.linalg.norm(b[:2] - C.BOWL[:2]) < 0.027
            and abs(b[2] - C.RICER[2]) < 0.025
            and axes[2, 2] > 0.94
        )
        delivered = min(C.POTATO_G, self.delivered)
        self.stable = (
            self.stable + elapsed
            if delivered >= 32.0
            and self.spilled <= 2.0
            and seated
            and clear
            and not self.pending
            and self.pending_grams < 0.05
            else 0.0
        )
        if self.spilled > 5.0:
            value = 0.0
        elif self.stable >= 0.40 - 1e-9:
            value = 1.0
        elif delivered >= 25.0:
            value = 0.75
        elif delivered >= 8.0:
            value = 0.50
        elif self.loaded:
            value = 0.25
        else:
            value = 0.0
        return {
            "score": value,
            "success": value == 1.0,
            "load_in_basket": self.loaded,
            "handle_angle_deg": float(np.degrees(ricer["joints"]["HandleHinge"])),
            "compression_mm": 1000 * self.depth,
            "ram_food_contact": self.contact,
            "peak_food_reaction_n": self.peak_force,
            "collected_g": delivered,
            "spilled_g": self.spilled,
            "in_flight_g": self.pending_grams + sum(p.grams for p in self.pending),
            "remaining_g": C.POTATO_G - self.emitted,
            "ricer_resting_on_bowl": seated,
            "hands_clear": clear,
            "stable_s": self.stable,
        }
