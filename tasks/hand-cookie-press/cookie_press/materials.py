"""Measured forward piston travel feeds a simple mass-conserving die model."""

from dataclasses import dataclass, field

import numpy as np

from . import config as C


@dataclass
class Deposit:
    xy: np.ndarray
    yaw: float
    grams: float = 0.0
    distorted: bool = False


@dataclass
class Dough:
    deposits: list[Deposit] = field(default_factory=list)
    remaining: float = C.DOUGH_G
    waste: float = 0.0
    stroke: float = 0.0
    current: Deposit | None = None
    largest_lever_angle: float = 0.0

    def advance(self, pose, angle, speed):
        xy = np.asarray(pose[:2])
        qx, qy, qz, qw = pose[3:]
        upright = 1.0 - 2.0 * (qx * qx + qy * qy)
        yaw = float(
            np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        )
        footprint_clear = all(
            np.abs(xy - C.TRAY[:2]) <= np.asarray(C.TRAY_SIZE) / 2.0 - C.BASE_RADIUS
        )
        # Seat the complete ring on the metal and retain its registration as
        # the piston advances. Away from the sheet, the same mass is expended.
        seated = (
            upright > np.cos(np.deg2rad(5.0))
            and abs(pose[2] - C.TRAY_TOP) <= C.SEAT_TOLERANCE
            and footprint_clear
        )
        self.largest_lever_angle = max(self.largest_lever_angle, angle)
        travel = float(np.clip((angle - 0.075) / (C.LEVER_PRESS - 0.075), 0.0, 1.0))
        if angle < 0.055:
            self.stroke = 0.0
            self.current = None
        increment = min(self.remaining, C.DOSE_G * max(0.0, travel - self.stroke))
        self.stroke = max(travel, self.stroke)
        self.remaining -= increment

        lowest_rim = (
            pose[2] - C.TRAY_TOP - C.BASE_RADIUS * np.sqrt(max(0.0, 1.0 - upright**2))
        )
        for item in self.deposits:
            distance = np.linalg.norm(xy - item.xy)
            rim_wipes = 0.021 < distance < 0.050 and lowest_rim < C.COOKIE_HEIGHT
            repeated_shot = (
                increment > 0.0
                and item is not self.current
                and seated
                and distance < 0.038
            )
            if rim_wipes or repeated_shot:
                item.distorted = True

        if increment > 0.0:
            if seated and speed < 0.065:
                if self.current is None:
                    self.current = Deposit(xy.copy(), yaw)
                    self.deposits.append(self.current)
                self.current.grams += increment
                if np.linalg.norm(xy - self.current.xy) > 0.006:
                    self.current.distorted = True
            else:
                self.waste += increment
                if self.current:
                    self.current.distorted = True

    def snapshot(self):
        return {
            "remaining_g": self.remaining,
            "wasted_g": self.waste,
            "largest_lever_angle_deg": float(np.rad2deg(self.largest_lever_angle)),
            "deposits": [
                {
                    "xy": item.xy.tolist(),
                    "yaw": item.yaw,
                    "mass_g": item.grams,
                    "distorted": item.distorted,
                }
                for item in self.deposits
            ],
        }
