"""Colour homogenization from measured, contact-coupled tube vibration."""
from collections import deque
import math

import numpy as np

from . import config as C

DARK = np.array([.025, .12, .55])
CLEAR = np.array([.93, .93, .88])
MEAN = np.array([.075, .45, .84])
FLUID_LUMINANCE = .25


class Sample:
    def __init__(self):
        self.reset()

    def reset(self):
        self.contrast = 1.
        self.phase = 0.
        self.window = deque(maxlen=15)
        self.coupling = 0.
        self.motor_on = False
        self.contact_s = 0.
        self.max_press_m = 0.
        self.max_tube_orbit_m = 0.

    def advance(self, dt, switch, cup, tube):
        self.max_press_m = max(self.max_press_m, -switch)
        self.motor_on = switch <= -C.PRESS_ON
        self.phase = (self.phase + math.tau * C.MOTOR_HZ * dt) % math.tau if self.motor_on else 0.
        offset = tube - cup
        coupled = (
            self.motor_on
            and abs(offset[2]) <= .0025
            and math.hypot(*offset[:2]) <= .010
        )
        if not coupled:
            self.window.clear()
            self.coupling = 0.
            return
        self.contact_s += dt
        self.window.append((tube[0], tube[1], self.phase))
        if len(self.window) < self.window.maxlen:
            return

        # Pull the measured tube-tip displacement component at the motor
        # frequency out of any slower placement motion. A static tube, contact
        # loss and pressing the cup with the gripper contribute no mixing.
        values = np.array(self.window)
        coordinates, phase = values[:, :2], values[:, 2]
        coordinates -= coordinates.mean(axis=0)
        x_sine = np.mean(coordinates * np.sin(phase)[:, None], axis=0)
        x_cosine = np.mean(coordinates * np.cos(phase)[:, None], axis=0)
        amplitude = math.sqrt(2 * (x_sine @ x_sine + x_cosine @ x_cosine))
        self.max_tube_orbit_m = max(self.max_tube_orbit_m, amplitude)
        self.coupling = min(amplitude / .0015, 1.0)
        self.contrast *= math.exp(-dt * 1.25 * self.coupling**2)

    def drive(self):
        if not self.motor_on:
            return (0., 0.), (0., 0.)
        sine, cosine = math.sin(self.phase), math.cos(self.phase)
        speed = C.PAD_ORBIT * math.tau * C.MOTOR_HZ
        return (C.PAD_ORBIT * cosine, C.PAD_ORBIT * sine), (-speed * sine, speed * cosine)

    def colors(self):
        return (MEAN + self.contrast * (layer - MEAN) for layer in (DARK, CLEAR))

    def snapshot(self):
        return {
            "contrast": self.contrast,
            "motor_on": self.motor_on,
            "coupling": self.coupling,
            "contact_s": self.contact_s,
            "max_press_m": self.max_press_m,
            "max_tube_orbit_m": self.max_tube_orbit_m,
        }
