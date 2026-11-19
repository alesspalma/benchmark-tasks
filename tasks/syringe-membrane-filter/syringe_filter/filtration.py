"""Conservative displaced-volume and particle-rejection model."""

import numpy as np
from yam_arena.ik import quat_xyzw_to_matrix

from . import config as C


class Filtration:
    def __init__(self):
        self.displaced_ml = 0.0
        self.collected_ml = 0.0
        self.spilled_ml = 0.0
        self.retained_mg = 0.0
        self.outlet = C.START.copy()
        self.outlet_velocity = np.zeros(3)
        self.stroke_speed = 0.0
        self.jet_intercept = None
        self.release_ml = 0.0
        self.is_captured = False

    def advance(self, pose, stroke, speed, dt):
        outlet = np.asarray(pose[:3])
        # The articulation reports center-of-mass velocity, while the jet
        # leaves the distal outlet. Estimate its velocity from position.
        # A 35 ms response averages contact-solver oscillations over a drop.
        blend = 1.0 - np.exp(-dt / 0.035)
        measured = (outlet - self.outlet) / dt
        self.outlet_velocity += blend * (measured - self.outlet_velocity)
        self.stroke_speed += blend * (speed - self.stroke_speed)
        self.outlet = outlet.copy()
        nozzle = -quat_xyzw_to_matrix(pose[3:])[:, 2]
        self.release_ml = 0.0
        self.is_captured = False
        # The first 0.12 mL of piston displacement takes up the wetted
        # filter and seal compliance before a parcel exits the outlet.
        displaced_to = float(
            np.clip(-stroke * C.PISTON_AREA * 1e6 - 0.12, 0.0, C.FILL_ML)
        )
        released = max(0.0, displaced_to - self.displaced_ml)
        if released <= 0.000002:
            return
        self.release_ml = released
        self.displaced_ml += released
        self.retained_mg += released * C.PARTICLE_CONCENTRATION_MG_ML
        # A reduced falling jet. Exit speed follows the measured positive
        # piston velocity. Once this parcel has left the outlet, arm motion
        # cannot direct it back into the vial.
        v = nozzle * min(
            0.8,
            max(
                0.015,
                -self.stroke_speed * C.PISTON_AREA / (np.pi * C.OUTLET_INNER_R**2),
            ),
        )
        v += self.outlet_velocity
        height = self.outlet[2] - (C.VIAL[2] + C.VIAL_LIP)
        self.jet_intercept = None
        if height >= 0:
            t = (v[2] + np.sqrt(v[2] ** 2 + 19.62 * height)) / 9.81
            self.jet_intercept = self.outlet[:2] + t * v[:2]
            self.is_captured = bool(
                np.linalg.norm(self.jet_intercept - C.VIAL[:2])
                <= C.VIAL_INNER_R - 0.001
            )
        elif (
            self.outlet[2] >= C.VIAL[2] + C.VIAL_FLOOR
            and np.linalg.norm(self.outlet[:2] - C.VIAL[:2]) <= C.VIAL_INNER_R - 0.001
            and nozzle[2] < 0.0
        ):
            self.is_captured = True
        if self.is_captured:
            self.collected_ml += released
        else:
            self.spilled_ml += released

    def snapshot(self):
        return {
            "collected_ml": self.collected_ml,
            "spilled_ml": self.spilled_ml,
            "remaining_ml": C.FILL_ML - self.displaced_ml,
            "retained_particles_mg": self.retained_mg,
            "filtrate_particles_mg": 0.0,
            "outlet_xyz": self.outlet.tolist(),
            "jet_intercept_xy": None
            if self.jet_intercept is None
            else self.jet_intercept.tolist(),
            "release_ml": self.release_ml,
            "captured": self.is_captured,
        }

    def score(self):
        # 0.2 g on a tared tray is a directly checkable loss criterion.
        clean_handling = self.spilled_ml <= C.SPILL_LIMIT_ML + 1e-10
        volume = self.collected_ml
        full = volume >= C.FLOW_TARGET_ML - 1e-10 and clean_handling
        return {
            "score": 1.0
            if full
            else round(min(0.99, volume / C.FLOW_TARGET_ML), 4)
            if clean_handling
            else 0.0,
            "success": full,
            **self.snapshot(),
        }
