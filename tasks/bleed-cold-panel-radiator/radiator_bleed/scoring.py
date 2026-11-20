"""Score removal of a trapped gas pocket at a physically turned vent."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


CUP_INNER_RADIUS = C.CUP_RADIUS - .002
CUP_INNER_DEPTH = C.CUP_HEIGHT - .003
_nodes, _weights = np.polynomial.legendre.leggauss(24)
_cut_x = CUP_INNER_RADIUS * _nodes
_cut_area = 2 * np.sqrt(CUP_INNER_RADIUS ** 2 - _cut_x ** 2) * CUP_INNER_RADIUS * _weights


def cup_capacity_ml(upright):
    """Volume below the lowest point of the circular lip at a given tilt."""
    if upright <= 1e-5:
        return 0.
    sideways = np.sqrt(max(0., 1. - upright ** 2))
    edge_height = CUP_INNER_DEPTH * upright - CUP_INNER_RADIUS * sideways
    depth = np.clip((edge_height - _cut_x * sideways) / upright, 0., CUP_INNER_DEPTH)
    return float(np.dot(_cut_area, depth) * 1e6)


class Fluid:
    def __init__(self):
        self.air = C.AIR_ML
        self.caught = 0.
        self.spilled = 0.
        self.flowing = False
        self.impact = C.PORT.copy()

    def advance(self, angle, cup, elapsed):
        r = quat_xyzw_to_matrix(cup[3:])
        capacity = cup_capacity_ml(r[2, 2]) if cup[2] >= C.TABLE - .018 else 0.
        self._retain(capacity)
        # Unseat the gasket by turning the native valve. The flow is continuous
        # through the same orifice as the initial air pocket is displaced.
        aperture = np.clip((angle - np.radians(12.)) / np.radians(67.), 0., 1.)
        if aperture <= 0.:
            self.flowing = False
            return
        rate = 26. * aperture
        air_time = self.air / rate
        self.air = max(0., self.air - rate * elapsed)
        water = 12. * aperture * max(0., elapsed - air_time)
        self.flowing = self.air <= 1e-5
        # The short outlet points down and slightly out. Water may enter only
        # through the open mouth; a cup anywhere else leaves it on the tray.
        mouth = cup[:3] + C.CUP_HEIGHT * r[:, 2]

        def landing_xy(height):
            drop = max(0., C.PORT[2] - height)
            speed_down = .55
            flight = (np.sqrt(speed_down ** 2 + 2 * 9.81 * drop) - speed_down) / 9.81
            return C.PORT[:2] + flight * np.array([-.08, .51])

        splash_xy = landing_xy(mouth[2])
        captured = (
            mouth[2] < C.PORT[2]
            and r[2, 2] >= .96
            and np.linalg.norm(splash_xy - mouth[:2]) <= C.CUP_RADIUS - .006
        )
        self.impact = np.r_[splash_xy, mouth[2] - .016] if captured else np.r_[landing_xy(C.TABLE), C.TABLE]
        if captured:
            self.caught += water
            self._retain(capacity)
        else:
            self.spilled += water

    def _retain(self, capacity):
        lost = max(0., self.caught - capacity)
        self.spilled += lost
        self.caught -= lost


class Scorer:
    def reset(self):
        self.fluid = Fluid()
        self.final_hold = 0

    def evaluate(self, state, elapsed):
        radiator = state["objects"]["radiator"]
        cup = np.asarray(state["objects"]["cup"]["pose"])
        key = np.asarray(state["objects"]["key"]["pose"])
        cup_v = np.asarray(state["objects"]["cup"]["velocity"])
        angle = -radiator["joints"]["VentAngle"]
        self.fluid.advance(angle, cup, elapsed)
        shut = abs(angle) <= np.radians(6.)
        upright = quat_xyzw_to_matrix(cup[3:])[2, 2] >= .985
        cup_parked = abs(cup[2] - C.TABLE) <= .006 and upright and np.linalg.norm(cup_v[:3]) < .025
        cup_rotation = quat_xyzw_to_matrix(cup[3:])
        clear = all(
            np.linalg.norm(point - key[:3]) >= .037 and (
                np.linalg.norm(local[:2]) >= .057
                or local[2] < -.025
                or local[2] > C.CUP_HEIGHT + .026
            )
            for hand in state["hands"].values()
            for point in map(np.asarray, hand["points"][:2])
            for local in [cup_rotation.T @ (point - cup[:3])]
        )
        enough = self.fluid.air <= 1e-5 and 3. <= self.fluid.caught <= 22.
        no_spill = self.fluid.spilled <= 2.
        key_v = np.asarray(state["objects"]["key"]["velocity"])
        key_set_down = C.TABLE - .002 <= key[2] <= C.TABLE + .039 and np.linalg.norm(key_v[:3]) < .035
        key_seated = np.linalg.norm(key[:3] - C.KEY_HOME) <= .013
        ready = shut and enough and no_spill and cup_parked and (key_seated or key_set_down) and clear
        self.final_hold = self.final_hold + 1 if ready else 0
        success = self.final_hold >= 20
        score = 0.
        if no_spill and self.fluid.caught <= 22. and self.fluid.air <= 1e-5:
            score = 1. if success else .75 if shut and enough else .4 if enough else 0.
        return dict(
            score=score, success=success, valve_degrees=float(np.degrees(angle)),
            air_ml=float(self.fluid.air), caught_ml=float(self.fluid.caught), spilled_ml=float(self.fluid.spilled),
            cup_supported=bool(cup_parked), vent_shut=bool(shut), clear=bool(clear),
            final_hold_steps=self.final_hold,
        )
