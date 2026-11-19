"""Pressure and liquid conservation for an air-bulb filler with check valves."""

from dataclasses import dataclass

import numpy as np

from . import config as C


def water_height(volume):
    """Height above a vessel's base, in metres for a volume in millilitres."""
    return C.CUP_BASE + volume * 1e-6 / (np.pi * C.CUP_RADIUS**2)


def column_height(volume):
    """Meniscus above the pipette tip, in metres."""
    return .014 + max(0., volume - C.TIP_RESERVOIR) * 1e-6 / C.GRADUATED_BORE


def valve_conductance(travel):
    return float(np.clip((travel - .0012) / .0022, 0., 1.))


@dataclass
class Water:
    source: float = C.WATER_START
    pipette: float = 0.
    receiver: float = 0.
    spilled: float = 0.
    air_equivalent: float = C.VOLUME_AT_REST
    tube_pressure: float = C.ATMOSPHERIC_PA
    bulb_pressure: float = C.ATMOSPHERIC_PA
    peak_column: float = 0.
    at_float: bool = False

    def step(self, dt, tip, upright, squeeze, air, suction, delivery):
        bulb_volume = C.VOLUME_AT_REST - C.COMPRESSION_PER_METRE * min(squeeze, C.BULB_TRAVEL)
        source_depth = C.TABLE + water_height(self.source) - tip[2]
        over_source = (
            np.linalg.norm(tip[:2] - C.SOURCE[:2]) < C.CUP_RADIUS - .002
            and C.TABLE + C.CUP_BASE < tip[2] < C.TABLE + C.CUP_HEIGHT + .038
        )
        in_source = over_source and source_depth > .002
        in_receiver = (
            np.linalg.norm(tip[:2] - C.RECEIVER[:2]) < C.CUP_RADIUS - .002
            and C.TABLE + C.CUP_BASE < tip[2] < C.TABLE + C.CUP_HEIGHT + .038
        )
        if air:
            # Valve A connects only the bulb to room air.
            self.air_equivalent += (bulb_volume - self.air_equivalent) * min(1., air * 35. * dt)
        if suction and not in_source and not self.pipette:
            # Empty pipette, tip in air: applying vacuum here loses the charge.
            self.air_equivalent += (bulb_volume - self.air_equivalent) * min(1., suction * 12. * dt)

        self.bulb_pressure = C.ATMOSPHERIC_PA * self.air_equivalent / bulb_volume
        head = 9810. * column_height(self.pipette) * max(.15, upright)
        if delivery:
            # E admits room air above the liquid column while the outlet drains.
            self.tube_pressure += (C.ATMOSPHERIC_PA - self.tube_pressure) * min(1., delivery * 45. * dt)
        if suction and not delivery:
            # Liquid entering the graduated tube displaces the equivalent air
            # volume into the bulb. Solve the pressure/head drop at this step.
            combined_volume = bulb_volume - self.pipette
            combined_pressure = C.ATMOSPHERIC_PA * self.air_equivalent / combined_volume
            self.tube_pressure += (combined_pressure - self.tube_pressure) * min(1., suction * 40. * dt)
            driving_pa = C.ATMOSPHERIC_PA - self.tube_pressure - head + 9810. * max(0., source_depth)
            if in_source:
                rise = min(self.source, suction * max(0., driving_pa) * .0024 * dt,
                           C.USABLE_VOLUME - self.pipette)
                self.source -= rise
                self.pipette += rise
                self.at_float |= self.pipette >= C.USABLE_VOLUME - .01
                self.peak_column = max(self.pipette, self.peak_column)

        if self.pipette:
            # Below the liquid column the open tip sees ambient pressure plus
            # submerged head, if any. Closed S and E trap air and hold the
            # column; they do not award volume or position.
            excess = self.tube_pressure - C.ATMOSPHERIC_PA + head
            if in_source:
                excess -= 9810. * source_depth
            # After S closes, the trapped tube air equilibrates against a
            # pinned meniscus. Venting or joining a pressurized bulb unpins it.
            if not delivery and not suction and upright > .6:
                self.tube_pressure = C.ATMOSPHERIC_PA - head
                excess = 0.
            drain = min(self.pipette, max(0., excess) * .0055 * dt)
            if upright < .4:
                drain = max(drain, min(self.pipette, dt * 4.))
            self.pipette -= drain
            if over_source:
                self.source += drain
            elif in_receiver:
                self.receiver += drain
            else:
                self.spilled += drain
