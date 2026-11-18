"""Evaluate the visible reading and unsupported, settled pointer."""

import numpy as np

from . import config as C


class Scorer:
    def reset(self):
        self.hold_since = None

    def evaluate(self, state):
        scale = state["objects"]["scale"]
        joints = scale["joints"]
        sample = state["objects"]["specimen"]
        beam_xyz = np.asarray(scale["links"]["Beam"][:3])
        sample_xyz = np.asarray(sample["pose"][:3])
        pan_xyz = np.asarray(scale["links"]["Pan"][:3]) + [0., 0., C.PAN_Z]
        on_pan = bool(np.linalg.norm((sample_xyz - pan_xyz)[:2]) <= .055
                      and abs(sample_xyz[2] - pan_xyz[2]) < .013)
        values = {r.name: -joints[r.name] * r.capacity_g / r.travel for r in C.RIDERS}
        reading = sum(values.values())
        notch_aligned = all(abs(values[r.name] / r.tick_g - round(values[r.name] / r.tick_g))
                            < .12 for r in C.RIDERS[:2])
        pointer_mm = float(np.sin(joints["Pivot"]) * -C.POINTER_Y * 1000)
        hand_points = [np.asarray(pos[:3])
                       for hand in state["hands"].values()
                       for pos, name in zip(hand["poses"], hand["names"])
                       if name in ("tip_left", "tip_right", "gripper")]
        for hand in state["hands"].values():
            hand_points.extend(np.asarray(hand[key]) for key in ("tcp", "tip"))
        controls = [np.asarray(scale["links"]["Poise" + r.name][:3]) for r in C.RIDERS]
        controls.extend((sample_xyz, beam_xyz, beam_xyz + [0., C.POINTER_Y, 0.]))
        clearance = min(float(np.linalg.norm(a - b)) for a in hand_points for b in controls)
        mass_error_g = abs(reading - 1000 * C.SPECIMEN_MASS)
        full = (on_pan and notch_aligned and mass_error_g <= 1.
                and abs(pointer_mm) <= 1.5 and clearance >= .045)
        if full:
            if self.hold_since is None:
                self.hold_since = state["time_s"]
        else:
            self.hold_since = None
        held = 0. if self.hold_since is None else state["time_s"] - self.hold_since
        success = held >= 1.2
        partial = (on_pan and notch_aligned and mass_error_g <= 10.
                   and abs(pointer_mm) <= 10. and clearance >= .045)
        return dict(score=1. if success else .5 if partial else 0., success=success,
                    reading_g=float(reading), pointer_mm=pointer_mm, riders_g=values,
                    on_pan=on_pan, notch_aligned=notch_aligned, clearance_m=clearance,
                    held_s=held)
