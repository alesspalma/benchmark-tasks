"""Give credit for observable seating, directed paint, and a stable can."""

from __future__ import annotations

import numpy as np

from . import config as C
from .mechanics import parts


class Scorer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.stable_time = 0.0
        self.peak_tilt = 0.0
        self.previous_base = None

    def evaluate(self, state, elapsed, gasket, material):
        can = state["objects"]["can"]
        base, rotation = parts(can["pose"])
        lid, lid_rotation = parts(state["objects"]["lid"]["pose"])
        tilt = float(np.rad2deg(np.arccos(np.clip(rotation[2, 2], -1, 1))))
        self.peak_tilt = max(self.peak_tilt, tilt)
        m = material.metrics()
        still = (
            self.previous_base is not None
            and np.linalg.norm(base - self.previous_base) <= 0.002
        )
        self.previous_base = base
        placed = (
            tilt <= 7.0
            and abs(base[2] - C.CAN[2]) < 0.013
            and np.linalg.norm(base[:2] - C.CAN[:2]) < 0.05
            and still
        )
        handles = [
            base + rotation @ C.HANDLE,
            base + rotation @ C.AUX_HANDLE,
            lid + lid_rotation @ C.LID_GRIP,
        ]
        clear = all(
            np.linalg.norm(np.asarray(p) - h) > 0.035
            for hand in state["hands"].values()
            for p in hand["points"]
            for h in handles
        )
        spill_ok = m["spilled_ml"] <= C.MAX_SPILL_ML and m["bypass_ml"] <= 5.0
        fill_ok = (
            C.TARGET_ML[0] <= m["through_spout_ml"] <= C.TARGET_ML[1]
            and m["tray_ml"] <= C.TARGET_ML[1]
        )
        full = (
            gasket.fitted
            and spill_ok
            and fill_ok
            and placed
            and clear
            and m["airborne_ml"] <= 0.1
        )
        self.stable_time = self.stable_time + elapsed if full else 0.0
        score = 0.0
        if gasket.fitted and spill_ok and m["tray_ml"] <= C.TARGET_ML[1]:
            score = 0.3 if m["through_spout_ml"] < 50 else 0.6
            if fill_ok:
                score = 0.8
            if self.stable_time >= 0.55 - 1e-10:
                score = 1.0
        return dict(
            score=score,
            success=score == 1.0,
            fitted=gasket.fitted,
            sectors_seated=int(np.count_nonzero(gasket.engaged)),
            rim_gaps_mm=(1000 * gasket.gaps).tolist(),
            maximum_snap_force_n=gasket.maximum_force,
            can_tilt_deg=tilt,
            peak_can_tilt_deg=self.peak_tilt,
            can_parked=bool(placed),
            arms_clear=bool(clear),
            stable_s=self.stable_time,
            **m,
        )
