"""Score measured water at the receiving vessel and outside both vessels."""

from . import config as C


class Scorer:
    def evaluate(self, state):
        delivered = state["receiving_ml"]
        error = abs(delivered - C.TARGET)
        score = 0.
        if state["spilled_ml"] <= C.SPILL_LIMIT and not state["bulb_flooded"]:
            if delivered > .5 and error <= 3.0:
                score = .25
            if error <= 1.0:
                score = .5
            if error <= .5:
                score = .75
            if error <= C.TOLERANCE and state["pipette_ml"] <= .06:
                score = 1.
        return {
            "score": score, "success": score == 1.,
            "receiving_ml": round(delivered, 4),
            "pipette_ml": round(state["pipette_ml"], 4),
            "peak_pipette_ml": round(state["peak_pipette_ml"], 4),
            "source_ml": round(state["source_ml"], 4),
            "spilled_ml": round(state["spilled_ml"], 4),
            "air_pressure_kpa": round(state["bulb_pressure_pa"] / 1000., 2),
            "squeeze_mm": round(1000. * state["squeeze_m"], 3),
            "valves_mm": {name: round(1000. * v, 3) for name, v in state["valves_m"].items()},
            "bulb_flooded": state["bulb_flooded"],
        }
