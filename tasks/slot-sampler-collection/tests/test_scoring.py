"""Synthetic state counterexamples; native granule feasibility is tested separately."""

import copy

import numpy as np
from slot_sampler import config as C
from slot_sampler.scoring import Scorer


def state(p=None, n=28):
    p = np.array(C.TUB + [0, 0, 0.01] if p is None else p)
    probe = {
        "pose": [*p, 0, 0, 0, 1.0],
        "velocity": [0.0] * 6,
        "joints": {"GateSlide": C.GATE_TRAVEL, "BottomSlide": 0.0},
    }
    return {
        "time_s": 0.0,
        "objects": {"probe": probe},
        "grains": np.tile(p + [0.026, 0, 0.025], (n, 1)),
        "grain_velocities": np.zeros((n, 6)),
        "bounds": {
            "probe:Outer": [p + [-0.018, -0.08, -0.01], p + [0.018, 0.08, 0.32]]
        },
        "hands": {n: {"opening_m": 0.09} for n in ("left", "right")},
        "robot_clearance_m": 0.1,
    }


def load(sc, s, close=True):
    sc.observe(s)
    s["grains"][:, 0] -= 0.005
    sc.observe(s)
    if close:
        s["objects"]["probe"]["joints"]["GateSlide"] = 0.0
        sc.observe(s)


def deliver(sc, s):
    # Synthetic physical exit through the separate bottom before the settled mass
    # is placed in its final jar positions. No captured IDs means no credit.
    origin = C.JAR + [0, 0, C.DISCHARGE_HEIGHT]
    s["objects"]["probe"]["pose"] = [*origin, 0, 0, 0, 1]
    s["objects"]["probe"]["joints"]["BottomSlide"] = C.DUMP_TRAVEL
    s["grains"][:] = origin + [0.010, 0, 0.025]
    sc.observe(s)
    s["grains"][:] = origin + [0.010, 0, -C.RADIUS - 0.001]
    sc.observe(s)
    n = len(s["grains"])
    s["grains"] = np.array(
        [
            C.JAR
            + [
                (k % 4 - 1.5) * 0.0071,
                ((k // 4) % 4 - 1.5) * 0.0071,
                0.007 + k // 16 * 0.0071,
            ]
            for k in range(n)
        ]
    )
    p = s["objects"]["probe"]
    p["pose"] = [*C.PROBE, 0, 0, 0, 1.0]
    p["joints"]["GateSlide"] = 0.0
    p["joints"]["BottomSlide"] = 0.0
    s["bounds"] = {
        "probe:Outer": [C.PROBE + [-0.018, -0.08, -0.01], C.PROBE + [0.018, 0.08, 0.32]]
    }


def finish(sc, s):
    for k in range(23):
        s["time_s"] = k / 20
        sc.observe(s)
        m = sc.evaluate(s)
    return m


def test_capture_close_and_deliver_actual_mass():
    sc = Scorer()
    s = state()
    load(sc, s)
    deliver(sc, s)
    assert finish(sc, s)["score"] == 1.0


def test_direct_material_in_jar_without_capture_is_zero():
    sc = Scorer()
    s = state()
    deliver(sc, s)
    assert finish(sc, s)["score"] == 0.0


def test_admission_without_closed_retention_is_zero():
    sc = Scorer()
    s = state()
    load(sc, s, False)
    deliver(sc, s)
    assert finish(sc, s)["score"] == 0.0


def test_inverted_or_unburied_scooping_does_not_mark_capture():
    sc = Scorer()
    s = state(C.TUB + [0, 0, 0.15])
    load(sc, s)
    deliver(sc, s)
    assert finish(sc, s)["score"] == 0.0


def test_partial_mass_is_measured_without_a_full_score():
    sc = Scorer()
    s = state(n=12)
    load(sc, s)
    deliver(sc, s)
    assert finish(sc, s)["score"] == 0.4


def test_sample_at_rest_but_robot_propping_probe_is_not_complete():
    sc = Scorer()
    s = state()
    load(sc, s)
    deliver(sc, s)
    s["robot_clearance_m"] = 0.0
    assert finish(sc, s)["score"] == 0.7


def test_missing_hand_measurements_cannot_finish():
    sc = Scorer()
    s = state()
    load(sc, s)
    deliver(sc, s)
    s["hands"] = {}
    assert finish(sc, s)["score"] == 0.7


def test_airborne_material_is_not_delivered_mass():
    sc = Scorer()
    s = state()
    load(sc, s)
    deliver(sc, s)
    s["grain_velocities"][:, 2] = 0.1
    assert finish(sc, s)["score"] == 0.0


def test_spill_is_permanent_even_after_recovery():
    sc = Scorer()
    s = state()
    load(sc, s)
    bad = copy.deepcopy(s)
    bad["grains"][0, 2] = C.TOP - 0.1
    sc.observe(bad)
    deliver(sc, s)
    assert finish(sc, s)["score"] == 0.0


def test_missing_parked_probe_only_earns_delivery_credit():
    sc = Scorer()
    s = state()
    load(sc, s)
    deliver(sc, s)
    s["objects"]["probe"]["pose"][0] += 0.03
    assert finish(sc, s)["score"] == 0.7


def test_reopened_slide_prevents_full_score_after_delivery():
    sc = Scorer()
    s = state()
    load(sc, s)
    deliver(sc, s)
    s["objects"]["probe"]["joints"]["GateSlide"] = C.GATE_TRAVEL
    assert finish(sc, s)["score"] == 0.7


def test_grains_entering_through_solid_back_do_not_count():
    sc = Scorer()
    s = state()
    origin = np.asarray(s["objects"]["probe"]["pose"][:3])
    s["grains"][:] = origin + [-0.026, 0, 0.030]
    sc.observe(s)
    s["grains"][:] = origin + [-0.015, 0, 0.030]
    sc.observe(s)
    s["objects"]["probe"]["joints"]["GateSlide"] = 0.0
    sc.observe(s)
    deliver(sc, s)
    assert finish(sc, s)["score"] == 0.0


def test_grains_below_floor_are_not_inside_sample():
    sc = Scorer()
    s = state()
    s["grains"][:, 2] -= 0.020
    load(sc, s)
    assert sc.inside_count == 0
    deliver(sc, s)
    assert finish(sc, s)["score"] == 0.0


def test_filled_probe_inside_jar_is_not_delivered_material():
    sc = Scorer()
    s = state()
    load(sc, s)
    origin = C.JAR + [0, 0, C.DISCHARGE_HEIGHT]
    s["objects"]["probe"]["pose"] = [*origin, 0, 0, 0, 1]
    s["grains"][:] = origin + [0.010, 0, 0.025]
    sc.observe(s)
    result = sc.evaluate(s)
    assert result["captured_grains"] == 28
    assert result["delivered_grams"] == 0
    assert result["score"] == 0


def test_open_bottom_cannot_capture_material():
    sc = Scorer()
    s = state()
    s["objects"]["probe"]["joints"]["BottomSlide"] = C.DUMP_TRAVEL
    load(sc, s)
    assert not sc.captured
    deliver(sc, s)
    assert finish(sc, s)["score"] == 0.0


def test_reopened_bottom_prevents_full_score():
    sc = Scorer()
    s = state()
    load(sc, s)
    deliver(sc, s)
    s["objects"]["probe"]["joints"]["BottomSlide"] = C.DUMP_TRAVEL
    assert finish(sc, s)["score"] == 0.7


def test_side_dump_does_not_count_as_bottom_discharge():
    sc = Scorer()
    s = state()
    load(sc, s)
    origin = C.JAR + [0, 0, C.DISCHARGE_HEIGHT]
    s["objects"]["probe"]["pose"] = [*origin, 0, 0, 0, 1]
    s["objects"]["probe"]["joints"]["GateSlide"] = C.GATE_TRAVEL
    s["grains"][:] = origin + [0.010, 0, 0.025]
    sc.observe(s)
    s["grains"][:] = origin + [0.040, 0, 0.025]
    sc.observe(s)
    assert not sc.discharged
