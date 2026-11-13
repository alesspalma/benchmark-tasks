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
        "joints": {"Sleeve": -1.5},
    }
    return {
        "time_s": 0.0,
        "objects": {"probe": probe},
        "grains": np.tile(p + [0.0105, 0, 0.04], (n, 1)),
        "grain_velocities": np.zeros((n, 6)),
        "bounds": {
            "probe:Outer": [p + [-0.018, -0.08, -0.01], p + [0.018, 0.08, 0.32]]
        },
        "hands": {n: {"opening_m": 0.09} for n in ("left", "right")},
        "robot_clearance_m": 0.1,
    }


def load(sc, s, close=True):
    sc.observe(s)
    s["grains"][:, 0] -= 0.002
    sc.observe(s)
    if close:
        s["objects"]["probe"]["joints"]["Sleeve"] = 0.0
        sc.observe(s)


def deliver(s):
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
    p["joints"]["Sleeve"] = 0.0
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
    deliver(s)
    assert finish(sc, s)["score"] == 1.0


def test_direct_material_in_jar_without_capture_is_zero():
    sc = Scorer()
    s = state()
    deliver(s)
    assert finish(sc, s)["score"] == 0.0


def test_admission_without_closed_retention_is_zero():
    sc = Scorer()
    s = state()
    load(sc, s, False)
    deliver(s)
    assert finish(sc, s)["score"] == 0.0


def test_inverted_or_unburied_scooping_does_not_mark_capture():
    sc = Scorer()
    s = state(C.TUB + [0, 0, 0.15])
    load(sc, s)
    deliver(s)
    assert finish(sc, s)["score"] == 0.0


def test_partial_mass_is_measured_without_a_full_score():
    sc = Scorer()
    s = state(n=12)
    load(sc, s)
    deliver(s)
    assert finish(sc, s)["score"] == 0.4


def test_sample_at_rest_but_robot_propping_probe_is_not_complete():
    sc = Scorer()
    s = state()
    load(sc, s)
    deliver(s)
    s["robot_clearance_m"] = 0.0
    assert finish(sc, s)["score"] == 0.7


def test_missing_hand_measurements_cannot_finish():
    sc = Scorer()
    s = state()
    load(sc, s)
    deliver(s)
    s["hands"] = {}
    assert finish(sc, s)["score"] == 0.7


def test_airborne_material_is_not_delivered_mass():
    sc = Scorer()
    s = state()
    load(sc, s)
    deliver(s)
    s["grain_velocities"][:, 2] = 0.1
    assert finish(sc, s)["score"] == 0.0


def test_spill_is_permanent_even_after_recovery():
    sc = Scorer()
    s = state()
    load(sc, s)
    bad = copy.deepcopy(s)
    bad["grains"][0, 2] = C.TOP - 0.1
    sc.observe(bad)
    deliver(s)
    assert finish(sc, s)["score"] == 0.0


def test_missing_parked_probe_only_earns_delivery_credit():
    sc = Scorer()
    s = state()
    load(sc, s)
    deliver(s)
    s["objects"]["probe"]["pose"][0] += 0.03
    assert finish(sc, s)["score"] == 0.7
