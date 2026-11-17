"""Outcome counterexamples, including reversible plug orientation and unsafe history."""

from copy import deepcopy

import numpy as np
import pytest

from damaged_powerbank import config as C
from damaged_powerbank import scoring


def state():
    return {
        "time_s": 0.0,
        "objects": {
            "intact": {"pose": [*C.GOOD, 0, 0, 0, 1], "velocity": [0]*6},
            "damaged": {"pose": [*C.BAD, 0, 0, 0, 1], "velocity": [0]*6},
            "socket": {"pose": [*C.SOCKET, 0, 0, 0, 1], "velocity": [0]*6,
                       "joints": {f"J_Leaf{i}_{k}_{sign}": 0.0
                                  for i in (0, 1) for k in (0, 1) for sign in ("P", "N")}},
        },
        "bounds": {"intact:Body": [[0.45, -0.15, 0.81], [0.49, -0.07, 0.88]],
                   "damaged:Body": [[C.BAD[0]-0.030, C.BAD[1]-0.036, C.TOP+C.PAD_H],
                                    [C.BAD[0]+0.032, C.BAD[1]+0.036, C.BAD[2]+C.BODY_Z/2]]},
        "hands": {},
    }


def insert(s, name="intact", outlet=0, roll=False):
    s["objects"][name]["pose"] = [
        C.SOCKET[0]-C.BODY_X/2-0.0007,
        C.SOCKET[1]+C.OUTLET_YS[outlet], C.SOCKET[2],
        *([1, 0, 0, 0] if roll else [0, 0, 0, 1]),
    ]
    for k in (0, 1):
        for sign in ("P", "N"):
            s["objects"]["socket"]["joints"][f"J_Leaf{outlet}_{k}_{sign}"] = 0.00026


def test_do_nothing_never_scores():
    s, c = state(), scoring.Scorer()
    for t in (0, 5, 100):
        s["time_s"] = t
        assert c.evaluate(s)["score"] == 0


@pytest.mark.parametrize("roll", [False, True])
@pytest.mark.parametrize("outlet", [0, 1])
def test_both_ordinary_type_a_orientations_and_outlets(roll, outlet):
    s = state()
    insert(s, outlet=outlet, roll=roll)
    assert scoring.connected_outlets(s, "intact") == [outlet]


def test_proximity_without_sprung_contact_is_not_charging():
    s = state()
    insert(s)
    for key in s["objects"]["socket"]["joints"]:
        s["objects"]["socket"]["joints"][key] = 0
    assert scoring.Scorer().evaluate(s)["score"] == 0


def test_one_missing_conductor_cannot_close_circuit():
    s = state()
    insert(s)
    s["objects"]["socket"]["joints"]["J_Leaf0_1_P"] = 0
    s["objects"]["socket"]["joints"]["J_Leaf0_1_N"] = 0
    assert not scoring.connected_outlets(s, "intact")


@pytest.mark.parametrize("roll", [False, True])
def test_bad_then_unplug_permanently_gates_good_credit(roll):
    s, c = state(), scoring.Scorer()
    insert(s, "damaged", outlet=1, roll=roll)
    c.observe(s)
    s["objects"]["damaged"]["pose"] = [*C.BAD, 0, 0, 0, 1]
    insert(s, "intact", outlet=0)
    assert c.evaluate(s)["score"] == 0
    assert "damaged_device_energized" in c.violations


def test_held_charging_is_partial_not_full(monkeypatch):
    s, c = state(), scoring.Scorer()
    insert(s)
    s["hands"] = {"measured": {}}
    monkeypatch.setattr(scoring.geometry, "robot_clearance", lambda *a: 0.0)
    assert c.evaluate(s)["score"] == 0.5
    s["time_s"] = 100
    assert c.evaluate(s)["score"] == 0.5


def test_released_stable_charge_qualifies_after_three_seconds(monkeypatch):
    s, c = state(), scoring.Scorer()
    insert(s)
    s["hands"] = {"measured": {}}
    monkeypatch.setattr(scoring.geometry, "robot_clearance", lambda *a: 0.10)
    assert c.evaluate(s)["score"] == 0.5
    s["time_s"] = 2.9
    assert c.evaluate(s)["score"] == 0.5
    s["time_s"] = 3.1
    assert c.evaluate(s)["score"] == 1.0


def test_damaged_drop_cannot_be_erased_by_good_charge():
    s, c = state(), scoring.Scorer()
    s["objects"]["damaged"]["pose"][2] = C.TOP - 0.08
    c.observe(s)
    s["objects"]["damaged"]["pose"] = [*C.BAD, 0, 0, 0, 1]
    insert(s)
    assert c.evaluate(s)["score"] == 0


def test_gentle_safe_repositioning_is_permitted():
    s, c = state(), scoring.Scorer()
    s["objects"]["damaged"]["pose"][1] += 0.03
    s["objects"]["damaged"]["velocity"][1] = 0.03
    c.observe(s)
    assert not c.violations


def test_partial_damaged_energization_invalidates_before_seating():
    s, c = state(), scoring.Scorer()
    insert(s, "damaged")
    s["objects"]["damaged"]["pose"][0] -= 0.006
    assert not scoring.plug_seated(s, "damaged")
    c.observe(s)
    assert "damaged_device_energized" in c.violations


def test_one_conducting_leaf_per_pole_is_enough():
    s = state()
    insert(s)
    for pole in (0,1):
        s["objects"]["socket"]["joints"][f"J_Leaf0_{pole}_P"] = 0
    assert scoring.connected_outlets(s, "intact") == [0]


def test_no_positive_score_while_damaged_device_is_unsupported(monkeypatch):
    s, c = state(), scoring.Scorer()
    insert(s)
    s["hands"] = {"measured": {}}
    monkeypatch.setattr(scoring.geometry, "robot_clearance", lambda *a: 0.10)
    s["bounds"]["damaged:Body"][0][2] += 0.10
    s["bounds"]["damaged:Body"][1][2] += 0.10
    c.evaluate(s)
    s["time_s"] = 100
    assert c.evaluate(s)["score"] == 0.0
    assert not c.violations
    s["bounds"]["damaged:Body"][0][2] -= 0.10
    s["bounds"]["damaged:Body"][1][2] -= 0.10
    assert c.evaluate(s)["score"] == 0.5
