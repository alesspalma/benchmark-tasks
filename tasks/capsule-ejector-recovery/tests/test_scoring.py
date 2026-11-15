"""Scorer counterexamples on synthetic states; these are not feasibility demos."""

import copy
import math

from capsule_recovery import config as C
from capsule_recovery.scoring import Scorer


def body(p):
    return {"pose": [*map(float, p), 0.0, 0.0, 0.0, 1.0], "velocity": [0.0] * 6}


def fixture():
    o = {
        "machine": {**body([0, 0, 0]), "joints": {"HeadHinge": 0.0}},
        "bin": body(C.BIN),
        "fresh": body(C.SLOT),
    }
    bounds = {
        "bin:Body": [C.BIN + [-0.16, -0.06, -0.0015], C.BIN + [0.085, 0.06, 0.15]],
        "fresh:Body": [
            C.SLOT + [-0.0185, -0.0185, 0],
            C.SLOT + [0.0185, 0.0185, 0.02825],
        ],
    }
    for k in range(C.USED_COUNT):
        p = C.WASTE + [(k % 3 - 1) * 0.05, (k // 3 - 1) * 0.045, 0.004]
        o[f"used{k}"] = body(p)
        bounds[f"used{k}:Body"] = [
            p + [-0.0185, -0.0185, 0],
            p + [0.0185, 0.0185, 0.02825],
        ]
    return {
        "time_s": 0.0,
        "objects": o,
        "bounds": bounds,
        "hands": {n: {"opening_m": 0.09} for n in ("left", "right")},
        "robot_clearance_m": 0.1,
    }


def finish(s, scorer=None):
    scorer = scorer or Scorer()
    for k in range(43):
        s["time_s"] = k / 20
        scorer.observe(s)
        m = scorer.evaluate(s)
    return m


def test_complete_supported_outcome():
    assert finish(fixture())["score"] == 1.0


def test_root_inside_but_flange_outside_does_not_count():
    s = fixture()
    b = s["bounds"]["used0:Body"]
    b[1][0] = C.WASTE[0] + C.WASTE_INNER[0] / 2 + 0.001
    assert finish(s)["score"] == 0.0


def test_wrong_fresh_capsule_height_cannot_finish():
    s = fixture()
    s["objects"]["fresh"]["pose"][2] += 0.01
    assert finish(s)["score"] == 0.65


def test_missing_collector_gets_only_collection_credit():
    s = fixture()
    s["objects"]["bin"]["pose"][0] -= 0.1
    assert finish(s)["score"] == 0.35


def test_open_head_cannot_finish():
    s = fixture()
    s["objects"]["machine"]["joints"]["HeadHinge"] = math.radians(8)
    assert finish(s)["score"] == 0.65


def test_open_fingers_propping_assembly_cannot_finish():
    s = fixture()
    s["robot_clearance_m"] = 0.0
    assert finish(s)["score"] == 0.65


def test_missing_hand_measurements_cannot_finish():
    s = fixture()
    s["hands"] = {}
    assert finish(s)["score"] == 0.65


def test_airborne_capsule_does_not_get_collection_credit():
    s = fixture()
    s["objects"]["used0"]["velocity"][2] = 0.2
    assert finish(s)["score"] == 0.0


def test_dropped_part_permanently_invalidates_trial():
    s = fixture()
    sc = Scorer()
    bad = copy.deepcopy(s)
    bad["bounds"]["used0:Body"][0][2] = C.TOP - 0.1
    sc.observe(bad)
    assert finish(s, sc)["score"] == 0.0


def test_no_credit_from_time_jump_or_repeated_evaluation():
    s = fixture()
    sc = Scorer()
    sc.evaluate(s)
    s["time_s"] = 100.0
    for _ in range(100):
        m = sc.evaluate(s)
    assert m["score"] == 0.0


def test_stationary_pose_is_not_failed_by_angular_velocity_chatter():
    s = fixture()
    s["objects"]["fresh"]["velocity"][3] = 0.3
    assert finish(s)["score"] == 1.0


def test_slow_real_translation_is_not_rest():
    s = fixture()
    sc = Scorer()
    s["objects"]["used0"]["velocity"][0] = 0.004
    for k in range(43):
        s["time_s"] = k / 20
        s["objects"]["used0"]["pose"][0] += 0.0002
        s["bounds"]["used0:Body"][0][0] += 0.0002
        s["bounds"]["used0:Body"][1][0] += 0.0002
        sc.observe(s)
        m = sc.evaluate(s)
    assert m["score"] == 0.0


def test_slow_real_rotation_is_not_rest():
    s = fixture()
    sc = Scorer()
    s["objects"]["fresh"]["velocity"][5] = 0.04
    for k in range(43):
        s["time_s"] = k / 20
        angle = 0.04 * k / 20
        s["objects"]["fresh"]["pose"][3:] = [
            0,
            0,
            math.sin(angle / 2),
            math.cos(angle / 2),
        ]
        sc.observe(s)
        m = sc.evaluate(s)
    assert m["score"] == 0.65


def test_reset_requires_new_observed_rest_history():
    s = fixture()
    sc = Scorer()
    assert finish(s, sc)["score"] == 1.0
    sc.reset()
    assert sc.evaluate(s)["score"] == 0.0


def test_oscillation_returning_to_start_is_not_rest():
    s = fixture()
    sc = Scorer()
    for k in range(43):
        s["time_s"] = k / 20
        angle = math.radians(5) * math.sin(2 * math.pi * k / 4)
        s["objects"]["fresh"]["pose"][3:] = [
            0,
            0,
            math.sin(angle / 2),
            math.cos(angle / 2),
        ]
        sc.observe(s)
        m = sc.evaluate(s)
    assert m["score"] == 0.65


def test_completion_is_revoked_when_outcome_changes_without_time_advance():
    s = fixture()
    sc = Scorer()
    assert finish(s, sc)["score"] == 1.0
    s["objects"]["fresh"]["pose"][2] += 0.01
    assert sc.evaluate(s)["score"] == 0.65
