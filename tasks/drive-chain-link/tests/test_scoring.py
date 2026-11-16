"""Regression checks for real-clearance joining, positive retention and release."""

import copy
import math
import unittest

import numpy as np
from chain_link import config as C
from chain_link.scoring import Scorer


def body(position, quaternion=(0, 0, 0, 1)):
    return {"pose": [*map(float, position), *quaternion], "velocity": [0.0] * 6}


def assembled():
    s = {
        "time_s": 0.0,
        "connector": body(C.CONNECTOR_POSITION, (0, 1, 0, 0)),
        "keeper": body(C.CONNECTOR_POSITION + [0, 0, -C.COVER_Z]),
        "tool": body(C.TOOL_POSITION),
        "robot_clearance_m": 0.10,
        "hands": {n: {"opening_m": 0.08} for n in ("left", "right")},
        "bodies": {},
        "clips": {},
    }
    for n, (a, b) in enumerate(zip(C.NODES[:-1], C.NODES[1:])):
        yaw = math.atan2(*(b - a)[1::-1])
        s["bodies"][f"Link{n:02}"] = body(
            C.CHAIN_POSITION + (a + b) / 2, (0, 0, math.sin(yaw / 2), math.cos(yaw / 2))
        )
    for n, y in enumerate((-0.05, 0.05)):
        s["clips"][f"clip{n}"] = body(C.CONNECTOR_POSITION + [0, y, -C.GROOVE_Z])
        s["clips"][f"clip{n}"]["joints"] = {"Spring0": 0.0, "Spring2": 0.0}
    return s


class ScoringTests(unittest.TestCase):
    def test_full_score_requires_continuous_release(self):
        s = assembled()
        c = Scorer()
        self.assertEqual(c.evaluate(s)["score"], 0.85)
        for i in range(1, 21):
            s["time_s"] = i / 20
            result = c.evaluate(s)
        self.assertTrue(result["success"])
        self.assertEqual(result["score"], 1.0)

    def test_gravity_supported_cover_stack_is_within_full_score_geometry(self):
        s = assembled()
        # Gravity seats each clip against the lower shoulder of its groove.
        # The cover's lower face then rests on the clips' upper faces.
        clip_z = C.GROOVE_Z + (C.GROOVE_WIDTH - C.CLIP_THICKNESS) / 2
        cover_z = clip_z - (C.CLIP_THICKNESS + C.PLATE_THICKNESS) / 2
        for clip in s["clips"].values():
            clip["pose"][2] = float(C.CONNECTOR_POSITION[2] - clip_z)
        s["keeper"]["pose"][2] = float(C.CONNECTOR_POSITION[2] - cover_z)
        scorer = Scorer()
        for i in range(21):
            s["time_s"] = i / C.CONTROL_HZ
            result = scorer.evaluate(s)
        self.assertEqual(result["score"], 1.0)

    def test_tight_bore_rejects_original_loose_alignment(self):
        s = assembled()
        s["bodies"]["Link00"]["pose"][0] += 0.0002
        self.assertEqual(Scorer().evaluate(s)["score"], 0.0)

    def test_unseated_cover(self):
        s = assembled()
        s["keeper"]["pose"][0] += 0.0002
        self.assertEqual(Scorer().evaluate(s)["score"], 0.35)

    def test_cover_can_rock_within_its_actual_bore_clearance(self):
        s = assembled()
        angle = 0.047
        # This short-axis tilt displaces the 4 mm straight bore faces by
        # 0.094 mm, within the 0.100 mm radial pin/bore clearance.
        s["keeper"]["pose"][3:] = [0, math.sin(angle / 2), 0, math.cos(angle / 2)]
        self.assertTrue(Scorer().evaluate(s)["plate_seated"])

    def test_cover_center_alone_does_not_establish_both_holes_seated(self):
        s = assembled()
        angle = 0.03
        s["keeper"]["pose"][3:] = [math.sin(angle / 2), 0, 0, math.cos(angle / 2)]
        self.assertEqual(Scorer().evaluate(s)["score"], 0.35)

    def test_each_clip_is_required(self):
        for n in ("clip0", "clip1"):
            with self.subTest(n=n):
                s = assembled()
                s["clips"][n]["pose"][2] -= 0.003
                self.assertEqual(Scorer().evaluate(s)["score"], 0.65)

    def test_identical_clips_are_interchangeable(self):
        s = assembled()
        s["clips"]["clip0"], s["clips"]["clip1"] = (
            s["clips"]["clip1"],
            s["clips"]["clip0"],
        )
        self.assertTrue(Scorer().evaluate(s)["retained"])

    def test_symmetric_connector_can_exchange_its_identical_pins(self):
        s = assembled()
        s["connector"]["pose"][3:] = [1, 0, 0, 0]
        c = Scorer()
        for i in range(21):
            s["time_s"] = i / 20
            result = c.evaluate(s)
        self.assertEqual(result["score"], 1.0)

    def test_symmetric_cover_can_exchange_its_identical_bores(self):
        s = assembled()
        s["keeper"]["pose"][3:] = [0, 0, 1, 0]
        self.assertTrue(Scorer().evaluate(s)["retained"])

    def test_two_clips_on_one_pin_cannot_retain_both_pins(self):
        s = assembled()
        s["clips"]["clip1"] = copy.deepcopy(s["clips"]["clip0"])
        self.assertEqual(Scorer().evaluate(s)["score"], 0.65)

    def test_clip_must_close_in_its_groove(self):
        for change in ("open", "axial"):
            s = assembled()
            p = s["clips"]["clip0"]
            if change == "open":
                p["joints"]["Spring0"] = math.radians(5)
            else:
                p["pose"][2] -= 0.001
            self.assertEqual(Scorer().evaluate(s)["score"], 0.65)

    def test_grasping_or_moving_or_displaced_chain_cannot_complete(self):
        for change in ("grasp", "velocity", "guide"):
            s = assembled()
            if change == "grasp":
                s["hands"]["left"]["opening_m"] = 0.01
            elif change == "velocity":
                s["connector"]["velocity"][0] = 0.03
            else:
                s["bodies"]["Link05"]["pose"][2] += 0.01
            c = Scorer()
            for i in range(30):
                s["time_s"] = i / 20
                r = c.evaluate(s)
            self.assertEqual(r["score"], 0.85)

    def test_round_groove_allows_retainer_rotation_while_captured(self):
        s = assembled()
        c = Scorer()
        for i in range(30):
            s["time_s"] = i / 20
            for clip in s["clips"].values():
                angle = i * 0.1
                clip["pose"][3:] = [0, 0, math.sin(angle / 2), math.cos(angle / 2)]
                clip["velocity"][5] = 2.0
            result = c.evaluate(s)
        self.assertEqual(result["score"], 1.0)
        s["clips"]["clip0"]["pose"][0] += 0.001
        self.assertEqual(c.evaluate(s)["score"], 0.65)

    def test_applicator_must_withdraw_from_both_captured_clips(self):
        s = assembled()
        s["tool"]["pose"][:2] = s["connector"]["pose"][:2]
        c = Scorer()
        for i in range(30):
            s["time_s"] = i / 20
            result = c.evaluate(s)
        self.assertEqual(result["score"], 0.85)
        self.assertFalse(result["applicator_clear"])

    def test_time_jump_does_not_establish_hold(self):
        s = assembled()
        c = Scorer()
        c.evaluate(s)
        s["time_s"] = 10
        self.assertFalse(c.evaluate(s)["success"])

    def test_release_interruption_restarts_hold(self):
        s = assembled()
        c = Scorer()
        for i in range(12):
            s["time_s"] = i / 20
            c.evaluate(s)
        s["hands"]["left"]["opening_m"] = 0.01
        s["time_s"] = 0.6
        c.evaluate(s)
        s["hands"]["left"]["opening_m"] = 0.08
        for i in range(13, 31):
            s["time_s"] = i / 20
            r = c.evaluate(s)
        self.assertFalse(r["success"])

    def test_disassembly_removes_earned_credit(self):
        s = assembled()
        c = Scorer()
        for i in range(21):
            s["time_s"] = i / 20
            c.evaluate(s)
        s["keeper"]["pose"][2] -= 0.01
        s["time_s"] = 1.05
        self.assertEqual(c.evaluate(s)["score"], 0.35)

    def test_invalid_or_incomplete_state_is_permanently_zero(self):
        for change in ("nan", "quaternion", "missing_joint"):
            s = assembled()
            c = Scorer()
            if change == "nan":
                s["connector"]["pose"][0] = float("nan")
            elif change == "quaternion":
                s["connector"]["pose"][4] = 0.0
            else:
                s["clips"]["clip0"]["joints"] = {}
            self.assertEqual(c.evaluate(s)["score"], 0.0)
            self.assertEqual(c.evaluate(assembled())["score"], 0.0)

    def test_physics_workspace_violation_is_permanent(self):
        s = assembled()
        c = Scorer()
        c.observe_workspace({"clip": np.array([[0.5, 0, C.TABLE_TOP - 0.01]])})
        self.assertEqual(c.evaluate(s)["score"], 0.0)


if __name__ == "__main__":
    unittest.main()
