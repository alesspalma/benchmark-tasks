"""Adversarial geometric repair states, independent of the reference motion."""

import copy, json, unittest
from pathlib import Path
import numpy as np
from chain_link.config import (
    CHAIN_POSITION,
    NODES,
    CONNECTOR_TARGET,
    CONNECTOR_POSITION,
    KEEPER_POSITION,
)
from chain_link.scoring import Scorer, routing, simple_loop


def chain_bodies(nodes=NODES):
    bodies = {}
    for i, (a, b) in enumerate(zip(nodes[:-1], nodes[1:])):
        yaw = np.arctan2(*(b - a)[1::-1])
        p = CHAIN_POSITION + (a + b) / 2
        bodies[f"Link{i:02}"] = {
            "pose": [*p, 0.0, 0.0, np.sin(yaw / 2), np.cos(yaw / 2)],
            "velocity": [0.0] * 6,
        }
    return bodies


def assembly():
    def part(p):
        return {"pose": [*p, 0.0, 0.0, 0.0, 1.0], "velocity": [0.0] * 6}

    return dict(
        time_s=0.0,
        bodies=chain_bodies(),
        connector=part(CONNECTOR_TARGET),
        keeper=part(CONNECTOR_TARGET + [0.014, 0, -0.037]),
        hands={n: {"opening_m": 0.09} for n in ("left_robot", "right_robot")},
        robot_clearance_m=0.04,
    )


class RepairTests(unittest.TestCase):
    def final(self, s, score=None):
        score = score or Scorer()
        score.evaluate(s)
        s["time_s"] += 1.01
        return score.evaluate(s)

    def test_real_closed_routed_loop_passes_after_hold(self):
        s = assembly()
        sc = Scorer()
        self.assertEqual(sc.evaluate(s)["score"], 0.7)
        s["time_s"] = 1.01
        m = sc.evaluate(s)
        self.assertEqual(m["score"], 1)
        self.assertTrue(m["closed_loop"])
        self.assertTrue(m["chain_routed"])

    def test_routed_but_disconnected_reset_is_zero(self):
        s = assembly()
        s["connector"]["pose"][:3] = CONNECTOR_POSITION.tolist()
        s["keeper"]["pose"][:3] = KEEPER_POSITION.tolist()
        m = self.final(s)
        self.assertTrue(m["chain_routed"])
        self.assertFalse(m["ends_joined"])
        self.assertEqual(m["score"], 0)

    def test_both_ends_without_keeper_only_partial(self):
        s = assembly()
        s["keeper"]["pose"][1] += 0.3
        self.assertEqual(self.final(s)["score"], 0.4)

    def test_keeper_at_slot_mouth_not_full(self):
        s = assembly()
        s["keeper"]["pose"][0] = CONNECTOR_TARGET[0] - 0.018
        m = self.final(s)
        self.assertTrue(m["plate_seated"])
        self.assertFalse(m["retained"])
        self.assertEqual(m["score"], 0.7)

    def test_stems_at_closed_slot_ends_are_valid(self):
        s = assembly()
        s["keeper"]["pose"][0] = CONNECTOR_TARGET[0] + 0.021
        self.assertEqual(self.final(s)["score"], 1)

    def test_joint_assembled_off_the_sprockets_gets_zero(self):
        s = assembly()
        for p in [s["connector"], s["keeper"], *s["bodies"].values()]:
            p["pose"][1] += 0.25
        m = self.final(s)
        self.assertTrue(m["retained"])
        self.assertFalse(m["chain_routed"])
        self.assertEqual(m["score"], 0)

    def test_closed_loop_lifted_above_sprockets_gets_zero(self):
        s = assembly()
        for p in [s["connector"], s["keeper"], *s["bodies"].values()]:
            p["pose"][2] += 0.04
        m = self.final(s)
        self.assertTrue(m["retained"])
        self.assertFalse(m["chain_routed"])
        self.assertEqual(m["score"], 0)

    def test_chain_derailed_from_one_wheel_is_zero(self):
        s = assembly()
        for i in (4, 5, 6):
            s["bodies"][f"Link{i:02}"]["pose"][0] -= 0.04
        m = self.final(s)
        self.assertTrue(m["ends_joined"])
        self.assertFalse(m["chain_routed"])
        self.assertEqual(m["score"], 0)

    def test_chain_not_on_pitch_seats_fails(self):
        s = assembly()
        for p in s["bodies"].values():
            p["pose"][1] += 0.006
        self.assertFalse(routing(s)["chain_routed"])

    def test_small_irrelevant_routing_offset_is_allowed(self):
        s = assembly()
        for p in [s["connector"], s["keeper"], *s["bodies"].values()]:
            p["pose"][0] += 0.002
        self.assertEqual(self.final(s)["score"], 1)

    def test_self_crossing_and_folded_loops_rejected(self):
        self.assertFalse(simple_loop(np.array([[0, 0], [1, 1], [0, 1], [1, 0.0]])))
        self.assertFalse(simple_loop(np.array([[0, 0], [1, 0], [0.5, 0], [0.5, 1.0]])))

    def test_either_end_missing_is_zero(self):
        for name in ("Link00", "Link10"):
            s = assembly()
            s["bodies"][name]["pose"][0] += 0.03
            self.assertEqual(self.final(s)["score"], 0)

    def test_inverted_keeper_rejected(self):
        s = assembly()
        s["keeper"]["pose"][3:] = [0, 0, 1, 0]
        self.assertEqual(self.final(s)["score"], 0.4)

    def test_keeper_under_heads_or_above_chain_rejected(self):
        for z in (-0.055, 0.02):
            s = assembly()
            s["keeper"]["pose"][2] = CONNECTOR_TARGET[2] + z
            self.assertEqual(self.final(s)["score"], 0.4)

    def test_each_hand_must_open(self):
        for name in ("left_robot", "right_robot"):
            s = assembly()
            s["hands"][name]["opening_m"] = 0.02
            self.assertEqual(self.final(s)["score"], 0.7)

    def test_whole_robot_must_clear(self):
        s = assembly()
        s["robot_clearance_m"] = 0.01
        self.assertEqual(self.final(s)["score"], 0.7)

    def test_all_links_must_settle(self):
        for i in range(11):
            s = assembly()
            s["bodies"][f"Link{i:02}"]["velocity"][0] = 0.03
            self.assertEqual(self.final(s)["score"], 0.7)

    def test_hold_restarts_after_contact(self):
        s = assembly()
        sc = Scorer()
        sc.evaluate(s)
        s["time_s"] = 0.9
        s["robot_clearance_m"] = 0
        sc.evaluate(s)
        s["time_s"] = 1.0
        s["robot_clearance_m"] = 0.04
        sc.evaluate(s)
        s["time_s"] = 1.2
        self.assertFalse(sc.evaluate(s)["success"])
        s["time_s"] = 2.01
        self.assertTrue(sc.evaluate(s)["success"])

    def test_score_does_not_keep_past_success(self):
        s = assembly()
        sc = Scorer()
        self.assertEqual(self.final(s, sc)["score"], 1)
        s["keeper"]["pose"][1] += 0.3
        self.assertEqual(sc.evaluate(s)["score"], 0.4)

    def test_escape_violation_permanent(self):
        sc = Scorer()
        sc.observe_physics({"part": np.array([[0.2, 0.66, 0.8]])})
        sc.observe_physics({"part": np.array([[0.2, 0, 0.8]])})
        self.assertEqual(self.final(assembly(), sc)["score"], 0)

    def test_invalid_state_fails_closed(self):
        s = assembly()
        s["bodies"]["Link05"]["pose"][0] = np.nan
        sc = Scorer()
        self.assertEqual(sc.evaluate(s)["score"], 0)
        self.assertEqual(self.final(assembly(), sc)["score"], 0)

    def test_state_not_mutated(self):
        s = assembly()
        a = copy.deepcopy(s)
        Scorer().evaluate(s)
        self.assertEqual(s, a)


class PhysicalForkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        records = json.loads(
            (Path(__file__).resolve().parents[1] / "scene-inventory.json").read_text()
        )
        cls.cells = [
            r for r in records if r.get("asset") == "keeper" and r.get("kind") == "box"
        ]

    def occupied(self, p):
        return any(
            np.all(abs(np.array(p) - r["center_m"]) < np.array(r["size_m"]) / 2 - 1e-9)
            for r in self.cells
        )

    def circle_hits(self, x, y, r):
        return any(
            self.occupied([x + r * np.cos(t), y + r * np.sin(t), 0])
            for t in np.linspace(0, 2 * np.pi, 180)
        )

    def test_heads_blocked_but_stems_fit(self):
        for y in (-0.05, 0.05):
            self.assertTrue(self.circle_hits(-0.014, y, 0.012))
            self.assertFalse(self.circle_hits(-0.014, y, 0.005))

    def test_two_stems_can_slide_from_open_edges(self):
        for y in (-0.05, 0.05):
            for x in np.linspace(0.05, -0.014, 50):
                self.assertFalse(self.circle_hits(x, y, 0.005))


if __name__ == "__main__":
    unittest.main()
