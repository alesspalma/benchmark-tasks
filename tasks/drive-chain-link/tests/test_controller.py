"""Only ordinary robot joints and time may reach the reference controller."""

import unittest

import numpy as np
from chain_link.controller import ScriptedPolicy
from inspect_robots.guard import ClampGuard, DeltaLimitGuard


class Observation:
    def __init__(self, t, joints):
        self.state_time = t
        self._joints = joints

    @property
    def state(self):
        return {"joint_pos": self._joints}

    def __getattr__(self, key):
        raise AssertionError("Unavailable scene input: " + key)


class ControllerTests(unittest.TestCase):
    def test_reference_uses_only_time_and_measured_joints(self):
        p = ScriptedPolicy("ends")
        joints = p.previous.copy()
        guards = [ClampGuard(p.info.action_space), DeltaLimitGuard(p.info.action_space)]
        store = {}
        for i in range(round(p.nominal_duration * 20) + 1):
            chunk = p.act(Observation(i / 20, joints))
            self.assertEqual(len(chunk.actions), 1)
            action = chunk.actions[0]
            expected = action.data.copy()
            for g in guards:
                action = g.review(action, store)
            np.testing.assert_array_equal(action.data, expected)
            joints = action.data.copy()

    def test_reset_repeats_program_without_scene_data(self):
        p = ScriptedPolicy("ends")
        first = p.action_at(0.05, p.previous).copy()
        digest = p.program_sha256
        p.action_at(0.1, p.previous)
        p.reset(None)
        np.testing.assert_array_equal(first, p.action_at(0.05, p.previous))
        self.assertEqual(digest, p.program_sha256)

    def test_incorrect_rate_fails_before_execution(self):
        class Wrong:
            control_hz = 10

        with self.assertRaises(ValueError):
            ScriptedPolicy().bind(Wrong())

    def test_grasp_waits_for_ordinary_joint_tracking(self):
        p = ScriptedPolicy("ends")
        end = round(p.precision_boundaries[0] * 20)
        for i in range(1, end + 1):
            p.action_at(i / 20, p.previous.copy())
        frozen = p.program_time
        wrong = p.nominal.copy()
        wrong[8] -= 0.1
        p.action_at((end + 1) / 20, wrong)
        self.assertEqual(p.program_time, frozen)

    def test_holding_arm_does_not_block_other_arms_precision_pose(self):
        p = ScriptedPolicy("ends")
        end = round(p.precision_boundaries[0] * 20)
        for i in range(1, end + 1):
            p.action_at(i / 20, p.previous.copy())
        frozen = p.program_time
        measured = p.nominal.copy()
        measured[1] -= 0.1
        p.action_at((end + 1) / 20, measured)
        self.assertGreater(p.program_time, frozen)

    def test_small_encoder_oscillation_cannot_stall_the_program_indefinitely(self):
        p = ScriptedPolicy("ends")
        boundary = float(p.precision_boundaries[0])
        for i in range(1, round(boundary * 20) + 1):
            p.action_at(i / 20, p.previous.copy())
        measured = p.nominal.copy()
        measured[p.precision_tracking[boundary][0]] += 0.0035
        for i in range(1, 61):
            p.action_at(boundary + i / 20, measured)
        self.assertGreater(p.program_time, boundary)

    def test_fine_precision_pose_uses_stricter_encoder_tolerance(self):
        p = ScriptedPolicy()
        boundary, tolerance = next(
            (b, tol) for b, tol in p.precision_tolerances.items() if tol < 0.002
        )
        for i in range(1, round(boundary * 20) + 1):
            p.action_at(i / 20)
        p.program_time = boundary
        p.last_time = boundary
        measured = p.nominal.copy()
        measured[p.precision_tracking[boundary][0]] += (tolerance + 0.002) / 2
        p.action_at(boundary + 0.05, measured)
        self.assertEqual(p.program_time, boundary)


if __name__ == "__main__":
    unittest.main()
