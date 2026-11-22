import unittest

import numpy as np

from vacuum_seal import config as C
from vacuum_seal.circuit import Circuit
from vacuum_seal.scoring import Scorer, supports_plate


def plate_pose(*, gap, compression):
    return np.r_[C.CUP_CENTER + [0, 0, compression - C.PLATE_SIZE[2] / 2 - gap], 0, 0, 0, 1]


def advance(circuit, joints, pose, dt, *, rim_load, rim_contact):
    lip = np.r_[C.CUP_CENTER + [0, 0, joints["BellowsTravel"]], 0, 0, 0, 1]
    return circuit.advance(joints, pose, lip, dt, rim_load=rim_load, rim_contact=rim_contact)


def observed_state(pose, *, hand=None):
    return {
        "objects": {"plate": {"pose": pose, "velocity": np.zeros(6)}},
        "hands": {"arm": {"points": [hand], "opening_m": 0.012}} if hand is not None else {},
        "plate_contacts": {"supported": False, "hand_touch": False},
    }


class ContactScoringTests(unittest.TestCase):
    def test_small_gap_and_light_touch_earn_no_vacuum(self):
        sequences = (
            (0.0018, 0.0020, 0.0, False),   # Within the old gap tolerance.
            (0.0001, 0.0020, 8.0, False),   # Speculative force without contact.
            (0.0018, 0.0020, 8.0, True),    # Rim contact on one side cannot bridge a gap.
            (0.0000, 0.00004, 8.0, True),   # Touch without useful bellows travel.
        )
        for gap, compression, load, contact in sequences:
            with self.subTest(gap=gap, compression=compression, contact=contact):
                circuit = Circuit()
                scorer = Scorer()
                pose = plate_pose(gap=gap, compression=compression)
                joints = {"BellowsTravel": compression, "SupplyTravel": 0.028, "VentTravel": 0.0}
                for _ in range(C.PHYSICS_HZ):
                    force, _ = advance(circuit, joints, pose, 1 / C.PHYSICS_HZ,
                                       rim_load=load, rim_contact=contact)
                    self.assertFalse(circuit.sealed)
                    np.testing.assert_array_equal(force, np.zeros(3))
                score = scorer.evaluate(observed_state(pose), circuit, C.CHECK_DT)
                self.assertEqual(score["score"], 0.0)
                self.assertLess(circuit.pressure, 2.0)

    def test_native_force_patterns(self):
        self.assertFalse(supports_plate(np.zeros((3, 3))))
        self.assertFalse(supports_plate(np.array([[0, 0, 0], [0, 4, 0], [0, 0, 0]])))
        self.assertTrue(supports_plate(np.array([[0, 0, 0], [0, 4, 0], [0, -4, 0]])))
        self.assertTrue(supports_plate(np.array([[0, 0, 1.2], [0, 0, 0], [0, 0, 0]])))

    def test_nearby_closed_hand_cannot_validate_vent(self):
        circuit = Circuit()
        scorer = Scorer()
        pose = plate_pose(gap=0.0, compression=0.002)
        joints = {"BellowsTravel": 0.002, "SupplyTravel": 0.028, "VentTravel": 0.0}
        for _ in range(20):
            advance(circuit, joints, pose, C.CHECK_DT, rim_load=8.0, rim_contact=True)
        self.assertEqual(scorer.evaluate(observed_state(pose), circuit, C.CHECK_DT)["score"], 0.35)
        joints["SupplyTravel"] = 0.0
        for _ in range(40):
            advance(circuit, joints, pose, C.CHECK_DT, rim_load=8.0, rim_contact=True)
            scorer.observe_hold(circuit, elevated=True, hand_touch=False, dt=C.CHECK_DT)
            measured = scorer.evaluate(observed_state(pose), circuit, C.CHECK_DT)
        self.assertEqual(measured["score"], 0.75)

        nearby = observed_state(pose, hand=pose[:3] + C.HANDLE_LOCAL + [0.010, 0, 0])
        scorer.observe_vent(vent=True, elevated=True, previous_pressure=circuit.pressure,
                           supported=nearby["plate_contacts"]["supported"])
        self.assertTrue(scorer.evaluate(nearby, circuit, C.CHECK_DT)["unsupported_vent"])
        # Even a later supported vent and correct parking cannot erase that event.
        scorer.observe_vent(vent=True, elevated=True, previous_pressure=circuit.pressure, supported=True)
        circuit.pressure = 0.0
        parked = observed_state(np.r_[C.PICK, 0, 0, 0, 1])
        for _ in range(20):
            final = scorer.evaluate(parked, circuit, C.CHECK_DT)
        self.assertEqual(final["score"], 0.75)
        self.assertTrue(final["plate_on_rest"])
        self.assertTrue(final["unsupported_vent"])

    def test_losing_contact_mid_hold_resets_timer(self):
        circuit = Circuit()
        scorer = Scorer()
        pose = plate_pose(gap=0.0, compression=0.002)
        joints = {"BellowsTravel": 0.002, "SupplyTravel": 0.0, "VentTravel": 0.0}
        circuit.pressure = 60.0
        for _ in range(C.PHYSICS_HZ):
            advance(circuit, joints, pose, 1 / C.PHYSICS_HZ, rim_load=8.0, rim_contact=True)
            scorer.observe_hold(circuit, elevated=True, hand_touch=False, dt=1 / C.PHYSICS_HZ)
        self.assertAlmostEqual(scorer.hold, 1.0)
        force, _ = advance(circuit, joints, plate_pose(gap=0.0018, compression=0.002),
                           1 / C.PHYSICS_HZ, rim_load=8.0, rim_contact=True)
        scorer.observe_hold(circuit, elevated=True, hand_touch=False, dt=1 / C.PHYSICS_HZ)
        self.assertFalse(circuit.sealed)
        np.testing.assert_array_equal(force, np.zeros(3))
        self.assertEqual(scorer.hold, 0.0)


if __name__ == "__main__":
    unittest.main()
