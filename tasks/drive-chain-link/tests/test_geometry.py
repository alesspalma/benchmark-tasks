"""Numeric clearance must include all robot collision hulls."""

from itertools import product
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import patch

import numpy as np
from chain_link.geometry import local_solids, robot_clearance
from yam_arena.urdf import quat_xyzw_to_matrix


class ClearanceTests(TestCase):
    def test_envelope_preserves_bounds_after_rigid_transforms(self):
        outer = np.array(list(product((-2.0, 2.0), (-1.0, 1.0), (-0.1, 0.1))))
        inside = np.random.default_rng(17).uniform(
            [-1, -0.5, -0.05], [1, 0.5, 0.05], (100, 3)
        )
        vertices = np.concatenate([outer, inside])
        reduced = local_solids(
            [
                {
                    "asset": "part",
                    "solid": "/Part/Body/Mesh",
                    "kind": "mesh",
                    "vertices_m": vertices.tolist(),
                }
            ]
        )["part:Body"]
        self.assertEqual(len(reduced), 8)
        for quaternion in np.random.default_rng(23).normal(size=(20, 4)):
            quaternion /= np.linalg.norm(quaternion)
            rotation = quat_xyzw_to_matrix(quaternion)
            full, envelope = vertices @ rotation.T, reduced @ rotation.T
            np.testing.assert_allclose(full.min(0), envelope.min(0), atol=1e-12)
            np.testing.assert_allclose(full.max(0), envelope.max(0), atol=1e-12)

    def test_box_separation_overlap_and_corner_distance(self):
        cube = np.array(list(product((-1.0, 1.0), repeat=3)))
        hull = SimpleNamespace(vertices=cube)
        with patch("yam_arena.analysis.link_hulls", return_value={"mock": [hull]}):
            for p, d in (([0, 0, 0], 0), ([3, 0, 0], 1), ([3, 3, 3], np.sqrt(3))):
                hands = {"robot": {"names": ["mock"], "poses": [[*p, 0, 0, 0, 1]]}}
                self.assertAlmostEqual(robot_clearance({"part": cube}, hands), d)

    def test_missing_robot_collision_geometry_is_an_error(self):
        with self.assertRaises(RuntimeError):
            robot_clearance({"part": np.zeros((8, 3))}, {})
