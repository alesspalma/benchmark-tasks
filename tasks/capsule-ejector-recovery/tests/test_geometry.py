"""Containment regressions for tilted solids and round native colliders."""

import numpy as np
from capsule_recovery.geometry import collision_points, world_bounds
from pxr import Gf, Usd, UsdGeom, UsdPhysics
from yam_arena.urdf import axis_angle_matrix


def scene(tmp_path):
    path = tmp_path / "shape.usda"
    stage = Usd.Stage.CreateNew(str(path))
    body = UsdGeom.Xform.Define(stage, "/Body")
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    spec = {"assets": [{"name": "part", "type": "rigid", "file": path}]}
    return stage, spec


def test_tilted_wedge_has_no_imaginary_corners_below_support(tmp_path):
    stage, spec = scene(tmp_path)
    vertices = np.array(
        [[x, y, z] for y in (-0.005, 0.005) for x, z in ((0, 0), (0.02, 0), (0, 0.02))]
    )
    mesh = UsdGeom.Mesh.Define(stage, "/Body/Wedge")
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in vertices])
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    stage.GetRootLayer().Save()
    angle = 3 * np.pi / 4
    rotation = axis_angle_matrix(np.array([0, 1, 0]), angle)
    q = [0, np.sin(angle / 2), 0, np.cos(angle / 2)]
    height = 0.02 / np.sqrt(2)
    bounds = world_bounds(
        collision_points(spec), {"objects": {"part": {"pose": [0, 0, height, *q]}}}
    )
    actual = np.array(bounds["part:Body"])
    expected = vertices @ rotation.T + [0, 0, height]
    np.testing.assert_allclose(actual, [expected.min(0), expected.max(0)], atol=1e-8)
    assert actual[0, 2] > -1e-8  # Its rotated box would extend 14 mm below the support.


def test_cylinder_envelope_contains_analytical_round_surface(tmp_path):
    stage, spec = scene(tmp_path)
    cylinder = UsdGeom.Cylinder.Define(stage, "/Body/Flange")
    cylinder.CreateRadiusAttr(0.0185)
    cylinder.CreateHeightAttr(0.0018)
    cylinder.CreateAxisAttr("Z")
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
    stage.GetRootLayer().Save()
    angle = 0.37
    axis = np.array([1, 2, 3]) / np.sqrt(14)
    rotation = axis_angle_matrix(axis, angle)
    q = [*(axis * np.sin(angle / 2)), np.cos(angle / 2)]
    bounds = np.array(
        world_bounds(
            collision_points(spec), {"objects": {"part": {"pose": [0, 0, 0, *q]}}}
        )["part:Body"]
    )
    direction = rotation[:, 2]
    exact_half = 0.0185 * np.sqrt(1 - direction**2) + 0.0009 * np.abs(direction)
    assert np.all(bounds[1] >= exact_half - 1e-10)
    assert np.all(bounds[1] - exact_half < 6e-6)
    np.testing.assert_allclose(bounds[0], -bounds[1], atol=1e-10)
