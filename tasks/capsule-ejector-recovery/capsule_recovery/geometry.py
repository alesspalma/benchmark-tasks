"""Read-only bounds of native collision geometry, used exclusively by scoring."""

from functools import lru_cache
from itertools import product

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

SIGNS = np.asarray(list(product((-1.0, 1.0), repeat=3)))


def collision_points(spec):
    from pxr import Usd, UsdGeom, UsdPhysics

    groups = {}
    for asset in spec["assets"]:
        if asset["type"] == "base":
            continue
        stage = Usd.Stage.Open(str(asset["file"]))
        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), ["default", "render", "proxy"]
        )
        for prim in stage.Traverse():
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            body = prim
            while body and not body.HasAPI(UsdPhysics.RigidBodyAPI):
                body = body.GetParent()
            if not body:
                raise RuntimeError("Collider without a native rigid body")
            box = cache.ComputeRelativeBound(prim, body).ComputeAlignedRange()
            lo, hi = np.asarray(box.GetMin()), np.asarray(box.GetMax())
            pts = (lo + hi) / 2 + SIGNS * (hi - lo) / 2
            groups.setdefault(asset["name"] + ":" + body.GetName(), []).append(pts)
    return {k: np.concatenate(v) for k, v in groups.items()}


def world_bounds(local, state):
    result = {}
    for key, points in local.items():
        asset, body = key.split(":")
        obj = state["objects"][asset]
        pose = obj["links"][body] if "links" in obj else obj["pose"]
        world = points @ quat_xyzw_to_matrix(np.asarray(pose[3:])).T + pose[:3]
        result[key] = [world.min(0).tolist(), world.max(0).tolist()]
    return result


@lru_cache(maxsize=1)
def robot_hulls():
    from yam_arena.analysis import link_hulls

    return {n: [h.vertices for h in hulls] for n, hulls in link_hulls().items()}


def robot_clearance(bounds, hands):
    if len(hands) != 2:
        raise ValueError("Two measured robot states are required")
    objects = np.asarray(list(bounds.values()))
    distance = float("inf")
    count = 0
    for hand in hands.values():
        for name, pose in zip(hand["names"], hand["poses"], strict=True):
            for vertices in robot_hulls().get(name, []):
                p = vertices @ quat_xyzw_to_matrix(np.asarray(pose[3:])).T + pose[:3]
                lo, hi = p.min(0), p.max(0)
                gap = np.maximum(np.maximum(objects[:, 0] - hi, lo - objects[:, 1]), 0)
                distance = min(distance, float(np.linalg.norm(gap, axis=1).min()))
                count += 1
    if not count:
        raise RuntimeError("Native robot collision geometry is required")
    return distance


def contained(bounds, lo, hi, tolerance=0.0005):
    box = np.asarray(bounds)
    return bool(
        np.all(box[0] >= np.asarray(lo) - tolerance)
        and np.all(box[1] <= np.asarray(hi) + tolerance)
    )
