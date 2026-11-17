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


@lru_cache(maxsize=1)
def skull_top_triangles():
    """Exactly the fixed head mesh's upper triangles, in head-local metres."""
    from . import config as C
    import math
    points=[]
    for j in range(25):
        lat=-math.pi/2+math.pi*j/24
        for k in range(48):
            lon=math.tau*k/48
            points.append([.095*math.cos(lat)*math.cos(lon),np.clip(.088*math.cos(lat)*math.sin(lon),-C.HALF_HEAD_WIDTH,C.HALF_HEAD_WIDTH),C.CROWN_HEIGHT-.120+.120*math.sin(lat)])
    points=np.asarray(points);tri=[]
    for j in range(12,24):
        for k in range(48):
            a=j*48+k;b=j*48+(k+1)%48;c=(j+1)*48+(k+1)%48;d=(j+1)*48+k
            for ids in ((a,b,c),(a,c,d)):
                q=points[list(ids)];cross=np.linalg.det(np.stack([q[1,:2]-q[0,:2],q[2,:2]-q[0,:2]]))
                if abs(cross)>1e-14:tri.append(q)
    return np.asarray(tri)


def _cross2(a,b):
    return a[0]*b[1]-a[1]*b[0]


def _clip_to_triangle(polygon,triangle):
    sign=1. if _cross2(triangle[1]-triangle[0],triangle[2]-triangle[0])>0 else -1.
    poly=[np.asarray(p) for p in polygon]
    for i in range(3):
        if not poly:break
        a=triangle[i];edge=triangle[(i+1)%3]-a
        result=[];previous=poly[-1];old=sign*_cross2(edge,previous-a)
        for point in poly:
            new=sign*_cross2(edge,point-a)
            if (old>=-1e-14)!=(new>=-1e-14):
                result.append(previous+(point-previous)*(old/(old-new)))
            if new>=-1e-14:result.append(point)
            previous,old=point,new
        poly=result
    return poly


def crown_surface_gap(pose):
    """Exact minimum vertical Band-underside/head-triangle gap.

    The gap is linear over each intersection polygon, so its minimum is at
    a polygon vertex. This avoids positive bias from sampling a face grid.
    """
    from . import config as C
    pose=np.asarray(pose);rot=quat_xyzw_to_matrix(pose[3:]);normal=rot[:,2]
    if normal[2]<.5:return None
    local=np.array([[-.012,-.033,0.],[.012,-.033,0.],[.012,.033,0.],[-.012,.033,0.]])
    polygon=(local@rot.T+pose[:3])[:,:2]
    triangles=skull_top_triangles().astype(np.float32).astype(np.float64)+C.HEAD
    lo=polygon.min(0);hi=polygon.max(0)
    candidates=triangles[np.all(triangles[:,:,:2].max(1)>=lo,axis=1)&np.all(triangles[:,:,:2].min(1)<=hi,axis=1)]
    best=None
    for tri in candidates:
        poly=_clip_to_triangle(polygon,tri[:,:2])
        if not poly:continue
        n=np.cross(tri[1]-tri[0],tri[2]-tri[0])
        if abs(n[2])<1e-14:continue
        for xy in poly:
            head=tri[0,2]-np.dot(n[:2],xy-tri[0,:2])/n[2]
            band=pose[2]-np.dot(normal[:2],xy-pose[:2])/normal[2]
            gap=float(band-head)
            if best is None or gap<best:best=gap
    return best
