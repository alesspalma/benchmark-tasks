"""Camera geometry for the web between the cassette slot and the native leader."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C


Z = C.FILM_Z - C.SEAT_Z
MOUTH = [(-0.0145, -0.010, Z), (0.0145, -0.010, Z)]
ATTACHMENT = np.array([(-0.012, 0.012, 0), (0.012, 0.012, 0)])


def define(writer, stage, parent, color):
    mesh = writer.Geom.Mesh.Define(stage, parent + "/FilmWeb")
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    endpoints = ATTACHMENT + [0, C.LEADER_DY, Z]
    mesh.CreatePointsAttr([MOUTH[0], *endpoints.tolist(), MOUTH[1]])
    mesh.CreateDisplayColorAttr([writer.Gf.Vec3f(*color)])


class FilmWeb:
    def __init__(self, stage):
        from pxr import UsdGeom

        path = "/World/envs/env_0/film_cartridge/Cassette/FilmWeb"
        self.points = UsdGeom.Mesh.Get(stage, path).GetPointsAttr()
        self.endpoints = None

    def update(self, cartridge):
        cassette = np.asarray(cartridge["pose"])
        leader = np.asarray(cartridge["links"]["FilmLeader"])
        leader_axes = quat_xyzw_to_matrix(leader[3:])
        cassette_axes = quat_xyzw_to_matrix(cassette[3:])
        endpoints = (ATTACHMENT @ leader_axes.T + leader[:3] - cassette[:3]) @ cassette_axes
        if self.endpoints is None or np.max(np.abs(endpoints - self.endpoints)) > 0.0001:
            self.points.Set([MOUTH[0], *endpoints.tolist(), MOUTH[1]])
            self.endpoints = endpoints
