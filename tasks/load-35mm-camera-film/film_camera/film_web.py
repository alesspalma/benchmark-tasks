"""Camera geometry for the web between the cassette slot and the native leader."""

from . import config as C


def corners(payout):
    mouth = -0.010
    end = C.LEADER_DY - payout + 0.012
    z = C.FILM_Z - C.SEAT_Z
    return [(-0.0145, mouth, z), (-0.012, end, z), (0.012, end, z), (0.0145, mouth, z)]


def define(writer, stage, parent, color):
    mesh = writer.Geom.Mesh.Define(stage, parent + "/FilmWeb")
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreatePointsAttr(corners(0.0))
    mesh.CreateDisplayColorAttr([writer.Gf.Vec3f(*color)])


class FilmWeb:
    def __init__(self, stage):
        from pxr import UsdGeom

        path = "/World/envs/env_0/film_cartridge/Cassette/FilmWeb"
        self.points = UsdGeom.Mesh.Get(stage, path).GetPointsAttr()
        self.payout = None

    def update(self, payout):
        if self.payout is None or abs(payout - self.payout) > 0.0001:
            self.points.Set(corners(payout))
            self.payout = payout
