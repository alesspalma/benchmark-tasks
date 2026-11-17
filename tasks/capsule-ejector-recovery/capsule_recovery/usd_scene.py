"""Original compound rigid geometry; all solids use native PhysX contacts."""

import json
import math
from pathlib import Path

import numpy as np


class Writer:
    def __init__(self, directory):
        from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

        self.Gf, self.Px, self.Usd, self.Geom, self.Phy, self.Shade = (
            Gf,
            PhysxSchema,
            Usd,
            UsdGeom,
            UsdPhysics,
            UsdShade,
        )
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.records = []

    def stage(self, name):
        s = self.Usd.Stage.CreateNew(str(self.directory / (name + ".usda")))
        self.Geom.SetStageUpAxis(s, "Z")
        self.Geom.SetStageMetersPerUnit(s, 1.0)
        root = self.Geom.Xform.Define(s, "/Asset")
        s.SetDefaultPrim(root.GetPrim())
        mat = self.Shade.Material.Define(s, "/Asset/Physics")
        p = self.Phy.MaterialAPI.Apply(mat.GetPrim())
        p.CreateStaticFrictionAttr(0.65)
        p.CreateDynamicFrictionAttr(0.5)
        p.CreateRestitutionAttr(0)
        return s

    def body(self, s, path, mass, position=(0, 0, 0), fixed=False, articulation=False):
        x = self.Geom.Xform.Define(s, path)
        x.AddTranslateOp().Set(self.Gf.Vec3d(*map(float, position)))
        x.AddOrientOp(precision=self.Geom.XformOp.PrecisionDouble).Set(
            self.Gf.Quatd(1, 0, 0, 0)
        )
        p = x.GetPrim()
        self.Phy.RigidBodyAPI.Apply(p)
        self.Phy.MassAPI.Apply(p).CreateMassAttr(mass)
        rigid = self.Px.PhysxRigidBodyAPI.Apply(p)
        rigid.CreateMaxDepenetrationVelocityAttr(0.5)
        rigid.CreateSolverPositionIterationCountAttr(32)
        rigid.CreateSolverVelocityIterationCountAttr(8)
        if articulation:
            self.Phy.ArticulationRootAPI.Apply(p)
            a = self.Px.PhysxArticulationAPI.Apply(p)
            a.CreateEnabledSelfCollisionsAttr(True)
            a.CreateSolverPositionIterationCountAttr(32)
            a.CreateSolverVelocityIterationCountAttr(8)
        if fixed:
            j = self.Phy.FixedJoint.Define(s, path + "/WorldFix")
            j.CreateBody1Rel().SetTargets([p.GetPath()])
            j.CreateLocalPos0Attr(self.Gf.Vec3f(*map(float, position)))
            j.CreateLocalPos1Attr(self.Gf.Vec3f(0, 0, 0))
        return path

    def _finish(self, s, geom, color, collide, contact=0.00015):
        geom.CreateDisplayColorAttr([self.Gf.Vec3f(*color)])
        if collide:
            p = geom.GetPrim()
            self.Phy.CollisionAPI.Apply(p)
            a = self.Px.PhysxCollisionAPI.Apply(p)
            a.CreateContactOffsetAttr(contact)
            a.CreateRestOffsetAttr(0)
            mat = self.Shade.Material.Get(s, "/Asset/Physics")
            self.Shade.MaterialBindingAPI.Apply(p).Bind(mat, materialPurpose="physics")
        return geom

    def box(self, s, path, size, position, color, rotation=None, collide=True):
        from pxr import Tf

        path = "/".join(Tf.MakeValidIdentifier(p) if p else "" for p in path.split("/"))
        x = self.Geom.Cube.Define(s, path)
        x.CreateSizeAttr(1)
        x.AddTranslateOp().Set(self.Gf.Vec3d(*map(float, position)))
        if rotation is not None:
            from yam_arena.ik import matrix_to_quat_xyzw

            q = matrix_to_quat_xyzw(np.asarray(rotation))
            x.AddOrientOp(precision=self.Geom.XformOp.PrecisionDouble).Set(
                self.Gf.Quatd(float(q[3]), self.Gf.Vec3d(*map(float, q[:3])))
            )
        x.AddScaleOp().Set(self.Gf.Vec3f(*map(float, size)))
        self._finish(s, x, color, collide)
        self.records.append(
            {
                "asset": Path(s.GetRootLayer().identifier).stem,
                "solid": path,
                "kind": "box",
                "size": list(size),
                "position": list(position),
                "color": list(color),
            }
        )

    def cylinder(
        self, s, path, radius, length, position, color, axis="Z", collide=True
    ):
        from pxr import Tf

        path = "/".join(Tf.MakeValidIdentifier(p) if p else "" for p in path.split("/"))
        x = self.Geom.Cylinder.Define(s, path)
        x.CreateRadiusAttr(radius)
        x.CreateHeightAttr(length)
        x.CreateAxisAttr(axis)
        x.AddTranslateOp().Set(self.Gf.Vec3d(*map(float, position)))
        self._finish(s, x, color, collide)
        return x

    def sphere(self, s, path, radius, position, color):
        x = self.Geom.Sphere.Define(s, path)
        x.CreateRadiusAttr(radius)
        x.AddTranslateOp().Set(self.Gf.Vec3d(*map(float, position)))
        self._finish(s, x, color, True, contact=0.00008)

    def ring(self, s, path, radius, thickness, length, z, color, n=32, omit=()):
        for k in range(n):
            if k in omit:
                continue
            a = (k + 0.5) * math.tau / n
            r = np.array(
                [
                    [math.cos(a), -math.sin(a), 0],
                    [math.sin(a), math.cos(a), 0],
                    [0, 0, 1],
                ]
            )
            self.box(
                s,
                f"{path}/S{k:02}",
                (thickness, 2 * radius * math.tan(math.pi / n) + 0.00005, length),
                (radius * math.cos(a), radius * math.sin(a), z),
                color,
                rotation=r,
            )

    def tray(self, s, path, inner, height, bottom, color, wall=0.003):
        x, y = inner
        self.box(
            s,
            path + "/Floor",
            (x + 2 * wall, y + 2 * wall, wall),
            (0, 0, bottom),
            color,
        )
        for d in (-1, 1):
            self.box(
                s,
                path + f"/X{d + 1}",
                (wall, y + 2 * wall, height),
                (d * (x + wall) / 2, 0, bottom + (height + wall) / 2),
                color,
            )
            self.box(
                s,
                path + f"/Y{d + 1}",
                (x, wall, height),
                (0, d * (y + wall) / 2, bottom + (height + wall) / 2),
                color,
            )

    def hinge(
        self,
        s,
        path,
        parent,
        child,
        axis,
        anchor0,
        anchor1,
        limits,
        friction=0.001,
        stiffness=0,
        damping=0.001,
        target=0,
        collide=False,
    ):
        j = self.Phy.RevoluteJoint.Define(s, path)
        j.CreateBody0Rel().SetTargets([parent])
        j.CreateBody1Rel().SetTargets([child])
        j.CreateAxisAttr(axis)
        j.CreateLocalPos0Attr(self.Gf.Vec3f(*map(float, anchor0)))
        j.CreateLocalPos1Attr(self.Gf.Vec3f(*map(float, anchor1)))
        j.CreateLowerLimitAttr(math.degrees(limits[0]))
        j.CreateUpperLimitAttr(math.degrees(limits[1]))
        j.CreateCollisionEnabledAttr(collide)
        self.Px.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(friction)
        drive = self.Phy.DriveAPI.Apply(j.GetPrim(), "angular")
        drive.CreateTypeAttr("force")
        drive.CreateStiffnessAttr(stiffness / math.degrees(1))
        drive.CreateDampingAttr(damping / math.degrees(1))
        drive.CreateTargetPositionAttr(math.degrees(target))
        return j

    def save(self, s):
        s.GetRootLayer().Save()
        return Path(s.GetRootLayer().identifier)

    def inventory(self):
        (self.directory / "geometry-inventory.json").write_text(
            json.dumps(self.records, indent=2) + "\n"
        )
