"""Standard RGB/joint observations; native task contacts; scoring-only measurements."""

from __future__ import annotations

import atexit
import hashlib
import math
import os
import tempfile
from dataclasses import replace
from pathlib import Path

import numpy as np
from inspect_robots.embodiment import PRIVILEGED_SUCCESS
from inspect_robots.types import Observation, StepResult
from inspect_robots_yam.config import observation_space
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY
from yam_arena.inspect_embodiment import (
    ARMS,
    JOINT_GUIDE,
    ROOM_DOCS,
    YamArenaEmbodiment,
)

from . import builder, config, geometry, scoring, materials

ROOT = Path(__file__).resolve().parents[1]
_APP = None


def array(value):
    if hasattr(value, "torch"):
        value = value.torch
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy().astype(float)
    if hasattr(value, "numpy"):
        return value.numpy().astype(float)
    return np.asarray(value, dtype=float)


def source_fingerprint():
    h = hashlib.sha256()
    for p in sorted(ROOT.rglob("*")):
        if (
            p.is_file()
            and (
                p.suffix == ".py"
                or p.name == "pyproject.toml"
                or p.parent.name == "motions"
            )
            and not any(
                x in str(p)
                for x in ("__pycache__", ".egg-info", "/runs/", "/validation/")
            )
        ):
            h.update(
                p.relative_to(ROOT).as_posix().encode()
                + b"\0"
                + hashlib.sha256(p.read_bytes()).digest()
            )
    return h.hexdigest()


class Environment(YamArenaEmbodiment):
    def __init__(
        self,
        *,
        physics_device="cpu",
        render_images=True,
        object_jitter=0,
        control_hz=20,
        **kwargs,
    ):
        if float(object_jitter) != 0 or float(control_hz) != 20:
            raise ValueError("Fixed reset and 20 Hz required")
        super().__init__(scene="station", object_jitter=0, control_hz=20, **kwargs)
        self.render_images = bool(render_images)
        self.physics_device = physics_device
        self.info = replace(
            self.info,
            name=config.KEY,
            supported_target_kinds=frozenset(),
            capabilities=self.info.capabilities | {PRIVILEGED_SUCCESS},
            docs="\n\n".join((JOINT_GUIDE, ROOM_DOCS["station"], config.DESCRIPTION)),
            environment_id=config.KEY + "-" + source_fingerprint()[:16],
            environment_revision=source_fingerprint()[:12],
        )
        if not self.render_images:
            self.info = replace(
                self.info,
                observation_space=observation_space(
                    self.cam_height, self.cam_width, ()
                ),
            )
        self.task = self
        self.metrics = {}
        self.scorer = scoring.Scorer()
        self._active = False
        self._temp = None
        self._audit_error = None
        self.material = materials.MaterialModel()
        self._material_accum = 0.
        self._material_callback = None
        self._mass_props = 0

    def _ensure_sim(self, goal=None):
        global _APP
        if self._env is not None:
            return
        if os.environ.get("OMNI_KIT_ACCEPT_EULA", "").upper() not in (
            "YES",
            "Y",
            "TRUE",
            "1",
        ):
            raise RuntimeError("Activate the accepted-license simulator environment")
        if _APP is None:
            from isaaclab.app import AppLauncher

            # Forward a documented SimulationApp option in this process only.
            AppLauncher._SIM_APP_CFG_TYPES = {
                **AppLauncher._SIM_APP_CFG_TYPES, 'enable_crashreporter': [bool]}
            _APP = AppLauncher(
                headless=self.headless,
                device=self.device,
                enable_cameras=self.render_images,
                multi_gpu=False,
                width=self.cam_width,
                height=self.cam_height,
                anti_aliasing=0,
                enable_crashreporter=False,
            ).app
            if Path('/proc/self/maps').exists() and 'libcarb.crashreporter' in Path('/proc/self/maps').read_text():
                _APP.close()
                raise RuntimeError('Crash reporter plugin unexpectedly loaded')
            atexit.register(_APP.close)
        import torch
        from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
        from isaaclab_arena.assets.object import Object, ObjectType
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from isaaclab_arena.environments.isaaclab_arena_environment import (
            IsaacLabArenaEnvironment,
        )
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.utils.pose import Pose
        from isaaclab_physx.physics.physx_manager import IsaacEvents
        from yam_arena.embodiment import BimanualYamEmbodiment
        from yam_arena.scene import env_cfg_callback, task_room

        self._temp = tempfile.TemporaryDirectory(prefix=config.KEY + "-")
        self.spec = builder.create_assets(self._temp.name)
        self._geometry = geometry.collision_points(self.spec)
        self._spec_by_name = {s["name"]: s for s in self.spec["assets"]}
        kind = {
            "rigid": ObjectType.RIGID,
            "articulation": ObjectType.ARTICULATION,
            "base": ObjectType.BASE,
        }
        objects = []
        for s in self.spec["assets"]:
            cls = {
                "rigid": RigidObjectCfg,
                "articulation": ArticulationCfg,
                "base": AssetBaseCfg,
            }[s["type"]]
            objects.append(
                Object(
                    name=s["name"],
                    object_type=kind[s["type"]],
                    usd_path=str(s["file"]),
                    initial_pose=Pose(
                        position_xyz=tuple(map(float, s.get("position", (0, 0, 0)))),
                        rotation_xyzw=tuple(
                            map(float, s.get("quaternion", (0, 0, 0, 1)))
                        ),
                    ),
                    asset_cfg_addon={"init_state": cls.InitialStateCfg()},
                )
            )
        all_objects = [*task_room("station", self.rig.top_cam), *objects]
        for o in all_objects:
            if isinstance(o, Object):
                cls = {
                    ObjectType.RIGID: RigidObjectCfg,
                    ObjectType.ARTICULATION: ArticulationCfg,
                }.get(o.object_type, AssetBaseCfg)
                o.asset_cfg_addon = {
                    **o.asset_cfg_addon,
                    "init_state": cls.InitialStateCfg(),
                }

        def configure(cfg):
            from isaaclab_physx.physics.physx_manager_cfg import PhysxCfg
            from yam_arena.constants import FINGER_OPEN

            cfg = env_cfg_callback(20)(cfg)
            cfg.sim.dt = 1. / config.PHYSICS_HZ
            cfg.decimation = config.PHYSICS_HZ // 20
            for name in ARMS:
                initial = getattr(cfg.scene, name).init_state
                initial.joint_pos = {
                    **dict(zip(ARM_JOINTS, READY)),
                    **dict.fromkeys(FINGER_JOINTS, FINGER_OPEN),
                }
                initial.joint_vel = {".*": 0.0}
            cfg.sim.use_newton_actuators = False
            if self.physics_device == "cpu":
                cfg.sim.use_fabric = False
            cfg.sim.physics = PhysxCfg(
                enable_ccd=True,
                gpu_max_rigid_contact_count=2**20,
                gpu_max_rigid_patch_count=2**18,
                gpu_found_lost_pairs_capacity=2**20,
                gpu_found_lost_aggregate_pairs_capacity=2**20,
                gpu_total_aggregate_pairs_capacity=2**20,
            )
            return cfg

        robot = BimanualYamEmbodiment(
            enable_cameras=self.render_images,
            rig=self.rig,
            contact_sensors=False,
            robot_contact_offset=0.001,
            cam_width=self.cam_width,
            cam_height=self.cam_height,
        )
        arena = IsaacLabArenaEnvironment(
            name=self.info.environment_id,
            scene=Scene(all_objects),
            embodiment=robot,
            env_cfg_callback=configure,
        )
        self._env = ArenaEnvBuilder(
            arena,
            ArenaEnvBuilderCfg(
                num_envs=1, solve_relations=False, device=self.physics_device
            ),
        ).make_registered()
        self._base = self._env.unwrapped
        self._torch = torch
        self._ready = torch.tensor(
            [[*READY, 1.0, *READY, 1.0]], dtype=torch.float32, device=self._base.device
        )
        self._ids = {
            n: (
                self._base.scene[n].find_joints(ARM_JOINTS, preserve_order=True)[0],
                self._base.scene[n].find_joints(FINGER_JOINTS, preserve_order=True)[0],
            )
            for n in ARMS
        }
        self._material_callback = self._base.sim.physics_manager.register_callback(
            self._material_step, IsaacEvents.POST_PHYSICS_STEP, order=50,
            name=config.KEY + "_material", wrap_weak_ref=False)
    def _material_step(self, dt):
        if not self._active:
            return
        self._material_accum += float(dt)
        if self._material_accum < .05 - 1.e-7:
            return
        try:
            from yam_arena.urdf import quat_xyzw_to_matrix
            view = self._base.scene["glass"].root_view
            tap = self._base.scene["tap"]
            valve = max(0., -float(array(tap.root_view.get_dof_positions())[0, 0]))
            pose = array(view.get_transforms())[0]
            rotation = quat_xyzw_to_matrix(pose[3:])
            hands = {}
            if valve > .055:
                for name in ARMS:
                    arm = self._base.scene[name]
                    hands[name] = {"names": list(arm.body_names),
                                   "poses": array(arm.root_view.get_link_transforms())[0]}
            self.material.advance(self._material_accum, pose[:3], rotation, valve, hands)
            self._material_accum = 0.
            self._update_load(view, rotation[2, :])
            if self.render_images:
                self._update_drink_visuals(rotation[2, :])
        except Exception as error:
            self._audit_error = error

    def _update_load(self, view, up):
        import warp as wp

        model = self.material
        volume = model.liquid_ml + model.foam_ml
        if volume == 0 and self._mass_props == 0:
            return
        mass, com, inertia = model.shape.properties(
            up, model.liquid_ml, model.foam_ml, model.liquid_level, model.total_level)
        ids = wp.array([0], dtype=wp.int32, device="cpu")
        view.set_masses(wp.array([[mass]], dtype=wp.float32, device="cpu"), indices=ids)
        view.set_coms(wp.array([[*com, 0, 0, 0, 1]], dtype=wp.float32, device="cpu"), indices=ids)
        view.set_inertias(wp.array([inertia.ravel()], dtype=wp.float32, device="cpu"), indices=ids)
        self._mass_props = volume

    def _update_drink_visuals(self, up):
        from pxr import Gf, UsdGeom
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        model = self.material
        base = "/World/envs/env_0/"
        liquid = model.shape.mesh(up, model.liquid_level) if model.liquid_ml > .05 else ([], [])
        foam = model.shape.mesh(up, model.total_level, model.liquid_level) if model.foam_ml > .05 else ([], [])
        points, faces = [], []
        if model.last_stream is not None and abs(model.last_stream[2] - config.SPOUT[2]) > .001:
            radius = .0035 * min(1., (model.last_flow_ml_s / config.FULL_FLOW_ML_S)**.5)
            for endpoint in (config.SPOUT, model.last_stream):
                points.extend(endpoint + [radius * math.cos(a), radius * math.sin(a), 0]
                              for a in np.arange(10) * math.tau / 10)
            for i in range(10):
                j = (i + 1) % 10
                faces.extend(((i, j, j + 10), (i, j + 10, i + 10)))
        for path, (points, faces) in (
            ("glass/Body/Liquid", liquid), ("glass/Body/Foam", foam),
            ("fixtures/Stream", (points, faces)),
        ):
            mesh = UsdGeom.Mesh.Get(stage, base + path)
            mesh.GetPointsAttr().Set([Gf.Vec3f(*v) for v in points])
            mesh.GetFaceVertexCountsAttr().Set([3] * len(faces))
            mesh.GetFaceVertexIndicesAttr().Set([i for face in faces for i in face])

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._active = False
        self._audit_error = None
        self._env.reset(seed=seed)
        self.material.reset()
        self._material_accum = 0.
        self._mass_props = -1
        self._update_load(self._base.scene["glass"].root_view, np.array([0., 0., 1.]))
        if self.render_images:
            self._update_drink_visuals(np.array([0., 0., 1.]))
        self._instruction = scene.instruction
        for _ in range(self.spec.get("settle_actions", 30)):
            self._env.step(self._ready)
        if self.render_images:
            for _ in range(4):
                self._base.sim.render()
        self._t = 0.0
        self.scorer.reset()
        if self._audit_error:
            raise RuntimeError("Material integration failed during reset") from self._audit_error
        self.metrics = self.scorer.evaluate(self.state())
        self._active = True
        obs = self._observation()
        if self.render_images and any(np.std(x) < 5 for x in obs.images.values()):
            raise RuntimeError("Blank native camera reset")
        return obs

    def state(self, light=False, include_bounds=True):
        result = {"time_s": self._t, "objects": {}}
        for s in self.spec["assets"]:
            if s["type"] == "base":
                continue
            o = self._base.scene[s["name"]]
            if s["type"] == "articulation":
                d = dict(
                    pose=array(o.root_view.get_root_transforms())[0].tolist(),
                    velocity=array(o.root_view.get_root_velocities())[0].tolist(),
                    joints={
                        n: float(q)
                        for n, q in zip(o.joint_names, array(o.root_view.get_dof_positions())[0])
                    },
                    links={
                        n: p.tolist()
                        for n, p in zip(o.body_names, array(o.root_view.get_link_transforms())[0])
                    },
                )
            else:
                d = {
                    "pose": array(o.root_view.get_transforms())[0].tolist(),
                    "velocity": array(o.root_view.get_velocities())[0].tolist(),
                }
            result["objects"][s["name"]] = d
        if include_bounds:
            result["bounds"] = geometry.world_bounds(self._geometry, result)
        result['material'] = self.material.snapshot()
        if not light:
            from yam_arena.runtime import tcp_world

            result["hands"] = {}
            for n in ARMS:
                a = self._base.scene[n]
                q = array(a.data.joint_pos)[0]
                _ids, fingers = self._ids[n]
                result["hands"][n] = {
                    "tcp": tcp_world(a).tolist(),
                    "opening_m": float(-q[fingers].sum()),
                    "poses": array(a.data.body_link_pose_w)[0].tolist(),
                    "names": list(a.body_names),
                }
            result["robot_clearance_m"] = geometry.robot_clearance(
                {"glass:Body": result["bounds"]["glass:Body"]}, result["hands"]
            )
        return result

    def _task_step(self):
        if self._audit_error:
            raise RuntimeError("Physics audit failed") from self._audit_error
        self.metrics = self.scorer.evaluate(self.state())
        return StepResult(
            observation=self._observation(),
            terminated=self.metrics["success"],
            termination_reason="success" if self.metrics["success"] else None,
            info=self.metrics.copy(),
        )

    def diagnostics(self):
        return self.state()

    def _observation(self):
        if self.render_images:
            return super()._observation()
        from yam_arena.constants import FINGER_OPEN

        values = []
        for n in ARMS:
            q = array(self._base.scene[n].data.joint_pos)[0]
            ids, fingers = self._ids[n]
            values.extend(
                [*q[ids], float(np.clip(q[fingers].mean() / FINGER_OPEN, 0, 1))]
            )
        return Observation(
            images={},
            state={"joint_pos": np.asarray(values)},
            instruction=self._instruction,
            state_time=self._t,
        )

    def close(self):
        self._active = False
        if self._material_callback:
            self._material_callback.deregister()
            self._material_callback = None
        if self._env is not None:
            self._env.close()
            self._env = self._base = None
        if self._temp:
            self._temp.cleanup()


def embodiment(**kwargs):
    return Environment(**kwargs)
