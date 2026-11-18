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
from yam_arena import ik
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.runtime import tcp_world
from yam_arena.inspect_embodiment import (
    ARMS,
    JOINT_GUIDE,
    YamArenaEmbodiment,
)

from . import builder, config, scoring, worker

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
        # Oblique overview on an independent camera stand.
        eye = np.array([-.61, .45, 1.94])
        target = np.array([.391, -.26, .78])
        optical_z = (eye-target) / np.linalg.norm(eye-target)
        optical_x = np.cross([0.,0.,1.], optical_z)
        optical_x /= np.linalg.norm(optical_x)
        optical_y = np.cross(optical_z, optical_x)
        orient = np.column_stack((optical_x, optical_y, optical_z))
        camera = replace(self.rig.top_cam, pos=tuple(eye),
                         rot_xyzw=tuple(ik.matrix_to_quat_xyzw(orient)))
        self.rig = replace(self.rig, top_cam=camera, bases=config.BASES)
        self.render_images = bool(render_images)
        self.physics_device = physics_device
        self.info = replace(
            self.info,
            name=config.KEY,
            supported_target_kinds=frozenset(),
            capabilities=self.info.capabilities | {PRIVILEGED_SUCCESS},
            docs="\n\n".join((
                JOINT_GUIDE,
                "Two YAM arms are mounted on independent pedestals 610 mm apart. "
                "The top camera sees the floor-standing cabinet at an angle. "
                "The left and right wrist views look outward along the fingers.",
                config.DESCRIPTION,
            )),
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
        self._temp = None
        self._audit_error = None
        self._counterbalance_callback = None
        self._worker_contact = False
        self._worker_elapsed = 0.
        self._worker_start = 0.
        self._worker_progress = 0.
        self._scene_running = False

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
        from isaaclab_physx.physics.physx_manager import IsaacEvents, PhysxManager
        from yam_arena.embodiment import BimanualYamEmbodiment
        from yam_arena.scene import env_cfg_callback, task_room

        self._temp = tempfile.TemporaryDirectory(prefix=config.KEY + "-")
        self.spec = builder.create_assets(self._temp.name)
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
        fixtures = task_room("station", self.rig.top_cam)
        ordinary_room = [prop for prop in fixtures if prop.name in ("floor", "dome_light", "lamp")]
        all_objects = [*ordinary_room, *objects]
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
                    **dict(zip(ARM_JOINTS, config.STOWED[name.split("_")[0]], strict=True)),
                    **dict.fromkeys(FINGER_JOINTS, FINGER_OPEN),
                }
                initial.joint_vel = {".*": 0.0}
            cfg.scene.freezer.init_state.joint_pos = {"Hinge": config.ANGLE_OPEN}
            cfg.scene.freezer.init_state.joint_vel = {".*": 0.0}
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
            [[*config.STOWED["left"], 1.0, *config.STOWED["right"], 1.0]], dtype=torch.float32, device=self._base.device
        )
        self._ids = {
            n: (
                self._base.scene[n].find_joints(ARM_JOINTS, preserve_order=True)[0],
                self._base.scene[n].find_joints(FINGER_JOINTS, preserve_order=True)[0],
            )
            for n in ARMS
        }
        import warp as wp
        self._freezer = self._base.scene["freezer"]
        self._effort = torch.zeros((1, 1), dtype=torch.float32, device=self._base.device)
        self._effort_wp = wp.from_torch(self._effort)
        self._counterbalance_callback = self._base.sim.physics_manager.register_callback(
            self._update_counterbalance, IsaacEvents.PRE_PHYSICS_STEP, order=50,
            name=config.KEY + "_counterbalance", wrap_weak_ref=False)

        sim_view = PhysxManager.get_physics_sim_view()
        self._stock_view = sim_view.create_rigid_body_view("/World/envs/env_0/stocker/Motion*")
        names = [path.rsplit("/", 1)[-1] for path in self._stock_view.prim_paths]
        if set(names) != set(worker.BODY_NAMES):
            raise RuntimeError(f"Expected the moving stocker bodies, got {names}")
        self._stock_order = [worker.BODY_NAMES.index(name) for name in names]
        self._stock_targets = torch.empty((len(names), 7), dtype=torch.float32,
                                          device=self._base.device)
        self._stock_wp = wp.from_torch(self._stock_targets)
        self._stock_ids = wp.array(np.arange(len(names), dtype=np.uint32),
                                   dtype=wp.uint32, device=self._base.device)
        bounds = self.spec["worker_bounds"]
        self._bound_bodies = [names.index(row[0]) for row in bounds]
        self._capsule_ends = np.asarray([row[1:3] for row in bounds])
        self._capsule_radii = np.asarray([row[3] for row in bounds])

        import carb.settings
        from pxr import PhysxSchema, Usd, UsdPhysics

        stage = self._base.sim.stage
        sensors = []
        for root in ("freezer/Lid", "LeftArm", "RightArm"):
            prim = stage.GetPrimAtPath(f"/World/envs/env_0/{root}")
            if not prim:
                raise RuntimeError(f"Cannot observe native contact on {root}")
            for body in Usd.PrimRange(prim):
                if body.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxContactReportAPI.Apply(body).CreateThresholdAttr(0.)
                    sensors.append(str(body.GetPath()))
        carb.settings.get_settings().set_bool("/physics/disableContactProcessing", False)
        self._worker_contacts = sim_view.create_rigid_contact_view(
            sensors, [list(self._stock_view.prim_paths)] * len(sensors)
        )
        if self._worker_contacts.sensor_count != len(sensors) or self._worker_contacts.filter_count != len(names):
            raise RuntimeError("Could not attach native worker contact filters")

    def _set_worker_targets(self, t):
        poses = worker.targets(t)[self._stock_order]
        self._stock_targets.copy_(self._torch.from_numpy(poses))
        self._stock_view.set_kinematic_targets(self._stock_wp, self._stock_ids)

    def _worker_clearance(self):
        poses = array(self._stock_view.get_transforms())[self._bound_bodies]
        # All actor joints rotate about X. Evaluate the native capsule extent
        # along Y, where the worker exits the full width of the lid's sweep.
        qx, qw = poses[:, 3:4], poses[:, 6:7]
        ys = (poses[:, 1:2] + (1. - 2. * qx**2) * self._capsule_ends[:, :, 1]
              - 2. * qw * qx * self._capsule_ends[:, :, 2])
        return -config.WIDTH / 2. - float(np.max(ys.max(axis=1) + self._capsule_radii))

    def _sample_worker_contacts(self):
        if not self._worker_contact:
            force = array(self._worker_contacts.get_contact_force_matrix(1. / config.PHYSICS_HZ))
            self._worker_contact = bool(np.any(np.linalg.norm(force, axis=-1) > .001))

    def _update_counterbalance(self, _dt):
        # A passive cam and spring carry the lid in its working range.
        # The counterbalance weakens below 24 degrees so the lid seats by gravity.
        try:
            view = self._freezer.root_view
            theta = float(array(view.get_dof_positions())[0, 0])
            speed = float(array(view.get_dof_velocities())[0, 0])
            if self._scene_running:
                self._sample_worker_contacts()
                occupied = self._worker_clearance() < config.WORKER_CLEARANCE
                self.scorer.observe_safety(theta, occupied, self._worker_contact)
                self._worker_elapsed += _dt
                progress = min(max(self._worker_elapsed - self._worker_start, 0.), worker.DURATION)
                if progress != self._worker_progress:
                    self._set_worker_targets(progress)
                    self._worker_progress = progress
            lever = config.DEPTH * .5 * math.cos(theta) - config.THICKNESS * .5 * math.sin(theta)
            fraction = float(np.clip(theta / math.radians(24.), 0., 1.))
            torque = config.LID_MASS * 9.81 * lever * fraction
            self._effort[0, 0] = torque - 1.8 * speed
            view.set_dof_actuation_forces(self._effort_wp, self._freezer._ALL_INDICES)
        except Exception as error:
            self._audit_error = error

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._audit_error = None
        self._scene_running = False
        self._env.reset(seed=seed)
        self._worker_elapsed = self._worker_progress = 0.
        self._worker_start = float(np.random.default_rng(seed).uniform(*config.DEPARTURE_RANGE))
        self._set_worker_targets(0.)
        self._stock_view.set_transforms(self._stock_wp, self._stock_ids)
        start = self._torch.full((1, 1), config.ANGLE_OPEN, device=self._base.device)
        self._freezer.write_joint_state_to_sim_index(position=start, velocity=self._torch.zeros_like(start))
        self._instruction = scene.instruction
        for _ in range(15):
            self._env.step(self._ready)
        if self.render_images:
            for _ in range(4):
                self._base.sim.render()
        self._t = 0.0
        self._worker_contact = False
        state = self.state()
        if self._audit_error:
            raise RuntimeError("Freezer physics integration failed") from self._audit_error
        if abs(state["lid_rad"] - config.ANGLE_OPEN) > math.radians(1.):
            forces = array(self._worker_contacts.get_contact_force_matrix(1. / config.PHYSICS_HZ))
            raise RuntimeError(f"Unexpected initial lid angle {math.degrees(state['lid_rad']):.3f}, "
                               f"worker clearance {state['worker_clearance_m']:.3f} m, "
                               f"peak contact {np.linalg.norm(forces, axis=-1).max():.3f} N")
        self.scorer.reset()
        self.metrics = self.scorer.evaluate(state)
        self._scene_running = True
        obs = self._observation()
        if self.render_images and any(np.std(x) < 5 for x in obs.images.values()):
            raise RuntimeError("Blank native camera reset")
        return obs

    def state(self):
        view = self._freezer.root_view
        clearance = self._worker_clearance()
        return {
            "time_s": self._t,
            "lid_rad": float(array(view.get_dof_positions())[0, 0]),
            "lid_speed_rad_s": float(array(view.get_dof_velocities())[0, 0]),
            "occupied": clearance < config.WORKER_CLEARANCE,
            "worker_clearance_m": clearance,
            "worker_departure_s": self._worker_start,
            "worker_progress_s": self._worker_progress,
            "worker_contact": self._worker_contact,
            "hands": {n: tcp_world(self._base.scene[n]).tolist() for n in ARMS},
        }

    def _task_step(self):
        if self._audit_error:
            raise RuntimeError("Freezer physics integration failed") from self._audit_error
        self._sample_worker_contacts()
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
        if self._counterbalance_callback:
            self._counterbalance_callback.deregister()
            self._counterbalance_callback = None
        if self._env is not None:
            self._env.close()
            self._env = self._base = None
        if self._temp:
            self._temp.cleanup()


def embodiment(**kwargs):
    return Environment(**kwargs)
