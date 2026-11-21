"""Standard RGB/joint observations; native task contacts; scoring-only measurements."""

from __future__ import annotations

import atexit
import hashlib
import os
import tempfile
from dataclasses import replace
from pathlib import Path

import numpy as np
from inspect_robots.embodiment import PRIVILEGED_SUCCESS
from inspect_robots.types import Observation, StepResult
from inspect_robots_yam.config import observation_space
from yam_arena import ik
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS, READY
from yam_arena.inspect_embodiment import (
    ARMS,
    JOINT_GUIDE,
    ROOM_DOCS,
    YamArenaEmbodiment,
)
from yam_arena.runtime import tcp_world

from . import builder, config, pedestrian
from .scoring import Scorer

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
        camera = self.rig.top_cam
        position = np.array([-0.840, -1.28, 2.10])
        sight = np.array([0.56, -0.015, 0.946]) - position
        sight /= np.linalg.norm(sight)
        screen_right = np.cross(sight, [0.0, 0.0, 1.0])
        screen_right /= np.linalg.norm(screen_right)
        camera = replace(
            camera,
            pos=tuple(position),
            rot_xyzw=tuple(
                ik.matrix_to_quat_xyzw(
                    np.column_stack(
                        (screen_right, np.cross(screen_right, sight), -sight)
                    )
                )
            ),
        )
        self.rig = replace(self.rig, top_cam=camera)
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
        self.scorer = Scorer()
        self._temp = None
        self._scene_running = False
        self._walker_time = 0.0
        self._begin_count = 0.0
        self._walker_started = False
        self._native_contact = False
        self._first_contact = None
        self._audit_error = None

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
                **AppLauncher._SIM_APP_CFG_TYPES,
                "enable_crashreporter": [bool],
            }
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
            if (
                Path("/proc/self/maps").exists()
                and "libcarb.crashreporter" in Path("/proc/self/maps").read_text()
            ):
                _APP.close()
                raise RuntimeError("Crash reporter plugin unexpectedly loaded")
            atexit.register(_APP.close)
        import torch
        from isaaclab import sim as sim_utils
        from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
        from isaaclab_arena.assets.object import Object, ObjectType
        from isaaclab_arena.environments.arena_env_builder import ArenaEnvBuilder
        from isaaclab_arena.environments.arena_env_builder_cfg import ArenaEnvBuilderCfg
        from isaaclab_arena.environments.isaaclab_arena_environment import (
            IsaacLabArenaEnvironment,
        )
        from isaaclab_arena.scene.scene import Scene
        from isaaclab_arena.utils.pose import Pose
        from yam_arena.embodiment import BimanualYamEmbodiment
        from yam_arena.scene import env_cfg_callback

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
        objects.extend(
            [
                Object(
                    name="room_fill",
                    object_type=ObjectType.BASE,
                    spawner_cfg=sim_utils.DomeLightCfg(intensity=580.0),
                ),
                Object(
                    name="bench_light",
                    object_type=ObjectType.BASE,
                    spawner_cfg=sim_utils.SphereLightCfg(intensity=670.0, radius=0.20),
                    initial_pose=Pose(position_xyz=(0.28, -0.32, 1.80)),
                ),
            ]
        )

        def configure(cfg):
            from isaaclab_physx.physics.physx_manager_cfg import PhysxCfg
            from yam_arena.constants import FINGER_OPEN

            cfg = env_cfg_callback(20)(cfg)
            cfg.sim.dt = 1.0 / config.PHYSICS_HZ
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
            scene=Scene(objects),
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
        self._attach_mechanics()

    def _attach_mechanics(self):
        import carb.settings
        import warp as wp
        from isaaclab_physx.physics.physx_manager import IsaacEvents, PhysxManager
        from pxr import PhysxSchema, Usd, UsdPhysics

        self._gate = self._base.scene["gate"]
        self._joint_ids = {name: i for i, name in enumerate(self._gate.joint_names)}
        self._effort = self._torch.zeros(
            (1, self._gate.num_joints), device=self._base.device
        )
        self._effort_view = wp.from_torch(self._effort)
        sim_view = PhysxManager.get_physics_sim_view()
        self._walker = sim_view.create_rigid_body_view(
            "/World/envs/env_0/walker/Motion*"
        )
        names = [path.rsplit("/", 1)[-1] for path in self._walker.prim_paths]
        if set(names) != set(pedestrian.BODY_NAMES):
            raise RuntimeError(f"Unexpected pedestrian rigid bodies {names}")
        self._walker_order = [pedestrian.BODY_NAMES.index(n) for n in names]
        self._torso_index = names.index("MotionTorso")
        self._poses = self._torch.empty(
            (len(names), 7), dtype=self._torch.float32, device=self._base.device
        )
        self._pose_view = wp.from_torch(self._poses)
        self._walker_ids = wp.array(
            np.arange(len(names), dtype=np.uint32),
            dtype=wp.uint32,
            device=self._base.device,
        )

        stage = self._base.sim.stage
        sensors = []
        for root in ("gate/Leaf", "LeftArm", "RightArm"):
            prim = stage.GetPrimAtPath(f"/World/envs/env_0/{root}")
            if not prim:
                raise RuntimeError(f"Cannot observe native contact on {root}")
            for body in Usd.PrimRange(prim):
                if body.HasAPI(UsdPhysics.RigidBodyAPI):
                    PhysxSchema.PhysxContactReportAPI.Apply(body).CreateThresholdAttr(
                        0.0
                    )
                    sensors.append(str(body.GetPath()))
        carb.settings.get_settings().set_bool(
            "/physics/disableContactProcessing", False
        )
        self._contacts = sim_view.create_rigid_contact_view(
            sensors, [list(self._walker.prim_paths)] * len(sensors)
        )
        self._sensor_names = sensors
        self._walker_names = names
        if self._contacts.sensor_count != len(
            sensors
        ) or self._contacts.filter_count != len(names):
            raise RuntimeError("Could not attach native person contact filters")
        self._callback = self._base.sim.physics_manager.register_callback(
            self._physics,
            IsaacEvents.PRE_PHYSICS_STEP,
            order=50,
            name=config.KEY + "_spring_and_walker",
            wrap_weak_ref=False,
        )

    def _set_walker(self, time_s):
        poses = pedestrian.targets(time_s)[self._walker_order]
        self._poses.copy_(self._torch.from_numpy(poses))
        self._walker.set_kinematic_targets(self._pose_view, self._walker_ids)

    def _walker_torso(self):
        return array(self._walker.get_transforms())[self._torso_index, :3]

    def _physics(self, dt):
        try:
            view = self._gate.root_view
            q = array(view.get_dof_positions())[0]
            v = array(view.get_dof_velocities())[0]
            hinge, bolt = self._joint_ids["Swing"], self._joint_ids["LiftBolt"]
            angle = max(0.0, q[hinge])
            # Two torsion hinges apply finite closing torque over the full arc.
            self._effort[0, hinge] = -1.42 - 1.20 * angle - 0.72 * v[hinge]
            self._effort[0, bolt] = -3.8 - 29.0 * max(q[bolt], 0.0) - 4.0 * v[bolt]
            view.set_dof_actuation_forces(self._effort_view, self._gate._ALL_INDICES)
            if self._scene_running:
                if not self._walker_started:
                    self._begin_count = (
                        self._begin_count + dt
                        if angle >= np.deg2rad(config.PASS_ANGLE)
                        else 0.0
                    )
                    self._walker_started = self._begin_count >= 0.23
                if self._walker_started:
                    if not self._native_contact:
                        force = array(
                            self._contacts.get_contact_force_matrix(
                                1.0 / config.PHYSICS_HZ
                            )
                        )
                        forces = np.linalg.norm(force, axis=-1)
                        self._native_contact = bool(np.any(forces > 0.04))
                        if self._native_contact:
                            i, j = np.unravel_index(np.argmax(forces), forces.shape)
                            self._first_contact = {
                                "struck_by": self._sensor_names[i].rsplit("/", 2)[-2:],
                                "person_part": self._walker_names[j],
                                "force_n": round(float(forces[i, j]), 3),
                            }
                    self.scorer.sample(
                        angle, self._walker_torso(), True, self._native_contact
                    )
                    if self._walker_time < config.WALK_DURATION:
                        self._walker_time = min(
                            config.WALK_DURATION, self._walker_time + dt
                        )
                        self._set_walker(self._walker_time)
        except (RuntimeError, ValueError, IndexError) as exc:
            self._audit_error = exc

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._scene_running = False
        self._audit_error = None
        self._env.reset(seed=seed)
        self._walker_time = 0.0
        self._begin_count = 0.0
        self._walker_started = False
        self._native_contact = False
        self._first_contact = None
        self._set_walker(0.0)
        self._walker.set_transforms(self._pose_view, self._walker_ids)
        self.scorer.reset()
        self._instruction = scene.instruction
        for _ in range(self.spec["settle_actions"]):
            self._env.step(self._ready)
        self._t = 0.0
        self._scene_running = True
        if self._audit_error:
            raise RuntimeError("Gate physics integration failed") from self._audit_error
        state = self.state()
        self.metrics = self.scorer.evaluate(state, elapsed=0.0)
        if self.render_images:
            for _ in range(4):
                self._base.sim.render()
        obs = self._observation()
        if self.render_images and any(np.std(x) < 5 for x in obs.images.values()):
            raise RuntimeError("Blank native camera reset")
        return obs

    def state(self):
        result = {
            "time_s": self._t,
            "objects": {},
            "pedestrian": {
                "torso": self._walker_torso().tolist(),
                "elapsed": self._walker_time,
                "walking": self._walker_started,
                "contact": self._native_contact,
                "first_contact": self._first_contact,
            },
        }
        for s in self.spec["assets"]:
            if s["type"] == "base":
                continue
            o = self._base.scene[s["name"]]
            if s["type"] == "articulation":
                d = {
                    "pose": array(o.root_view.get_root_transforms())[0].tolist(),
                    "velocity": array(o.root_view.get_root_velocities())[0].tolist(),
                    "joints": {
                        n: float(q)
                        for n, q in zip(
                            o.joint_names, array(o.root_view.get_dof_positions())[0]
                        )
                    },
                    "joint_speeds": {
                        n: float(v)
                        for n, v in zip(
                            o.joint_names, array(o.root_view.get_dof_velocities())[0]
                        )
                    },
                    "links": {
                        n: p.tolist()
                        for n, p in zip(
                            o.body_names, array(o.root_view.get_link_transforms())[0]
                        )
                    },
                }
            else:
                d = {
                    "pose": array(o.root_view.get_transforms())[0].tolist(),
                    "velocity": array(o.root_view.get_velocities())[0].tolist(),
                }
            result["objects"][s["name"]] = d
        result["hands"] = {}
        for name in ARMS:
            arm = self._base.scene[name]
            poses = array(arm.data.body_link_pose_w)[0]
            _, fingers = self._ids[name]
            result["hands"][name] = {
                "points": [
                    tcp_world(arm).tolist(),
                    tcp_world(arm, past_tcp=0.013).tolist(),
                    *[
                        pose[:3].tolist()
                        for part, pose in zip(arm.body_names, poses)
                        if part in ("gripper", "tip_left", "tip_right")
                    ],
                ],
                "opening_m": float(-array(arm.data.joint_pos)[0, fingers].sum()),
            }
        return result

    def _task_step(self):
        if self._audit_error:
            raise RuntimeError("Gate physics integration failed") from self._audit_error
        state = self.state()
        self.metrics = self.scorer.evaluate(state, elapsed=config.CHECK_DT)
        if self.render_images:
            self._base.sim.render()
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
        if self._env is not None:
            self._env.close()
            self._env = self._base = None
        if self._temp:
            self._temp.cleanup()


def embodiment(**kwargs):
    return Environment(**kwargs)
