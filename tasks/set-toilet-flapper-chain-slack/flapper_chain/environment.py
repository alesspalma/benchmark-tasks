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

from . import builder, config
from .materials import TANK_AREA, Mechanism
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
        position = np.array([-0.12, -0.020, 1.535])
        sight = np.array([0.432, -0.018, 0.938]) - position
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
        self.mechanism = Mechanism()
        self._material_error = None
        self._temp = None

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
                    asset_cfg_addon={
                        "init_state": cls.InitialStateCfg(
                            **({"joint_pos": s["joints"]} if "joints" in s else {})
                        )
                    },
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
        import warp as wp
        from isaaclab_physx.physics.physx_manager import IsaacEvents

        self._tank = self._base.scene["tank"]
        self._clip = self._base.scene["clip"]
        self._tank_dof = [
            self._tank.joint_names.index(n) for n in ("TripPivot", "FlapperPivot")
        ]
        self._tank_links = [
            self._tank.body_names.index(n) for n in ("TripLever", "Flapper")
        ]
        self._effort = torch.zeros(
            (1, 2), dtype=torch.float32, device=self._base.device
        )
        self._effort_view = wp.from_torch(self._effort)
        self._clip_forces = torch.zeros(
            (2, 1, 3), dtype=torch.float32, device=self._base.device
        )
        self._force_views = [wp.from_torch(tensor) for tensor in self._clip_forces]
        self._drive_callback = self._base.sim.physics_manager.register_callback(
            self._material_step,
            IsaacEvents.PRE_PHYSICS_STEP,
            order=50,
            name=config.KEY + "_material",
            wrap_weak_ref=False,
        )
        if self.render_images:
            self._resolve_visuals()

    def _material_step(self, dt):
        if self._material_error:
            return
        try:
            tank = self._tank.root_view
            clip = self._clip.root_view
            poses = array(tank.get_link_transforms())[0]
            speeds = array(tank.get_link_velocities())[0]
            angles = array(tank.get_dof_positions())[0]
            rates = array(tank.get_dof_velocities())[0]
            trip, flapper = self._tank_links
            a, b = self._tank_dof
            impulses, force, torque = self.mechanism.advance(
                poses[trip],
                speeds[trip],
                poses[flapper],
                speeds[flapper],
                array(clip.get_transforms())[0],
                array(clip.get_velocities())[0],
                angles[a],
                rates[a],
                angles[b],
                rates[b],
            )
            self._effort[0, a], self._effort[0, b] = map(float, impulses)
            tank.set_dof_actuation_forces(self._effort_view, self._tank._ALL_INDICES)
            self._clip_forces[:] = self._torch.as_tensor(
                np.stack((force, torque))[:, None, :], device=self._base.device
            )
            clip.apply_forces_and_torques_at_position(
                force_data=self._force_views[0],
                torque_data=self._force_views[1],
                position_data=None,
                indices=self._clip._ALL_INDICES,
                is_global=True,
            )
        except Exception as error:  # noqa: BLE001 - Propagate callbacks to the rollout.
            self._material_error = error

    def _resolve_visuals(self):
        from isaaclab.sim import find_matching_prims
        from pxr import Gf

        self._Gf = Gf
        self._visual = {}
        path = "/World/envs/env_0/water"
        for name in ("WaterSurface", "CatchWater", "Outflow"):
            prim = find_matching_prims(f"{path}/{name}")[0]
            key = "xformOp:scale" if name == "Outflow" else "xformOp:translate"
            self._visual[name] = prim.GetAttribute(key)
        self._links = []
        for index in range(config.CHAIN_LINK_COUNT):
            prim = find_matching_prims(f"{path}/Chain/Link{index:02}")[0]
            self._links.append(
                (
                    prim.GetAttribute("xformOp:translate"),
                    prim.GetAttribute("xformOp:orient"),
                )
            )

    def _update_visuals(self):
        if not self.render_images:
            return
        model = self.mechanism
        cx = (config.WALL_FRONT + config.WALL_BACK) * 0.5
        self._visual["WaterSurface"].Set(
            self._Gf.Vec3d(cx, 0, config.FLOOR + model.litres / (1000 * TANK_AREA))
        )
        catch_area = config.CATCH_FOOTPRINT[0] * config.CATCH_FOOTPRINT[1]
        catch_z = config.TABLE + 0.011 + model.delivered / (1000 * catch_area)
        self._visual["CatchWater"].Set(self._Gf.Vec3d(cx, 0, catch_z))
        flow = float(np.clip(model.flow / 2.0, 0.005, 1.0))
        self._visual["Outflow"].Set(self._Gf.Vec3d(flow, flow, 1))
        t = (np.arange(config.CHAIN_LINK_COUNT) + 0.5) / config.CHAIN_LINK_COUNT
        slack = max(config.CHAIN_LENGTH - model.chain_span, 0.0)
        span_vector = model.end - model.anchor
        bow = np.sqrt(slack * max(model.chain_span, 0.01) / 2)
        points = model.anchor[None, :] + t[:, None] * span_vector[None, :]
        points[:, 1] -= np.sin(np.pi * t) * bow
        for i, (alpha, point, (position, rotation)) in enumerate(
            zip(t, points, self._links)
        ):
            tangent = span_vector - np.array(
                [0, np.pi * np.cos(np.pi * alpha) * bow, 0]
            )
            tangent /= max(np.linalg.norm(tangent), 1e-8)
            normal = [0.0, 1.0, 0.0] if abs(tangent[0]) > 0.94 else [1.0, 0.0, 0.0]
            side = np.cross(tangent, normal)
            side /= max(np.linalg.norm(side), 1e-8)
            up = np.cross(tangent, side)
            if i % 2:
                side, up = up, -side
            x, y, z, w = ik.matrix_to_quat_xyzw(np.column_stack((side, up, tangent)))
            position.Set(self._Gf.Vec3d(*map(float, point)))
            rotation.Set(
                self._Gf.Quatd(float(w), self._Gf.Vec3d(float(x), float(y), float(z)))
            )

    def _check_material(self):
        if self._material_error is not None:
            raise RuntimeError(
                "Native flapper or clip reaction failed"
            ) from self._material_error

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self.mechanism.reset()
        self._material_error = None
        self._env.reset(seed=seed)
        self.scorer.reset()
        self._instruction = scene.instruction
        for _ in range(self.spec["settle_actions"]):
            self._env.step(self._ready)
        self._t = 0.0
        self._check_material()
        state = self.state()
        self.metrics = self.scorer.evaluate(state, elapsed=0.0)
        if self.render_images:
            self._update_visuals()
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
            "material": self.mechanism.metrics(),
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
        self._check_material()
        state = self.state()
        self.metrics = self.scorer.evaluate(state, elapsed=config.CHECK_DT)
        if self.render_images:
            self._update_visuals()
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
