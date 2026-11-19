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

from . import builder, config, materials, scoring

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
        # Keep the press lever and entire sheet visible from the work side.
        position = np.array([0.80, -0.14, 1.37])
        sight = np.array([0.385, 0.017, 0.844]) - position
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
        self.scorer = scoring.Scorer()
        self._active = False
        self._temp = None
        self._audit_error = None
        self.material = materials.Dough()
        self._cookie_meshes = {}
        self._material_callback = None

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
        room = task_room("station", self.rig.top_cam)
        all_objects = [
            *(o for o in room if "stand" not in o.name and "housing" not in o.name),
            *objects,
        ]
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
        import warp as wp

        self._press = self._base.scene["press"]
        self._press_view = self._press.root_view
        self._lever = self._press.joint_names.index("Actuate")
        self._effort = torch.zeros(
            (1, self._press.num_joints), device=self._base.device
        )
        self._effort_wp = wp.from_torch(self._effort)
        self._material_callback = self._base.sim.physics_manager.register_callback(
            self._material_step,
            IsaacEvents.PRE_PHYSICS_STEP,
            order=50,
            name=config.KEY + "_dough",
            wrap_weak_ref=False,
        )

    def _material_step(self, dt):
        if not self._active:
            return
        try:
            pose = array(self._press_view.get_root_transforms())[0]
            velocity = array(self._press_view.get_root_velocities())[0]
            angle = float(array(self._press_view.get_dof_positions())[0, self._lever])
            omega = float(array(self._press_view.get_dof_velocities())[0, self._lever])
            # Return spring, viscous loss and dough back-pressure in the real
            # actuation direction. The ratchet supplies only forward dosage.
            filled = self.material.remaining > 0.005
            travel = max(angle, 0.0)
            spring = 0.12 * min(travel / 0.10, 1.0) + 0.38 * travel
            load = 0.34 * filled * float(np.clip(omega * 12.0, 0.0, 1.0))
            self._effort[0, self._lever] = (
                -spring - load - 0.09 * float(np.clip(omega, -3.0, 3.0))
            )
            self._press_view.set_dof_actuation_forces(
                self._effort_wp, self._press._ALL_INDICES
            )
            if not self._settling:
                self.material.advance(pose, angle, np.linalg.norm(velocity[:3]))
        except Exception as error:  # noqa: BLE001 - propagate failures from the physics callback
            self._audit_error = error

    def _update_dough_meshes(self):
        from pxr import Gf, UsdGeom

        prefix = "/World/envs/env_0/tray/ExtrudedDough"
        sectors = 48
        for index, item in enumerate(self.material.deposits):
            if item.grams < 0.04:
                continue
            scale = float(np.clip(np.sqrt(item.grams / config.DOSE_G), 0.11, 1.20))
            height = config.COOKIE_HEIGHT * scale * (0.44 if item.distorted else 1.0)
            radius = (
                config.COOKIE_DIAMETER / 2 * scale * (1.18 if item.distorted else 1.0)
            )
            x, y = item.xy - config.TRAY[:2]
            points = []
            rings = ((1.0, 0.20), (0.99, 0.52), (0.80, 0.90), (0.55, 0.97), (0.22, 1.0))
            for r, z in rings:
                for k in range(sectors):
                    phi = (2.0 * np.pi * k) / sectors
                    flower = 1.0 - 0.22 * (1.0 - np.cos(6 * phi))
                    phi += item.yaw
                    points.append(
                        Gf.Vec3f(
                            float(x + r * radius * flower * np.cos(phi)),
                            float(y + r * radius * flower * np.sin(phi)),
                            float(config.TRAY_TOP - config.TRAY[2] + z * height),
                        )
                    )
            faces = [
                (
                    i * sectors + k,
                    i * sectors + (k + 1) % sectors,
                    (i + 1) * sectors + (k + 1) % sectors,
                    (i + 1) * sectors + k,
                )
                for i in range(len(rings) - 1)
                for k in range(sectors)
            ]
            faces.append(tuple(range((len(rings) - 1) * sectors, len(rings) * sectors)))
            if index not in self._cookie_meshes:
                mesh = UsdGeom.Mesh.Define(
                    self._base.sim.stage, f"{prefix}/Dose{index}"
                )
                mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
                mesh.CreateFaceVertexIndicesAttr([k for face in faces for k in face])
                mesh.CreateDisplayColorAttr([Gf.Vec3f(0.83, 0.51, 0.21)])
                mesh.CreateSubdivisionSchemeAttr("none")
                self._cookie_meshes[index] = mesh
            mesh = self._cookie_meshes[index]
            mesh.CreatePointsAttr(points).Set(points)

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._active = False
        self._audit_error = None
        self._env.reset(seed=seed)
        self.material = materials.Dough()
        for mesh in self._cookie_meshes.values():
            self._base.sim.stage.RemovePrim(mesh.GetPath())
        self._cookie_meshes.clear()
        self._settling = True
        self._active = True
        self._instruction = scene.instruction
        for _ in range(self.spec.get("settle_actions", 24)):
            self._env.step(self._ready)
        self._settling = False
        if self.render_images:
            for _ in range(4):
                self._base.sim.render()
        self._t = 0.0
        self.scorer.reset()
        if self._audit_error:
            raise RuntimeError(
                "Material integration failed during reset"
            ) from self._audit_error
        self.metrics = self.scorer.evaluate(self.state())
        self._active = True
        obs = self._observation()
        if self.render_images and any(np.std(x) < 5 for x in obs.images.values()):
            raise RuntimeError("Blank native camera reset")
        return obs

    def state(self):
        result = {"time_s": self._t, "objects": {}}
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
        result["material"] = self.material.snapshot()
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
            raise RuntimeError("Physics audit failed") from self._audit_error
        self.metrics = self.scorer.evaluate(self.state())
        if self.render_images:
            self._update_dough_meshes()
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
