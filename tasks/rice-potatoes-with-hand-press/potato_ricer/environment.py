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

from . import builder, config, render
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
        position = np.array([-0.20, -0.017, 1.275])
        sight = np.array([0.398, 0.026, 0.895]) - position
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
        self._sampling = False
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

        self._ricer = self._base.scene["ricer"]
        self._potato = self._base.scene["potato"]
        self._hinge_dof = self._ricer.joint_names.index("HandleHinge")
        self._ram_dof = self._ricer.joint_names.index("RamSwivel")
        self._ram_id = self._ricer.body_names.index("Ram")
        self._yield_dof = self._potato.joint_names.index("YieldTravel")
        self._ricer_targets = torch.zeros(
            (1, self._ricer.num_joints), device=self._base.device
        )
        self._potato_targets = torch.zeros(
            (1, self._potato.num_joints), device=self._base.device
        )
        self._ricer_warp = wp.from_torch(self._ricer_targets)
        self._potato_warp = wp.from_torch(self._potato_targets)
        self._drive_callback = self._base.sim.physics_manager.register_callback(
            self._material_step,
            IsaacEvents.PRE_PHYSICS_STEP,
            order=50,
            name=config.KEY + "_yield",
            wrap_weak_ref=False,
        )
        if self.render_images:
            self._prepare_scene_indicators()

    def _material_step(self, dt):
        q = array(self._ricer.root_view.get_dof_positions())[0]
        hinge_angle = q[self._hinge_dof]
        # The light open detent supplies holding torque near the open stop.
        # A small counterbalance keeps gravity from pressing food on its own.
        self._ricer_targets[0, self._hinge_dof] = (
            np.radians(81.0)
            if not self._sampling or hinge_angle > np.radians(65.0)
            else hinge_angle + 0.070
        )
        # The loose ram swivel levels the disc as the upper lever pivots.
        slide_offset = config.HINGE[0] + config.PLATE_PIN[0] * np.cos(hinge_angle)
        vertical_correction = np.arcsin(
            np.clip(slide_offset / config.PLATE_DROP, 0, 0.85)
        )
        clear_pocket = np.clip((hinge_angle - 0.78) / 0.42, 0.0, 1.0)
        ram_angle = vertical_correction * (1.0 - clear_pocket) - 0.61 * clear_pocket
        self._ricer_targets[0, self._ram_dof] = ram_angle - hinge_angle
        self._ricer.root_view.set_dof_position_targets(
            self._ricer_warp, self._ricer._ALL_INDICES
        )
        if self._sampling:
            self.scorer.physics_sample(
                array(self._ricer.root_view.get_root_transforms())[0],
                array(self._ricer.root_view.get_link_transforms())[0, self._ram_id],
                array(self._potato.root_view.get_root_transforms())[0],
                array(self._potato.root_view.get_dof_positions())[0, self._yield_dof],
                dt,
            )
        self._potato_targets[0, self._yield_dof] = 0.003 - self.scorer.set_depth
        self._potato.root_view.set_dof_position_targets(
            self._potato_warp, self._potato._ALL_INDICES
        )

    def _prepare_scene_indicators(self):
        from isaaclab.sim import find_matching_prims, get_current_stage
        from pxr import UsdGeom

        stage = get_current_stage()
        potato = find_matching_prims(self._potato.cfg.prim_path, stage)[0].GetPath()
        self._skin = UsdGeom.Mesh.Get(stage, potato.AppendPath("LowerPotato/Skin"))
        self._deposits = render.create_deposits(stage)

    def _show_status(self, state):
        from pxr import Gf

        joints = state["objects"]["potato"]["joints"]
        height = max(0.008, config.POTATO_HEIGHT + joints["YieldTravel"])
        self._skin.GetPointsAttr().Set(
            [Gf.Vec3f(*point) for point in render.potato_skin(height)]
        )
        render.update_deposits(
            self._deposits, self.scorer, state["objects"]["ricer"]["pose"]
        )

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._sampling = False
        self._env.reset(seed=seed)
        self.scorer.reset()
        self._instruction = scene.instruction
        for _ in range(self.spec["settle_actions"]):
            self._env.step(self._ready)
        self._t = 0.0
        self._sampling = True
        state = self.state()
        self.metrics = self.scorer.evaluate(state, elapsed=0.0)
        if self.render_images:
            self._show_status(state)
            for _ in range(4):
                self._base.sim.render()
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
        state = self.state()
        self.metrics = self.scorer.evaluate(state, elapsed=config.CHECK_DT)
        if self.render_images:
            self._show_status(state)
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
            self._sampling = False
            self._env.close()
            self._env = self._base = None
        if self._temp:
            self._temp.cleanup()


def embodiment(**kwargs):
    return Environment(**kwargs)
