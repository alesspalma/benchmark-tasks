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
from .film_web import FilmWeb
from .scoring import Scorer, seated

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
        position = np.array([0.025, -0.070, 1.285])
        sight = np.array([0.389, 0.024, 0.848]) - position
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
        import warp as wp
        from isaaclab_physx.physics.physx_manager import IsaacEvents

        self._wp = wp
        self._camera = self._base.scene["camera"]
        self._cartridge = self._base.scene["film_cartridge"]
        if self.render_images:
            from isaaclab.sim.utils import get_current_stage

            self._film_web = FilmWeb(get_current_stage())
        self._cam_ids = {name: i for i, name in enumerate(self._camera.joint_names)}
        self._torques = torch.zeros(
            (1, self._camera.num_joints), device=self._base.device
        )
        self._payout_force = torch.zeros((1, 1), device=self._base.device)
        self._torque_view = wp.from_torch(self._torques)
        self._payout_view = wp.from_torch(self._payout_force)
        self._engaged = False
        self._closed = False
        self._start_lever = 0.0
        self._stroke = 0.0
        self._callback = self._base.sim.physics_manager.register_callback(
            self._transport_forces,
            IsaacEvents.PRE_PHYSICS_STEP,
            order=50,
            name=config.KEY + "_transport",
            wrap_weak_ref=False,
        )
        self._ids = {
            n: (
                self._base.scene[n].find_joints(ARM_JOINTS, preserve_order=True)[0],
                self._base.scene[n].find_joints(FINGER_JOINTS, preserve_order=True)[0],
            )
            for n in ARMS
        }

    def _transport_forces(self, _dt):
        # Rear-cover detent and one-way transport clutch. The cam draws the
        # film only after its leader has reached the sprocket and is held flat
        # by the closed back. Apply explicit finite efforts to native bearings.
        q = array(self._camera.root_view.get_dof_positions())[0]
        qd = array(self._camera.root_view.get_dof_velocities())[0]
        film_q = float(array(self._cartridge.root_view.get_dof_positions())[0, 0])
        film_v = float(array(self._cartridge.root_view.get_dof_velocities())[0, 0])
        hinge, lever, rewind = (
            self._cam_ids[n] for n in ("BackHinge", "AdvancePivot", "RewindPivot")
        )
        cover = -q[hinge]
        cover_rest = (
            np.deg2rad(config.DOOR_OPEN)
            if cover > np.deg2rad(config.DOOR_OPEN - 5)
            else -0.04
        )
        self._torques[0, hinge] = np.clip(
            -0.52 * (cover_rest - cover) - 0.037 * qd[hinge], -0.14, 0.14
        )
        self._torques[0, lever] = np.clip(
            -0.065 * q[lever] - 0.004 * qd[lever], -0.06, 0.015
        )
        if cover > 0.16:
            self._closed = False
            self._engaged = False
            self._stroke = 0.0
        elif cover < 0.11 and not self._closed:
            self._closed = True
            self._start_lever = q[lever]
            pose = array(self._cartridge.root_view.get_root_transforms())[0]
            self._engaged = (
                seated(pose) and config.PULL - 0.007 <= -film_q <= config.PULL + 0.006
            )
        if self._engaged:
            self._start_lever = min(self._start_lever, q[lever])
            stroke = np.clip((q[lever] - self._start_lever - 0.08) / 0.91, 0.0, 1.0)
            self._stroke = max(self._stroke, float(stroke))
            target = -(config.PULL + config.ADVANCE * self._stroke)
            self._payout_force[0, 0] = np.clip(
                95.0 * (target - film_q) - 0.8 * film_v, -1.0, 1.0
            )
        else:
            # The first registration tooth lightly holds a leader drawn into
            # the index range. It can still pull free until the back latches.
            if abs((-film_q) - config.PULL) < 0.013:
                registration = 36.0 * (-config.PULL - film_q)
                drag = 0.32
            else:
                registration = (
                    np.clip(-200.0 * film_q, -0.28, 0.28) if film_q > -0.004 else 0.0
                )
                drag = 0.23
            self._payout_force[0, 0] = np.clip(
                registration - drag * film_v, -0.34, 0.34
            )
        knob_goal = (
            np.clip((-film_q - config.PULL) * 100.0, 0.0, 1.85)
            if self._engaged
            else 0.0
        )
        self._torques[0, rewind] = np.clip(
            0.070 * (knob_goal - q[rewind]) - 0.002 * qd[rewind], -0.04, 0.04
        )
        self._camera.root_view.set_dof_actuation_forces(
            self._torque_view, self._camera._ALL_INDICES
        )
        self._cartridge.root_view.set_dof_actuation_forces(
            self._payout_view, self._cartridge._ALL_INDICES
        )

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._env.reset(seed=seed)
        self._engaged = False
        self._closed = False
        self._start_lever = 0.0
        self._stroke = 0.0
        camera_reset = self._torch.zeros_like(self._torques)
        camera_reset[0, self._cam_ids["BackHinge"]] = -np.deg2rad(config.DOOR_OPEN)
        self._camera.write_joint_state_to_sim(
            camera_reset, self._torch.zeros_like(camera_reset)
        )
        self._cartridge.write_joint_state_to_sim(
            self._torch.zeros_like(self._payout_force),
            self._torch.zeros_like(self._payout_force),
        )
        self.scorer.reset()
        self._instruction = scene.instruction
        for _ in range(self.spec["settle_actions"]):
            self._env.step(self._ready)
        self._t = 0.0
        state = self.state()
        if not all(np.isfinite(obj["pose"]).all() for obj in state["objects"].values()):
            raise RuntimeError(f"Non-finite native reset: {state}")
        self.metrics = self.scorer.evaluate(state, elapsed=0.0)
        if self.render_images:
            self._film_web.update(self.metrics["film_payout_m"])
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
        result["transport"] = {
            "engaged": self._engaged,
            "stroke_fraction": self._stroke,
        }
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
            self._film_web.update(self.metrics["film_payout_m"])
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
