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
from yam_arena.constants import ARM_JOINTS, FINGER_JOINTS
from yam_arena.runtime import tcp_world
from yam_arena.inspect_embodiment import (
    ARMS,
    JOINT_GUIDE,
    YamArenaEmbodiment,
)

from . import builder, config, scoring

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
        eye = np.array([-.57, -.80, 1.79])
        target = np.array([.80, .07, .84])
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
                "The top camera faces the near net post and regulation-size tabletop. "
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
        self._joint_callback = None
        self._tooth = 0
        self._height_target = 0.

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
            cfg.sim.render_interval = cfg.decimation
            for name in ARMS:
                initial = getattr(cfg.scene, name).init_state
                initial.joint_pos = {
                    **dict(zip(ARM_JOINTS, config.STOWED[name.split("_")[0]], strict=True)),
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

        self._adjuster = self._base.scene["adjuster"]
        self._wind = self._adjuster.joint_names.index("Wind")
        self._guide = self._adjuster.joint_names.index("Height")
        self._joint_targets = torch.zeros((1, self._adjuster.num_joints), dtype=torch.float32,
                                          device=self._base.device)
        self._joint_targets_wp = wp.from_torch(self._joint_targets)
        self._joint_forces = torch.zeros_like(self._joint_targets)
        self._joint_forces_wp = wp.from_torch(self._joint_forces)
        self._joint_callback = self._base.sim.physics_manager.register_callback(
            self._apply_joint_loads, IsaacEvents.PRE_PHYSICS_STEP, order=55,
            name=config.KEY + "_passive_loads", wrap_weak_ref=False)

    def _apply_joint_loads(self, _dt):
        """Passive tooth bearing, guide friction and cord restoring torque."""
        try:
            view = self._adjuster.root_view
            joints = array(view.get_dof_positions())[0]
            speeds = array(view.get_dof_velocities())[0]
            angle = joints[self._wind]
            height = joints[self._guide]
            # Load is transferred at a physical tooth pitch traversed by the
            # hand. The load-limited bearing cannot advance the reel.
            next_tooth = max(0, min(4, int((angle - .010) / config.TOOTH)))
            self._tooth = max(self._tooth, next_tooth)
            seat = self._tooth * config.TOOTH
            self._joint_targets[0, self._wind] = seat if angle < seat else angle
            # Implicit friction shoe opposes sliding motion. Update its
            # reference only after movement exceeds the bounded-load band.
            if abs(height - self._height_target) >= .0017:
                direction = np.sign(height - self._height_target)
                self._height_target = float(np.clip(height - direction * .0016,
                                                    -config.POST_TRAVEL, 0.))
            self._joint_targets[0, self._guide] = self._height_target
            self._joint_forces[0, self._wind] = (
                -config.tension(angle) * config.SPOOL_RADIUS - .008 * speeds[self._wind])
            view.set_dof_position_targets(self._joint_targets_wp, self._adjuster._ALL_INDICES)
            view.set_dof_actuation_forces(self._joint_forces_wp, self._adjuster._ALL_INDICES)
        except Exception as error:
            self._audit_error = error

    def _update_net(self):
        if self.render_images:
            joints = array(self._adjuster.root_view.get_dof_positions())[0]
            builder.update_fabric(self._base.sim.stage, "/World/envs/env_0/fabric",
                                  config.NEAR_HEIGHT + joints[self._guide],
                                  config.tension(joints[self._wind]))

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._audit_error = None
        self._tooth = 0
        self._height_target = 0.
        self._joint_targets.zero_()
        self._joint_forces.zero_()
        self._env.reset(seed=seed)
        self._instruction = scene.instruction
        for _ in range(self.spec["settle_actions"]):
            self._env.step(self._ready)
        if self.render_images:
            self._update_net()
            for _ in range(4):
                self._base.sim.render()
        self._t = 0.
        self.scorer.reset()
        self.metrics = self.scorer.evaluate(self.state())
        obs = self._observation()
        if self.render_images and any(np.std(x) < 5 for x in obs.images.values()):
            raise RuntimeError("Blank native camera reset")
        return obs

    def state(self):
        view = self._adjuster.root_view
        q = array(view.get_dof_positions())[0]
        dq = array(view.get_dof_velocities())[0]
        return {
            "time_s": self._t,
            "angle": float(q[self._wind]),
            "height": float(q[self._guide]),
            "angular_speed": float(dq[self._wind]),
            "guide_speed": float(dq[self._guide]),
            "tooth": self._tooth,
            "hands": {n: tcp_world(self._base.scene[n]).tolist() for n in ARMS},
        }

    def _task_step(self):
        if self._audit_error:
            raise RuntimeError("Native net load update failed") from self._audit_error
        self._update_net()
        self.metrics = self.scorer.evaluate(self.state())
        return StepResult(
            observation=self._observation(),
            terminated=self.metrics["success"],
            termination_reason="success" if self.metrics["success"] else None,
            info=self.metrics.copy(),
        )

    def diagnostics(self):
        state = self.state()
        view = self._adjuster.root_view
        state["fixture_links"] = dict(zip(self._adjuster.body_names,
                                          array(view.get_link_transforms())[0].tolist(), strict=True))
        return state

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
        if self._joint_callback:
            self._joint_callback.deregister()
            self._joint_callback = None
        if self._env is not None:
            self._env.close()
            self._env = self._base = None
        if self._temp:
            self._temp.cleanup()


def embodiment(**kwargs):
    return Environment(**kwargs)
