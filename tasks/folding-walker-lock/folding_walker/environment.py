"""Inspect Robots embodiment for the full-size bilateral walker mechanism."""

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
from yam_arena.urdf import quat_xyzw_to_matrix
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
        eye = np.array([-.39, -.11, 2.12])
        target = np.array([.51, 0., .635])
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
                "The top camera sees the walker and seated training mannequin. "
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
        self._material_callback = None
        self._locked = dict.fromkeys(config.SIDES, False)
        self._proof_force = 0.0

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
            cfg.scene.walker.init_state.joint_pos = {
                **{side + "Hinge": -sign * config.START_FOLD[side]
                   for side, sign in config.SIDES.items()},
                ".*Button": 0.,
            }
            cfg.scene.walker.init_state.joint_vel = {".*": 0.}
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

        self._walker = self._base.scene["walker"]
        self._hinges = {side: self._walker.joint_names.index(side + "Hinge") for side in config.SIDES}
        self._buttons = {side: self._walker.joint_names.index(side + "Button") for side in config.SIDES}
        self._legs = {side: self._walker.body_names.index(side.capitalize() + "Side") for side in config.SIDES}
        self._effort = torch.zeros((1, self._walker.num_joints), dtype=torch.float32, device=self._base.device)
        self._effort_wp = wp.from_torch(self._effort)
        self._material_callback = self._base.sim.physics_manager.register_callback(
            self._apply_latches, IsaacEvents.PRE_PHYSICS_STEP, order=55,
            name=config.KEY + "_latches", wrap_weak_ref=False
        )

    def _apply_latches(self, _dt):
        """Apply the spring-pin cam load and the unilateral lock reaction."""
        try:
            view = self._walker.root_view
            q = array(view.get_dof_positions())[0]
            qd = array(view.get_dof_velocities())[0]
            for side, sign in config.SIDES.items():
                hinge, button = self._hinges[side], self._buttons[side]
                fold = -sign * q[hinge]
                fold_speed = -sign * qd[hinge]
                # Outside the recess, the lock's cam holds the pin down.
                if fold > config.LATCH_SEAT + 0.007:
                    self._effort[0, button] = -5.8
                else:
                    self._effort[0, button] = 0.
                if q[button] < 0.0016:
                    self._locked[side] = False
                elif q[button] > 0.0044 and fold <= config.LATCH_SEAT:
                    self._locked[side] = True
                elif fold > np.deg2rad(7):
                    self._locked[side] = False
                torque = 0.
                if self._locked[side] and fold > config.LATCH_CLEARANCE:
                    torque = np.clip(
                        config.LATCH_STIFFNESS*(fold-config.LATCH_CLEARANCE)
                        + config.LATCH_DAMPING * fold_speed,
                        0., config.LATCH_MAX_TORQUE
                    )
                # The polymer hinge bushing resists the onset of free swing and
                # damps frame chatter without an angular position target.
                bushing = 1.05*np.tanh(fold_speed/.018) + .65*fold_speed
                proof_torque = self._proof_force * config.GRASP_DISTANCE
                self._effort[0, hinge] = sign * (torque + bushing - proof_torque)
            view.set_dof_actuation_forces(self._effort_wp, self._walker._ALL_INDICES)
        except Exception as error:
            self._audit_error = error

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._audit_error = None
        self._locked = dict.fromkeys(config.SIDES, False)
        self._proof_force = 0.0
        self._env.reset(seed=seed)
        initial = self._torch.zeros((1, self._walker.num_joints), device=self._base.device)
        for side, sign in config.SIDES.items():
            initial[0, self._hinges[side]] = -sign*config.START_FOLD[side]
        self._walker.write_joint_state_to_sim_index(
            position=initial, velocity=self._torch.zeros_like(initial)
        )
        self._instruction = scene.instruction
        for _ in range(self.spec["settle_actions"]):
            self._env.step(self._ready)
        if self.render_images:
            for _ in range(4):
                self._base.sim.render()
        self._t = 0.0
        self.scorer.reset()
        self.metrics = self.scorer.evaluate(self.state())
        obs = self._observation()
        if self.render_images and any(np.std(x) < 5 for x in obs.images.values()):
            raise RuntimeError("Blank native camera reset")
        return obs

    def state(self):
        view = self._walker.root_view
        q = array(view.get_dof_positions())[0]
        root = array(view.get_root_transforms())[0]
        links = array(view.get_link_transforms())[0]
        grips, feet = {}, []
        for side, sign in config.SIDES.items():
            p = links[self._legs[side]]
            rot = quat_xyzw_to_matrix(p[3:])
            grips[side] = (p[:3] + rot @ np.array([config.GRASP_DISTANCE, 0., config.GRIP_HEIGHT])).tolist()
            feet.extend((p[:3], p[:3] + rot @ [config.BACK_FOOT_DISTANCE, sign*config.BACK_FOOT_SPLAY, 0.]))
        root_velocity = array(view.get_root_velocities())[0]
        return {
            "time_s": self._t,
            "root": root.tolist(),
            "fold_rad": {side: float(-sign*q[self._hinges[side]]) for side, sign in config.SIDES.items()},
            "button_mm": {side: float(1000*q[self._buttons[side]]) for side in config.SIDES},
            "locked": self._locked.copy(),
            "feet": np.asarray(feet).tolist(),
            "grips": grips,
            "root_speed": float(np.linalg.norm(root_velocity[:3])),
            "root_angular_speed": float(np.linalg.norm(root_velocity[3:])),
            "hands": {side: tcp_world(self._base.scene[side + "_robot"]).tolist()[:3]
                      for side in config.SIDES},
        }

    def _task_step(self):
        if self._audit_error:
            raise RuntimeError("Native walker latch update failed") from self._audit_error
        self.metrics = self.scorer.evaluate(self.state())
        return StepResult(
            observation=self._observation(),
            terminated=self.metrics["success"],
            termination_reason="success" if self.metrics["success"] else None,
            info=self.metrics.copy(),
        )

    def proof_load(self, last_action):
        """Native post-trial bench check with the grippers stationary and clear."""
        if not self.metrics["success"]:
            raise ValueError("A released, ready walker is required before the load check")
        hold = self._torch.tensor(
            last_action, dtype=self._torch.float32, device=self._base.device
        ).unsqueeze(0)
        peak_fold = dict.fromkeys(config.SIDES, 0.)
        passed = True
        self._proof_force = config.PROBE_FORCE
        try:
            for _ in range(20):
                self._env.step(hold)
                state = self.state()
                for side in config.SIDES:
                    peak_fold[side] = max(peak_fold[side], abs(np.rad2deg(state["fold_rad"][side])))
                    passed &= state["locked"][side] and state["button_mm"][side] >= 4.4
        finally:
            self._proof_force = 0.
        if self._audit_error:
            raise RuntimeError("Native load check failed") from self._audit_error
        return {
            "inward_force_per_grip_n": config.PROBE_FORCE,
            "duration_s": 1.0,
            "max_fold_degrees": peak_fold,
            "passed": bool(passed and max(peak_fold.values()) <= 4.),
        }

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
