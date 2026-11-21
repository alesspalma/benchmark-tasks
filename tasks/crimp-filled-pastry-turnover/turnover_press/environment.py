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
from yam_arena.urdf import quat_xyzw_to_matrix

from . import builder, config
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
        position = np.array([-0.145, -0.017, 1.255])
        sight = np.array([0.39, -0.07, 0.80]) - position
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
        self._detent_engaged = False
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

        self._press = self._base.scene["press"]
        self._pastry = self._base.scene["pastry"]
        self._configure_mold_contact()
        self._press_effort = torch.zeros((1, 1), device=self._base.device)
        self._pastry_effort = torch.zeros((1, 1), device=self._base.device)
        self._press_warp = wp.from_torch(self._press_effort)
        self._pastry_warp = wp.from_torch(self._pastry_effort)
        self._contact = {}
        for name, body in (("press", self._press), ("pastry", self._pastry)):
            force = torch.zeros((1, body.num_bodies, 3), dtype=torch.float32, device=self._base.device)
            torque = torch.zeros_like(force)
            self._contact[name] = (body, force, torque, wp.from_torch(force).flatten(), wp.from_torch(torque).flatten())
        self._drive_callback = self._base.sim.physics_manager.register_callback(
            self._material_step,
            IsaacEvents.PRE_PHYSICS_STEP,
            order=50,
            name=config.KEY + "_material",
            wrap_weak_ref=False,
        )
        if self.render_images:
            self._prepare_impressions()

    def _configure_mold_contact(self):
        from isaaclab.sim import find_matching_prims, get_current_stage
        from pxr import UsdPhysics

        stage = get_current_stage()
        lower = find_matching_prims(self._press.cfg.prim_path, stage)[0].GetPath()
        pastry = find_matching_prims(self._pastry.cfg.prim_path, stage)[0].GetPath()
        # The upper die deforms the dough through the material response below.
        # The native hulls remain active for the lower well and gripper contact.
        die = stage.GetPrimAtPath(lower.AppendPath("UpperPress"))
        faces = [prim.GetPath() for prim in die.GetChildren() if prim.HasAPI(UsdPhysics.CollisionAPI)]
        for side in ("LowerDough", "UpperDough"):
            shape = stage.GetPrimAtPath(pastry.AppendPath(f"{side}/DoughContact"))
            relation = UsdPhysics.FilteredPairsAPI.Apply(shape).CreateFilteredPairsRel()
            relation.SetTargets(faces)

    def _material_step(self, dt):
        press_view = self._press.root_view
        pastry_view = self._pastry.root_view
        fold = -array(press_view.get_dof_positions())[0, 0]
        dough_fold = -array(pastry_view.get_dof_positions())[0, 0]
        rate = -array(pastry_view.get_dof_velocities())[0, 0]
        # Positive applied torque opens both native hinges. Contact bends the
        # supported dough lobe, pressure on the actual rim is felt at the tab.
        # A friction hinge holds the working opening during loading. The
        # finite detent releases when pressed through the working arc.
        speed = -array(press_view.get_dof_velocities())[0, 0]
        opened = fold - config.READY_FOLD
        press_effort = 0.0
        if fold >= config.READY_FOLD + np.radians(18.0):
            self._detent_engaged = False
        if self._detent_engaged and np.radians(39.0) < fold:
            press_effort += np.clip(0.038 + 3.1 * opened + 0.028 * speed, -0.36, 0.22)
        if self._sampling:
            press = array(press_view.get_root_transforms())[0]
            pastry = array(pastry_view.get_root_transforms())[0]
            self._support_dough(press, pastry)
            press_effort -= self.scorer.sample(
                press,
                pastry,
                fold,
                dough_fold,
                dt,
            )
        if self.scorer.sealed:
            pastry_effort = np.clip(1.4 * (dough_fold - config.FOLDED) + 0.022 * rate, -0.115, 0.115)
        else:
            pastry_effort = 0.0018 * dough_fold + 0.0006 * rate
            if self.scorer.registered:
                margin = max(0.006, (np.pi - fold) * 0.30)
                shape_error = dough_fold - min(config.FOLDED, fold + margin)
                if abs(shape_error) <= 0.44:
                    contact = np.clip(0.25 * shape_error + 0.003 * (rate - speed), -0.032, 0.032)
                    pastry_effort += contact
                    press_effort -= contact
                compression = min(0.065, 0.65 * max(0.0, fold - dough_fold - 0.012))
                pastry_effort -= compression
                press_effort += compression
        self._press_effort[0, 0] = press_effort
        self._pastry_effort[0, 0] = pastry_effort
        press_view.set_dof_actuation_forces(self._press_warp, self._press._ALL_INDICES)
        pastry_view.set_dof_actuation_forces(self._pastry_warp, self._pastry._ALL_INDICES)

    def _support_dough(self, press, pastry):
        # The moist dough sticks weakly to the lower well. A finite local
        # contact load resists tangential slip and lets a normal lift peel it.
        axes = quat_xyzw_to_matrix(press[3:])
        offset = axes.T @ (pastry[:3] - press[:3])
        within_well = np.linalg.norm(offset[:2]) <= 0.021 and -0.005 <= offset[2] <= 0.022
        support_velocity = array(self._press.root_view.get_root_velocities())[0]
        brace_force = np.clip(1000. * (config.PRESS - press[:3]) - 10. * support_velocity[:3], -21.0, 21.0)
        brace_torque = np.clip(2.2 * (np.cross(axes[:, 2], [0., 0., 1.]) + np.cross(axes[:, 0], [1., 0., 0.])) - .026 * support_velocity[3:], -.38, .38)
        motion = array(self._pastry.root_view.get_root_velocities())[0] - array(self._press.root_view.get_root_velocities())[0]
        drift = axes.T @ motion[:3]
        force = -np.array([260., 260., 48.]) * (offset - [0., 0., 0.0075]) - np.array([2.6, 2.6, 0.6]) * drift
        force = axes @ np.clip(force, [-3.0, -3.0, -0.65], [3.0, 3.0, 0.65])
        pastry_axes = quat_xyzw_to_matrix(pastry[3:])
        torque = 0.25 * np.cross(pastry_axes[:, 2], axes[:, 2]) + 0.085 * np.cross(pastry_axes[:, 0], axes[:, 0]) - 0.002 * motion[3:]
        torque = np.clip(torque, -0.026, 0.026)
        if not within_well:
            force.fill(0.)
            torque.fill(0.)
        for name in ("press", "pastry"):
            applied = brace_force - force if name == "press" else force
            moment = brace_torque - torque if name == "press" else torque
            body, forces, torques, force_view, torque_view = self._contact[name]
            root_index = body.body_names.index("LowerPress" if name == "press" else "LowerDough")
            forces[0, root_index] = self._torch.as_tensor(applied, device=self._base.device)
            torques[0, root_index] = self._torch.as_tensor(moment, device=self._base.device)
            body.root_view.apply_forces_and_torques_at_position(
                force_data=force_view,
                torque_data=torque_view,
                position_data=None,
                indices=body._ALL_INDICES,
                is_global=True,
            )

    def _prepare_impressions(self):
        from isaaclab.sim import find_matching_prims, get_current_stage
        from pxr import UsdGeom

        stage = get_current_stage()
        root = find_matching_prims(self._pastry.cfg.prim_path, stage)[0].GetPath()
        self._impressions = [
            UsdGeom.Imageable.Get(stage, root.AppendPath(f"{side}/Impression{k}"))
            for side in ("LowerDough", "UpperDough")
            for k in range(config.SECTOR_COUNT)
        ]
        self._emboss_visible = None

    def _show_status(self):
        from pxr import UsdGeom

        embossed = self.scorer.sealed
        if embossed != self._emboss_visible:
            token = UsdGeom.Tokens.inherited if embossed else UsdGeom.Tokens.invisible
            for prim in self._impressions:
                prim.CreateVisibilityAttr(token)
                prim.GetVisibilityAttr().Set(token)
            self._emboss_visible = embossed

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._sampling = False
        self._env.reset(seed=seed)
        self._detent_engaged = True
        for body, effort, initial in (
            (self._press, self._press_effort, config.READY_FOLD),
            (self._pastry, self._pastry_effort, config.READY_DOUGH),
        ):
            angle = self._torch.full_like(effort, -initial)
            body.write_joint_state_to_sim(angle, self._torch.zeros_like(angle))
        self.scorer.reset()
        self._instruction = scene.instruction
        self._emboss_visible = None
        self._sampling = True
        for _ in range(self.spec["settle_actions"]):
            self._env.step(self._ready)
        self.scorer.reset()
        self._t = 0.0
        state = self.state()
        self.metrics = self.scorer.evaluate(state, elapsed=0.0)
        if self.render_images:
            self._show_status()
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
            self._show_status()
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
