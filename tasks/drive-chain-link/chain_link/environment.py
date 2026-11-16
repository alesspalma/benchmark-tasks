"""Pre-clamped chain fixture and free joint hardware; standard robot controls."""

from __future__ import annotations

import atexit
import hashlib
import importlib.util
import json
import os
import tempfile
from dataclasses import replace

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

from . import TASK_ROOT
from .config import (
    CLIP_POSITIONS,
    CLIP_QUATERNION,
    CONNECTOR_PICKUP_POSITION,
    CONNECTOR_QUATERNION,
    CONTROL_HZ,
    KEEPER_POSITION,
    KEEPER_QUATERNION,
    PHYSICS_HZ,
    ROOT_POSITION,
    ROOT_QUATERNION,
    TOOL_POSITION,
    TOOL_QUATERNION,
)
from .geometry import local_solids, robot_clearance, world_points
from .scoring import Scorer

_APP = None


def numpy(value):
    if hasattr(value, "torch"):
        value = value.torch
    if hasattr(value, "detach"):
        return value.detach().cpu().numpy().astype(float)
    if hasattr(value, "numpy"):
        return value.numpy().astype(float)
    return np.asarray(value, dtype=float)


def source_fingerprint():
    d = hashlib.sha256()
    for p in sorted(TASK_ROOT.rglob("*")):
        if (
            not p.is_file()
            or any(
                s in p.parts for s in ("__pycache__", ".pytest_cache", ".ruff_cache")
            )
            or ".egg-info" in str(p)
        ):
            continue
        if (
            p.suffix == ".py"
            or p.name == "pyproject.toml"
            or (p.parent.name == "motions" and p.suffix == ".json")
        ):
            d.update(p.relative_to(TASK_ROOT).as_posix().encode() + b"\0")
            d.update(hashlib.sha256(p.read_bytes()).digest())
    return d.hexdigest()


def close_app():
    if _APP is not None:
        _APP.close()


class ChainLinkEnvironment(YamArenaEmbodiment):
    def __init__(
        self,
        *,
        object_jitter=0,
        control_hz=CONTROL_HZ,
        physics_device=None,
        render_images=True,
        **kwargs,
    ):
        if float(object_jitter) != 0 or float(control_hz) != CONTROL_HZ:
            raise ValueError("Fixed layout at 20 Hz required")
        super().__init__(
            scene="station", object_jitter=0, control_hz=CONTROL_HZ, **kwargs
        )
        self.info = replace(
            self.info,
            name="chain_link",
            supported_target_kinds=frozenset(),
            capabilities=self.info.capabilities | {PRIVILEGED_SUCCESS},
            docs="\n\n".join(
                (
                    JOINT_GUIDE,
                    ROOM_DOCS["station"],
                    (
                        "The dark grey conveyor chain is held in a service fixture, with an open joint at the front. "
                        "The yellow two-pin connecting link rests on the right loading stand. "
                        "The blue closed-hole cover plate rests on the left stand. Two spring clips sit freely in the bench-mounted black applicator beside the joint. Its passive guide allows horizontal motion along two axes. "
                        "The end nests and hold-down straps stay tightened."
                    ),
                )
            ),
            environment_id="chain-link-" + source_fingerprint()[:16],
            environment_revision=source_fingerprint()[:12],
        )
        self.render_images = bool(render_images)
        if not self.render_images:
            self.info = replace(
                self.info,
                observation_space=observation_space(
                    self.cam_height, self.cam_width, ()
                ),
                environment_id=self.info.environment_id + "-physics-only",
            )
        self.physics_device = self.device if physics_device is None else physics_device
        self.task = self
        self.scorer = Scorer()
        self.metrics = {}
        self._temp = None

    def _ensure_sim(self, goal=None):
        global _APP
        if self._env is not None:
            return
        if os.environ.get("OMNI_KIT_ACCEPT_EULA", "").upper() not in {
            "YES",
            "Y",
            "TRUE",
            "1",
        }:
            raise RuntimeError("Operator license acceptance is required")
        if _APP is None:
            from isaaclab.app import AppLauncher

            _APP = AppLauncher(
                headless=self.headless,
                device=self.device,
                enable_cameras=self.render_images,
                multi_gpu=False,
                width=self.cam_width,
                height=self.cam_height,
                anti_aliasing=0,
            ).app
            atexit.register(close_app)
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
        from yam_arena.embodiment import BimanualYamEmbodiment
        from yam_arena.scene import env_cfg_callback, task_room

        spec = importlib.util.spec_from_file_location(
            "chain_asset_builder", TASK_ROOT / "build_scene.py"
        )
        builder = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(builder)
        self._temp = tempfile.TemporaryDirectory(prefix="chain-link-assets-")
        assets = builder.create_assets(self._temp.name)
        self._local = local_solids(
            json.loads(
                (assets["fixtures"].parent / "geometry-inventory.json").read_text()
            )
        )
        robot = BimanualYamEmbodiment(
            enable_cameras=self.render_images,
            rig=self.rig,
            contact_sensors=False,
            robot_contact_offset=0.001,
            cam_width=self.cam_width,
            cam_height=self.cam_height,
        )
        objects = [
            Object(
                name="chain",
                object_type=ObjectType.ARTICULATION,
                usd_path=str(assets["chain"]),
                initial_pose=Pose(
                    position_xyz=tuple(map(float, ROOT_POSITION)),
                    rotation_xyzw=tuple(map(float, ROOT_QUATERNION)),
                ),
            ),
            Object(
                name="connector",
                object_type=ObjectType.RIGID,
                usd_path=str(assets["connector"]),
                initial_pose=Pose(
                    position_xyz=tuple(map(float, CONNECTOR_PICKUP_POSITION)),
                    rotation_xyzw=tuple(map(float, CONNECTOR_QUATERNION)),
                ),
            ),
            Object(
                name="keeper",
                object_type=ObjectType.RIGID,
                usd_path=str(assets["keeper"]),
                initial_pose=Pose(
                    position_xyz=tuple(map(float, KEEPER_POSITION)),
                    rotation_xyzw=tuple(map(float, KEEPER_QUATERNION)),
                ),
            ),
            Object(
                name="fixtures",
                object_type=ObjectType.BASE,
                usd_path=str(assets["fixtures"]),
                initial_pose=Pose(),
            ),
        ]
        objects += [
            Object(
                name="tool",
                object_type=ObjectType.ARTICULATION,
                usd_path=str(assets["tool"]),
                initial_pose=Pose(
                    position_xyz=tuple(map(float, TOOL_POSITION)),
                    rotation_xyzw=tuple(map(float, TOOL_QUATERNION)),
                ),
            )
        ]
        objects += [
            Object(
                name=n,
                object_type=ObjectType.ARTICULATION,
                usd_path=str(assets[n]),
                initial_pose=Pose(
                    position_xyz=tuple(map(float, p)),
                    rotation_xyzw=tuple(map(float, CLIP_QUATERNION)),
                ),
            )
            for n, p in CLIP_POSITIONS.items()
        ]
        all_objects = [*task_room("station", self.rig.top_cam), *objects]
        # Arena mutates init_state when applying a pose. Supply a separate
        # supported configuration object to each asset, avoiding shared defaults.
        for item in all_objects:
            if not isinstance(item, Object):
                continue
            cls = {
                ObjectType.RIGID: RigidObjectCfg,
                ObjectType.ARTICULATION: ArticulationCfg,
            }.get(item.object_type, AssetBaseCfg)
            initial = (
                cls.InitialStateCfg(joint_pos={}, joint_vel={})
                if item.name == "chain"
                else cls.InitialStateCfg()
            )
            item.asset_cfg_addon = {**item.asset_cfg_addon, "init_state": initial}
            if item.name in {"chain", "connector", "keeper", "tool", "clip0", "clip1"}:
                item.spawn_cfg_addon = {
                    **item.spawn_cfg_addon,
                    "activate_contact_sensors": False,
                }

        def configure(cfg):
            from isaaclab_physx.physics.physx_manager_cfg import PhysxCfg
            from yam_arena.constants import FINGER_OPEN

            cfg = env_cfg_callback(CONTROL_HZ)(cfg)
            cfg.sim.dt = 1.0 / PHYSICS_HZ
            cfg.decimation = PHYSICS_HZ // CONTROL_HZ
            cfg.sim.render_interval = cfg.decimation
            # Arena merges nested dictionaries when building the scene. Clear
            # its generic joint selector after that merge: this pre-clamped
            # fixture has fixed links and no movable joints.
            cfg.scene.chain.init_state.joint_pos = {}
            cfg.scene.chain.init_state.joint_vel = {}
            # Reset directly to the standard READY posture. Starting at all
            # zero joint angles sweeps the forearms through raised fixtures
            # before the first observation; it is not part of the task.
            for name in ARMS:
                initial = getattr(cfg.scene, name).init_state
                initial.joint_pos = {
                    **dict(zip(ARM_JOINTS, READY)),
                    **dict.fromkeys(FINGER_JOINTS, FINGER_OPEN),
                }
                initial.joint_vel = {".*": 0.0}
            # Empty passive actuator groups use the standard PhysX path.
            # The optional Newton actuator wrapper must not drive free hinges.
            cfg.sim.use_newton_actuators = False
            if self.physics_device == "cpu":
                cfg.sim.use_fabric = False
            # Capacity settings only: avoid reserving multi-environment buffers
            # for this single small scene. Contact dynamics are unchanged.
            cfg.sim.physics = PhysxCfg(
                gpu_max_rigid_contact_count=2**19,
                gpu_max_rigid_patch_count=2**17,
                gpu_found_lost_pairs_capacity=2**18,
                gpu_found_lost_aggregate_pairs_capacity=2**18,
                gpu_total_aggregate_pairs_capacity=2**18,
            )
            return cfg

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
        self._base, self._torch = self._env.unwrapped, torch
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

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._env.reset(seed=seed)
        from yam_arena.constants import FINGER_OPEN

        for name in ARMS:
            robot = self._base.scene[name]
            values = dict(zip(ARM_JOINTS, READY)) | dict.fromkeys(
                FINGER_JOINTS, FINGER_OPEN
            )
            q = self._torch.tensor(
                [[values[n] for n in robot.joint_names]],
                dtype=self._torch.float32,
                device=self._base.device,
            )
            robot.write_joint_state_to_sim_index(
                position=q, velocity=self._torch.zeros_like(q)
            )
            robot.actuators.target_command.set_position_index(value=q)
            robot.actuators.target_command.set_velocity_index(
                value=self._torch.zeros_like(q)
            )
        # Restore the declared layout after initializing the robot posture;
        # these writes are reset-only and precede all scoring/observations.
        for name in ("chain", "connector", "keeper", "tool", "clip0", "clip1"):
            asset = self._base.scene[name]
            pose = asset.data.default_root_pose.torch.clone()
            pose[:, :3] += self._base.scene.env_origins
            asset.write_root_pose_to_sim_index(root_pose=pose)
            vel = asset.data.default_root_vel.torch.clone()
            asset.write_root_velocity_to_sim_index(
                root_velocity=self._torch.zeros_like(vel)
            )
        for name in ("tool", "clip0", "clip1"):
            asset = self._base.scene[name]
            q = asset.data.joint_pos
            q = q.torch if hasattr(q, "torch") else q
            zeros = self._torch.zeros(q.shape, dtype=q.dtype, device=q.device)
            asset.write_joint_state_to_sim_index(position=zeros, velocity=zeros)
        self._instruction = scene.instruction
        for _ in range(10):
            self._env.step(self._ready)
        if self.render_images:
            for _ in range(4):
                self._base.sim.render()
        self.scorer.reset()
        self._t = 0.0
        self.metrics = self.scorer.evaluate(self.state())
        if self.metrics["score"] != 0 or self.metrics["violations"]:
            raise RuntimeError(f"Invalid reset: {self.metrics}")
        state = self.state()
        for name, expected in [
            ("connector", CONNECTOR_PICKUP_POSITION),
            ("keeper", KEEPER_POSITION),
        ]:
            if np.linalg.norm(np.array(state[name]["pose"][:3]) - expected) > 0.012:
                raise RuntimeError(
                    f"{name} reset pose {state[name]['pose'][:3]} differs from {expected.tolist()}"
                )
        observation = self._observation()
        from pxr import UsdGeom

        cache = UsdGeom.XformCache()
        self.reset_diagnostics = {"cameras": {}, "state": state}
        for prim in self._base.sim.stage.Traverse():
            if prim.GetTypeName() == "Camera":
                m = cache.GetLocalToWorldTransform(prim)
                self.reset_diagnostics["cameras"][str(prim.GetPath())] = {
                    "position": list(m.ExtractTranslation()),
                    "rotation_matrix": [list(row) for row in m.ExtractRotationMatrix()],
                }
        self.reset_image_stats = {
            n: {
                "std": float(np.std(image)),
                "min": int(image.min()),
                "max": int(image.max()),
            }
            for n, image in observation.images.items()
        }
        if self.render_images and any(
            np.std(image) < 1 for image in observation.images.values()
        ):
            raise RuntimeError(
                f"Invalid uniform camera frames: {self.reset_image_stats}"
            )
        return observation

    def state(self):
        from yam_arena.runtime import tcp_world

        chain = self._base.scene["chain"]
        poses = numpy(chain.data.body_link_pose_w)[0]
        velocity = numpy(chain.data.body_com_vel_w)[0]
        result = {
            "time_s": self._t,
            "bodies": {
                n: {"pose": p.tolist(), "velocity": v.tolist()}
                for n, p, v in zip(chain.body_names, poses, velocity)
            },
            "mechanism": {},
            "hands": {},
        }
        for n in ("connector", "keeper"):
            obj = self._base.scene[n]
            result[n] = {
                "pose": numpy(obj.data.root_link_pose_w)[0].tolist(),
                "velocity": numpy(obj.data.root_com_vel_w)[0].tolist(),
            }
        obj = self._base.scene["tool"]
        tool_names = obj.body_names
        tool_poses = numpy(obj.data.body_link_pose_w)[0]
        tool_velocities = numpy(obj.data.body_com_vel_w)[0]
        tool_id = obj.body_names.index("Body")
        result["tool"] = {
            "pose": tool_poses[tool_id].tolist(),
            "velocity": tool_velocities[tool_id].tolist(),
        }
        result["mechanism"]["applicator_slide_m"] = dict(
            zip(obj.joint_names, numpy(obj.data.joint_pos)[0].tolist())
        )
        result["clips"] = {}
        clip_poses = {}
        for n in ("clip0", "clip1"):
            obj = self._base.scene[n]
            result["clips"][n] = {
                "pose": numpy(obj.data.root_link_pose_w)[0].tolist(),
                "velocity": numpy(obj.data.root_com_vel_w)[0].tolist(),
                "joints": dict(
                    zip(obj.joint_names, numpy(obj.data.joint_pos)[0].tolist())
                ),
            }
            clip_poses.update(
                {
                    n + ":" + name: pose
                    for name, pose in zip(
                        obj.body_names, numpy(obj.data.body_link_pose_w)[0]
                    )
                }
            )
        for n in ARMS:
            robot = self._base.scene[n]
            q = numpy(robot.data.joint_pos)[0]
            result["hands"][n] = {
                "tcp": tcp_world(robot).tolist(),
                "joint_positions": q.tolist(),
                "opening_m": float(-q[self._ids[n][1]].sum()),
                "poses": numpy(robot.data.body_link_pose_w)[0].tolist(),
                "names": robot.body_names,
            }
        points = world_points(
            self._local,
            {
                **{"chain:" + n: v["pose"] for n, v in result["bodies"].items()},
                **{n + ":Body": result[n]["pose"] for n in ("connector", "keeper")},
                **{"tool:" + name: pose for name, pose in zip(tool_names, tool_poses)},
                **clip_poses,
            },
        )
        result["bounds"] = {
            n: [p.min(0).tolist(), p.max(0).tolist()] for n, p in points.items()
        }
        result["robot_clearance_m"] = robot_clearance(points, result["hands"])
        return result

    def _task_step(self):
        state = self.state()
        # Workspace checks share the declared control/video sampling rate.
        # The min/max corners retain every body's exact coordinate extrema.
        self.scorer.observe_workspace(
            {name: np.asarray(bounds) for name, bounds in state["bounds"].items()}
        )
        if "invalid_state" in self.scorer.violations:
            raise RuntimeError("Nonfinite native physics state; trial is invalid")
        if self.scorer.workspace_samples != round(self._t * CONTROL_HZ):
            raise RuntimeError("Incomplete control-frame workspace audit")
        self.metrics = self.scorer.evaluate(state)
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
        per_arm = []
        for n in ARMS:
            q = numpy(self._base.scene[n].data.joint_pos)[0]
            arm_ids, finger_ids = self._ids[n]
            from yam_arena.constants import FINGER_OPEN

            grip = float(np.clip(q[finger_ids].mean() / FINGER_OPEN, 0, 1))
            per_arm.append(np.r_[q[arm_ids], grip])
        return Observation(
            images={},
            state={"joint_pos": np.r_[*per_arm]},
            instruction=self._instruction,
            state_time=self._t,
        )

    def close(self):
        if self._env is not None:
            self._env.close()
            self._env = self._base = None
        if self._temp is not None:
            self._temp.cleanup()


def chain_link_embodiment(**kwargs):
    return ChainLinkEnvironment(**kwargs)
