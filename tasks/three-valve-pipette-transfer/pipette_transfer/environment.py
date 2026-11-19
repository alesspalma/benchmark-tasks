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

from . import builder, config, fluid, scoring

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
        position = np.array([.82, -.20, 1.46])
        sight = np.array([.34, .00, 1.00]) - position
        sight /= np.linalg.norm(sight)
        screen_right = np.cross(sight, [0.0, 0.0, 1.0])
        screen_right /= np.linalg.norm(screen_right)
        camera = replace(
            camera, pos=tuple(position),
            rot_xyzw=tuple(ik.matrix_to_quat_xyzw(
                np.column_stack((screen_right, np.cross(screen_right, sight), -sight))
            )),
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
        self._water_callback = None

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
        # The overhead is mounted off the active worktop. The stock rear
        # mast would obscure this closer forward-facing camera pose.
        fixtures = [item for item in room if item.name in ("floor", "table", "dome_light", "lamp")]
        all_objects = [*fixtures, *objects]
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
        self._pipette = self._base.scene["pipette"]
        self._joint_ids = {
            joint: self._pipette.joint_names.index("Slide" + joint)
            for joint in ("Squeeze", *config.VALVES)
        }
        self._prim_root = self._base.scene.env_prim_paths[0]
        self._water_callback = self._base.sim.physics_manager.register_callback(
            self._update_water_state, IsaacEvents.PRE_PHYSICS_STEP, order=55,
            name=config.KEY + "_water", wrap_weak_ref=False,
        )

    def _pipe_state(self):
        from yam_arena.urdf import quat_xyzw_to_matrix

        view = self._pipette.root_view
        joints = array(view.get_dof_positions())[0]
        poses = dict(zip(self._pipette.body_names, array(view.get_link_transforms())[0],
                         strict=True))
        pose = poses["Pipe"]
        up = quat_xyzw_to_matrix(pose[3:])[:, 2]
        travel = {name: max(0., float(-joints[i]))
                  for name, i in self._joint_ids.items()}
        return pose, float(up[2]), travel

    def _update_water_state(self, dt):
        if not self._active:
            return
        try:
            pose, upright, travel = self._pipe_state()
            self.water.step(
                dt, pose[:3], upright, travel["Squeeze"],
                *(fluid.valve_conductance(travel[name]) for name in config.VALVES),
            )
        except Exception as error:
            self._audit_error = error

    def state(self):
        pose, upright, travel = self._pipe_state()
        return {
            "pipe_pose": pose.tolist(),
            "upright": upright,
            "squeeze_m": travel.pop("Squeeze"),
            "valves_m": travel,
            "bulb_pressure_pa": self.water.bulb_pressure,
            "tube_pressure_pa": self.water.tube_pressure,
            "air_equivalent_ml": self.water.air_equivalent,
            "pipette_ml": self.water.pipette,
            "peak_pipette_ml": self.water.peak_column,
            "source_ml": self.water.source,
            "receiving_ml": self.water.receiver,
            "spilled_ml": self.water.spilled,
            "bulb_flooded": self.water.at_float,
            "hands": {name: tcp_world(self._base.scene[name]).tolist() for name in ARMS},
            "time_s": self._t,
        }

    def _render_water(self, state):
        builder.update_water(self._base.sim.stage, self._prim_root, self.water, state["squeeze_m"])
        self._base.sim.render()

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._active = False
        self._audit_error = None
        self._env.reset(seed=seed)
        self._instruction = scene.instruction
        self.water = fluid.Water()
        for _ in range(self.spec["settle_actions"]):
            self._env.step(self._ready)
        self._t = 0.
        initial = self.state()
        if abs(initial["pipe_pose"][2] - config.PARKED[2]) > .008 or initial["upright"] < .98:
            raise RuntimeError(f"Pipette did not settle upright on its support: {initial}")
        self.metrics = self.scorer.evaluate(initial)
        if self.render_images:
            for _ in range(4):
                self._render_water(initial)
        obs = self._observation()
        if self.render_images and any(np.std(x) < 5 for x in obs.images.values()):
            raise RuntimeError("Blank native camera reset")
        self._active = True
        return obs

    def _task_step(self):
        if self._audit_error:
            raise RuntimeError("Native water update failed") from self._audit_error
        state = self.state()
        self.metrics = self.scorer.evaluate(state)
        if self.render_images:
            self._render_water(state)
        return StepResult(
            observation=self._observation(), terminated=self.metrics["success"],
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
            values.extend([*q[ids], float(np.clip(q[fingers].mean() / FINGER_OPEN, 0, 1))])
        return Observation(
            images={}, state={"joint_pos": np.asarray(values)}, instruction=self._instruction,
            state_time=self._t,
        )

    def close(self):
        self._active = False
        if self._water_callback is not None:
            self._water_callback.deregister()
            self._water_callback = None
        if self._env is not None:
            self._env.close()
            self._env = self._base = None
        if self._temp is not None:
            self._temp.cleanup()


def embodiment(**kwargs):
    return Environment(**kwargs)
