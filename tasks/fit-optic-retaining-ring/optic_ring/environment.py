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
        eye = np.array([.074, -.527, 1.24])
        target = np.array([.32, -.040, .849])
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
                "Two YAM arms are mounted on independent pedestals 488 mm apart. "
                "The overview shows the optical inspection pad and spanner rest. "
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
        self._thread_rest = 0.

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

        self._tube = self._base.scene["tube"]
        self._lens = self._base.scene["optic"]
        self._spanner = self._base.scene["spanner"]
        self._lead = self._tube.joint_names.index("Lead")
        self._thread = self._tube.joint_names.index("Thread")
        self._targets = torch.zeros((1, self._tube.num_joints), dtype=torch.float32,
                                    device=self._base.device)
        self._forces = torch.zeros_like(self._targets)
        self._targets_wp = wp.from_torch(self._targets)
        self._forces_wp = wp.from_torch(self._forces)
        self._joint_callback = self._base.sim.physics_manager.register_callback(
            self._thread_load, IsaacEvents.PRE_PHYSICS_STEP, order=55,
            name=config.KEY + "_thread_load", wrap_weak_ref=False)

    def _thread_load(self, _dt):
        """Couple native ring rotation to the documented 40-TPI axial lead."""
        try:
            view = self._tube.root_view
            q = array(view.get_dof_positions())[0]
            dq = array(view.get_dof_velocities())[0]
            angle, speed = q[self._thread], dq[self._thread]
            lead = config.PITCH_PER_RADIAN * angle
            self._targets[0, self._lead] = lead
            pressure = np.clip(75000. * (q[self._lead] - lead), 0., config.THREAD_FORCE_LIMIT)
            # The native angular drive stores finite static friction at
            # the most recent stick point. The reference can follow slip
            # only after external motion has crossed the friction band.
            elastic_limit = .010 + pressure * .00062
            self._thread_rest = float(np.clip(self._thread_rest,
                                               angle - elastic_limit,
                                               angle + elastic_limit))
            self._targets[0, self._thread] = self._thread_rest
            ring_inertia = .018 * (.0248 ** 2)
            stopping_torque = .65 * ring_inertia / _dt * speed
            self._forces[0, self._thread] = -np.clip(
                stopping_torque, -config.TURN_FRICTION, config.TURN_FRICTION)
            view.set_dof_position_targets(self._targets_wp, self._tube._ALL_INDICES)
            view.set_dof_actuation_forces(self._forces_wp, self._tube._ALL_INDICES)
        except Exception as error:
            self._audit_error = error

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._audit_error = None
        self._thread_rest = 0.
        self._targets.zero_()
        self._forces.zero_()
        self._env.reset(seed=seed)
        self._instruction = scene.instruction
        for _ in range(self.spec["settle_actions"]):
            self._env.step(self._ready)
        if self.render_images:
            for _ in range(4):
                self._base.sim.render()
        self._t = 0.
        self.scorer.reset()
        self.metrics = self.scorer.evaluate(self.state())
        obs = self._observation()
        if self.render_images and any(np.std(x) < 5 for x in obs.images.values()):
            raise RuntimeError("Blank native camera reset")
        return obs

    @staticmethod
    def _pose(body):
        pose = array(body.data.root_pose_w)[0]
        frame = ik.quat_xyzw_to_matrix(pose[3:])
        return pose[:3], frame, np.linalg.norm(array(body.data.root_lin_vel_w)[0])

    def state(self):
        view = self._tube.root_view
        q = array(view.get_dof_positions())[0]
        dq = array(view.get_dof_velocities())[0]
        tube, tube_frame, tube_speed = self._pose(self._tube)
        lens, lens_frame, lens_speed = self._pose(self._lens)
        tool, tool_frame, tool_speed = self._pose(self._spanner)
        hands, tips, hand_frames = {}, {}, {}
        for n in ARMS:
            robot = self._base.scene[n]
            q = array(robot.data.joint_pos)[0]
            ids, fingers = self._ids[n]
            tcp = np.asarray(tcp_world(robot))
            _, r = ik.tcp_pose(q[ids], config.BASES[n.split("_")[0]])
            hands[n] = tcp.tolist()
            hand_frames[n] = r.tolist()
            tips[n] = [
                (tcp + r @ [x, y, z]).tolist()
                for x in (q[fingers[0]], -q[fingers[1]])
                for y in (-.007, 0., .007)
                for z in (-.004, .006, .012)
            ]
        angles = array(view.get_dof_positions())[0]
        return {
            "time_s": self._t,
            "angle": float(angles[self._thread]),
            "lead": float(angles[self._lead]),
            "angular_speed": float(dq[self._thread]),
            "tube_pos": tube.tolist(),
            "tube_frame": tube_frame.tolist(),
            "tube_speed": float(tube_speed),
            "lens_pos": lens.tolist(),
            "lens_frame": lens_frame.tolist(),
            "lens_speed": float(lens_speed),
            "tool_pos": tool.tolist(),
            "tool_frame": tool_frame.tolist(),
            "tool_speed": float(tool_speed),
            "hands": hands,
            "hand_frames": hand_frames,
            "fingertips": tips,
        }

    def _task_step(self):
        if self._audit_error:
            raise RuntimeError("Native thread load update failed") from self._audit_error
        self.metrics = self.scorer.evaluate(self.state())
        return StepResult(
            observation=self._observation(),
            terminated=self.metrics["success"],
            termination_reason="success" if self.metrics["success"] else None,
            info=self.metrics.copy(),
        )

    def proof_load(self, last_action):
        """Independently proof the released optic with a small upward edge load."""
        if not self.metrics["success"]:
            raise ValueError("Proof requires a released assembly")
        t = self._torch
        device = self._base.device
        hold = t.tensor(last_action, dtype=t.float32, device=device).unsqueeze(0)
        composer = self._lens.permanent_wrench_composer
        forces = t.tensor([[[0., 0., .80]]], dtype=t.float32, device=device)
        torques = t.zeros_like(forces)
        local_edge = t.tensor([[[.0245, 0., -.0020]]], dtype=t.float32, device=device)
        start = self.state()
        origin = np.array(start["tube_pos"])
        direction = np.array(start["tube_frame"])[:, 2]
        lens_reference = np.dot(np.array(start["lens_pos"]) - origin, direction)
        max_shift = 0.
        composer.add_forces_and_torques_index(forces, torques, positions=local_edge)
        try:
            for _ in range(14):
                self._env.step(hold)
                state = self.state()
                axis = np.asarray(state["tube_frame"])[:, 2]
                axial = np.dot(np.asarray(state["lens_pos"]) - np.asarray(state["tube_pos"]), axis)
                max_shift = max(max_shift, abs(axial - lens_reference))
        finally:
            composer.reset()
        return {
            "upward_edge_force_n": .80,
            "duration_s": .70,
            "peak_axial_shift_um": 1e6 * max_shift,
            "passed": bool(max_shift < .00005),
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
