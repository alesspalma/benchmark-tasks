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
from .scoring import Scorer, at_seat, routed_tip, tip_from

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
        position = np.array([-0.335, 0.205, 0.950])
        sight = np.array([0.433, 0.018, 0.875]) - position
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
        self._ids = {
            n: (
                self._base.scene[n].find_joints(ARM_JOINTS, preserve_order=True)[0],
                self._base.scene[n].find_joints(FINGER_JOINTS, preserve_order=True)[0],
            )
            for n in ARMS
        }

        import warp as wp
        from isaaclab.sim import find_matching_prims, get_current_stage
        from isaaclab_physx.physics.physx_manager import IsaacEvents

        self._roll = self._base.scene["roll"]
        self._mech = self._base.scene["mechanism"]
        self._drive = {}
        for label, asset in (("roll", self._roll), ("mechanism", self._mech)):
            tensor = torch.zeros((1, asset.num_joints), device=self._base.device)
            self._drive[label] = (tensor, wp.from_torch(tensor), asset)
        self._cover_torque = torch.zeros_like(self._drive["mechanism"][0])
        self._cover_torque_view = wp.from_torch(self._cover_torque)
        self._hinge_idx = self._mech.joint_names.index("Hinge")
        self._drive_callback = self._base.sim.physics_manager.register_callback(
            self._native_drives,
            IsaacEvents.PRE_PHYSICS_STEP,
            order=50,
            name=config.KEY + "_loads",
            wrap_weak_ref=False,
        )
        self._sheet = None
        self._roller_marks = []
        from pxr import Usd, UsdGeom, UsdPhysics

        stage = get_current_stage()
        root = find_matching_prims(self._roll.cfg.prim_path, stage)[0]
        fixture = self._base.scene["dispenser"]
        housing = find_matching_prims(fixture.prim_path, stage)[0].GetPath()
        # The contact envelope of the hinged sheet stands in for flexible paper.
        # Model compliant passage past the drum and the moving outlet lips.
        # The collision envelope remains active against the grippers and cradle.
        fixture_root = find_matching_prims(self._mech.cfg.prim_path, stage)[0].GetPath()
        edge = root.GetPath().AppendPath("FreeEdge/PaperContact")
        pairs = UsdPhysics.FilteredPairsAPI.Apply(stage.GetPrimAtPath(edge))
        clearance = [housing.AppendChild(f"Drum{i}") for i in range(2)]
        for link in ("Cover", "Paddle"):
            for shape in Usd.PrimRange(
                stage.GetPrimAtPath(fixture_root.AppendChild(link))
            ):
                if shape.HasAPI(UsdPhysics.CollisionAPI):
                    clearance.append(shape.GetPath())
        pairs.CreateFilteredPairsRel().SetTargets(clearance)
        if self.render_images:
            self._sheet = UsdGeom.Mesh.Get(
                stage, root.GetPath().AppendChild("UpperWeb")
            )
            self._paper_frame = (
                UsdGeom.XformCache().GetLocalToWorldTransform(root).GetInverse()
            )
            for i in range(2):
                drum = UsdGeom.Xform.Get(stage, housing.AppendChild(f"RollerMarks{i}"))
                self._roller_marks.append(drum.GetOrderedXformOps()[-1])

    def _target(self, label, **values):
        tensor, _, asset = self._drive[label]
        for name, value in values.items():
            tensor[0, asset.joint_names.index(name)] = value

    def _native_drives(self, _dt):
        for _, view, asset in self._drive.values():
            asset.root_view.set_dof_position_targets(view, asset._ALL_INDICES)
        self._mech.root_view.set_dof_actuation_forces(
            self._cover_torque_view,
            self._mech._ALL_INDICES,
        )

    def _machine(self, state):
        # One-way rack advances the guide path only while the leading edge is
        # inside the rotating nip. The dimensions are in roll/paddle metres.
        roll = state["objects"]["roll"]
        tip = tip_from(roll)
        joints = state["objects"]["mechanism"]["joints"]
        stroke = joints["Stroke"]
        increment = max(0.0, stroke - max(self._stroke, 0.008))
        self._stroke = stroke
        self._drum_rotation += 4.55 * increment / 0.014
        displacement = np.abs(np.asarray(roll["pose"][:3]) - config.SEAT)
        out_of_guide = np.any(displacement > (0.046, 0.052, 0.053))
        self._slip_steps = self._slip_steps + 1 if out_of_guide else 0
        if self._slip_steps >= 7:
            self._engaged = False
        if (
            increment > 0.0
            and not self._engaged
            and at_seat(roll)
            and abs(tip[0] - config.NIP_X) <= 0.012
            and abs(tip[1] - config.SEAT[1]) <= 0.020
            and config.NIP_Z - 0.027 <= tip[2] <= config.NIP_Z + 0.028
        ):
            self._engaged = True
            self._bend = roll["joints"]["Bend"]
        bend = self._bend
        if self._engaged:
            self._payout = min(0.191, self._payout + 4.55 * increment)
            frame = quat_xyzw_to_matrix(roll["pose"][3:])
            phase = np.arctan2(frame[0, 2], frame[0, 0])
            pivot_x = roll["links"]["PayOut"][0]
            norm = np.hypot(frame[0, 0], frame[0, 2])
            bend = float(
                np.arcsin(
                    np.clip(
                        (pivot_x - config.NIP_X) / (config.PAPER_LENGTH * norm),
                        -0.89,
                        0.35,
                    )
                )
                - phase
            )
        self._target("roll", Travel=-self._payout, Bend=bend)
        angle = joints["Hinge"]
        self._closing = self._closing or angle > -0.71
        self._latched = self._closing and angle >= -0.035
        self._target(
            "mechanism", Stroke=0.0, Hinge=0.0 if self._closing else config.COVER_OPEN
        )
        # The spring-assisted close adds positive torque after the cover has
        # physically crossed the cam. A hand or an object can still obstruct it.
        speed = float(
            array(self._mech.root_view.get_dof_velocities())[0, self._hinge_idx]
        )
        load = (
            float(np.clip(-2.6 * angle - 0.35 * speed, 0.0, 2.3))
            if self._closing
            else 0.0
        )
        self._cover_torque[0, self._hinge_idx] = load

    def _draw_web(self, state):
        if self._sheet is None:
            return
        from pxr import Gf

        roll = state["objects"]["roll"]
        position = np.asarray(roll["pose"][:3])
        frame = quat_xyzw_to_matrix(roll["pose"][3:])
        carriage = np.asarray(roll["links"]["PayOut"][:3])
        top = position + frame @ np.array([-0.066, 0.0, -0.064])
        tip = routed_tip(roll, self._engaged)
        if self._engaged:
            mouth = np.array([config.NIP_X, tip[1], config.NIP_Z + 0.012])
            lip = np.array([config.NIP_X, tip[1], config.OUTLET_Z])
            if tip[2] >= config.OUTLET_Z:
                lip = (mouth + tip) / 2
            approach = 0.40 * top + 0.60 * mouth
            section = (top, approach, mouth, lip, tip)
        else:
            curl = position + frame @ np.array([config.PAPER_X + 0.009, 0.0, -0.080])
            section = (top, curl, carriage, 0.45 * carriage + 0.55 * tip, tip)
        points = [
            self._paper_frame.Transform(
                Gf.Vec3d(
                    *map(float, centre + sign * config.PAPER_WIDTH / 2 * frame[:, 1])
                )
            )
            for centre in section
            for sign in (-1, 1)
        ]
        self._sheet.GetPointsAttr().Set([Gf.Vec3f(*p) for p in points])
        for i, mark in enumerate(self._roller_marks):
            mark.Set(float(180.0 / np.pi * self._drum_rotation * (2 * i - 1)))

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._env.reset(seed=seed)
        q = self._mech.data.default_joint_pos.clone()
        q[:, self._mech.joint_names.index("Hinge")] = config.COVER_OPEN
        self._mech.write_joint_state_to_sim(q, self._torch.zeros_like(q))
        self.scorer.reset()
        self._stroke = self._bend = self._payout = self._drum_rotation = 0.0
        self._engaged = self._latched = self._closing = False
        self._slip_steps = 0
        self._cover_torque.zero_()
        self._target("roll", Travel=0.0, Bend=0.0)
        self._target("mechanism", Stroke=0.0, Hinge=config.COVER_OPEN)
        self._instruction = scene.instruction
        for _ in range(self.spec["settle_actions"]):
            self._env.step(self._ready)
        self._t = 0.0
        state = self.state()
        self.metrics = self.scorer.evaluate(state, elapsed=0.0)
        self._draw_web(state)
        if self.render_images:
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
            "latched": self._latched,
            "engaged": self._engaged,
            "closing": self._closing,
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
        state = self.state()
        self._machine(state)
        state["latched"], state["engaged"] = self._latched, self._engaged
        self.metrics = self.scorer.evaluate(state, elapsed=config.CHECK_DT)
        self._draw_web(state)
        if self.render_images:
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
