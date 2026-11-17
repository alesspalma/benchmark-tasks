"""Standard RGB/joint observations; native task contacts; scoring-only measurements."""

from __future__ import annotations

import atexit
import hashlib
import math
import os
import tempfile
from dataclasses import replace
from pathlib import Path

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

from . import builder, config, geometry, scoring, materials

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
        self._callback = None
        self._temp = None
        self._audit_error = None
        self.material = materials.MaterialModel()
        self._material_accum = 0.
        self._material_callback = None
        self._mass_props = {}

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
        self._geometry = geometry.collision_points(self.spec)
        self._spec_by_name = {s["name"]: s for s in self.spec["assets"]}
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
        all_objects = [*task_room("station", self.rig.top_cam), *objects]
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
        self._material_callback = self._base.sim.physics_manager.register_callback(
            self._material_step, IsaacEvents.PRE_PHYSICS_STEP, order=50,
            name=config.KEY + "_material", wrap_weak_ref=False)
        self._callback = self._base.sim.physics_manager.register_callback(
            self._audit,
            IsaacEvents.POST_PHYSICS_STEP,
            order=100,
            name=config.KEY + "_audit",
            wrap_weak_ref=False,
        )

    def _audit(self, dt):
        if not self._active:
            return
        try:
            self.scorer.observe(self.state(light=True, include_bounds=False))
        except Exception as e:  # noqa: BLE001 - Re-raised by the control step.
            self._audit_error = e

    def _apply(self,name,force,torque,position=None):
        import warp as wp
        self._base.scene[name].root_view.apply_forces_and_torques_at_position(
            force_data=wp.array(np.asarray(force,dtype=np.float32).reshape(1,3),device=self._base.device),
            torque_data=wp.array(np.asarray(torque,dtype=np.float32).reshape(1,3),device=self._base.device),
            position_data=None if position is None else wp.array(np.asarray(position,dtype=np.float32).reshape(1,3),device=self._base.device),
            indices=wp.array([0],dtype=wp.int32,device=self._base.device),is_global=True)

    def _material_step(self,dt):
        from yam_arena.urdf import quat_xyzw_to_matrix
        try:
            state=self.state(light=True, include_bounds=False)
            tool=state['objects']['restrainer'];bar=state['objects']['bar']
            tr=quat_xyzw_to_matrix(np.asarray(tool['pose'][3:]));br=quat_xyzw_to_matrix(np.asarray(bar['pose'][3:]))
            tp=np.asarray(tool['pose'][:3])+tr@config.MAGNET_OFFSET;bp=np.asarray(bar['pose'][:3])
            f,tb,tt=materials.dipoles(tp,tr,bp,br)
            self.material.last_force=f;self.material.max_force=max(self.material.max_force,float(np.linalg.norm(f)))
            self._apply('bar',f,tb,bp);self._apply('restrainer',-f,tt,tp)
            # Submerged PTFE bar buoyancy and weak quadratic water drag, with
            # equal/opposite liquid reaction applied to its containing vessel.
            for name in ('source','receiver'):
                obj=state['objects'][name];r=quat_xyzw_to_matrix(np.asarray(obj['pose'][3:]));p=np.asarray(obj['pose'][:3]);local=r.T@(bp-p)
                if np.linalg.norm(local[:2])<config.R_IN and config.FLOOR<local[2]<config.HEIGHT and self.material.volumes[name]>1e-9:
                    level=self.material.levels.get(name)
                    if level is None:
                        level=self.material.shape.level(p,r,self.material.volumes[name]);self.material.levels[name]=level
                    tips=np.array([bp+br[:,0]*x for x in np.linspace(-config.BAR_LENGTH/2,config.BAR_LENGTH/2,9)])
                    fraction=float(np.mean(tips[:,2]<level))
                    buoy=np.array([0,0,1000*9.81*math.pi*config.BAR_RADIUS**2*config.BAR_LENGTH*fraction])
                    flow=np.asarray(obj['velocity'][:3])+np.cross(obj['velocity'][3:],bp-p)
                    rel=np.asarray(bar['velocity'][:3])-flow
                    drag=-.5*1000*1.2*(config.BAR_LENGTH*config.BAR_RADIUS*2)*np.linalg.norm(rel)*rel*fraction
                    self._apply('bar',buoy+drag,np.zeros(3),bp);self._apply(name,-buoy-drag,np.zeros(3),bp)
                    break
            self._material_accum+=float(dt)
            if self._material_accum>=.05-1e-7:
                state['hands']={}
                for side in ARMS:
                    arm=self._base.scene[side]
                    state['hands'][side]={'names':list(arm.body_names),'poses':array(arm.root_view.get_link_transforms())[0].tolist()}
                self.material.transport(state,self._material_accum);self._material_accum=0.
                self._update_water_mass()
                if self.render_images:self._update_water_visuals()
        except Exception as e:
            self._audit_error=e

    def _update_water_mass(self):
        import warp as wp
        from yam_arena.urdf import quat_xyzw_to_matrix
        for name in ('source','receiver'):
            view=self._base.scene[name].root_view;pose=array(view.get_transforms())[0]
            r=quat_xyzw_to_matrix(pose[3:]);props=self.material.shape.properties(pose[:3],r,self.material.volumes[name])
            h,mass,com,inertia=props;old=self._mass_props.get(name)
            if old is None or abs(mass-old[0])>1e-6 or np.linalg.norm(com-old[1])>1e-5:
                ids=wp.array([0],dtype=wp.int32,device='cpu')
                view.set_masses(wp.array([[mass]],dtype=wp.float32,device='cpu'),indices=ids)
                view.set_coms(wp.array([[*com,0,0,0,1]],dtype=wp.float32,device='cpu'),indices=ids)
                view.set_inertias(wp.array([inertia.ravel()],dtype=wp.float32,device='cpu'),indices=ids)
                self._mass_props[name]=(mass,com)

    def _update_water_visuals(self):
        # Visual meshes only; these have no CollisionAPI and do not move bodies.
        from pxr import UsdGeom,Gf
        import omni.usd
        from yam_arena.urdf import quat_xyzw_to_matrix
        stage=omni.usd.get_context().get_stage()
        for name in ('source','receiver'):
            obj=self._base.scene[name];pose=array(obj.root_view.get_transforms())[0]
            points,faces=self.material.water_mesh(pose[:3],quat_xyzw_to_matrix(pose[3:]),self.material.volumes[name])
            mesh=UsdGeom.Mesh.Get(stage,f'/World/envs/env_0/{name}/Body/Liquid')
            mesh.GetPointsAttr().Set([Gf.Vec3f(*v) for v in points]);mesh.GetFaceVertexCountsAttr().Set([3]*len(faces));mesh.GetFaceVertexIndicesAttr().Set([i for f in faces for i in f])
        stream=UsdGeom.Mesh.Get(stage,'/World/envs/env_0/fixtures/Stream');pts=[];faces=[]
        for origin,end,volume in self.material.segments:
            delta=end-origin;length=float(np.linalg.norm(delta))
            if length<1e-8:continue
            axis=delta/length;u=np.cross(axis,[0,0,1])
            if np.linalg.norm(u)<1e-6:u=np.cross(axis,[0,1,0])
            u/=np.linalg.norm(u);v=np.cross(axis,u)
            radius=min(.006,max(.0006,math.sqrt(volume/(math.pi*length))))
            offset=len(pts)
            for center in (origin,end):
                for a in np.arange(8)*math.tau/8:pts.append(center+radius*(u*math.cos(a)+v*math.sin(a)))
            for k in range(8):
                a=offset+k;b=offset+(k+1)%8;c=b+8;d=a+8;faces.extend([[a,b,c],[a,c,d]])
        stream.GetPointsAttr().Set([Gf.Vec3f(*map(float,v)) for v in pts]);stream.GetFaceVertexCountsAttr().Set([3]*len(faces));stream.GetFaceVertexIndicesAttr().Set([i for f in faces for i in f])

    def reset(self, scene, *, seed=None):
        self._ensure_sim()
        self._active = False
        self._audit_error = None
        self._env.reset(seed=seed)
        self.material.reset()
        self._material_accum = 0.
        self._mass_props = {}
        self._update_water_mass()
        self._instruction = scene.instruction
        for _ in range(self.spec.get("settle_actions", 30)):
            self._env.step(self._ready)
        if self.render_images:
            for _ in range(4):
                self._base.sim.render()
        self._t = 0.0
        self.scorer.reset()
        if self._audit_error:
            raise RuntimeError("Material integration failed during reset") from self._audit_error
        self.metrics = self.scorer.evaluate(self.state())
        self._active = True
        obs = self._observation()
        if self.render_images and any(np.std(x) < 5 for x in obs.images.values()):
            raise RuntimeError("Blank native camera reset")
        return obs

    def state(self, light=False, include_bounds=True):
        result = {"time_s": self._t, "objects": {}}
        for s in self.spec["assets"]:
            if s["type"] == "base":
                continue
            o = self._base.scene[s["name"]]
            d = {
                "pose": array(o.root_view.get_transforms())[0].tolist(),
                "velocity": array(o.root_view.get_velocities())[0].tolist(),
            }
            if s["type"] == "articulation":
                d.update(
                    joints={
                        n: float(q)
                        for n, q in zip(o.joint_names, array(o.data.joint_pos)[0])
                    },
                    links={
                        n: p.tolist()
                        for n, p in zip(o.body_names, array(o.data.body_link_pose_w)[0])
                    },
                )
            if s['name'] in self._mass_props:
                d['native_mass_kg']=self._mass_props[s['name']][0]
                d['com_local']=self._mass_props[s['name']][1].tolist()
            result["objects"][s["name"]] = d
        if include_bounds:
            result["bounds"] = geometry.world_bounds(self._geometry, result)
        result['material'] = self.material.snapshot()
        if not light:
            from yam_arena.runtime import tcp_world

            result["hands"] = {}
            for n in ARMS:
                a = self._base.scene[n]
                q = array(a.data.joint_pos)[0]
                _ids, fingers = self._ids[n]
                result["hands"][n] = {
                    "tcp": tcp_world(a).tolist(),
                    "opening_m": float(-q[fingers].sum()),
                    "poses": array(a.data.body_link_pose_w)[0].tolist(),
                    "names": list(a.body_names),
                }
            result["robot_clearance_m"] = geometry.robot_clearance(
                result["bounds"], result["hands"]
            )
        return result

    def _task_step(self):
        if self._audit_error:
            raise RuntimeError("Physics audit failed") from self._audit_error
        self.metrics = self.scorer.evaluate(self.state())
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
        if self._callback:
            self._callback.deregister()
            self._callback = None
        if self._env is not None:
            self._env.close()
            self._env = self._base = None
        if self._temp:
            self._temp.cleanup()


def embodiment(**kwargs):
    return Environment(**kwargs)
