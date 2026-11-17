"""Ordinary bimanual fitting phases, initialized from the current run only.

No recorded states, object writes, fixture actions or altered gains/effort limits.
Pose/contact reads are for the scripted reference; model observations stayRGB/joints.
"""
import numpy as np
from yam_arena import ik, station, constants, urdf
from . import config as C
from .geometry import robot_hulls, crown_surface_gap
from .motion import SIDE_Y, rotation
from .robot_press import preload, preload_vector

ROTATION=rotation('Y',np.deg2rad(25.))@rotation('Z',-np.pi/2)@SIDE_Y


def _toe(arm,q_seed):
    tcp=np.array([.48,-.05 if arm=='right' else .05,1.135])
    q,pe,re=ik.solve(tcp,ik.matrix_to_quat_xyzw(ROTATION),q_seed,base_pos=station.BASES[arm],iters=120)
    if pe>.0002 or re>.001:raise RuntimeError('Fitting toe calibration unreachable')
    mapped=dict(zip(constants.ARM_JOINTS,q));mapped.update(dict.fromkeys(constants.FINGER_JOINTS,constants.FINGER_OPEN))
    name='tip_left' if arm=='right' else 'tip_right'
    transform=urdf.forward_kinematics(ik.YAM_URDF,mapped)[name]
    points=np.concatenate(robot_hulls()[name])@transform[:3,:3].T+transform[:3,3]+station.BASES[arm]-tcp
    toe=points[points[:,0]>points[:,0].max()-.008]
    sole=toe[toe[:,2]<toe[:,2].min()+.001]
    return sole.mean(0)


class _Stage:
    def __init__(self,env,info,joints,previous):
        self.info=info
        self.start_state=env.state()
        self.start_previous=np.asarray(previous).copy()
        self.initial={}
        for name,ids in [('left',slice(0,6)),('right',slice(7,13))]:
            hand=self.start_state['hands'][name+'_robot']
            pose=hand['poses'][hand['names'].index('gripper')]
            self.initial[name]=(np.asarray(joints)[ids].copy(),np.asarray(hand['tcp']),urdf.quat_xyzw_to_matrix(np.asarray(pose[3:])))
        self.rotation=ROTATION
        self.sole=_toe(self.PRESS_ARM,self.initial[self.PRESS_ARM][0])
        self.reset(None)
        self.native_feedback=env


class RightPressStage(_Stage):
    PRESS_ARM='right'
    def reset(self, scene):
        self.q = {n: self.initial[n][0].copy() for n in self.initial}
        self.previous = self.start_previous.copy()
        self.integral = np.zeros(14)
        self.duration = 20.5
        self.phase = 'fit component ready'
        self.worst_ik_m = self.worst_ik_rad = 0.
        self.native_feedback = None
        self.lift_start = None
        self.approach = None
        self.force_since = None
        self.left_grasp=None
        self.left_raise_tcp=None
        self.left_achieved=None
        self.left_clear_q=None
        self.cup_z_history=[]
        self.cup_brake_horizon=.15
        self.cup_admittance_force=0.
        self.left_rot=rotation('Z',np.pi)@SIDE_Y
        self.left_cup_initial=np.asarray(self.start_state['objects']['protector']['links']['LeftCup'])[:3]
        self.left_outside=self.left_cup_initial+[0,.035,.015]
        self.left_outside_q,pe,re=ik.solve(self.left_outside,ik.matrix_to_quat_xyzw(self.left_rot),self.initial['left'][0],base_pos=station.BASES['left'],iters=160)
        assert pe<.002 and re<.03

    def contact_tcp(self, state, pressing):
        root = np.asarray(state['objects']['protector']['links']['Crown'])
        band_rotation = urdf.quat_xyzw_to_matrix(root[3:])
        # Centre the real toe on the band's front half, avoiding a heel press.
        point = root[:3] + band_rotation @ np.array([-.005, 0., .006])
        tcp = point - self.sole
        if pressing:
            gap = crown_surface_gap(root)
            tcp[2] -= min(.022, max(0., gap-.0001)+.003)
        return tcp

    def action_at(self, t, joint_pos=None):
        assert self.native_feedback is not None
        state = self.native_feedback.state(light=True)
        if self.approach is None:
            self.approach = self.contact_tcp(state,False)+[0,0,.035]
        if t <= 4.5:
            s=ik.smoothstep(t/4.5)
            pos=self.initial['right'][1]+s*(self.approach-self.initial['right'][1])
            rot=ik.slerp_matrix(self.initial['right'][2],self.rotation,s)
            self.phase='fit orient toe clear above band'
        elif t <= 6.5:
            s=ik.smoothstep((t-4.5)/2.)
            pos=self.approach+s*(self.contact_tcp(state,True)-self.approach)
            rot=self.rotation
            self.phase='fit ordinary fingertip press'
        elif t <= 16.:
            pos=self.contact_tcp(state,True);rot=self.rotation
            self.phase='fit hold real crown contact'
        elif t <= 17.:
            if self.lift_start is None:
                self.lift_start=np.asarray(self.native_feedback.state()['hands']['right_robot']['tcp'])
            pos=self.lift_start+np.array([0,0,.030])*ik.smoothstep(t-16.);rot=self.rotation
            self.phase='fit lift finger and release'
        else:
            start=self.lift_start+np.array([0,0,.030])
            s=ik.smoothstep((t-17.)/2.)
            pos=start+s*(np.array([.30,-.24,1.07])-start)
            rot=ik.slerp_matrix(self.rotation,rotation('Z',-np.pi/2)@SIDE_Y,s)
            self.phase='fit hands clear; passive support'
        q,pe,re=ik.solve(pos,ik.matrix_to_quat_xyzw(rot),self.q['right'],base_pos=station.BASES['right'],iters=90)
        assert pe<.002 and re<.03, (pe,re)
        force_mode=False
        contact=float(state['crown_total_contact_force_N'])
        if 4. <= t <= 16. and joint_pos is not None and (contact>.2 or self.force_since is not None):
            if self.force_since is None:self.force_since=t
            requested=(3.+13.*ik.smoothstep((t-self.force_since)/.8))*(1.-ik.smoothstep((t-14.)/2.))
            actual_q=np.asarray(joint_pos)[7:13]
            actual_tcp=np.asarray(self.native_feedback.state()['hands']['right_robot']['tcp'])
            aligned=self.contact_tcp(state,False);aligned[2]=actual_tcp[2]
            equilibrium,eq_pe,eq_re=ik.solve(aligned,ik.matrix_to_quat_xyzw(self.rotation),actual_q,base_pos=station.BASES['right'],iters=90)
            assert eq_pe<.002 and eq_re<.03
            loaded,report=preload(actual_q,self.rotation.T@self.sole,requested)
            q=equilibrium+(loaded-actual_q)
            self.native_feedback._fit_requested_preload=report
            self.integral[7:13]=0.
            force_mode=True
            self.phase='fit bounded16N preload; gradual16to0N unload' if t>14. else 'fit bounded16N ordinary joint-target preload'
        else:
            self.native_feedback._fit_requested_preload=None
        self.q['right']=q
        left_grip=1.
        cup=np.asarray(state['objects']['protector']['links']['LeftCup'])[:3]
        self.cup_z_history.append(float(cup[2]))
        cup_vertical_speed=(self.cup_z_history[-1]-self.cup_z_history[-2])/.05 if len(self.cup_z_history)>1 else 0.
        cup_up_speed=max(0.,cup_vertical_speed)
        if t<=4.5:
            left_q=self.initial['left'][0]+ik.smoothstep(t/4.5)*(self.left_outside_q-self.initial['left'][0])
        else:
            if self.left_grasp is None:
                self.left_grasp=cup+np.array([0,.004,.002])
            if t<=6.:
                p=self.left_outside+ik.smoothstep((t-4.5)/1.5)*(self.left_grasp-self.left_outside)
            elif t<=7.:
                p=self.left_grasp;left_grip=1.-.45*ik.smoothstep(t-6.)
            elif t<=12.:
                left_grip=.55
                if self.left_raise_tcp is None:
                    self.left_raise_tcp=np.asarray(self.native_feedback.state()['hands']['left_robot']['tcp'])
                    self.left_raise_start=cup[2]
                error=C.HEAD[2]+C.EAR_HEIGHT-cup[2]
                if abs(error)<=.0003 and abs(cup_vertical_speed)<.001 and self.left_achieved is None:
                    self.left_achieved=np.asarray(self.native_feedback.state()['hands']['left_robot']['tcp'])
                p=self.left_achieved.copy() if self.left_achieved is not None else self.left_raise_tcp.copy()
                if self.left_achieved is None:p[2]+=.002+np.clip(4.*error,-.010,.010)
            elif t<=13.:
                if self.left_achieved is None:self.left_achieved=np.asarray(self.native_feedback.state()['hands']['left_robot']['tcp'])
                p=self.left_achieved;left_grip=.55+.45*ik.smoothstep(t-12.)
            elif t<=14.:
                p=self.left_achieved+np.array([0,.035,.01])*ik.smoothstep(t-13.)
            else:
                if self.left_clear_q is None:self.left_clear_q=self.q['left'].copy()
                left_q=self.left_clear_q+ik.smoothstep((t-14.)/4.5)*(self.initial['left'][0]-self.left_clear_q)
                p=None
            if t<=14.:
                left_q,lpe,lre=ik.solve(p,ik.matrix_to_quat_xyzw(self.left_rot),self.q['left'],base_pos=station.BASES['left'],iters=100)
                assert lpe<.002 and lre<.03,(lpe,lre,t)
        left_force_mode=False
        if 7. < t <= 12. and self.left_achieved is None and joint_pos is not None:
            q_actual=np.asarray(joint_pos)[:6]
            measured=self.native_feedback.state()['hands']['left_robot']
            measured_tcp=np.asarray(measured['tcp'])
            gripper_pose=measured['poses'][measured['names'].index('gripper')]
            measured_R=urdf.quat_xyzw_to_matrix(np.asarray(gripper_pose[3:]))
            local=measured_R.T@(cup-measured_tcp)
            desired_speed=float(np.clip((C.HEAD[2]+C.EAR_HEIGHT-cup[2]-cup_vertical_speed*self.cup_brake_horizon)*2.,-.004,.004))
            self.cup_admittance_force=float(np.clip(self.cup_admittance_force+2000.*(desired_speed-cup_vertical_speed)*.05,-16.,16.))
            force=self.cup_admittance_force
            loaded,left_report=preload_vector(q_actual,local,np.array([0.,0.,force]))
            self.cup_brake_horizon=float(np.max(abs(loaded-q_actual))/(.04*20.)*.5)
            aligned=self.left_raise_tcp.copy();aligned[2]=measured_tcp[2]
            equilibrium,lpe,lre=ik.solve(aligned,ik.matrix_to_quat_xyzw(self.left_rot),q_actual,base_pos=station.BASES['left'],iters=100)
            assert lpe<.002 and lre<.03
            left_q=equilibrium+loaded-q_actual
            self.integral[:6]=0.
            left_force_mode=True
            self.native_feedback._fit_left_preload=left_report
        else:self.native_feedback._fit_left_preload=None
        self.q['left']=left_q
        self.native_feedback._fit_cup_raise={'cup_label':'LeftCup','adjustment_arm':'left','cup_height_m':float(cup[2]),'initial_height_m':getattr(self,'left_raise_start',None),'hold_pose_set':self.left_achieved is not None,'braking_horizon_s':self.cup_brake_horizon,'upward_speed_m_per_s':cup_up_speed,'measured_goal_reached':abs(cup[2]-(C.HEAD[2]+C.EAR_HEIGHT))<.001,'grip_target':left_grip}
        target=np.r_[left_q,left_grip,q,1.]
        if joint_pos is not None:
            error=target-np.asarray(joint_pos);error[[6,13]]=0
            if force_mode:error[7:13]=0.
            if left_force_mode:error[:6]=0.
            self.integral=np.clip(self.integral+.6*.05*error,-.30,.30)
            target+=self.integral
        target=np.clip(target,self.info.action_space.low,self.info.action_space.high)
        result=self.previous+np.clip(target-self.previous,-.04,.04)
        self.previous=result
        self.worst_ik_m=max(self.worst_ik_m,pe);self.worst_ik_rad=max(self.worst_ik_rad,re)
        return result


class LeftPressStage(_Stage):
    PRESS_ARM='left'
    def reset(self, scene):
        self.q = {n: self.initial[n][0].copy() for n in self.initial}
        self.previous = self.start_previous.copy()
        self.integral = np.zeros(14)
        self.duration = 20.5
        self.phase = 'fit component ready'
        self.worst_ik_m = self.worst_ik_rad = 0.
        self.native_feedback = None
        self.lift_start = None
        self.approach = None
        self.force_since = None
        self.left_grasp=None
        self.left_raise_tcp=None
        self.left_achieved=None
        self.left_clear_q=None
        self.cup_z_history=[]
        self.cup_brake_horizon=.15
        self.cup_admittance_force=0.
        self.left_rot=SIDE_Y
        self.left_cup_initial=np.asarray(self.start_state['objects']['protector']['links']['RightCup'])[:3]
        self.left_outside=self.left_cup_initial+[0,-.035,.015]
        self.left_outside_q,pe,re=ik.solve(self.left_outside,ik.matrix_to_quat_xyzw(self.left_rot),self.initial['right'][0],base_pos=station.BASES['right'],iters=160)
        assert pe<.002 and re<.03

    def contact_tcp(self, state, pressing):
        root = np.asarray(state['objects']['protector']['links']['Crown'])
        band_rotation = urdf.quat_xyzw_to_matrix(root[3:])
        # Centre the real toe on the band's front half, avoiding a heel press.
        point = root[:3] + band_rotation @ np.array([-.005, 0., .006])
        tcp = point - self.sole
        if pressing:
            gap = crown_surface_gap(root)
            tcp[2] -= min(.022, max(0., gap-.0001)+.003)
        return tcp

    def action_at(self, t, joint_pos=None):
        assert self.native_feedback is not None
        state = self.native_feedback.state(light=True)
        if self.approach is None:
            self.approach = self.contact_tcp(state,False)+[0,0,.035]
        if t <= 4.5:
            s=ik.smoothstep(t/4.5)
            pos=self.initial['left'][1]+s*(self.approach-self.initial['left'][1])
            rot=ik.slerp_matrix(self.initial['left'][2],self.rotation,s)
            self.phase='fit orient toe clear above band'
        elif t <= 6.5:
            s=ik.smoothstep((t-4.5)/2.)
            pos=self.approach+s*(self.contact_tcp(state,True)-self.approach)
            rot=self.rotation
            self.phase='fit ordinary fingertip press'
        elif t <= 16.:
            pos=self.contact_tcp(state,True);rot=self.rotation
            self.phase='fit hold real crown contact'
        elif t <= 17.:
            if self.lift_start is None:
                self.lift_start=np.asarray(self.native_feedback.state()['hands']['left_robot']['tcp'])
            pos=self.lift_start+np.array([0,0,.030])*ik.smoothstep(t-16.);rot=self.rotation
            self.phase='fit lift finger and release'
        else:
            start=self.lift_start+np.array([0,0,.030])
            s=ik.smoothstep((t-17.)/2.)
            pos=start+s*(np.array([.30,.24,1.07])-start)
            rot=ik.slerp_matrix(self.rotation,rotation('Z',-np.pi/2)@SIDE_Y,s)
            self.phase='fit hands clear; passive support'
        q,pe,re=ik.solve(pos,ik.matrix_to_quat_xyzw(rot),self.q['left'],base_pos=station.BASES['left'],iters=90)
        assert pe<.002 and re<.03, (pe,re)
        force_mode=False
        contact=float(state['crown_total_contact_force_N'])
        if 4. <= t <= 16. and joint_pos is not None and (contact>.2 or self.force_since is not None):
            if self.force_since is None:self.force_since=t
            requested=(3.+13.*ik.smoothstep((t-self.force_since)/.8))*(1.-ik.smoothstep((t-14.)/2.))
            actual_q=np.asarray(joint_pos)[:6]
            actual_tcp=np.asarray(self.native_feedback.state()['hands']['left_robot']['tcp'])
            aligned=self.contact_tcp(state,False);aligned[2]=actual_tcp[2]
            equilibrium,eq_pe,eq_re=ik.solve(aligned,ik.matrix_to_quat_xyzw(self.rotation),actual_q,base_pos=station.BASES['left'],iters=90)
            assert eq_pe<.002 and eq_re<.03
            loaded,report=preload(actual_q,self.rotation.T@self.sole,requested)
            q=equilibrium+(loaded-actual_q)
            self.native_feedback._fit_requested_preload=report
            self.integral[:6]=0.
            force_mode=True
            self.phase='fit bounded16N preload; gradual16to0N unload' if t>14. else 'fit bounded16N ordinary joint-target preload'
        else:
            self.native_feedback._fit_requested_preload=None
        self.q['left']=q
        left_grip=1.
        cup=np.asarray(state['objects']['protector']['links']['RightCup'])[:3]
        self.cup_z_history.append(float(cup[2]))
        cup_vertical_speed=(self.cup_z_history[-1]-self.cup_z_history[-2])/.05 if len(self.cup_z_history)>1 else 0.
        cup_up_speed=max(0.,cup_vertical_speed)
        if t<=4.5:
            left_q=self.initial['right'][0]+ik.smoothstep(t/4.5)*(self.left_outside_q-self.initial['right'][0])
        else:
            if self.left_grasp is None:
                self.left_grasp=cup+np.array([0,-.004,.002])
            if t<=6.:
                p=self.left_outside+ik.smoothstep((t-4.5)/1.5)*(self.left_grasp-self.left_outside)
            elif t<=7.:
                p=self.left_grasp;left_grip=1.-.45*ik.smoothstep(t-6.)
            elif t<=12.:
                left_grip=.55
                if self.left_raise_tcp is None:
                    self.left_raise_tcp=np.asarray(self.native_feedback.state()['hands']['right_robot']['tcp'])
                    self.left_raise_start=cup[2]
                error=C.HEAD[2]+C.EAR_HEIGHT-cup[2]
                if abs(error)<=.0003 and abs(cup_vertical_speed)<.001 and self.left_achieved is None:
                    self.left_achieved=np.asarray(self.native_feedback.state()['hands']['right_robot']['tcp'])
                p=self.left_achieved.copy() if self.left_achieved is not None else self.left_raise_tcp.copy()
                if self.left_achieved is None:p[2]+=.002+np.clip(4.*error,-.010,.010)
            elif t<=13.:
                if self.left_achieved is None:self.left_achieved=np.asarray(self.native_feedback.state()['hands']['right_robot']['tcp'])
                p=self.left_achieved;left_grip=.55+.45*ik.smoothstep(t-12.)
            elif t<=14.:
                p=self.left_achieved+np.array([0,-.035,.01])*ik.smoothstep(t-13.)
            else:
                if self.left_clear_q is None:self.left_clear_q=self.q['right'].copy()
                left_q=self.left_clear_q+ik.smoothstep((t-14.)/4.5)*(self.initial['right'][0]-self.left_clear_q)
                p=None
            if t<=14.:
                left_q,lpe,lre=ik.solve(p,ik.matrix_to_quat_xyzw(self.left_rot),self.q['right'],base_pos=station.BASES['right'],iters=100)
                assert lpe<.002 and lre<.03,(lpe,lre,t)
        left_force_mode=False
        if 7. < t <= 12. and self.left_achieved is None and joint_pos is not None:
            q_actual=np.asarray(joint_pos)[7:13]
            measured=self.native_feedback.state()['hands']['right_robot']
            measured_tcp=np.asarray(measured['tcp'])
            gripper_pose=measured['poses'][measured['names'].index('gripper')]
            measured_R=urdf.quat_xyzw_to_matrix(np.asarray(gripper_pose[3:]))
            local=measured_R.T@(cup-measured_tcp)
            desired_speed=float(np.clip((C.HEAD[2]+C.EAR_HEIGHT-cup[2]-cup_vertical_speed*self.cup_brake_horizon)*2.,-.004,.004))
            self.cup_admittance_force=float(np.clip(self.cup_admittance_force+2000.*(desired_speed-cup_vertical_speed)*.05,-16.,16.))
            force=self.cup_admittance_force
            loaded,left_report=preload_vector(q_actual,local,np.array([0.,0.,force]))
            self.cup_brake_horizon=float(np.max(abs(loaded-q_actual))/(.04*20.)*.5)
            aligned=self.left_raise_tcp.copy();aligned[2]=measured_tcp[2]
            equilibrium,lpe,lre=ik.solve(aligned,ik.matrix_to_quat_xyzw(self.left_rot),q_actual,base_pos=station.BASES['right'],iters=100)
            assert lpe<.002 and lre<.03
            left_q=equilibrium+loaded-q_actual
            self.integral[7:13]=0.
            left_force_mode=True
            self.native_feedback._fit_left_preload=left_report
        else:self.native_feedback._fit_left_preload=None
        self.q['right']=left_q
        self.native_feedback._fit_cup_raise={'cup_label':'RightCup','adjustment_arm':'right','cup_height_m':float(cup[2]),'initial_height_m':getattr(self,'left_raise_start',None),'hold_pose_set':self.left_achieved is not None,'braking_horizon_s':self.cup_brake_horizon,'upward_speed_m_per_s':cup_up_speed,'measured_goal_reached':abs(cup[2]-(C.HEAD[2]+C.EAR_HEIGHT))<.001,'grip_target':left_grip}
        target=np.r_[q,1.,left_q,left_grip]
        if joint_pos is not None:
            error=target-np.asarray(joint_pos);error[[6,13]]=0
            if force_mode:error[:6]=0.
            if left_force_mode:error[7:13]=0.
            self.integral=np.clip(self.integral+.6*.05*error,-.30,.30)
            target+=self.integral
        target=np.clip(target,self.info.action_space.low,self.info.action_space.high)
        result=self.previous+np.clip(target-self.previous,-.04,.04)
        self.previous=result
        self.worst_ik_m=max(self.worst_ik_m,pe);self.worst_ik_rad=max(self.worst_ik_rad,re)
        return result


class FittingSequence:
    """Three bounded alternating stages; scoring may finish earlier on valid fit."""
    def __init__(self,env,info,joints,previous,held=False):
        self.env=env;self.info=info;self.held=held
        self.types=(RightPressStage,LeftPressStage,RightPressStage)
        self.index=0;self.start=0.;self.duration=13.5 if held else 3*20.5
        self.stage=self.types[0](env,info,joints,previous)
        self.previous=np.asarray(previous).copy()
        self.phase='fitting';self.worst_ik_m=0.;self.worst_ik_rad=0.
    def action_at(self,t,joints):
        if not self.held and t-self.start>self.stage.duration and self.index<2:
            self.start+=self.stage.duration;self.index+=1
            self.stage=self.types[self.index](self.env,self.info,joints,self.previous)
        result=self.stage.action_at(t-self.start,joints)
        self.previous=result
        self.phase=f'{self.index+1}/3 '+self.stage.phase
        self.worst_ik_m=max(self.worst_ik_m,self.stage.worst_ik_m)
        self.worst_ik_rad=max(self.worst_ik_rad,self.stage.worst_ik_rad)
        return result
