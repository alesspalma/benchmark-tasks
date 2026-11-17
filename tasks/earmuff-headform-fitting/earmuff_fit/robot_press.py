"""Ordinary position-target preload using unchanged official YAM gains/limits."""
from functools import lru_cache
import xml.etree.ElementTree as ET
import numpy as np
from yam_arena import ik, constants, station, urdf

GAIN=np.array([80.,80.,80.,10.,10.,10.])
LIMIT=np.array([27.,27.,27.,7.,7.,7.])


@lru_cache(maxsize=1)
def masses():
    rows=[]
    for link in ET.parse(ik.YAM_URDF).getroot().findall('link'):
        inertial=link.find('inertial')
        if inertial is None: continue
        mass=float(inertial.find('mass').attrib['value'])
        origin=inertial.find('origin')
        xyz=np.fromstring(origin.attrib.get('xyz','0 0 0') if origin is not None else '0 0 0',sep=' ')
        rows.append((link.attrib['name'],mass,xyz))
    return rows


def fk(q):
    mapped=dict(zip(constants.ARM_JOINTS,q))
    mapped.update(dict.fromkeys(constants.FINGER_JOINTS,constants.FINGER_OPEN))
    return urdf.forward_kinematics(ik.YAM_URDF,mapped)


def gravity(q):
    def energy(q):
        transforms=fk(q)
        return sum(mass*9.81*(transforms[name][:3,:3]@point+transforms[name][:3,3])[2] for name,mass,point in masses())
    eps=1e-5
    result=[]
    for i in range(6):
        a=np.array(q);b=a.copy();a[i]+=eps;b[i]-=eps
        result.append((energy(a)-energy(b))/(2*eps))
    return np.asarray(result)


def contact_jacobian(q,toe_local):
    def point(q):
        tcp,rot=ik.tcp_pose(q,station.BASES['right'])
        return tcp+rot@toe_local
    eps=1e-5;p=point(q)
    result=np.zeros((3,6))
    for i in range(6):
        a=np.array(q);a[i]+=eps
        result[:,i]=(point(a)-p)/eps
    return result


def preload(q,toe_local,force_N):
    return preload_vector(q,toe_local,np.array([0.,0.,-force_N]))


def preload_vector(q,toe_local,force_world):
    tau_gravity=gravity(q)
    tau_press=contact_jacobian(q,toe_local).T@np.asarray(force_world)
    tau=tau_gravity+tau_press
    assert np.all(abs(tau)<LIMIT), (tau,LIMIT)
    return np.asarray(q)+tau_press/GAIN,{'existing_native_gravity_feedforward':True,'gravity_torque_Nm':tau_gravity.tolist(),'press_torque_Nm':tau_press.tolist(),'sum_torque_Nm':tau.tolist(),'requested_downward_force_N':float(-np.asarray(force_world)[2]),'force_world_N':np.asarray(force_world).tolist()}
