"""Read-only mechanical-fit checks, corresponding to photos and a 1 mm feeler."""
import math
import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix
from . import config as C
from .geometry import crown_surface_gap


PINNA_SURFACE=np.asarray([
    [.014*math.cos(lat)*math.cos(a),.009*math.cos(lat)*math.sin(a),.030*math.sin(lat)]
    for lat in np.linspace(-math.pi/2,math.pi/2,25)
    for a in np.arange(48)*math.tau/48
])


def fitted_side(links,side,label):
    pose=np.asarray(links[label+'Cup']);r=quat_xyzw_to_matrix(pose[3:]);p=pose[:3]
    ear=C.HEAD+np.array([.004,side*(C.HALF_HEAD_WIDTH+.005),C.EAR_HEIGHT])
    # Project the real fixed3D pinna, not a cup-aligned substitute ellipse.
    local=(PINNA_SURFACE+ear-p)@r
    enclosure=float(np.max((local[:,0]/C.CUP_INNER[0])**2+(local[:,2]/C.CUP_INNER[1])**2))
    gaps=[]
    for k in range(C.PAD_COUNT):
        q=np.asarray(links[f'{label}Pad{k:02}']);rot=quat_xyzw_to_matrix(q[3:])
        for t in (k*math.tau/C.PAD_COUNT,(k+.5)*math.tau/C.PAD_COUNT,(k+1)*math.tau/C.PAD_COUNT):
            for radii in (C.CUP_INNER,C.CUSHION_OUTER):
                v=np.array([radii[0]*math.cos(t),-side*C.CUSHION_FACE,radii[1]*math.sin(t)])
                world=q[:3]+rot@v
                gaps.append(side*(world[1]-C.HEAD[1])-C.HALF_HEAD_WIDTH)
    max_gap=max(gaps);min_gap=min(gaps)
    centered=bool(abs(p[0]-(C.HEAD[0]+.004))<=.010 and abs(p[2]-(C.HEAD[2]+C.EAR_HEIGHT))<=.010)
    normal=float(r[1,1])
    ok=bool(enclosure<=1 and max_gap<=.001 and min_gap>=-.0015 and normal>=.97 and centered)
    return {'fit':ok,'pinna_opening_fraction':enclosure,'max_cushion_gap_m':max_gap,'min_cushion_gap_m':min_gap,'cup_center_error_m':[float(p[0]-(C.HEAD[0]+.004)),float(p[2]-(C.HEAD[2]+C.EAR_HEIGHT))],'cup_normal_cosine':normal}


class Scorer:
    def __init__(self):self.reset()
    def reset(self):self.clear_since=None
    def observe(self,state):pass
    def evaluate(self,state):
        protector=state['objects']['protector'];links=protector['links'];t=float(state['time_s'])
        left=fitted_side(links,1,'Left');right=fitted_side(links,-1,'Right')
        root=np.asarray(links['Crown']);r=quat_xyzw_to_matrix(root[3:])
        crown_error=root[:3]-(C.HEAD+[0,0,C.CROWN_HEIGHT])
        support_gap=crown_surface_gap(root)
        contact_force=float(state.get('crown_contact_force_N',0.))
        crown=bool(abs(crown_error[0])<=.015 and abs(crown_error[1])<=.012 and r[2,2]>=math.cos(math.radians(14.)) and support_gap is not None and -.0015<=support_gap<=.0005)
        hands=state.get('hands',{});clear=len(hands)==2 and state.get('robot_clearance_m',0.)>=.003
        for h in hands.values():
            tcp=np.asarray(h['tcp'])
            if min(np.linalg.norm(tcp-np.asarray(links[x+'Cup'])[:3]) for x in ('Left','Right'))<.09:clear=False
            if np.linalg.norm(tcp-root[:3])<.09:clear=False
        stable=float(np.linalg.norm(protector['velocity'][:3]))<.015 and float(np.linalg.norm(protector['velocity'][3:]))<.15
        full=left['fit'] and right['fit'] and crown and clear and stable
        if full:
            if self.clear_since is None:self.clear_since=t
        else:self.clear_since=None
        success=bool(full and t-self.clear_since>=1.)
        n=int(left['fit'])+int(right['fit'])
        score=1. if success else .7 if n==2 and crown else .3 if n==1 and crown else 0.
        return {'score':score,'success':success,'left':left,'right':right,'crown_seated':crown,'crown_surface_gap_m':support_gap,'crown_contact_force_N':contact_force,'crown_error_m':crown_error.tolist(),'hands_clear':clear,'stable':stable,'joints':protector['joints'],'violations':[]}
