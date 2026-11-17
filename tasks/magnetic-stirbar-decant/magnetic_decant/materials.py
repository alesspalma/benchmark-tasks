"""Force-only magnetic coupling and conservative free-surface water transport.

The bar/tool remain native free rigid bodies. Water is an analytic material:
gravity defines its free surface, a weir law emits volume through the low lip,
ballistic flight determines collection, and native mass/COM/inertia follow it.
No controller phase, target pose, capture latch or task-object pose write occurs.
"""
import math
import numpy as np
from . import config as C


def dipoles(p1,R1,p2,R2):
    """Return bar force and intrinsic bar/tool torques from a softened potential."""
    # The encapsulated tool core is magnetized through its face (local Z);
    # the cylindrical stirring bar is magnetized along its length (local X).
    m1=R1[:,2]*C.TOOL_MOMENT;m2=R2[:,0]*C.BAR_MOMENT
    r=np.asarray(p2)-p1;s2=float(r@r+C.MAG_SOFTENING**2)
    s3=s2**1.5;s5=s2**2.5;s7=s2**3.5;k=1e-7
    a,b=float(m1@r),float(m2@r)
    force=3*k*((m1@m2)*r+a*m2+b*m1)/s5-15*k*a*b*r/s7
    field1=k*(3*r*a/s5-m1/s3)
    field2=k*(3*r*b/s5-m2/s3)
    return force,np.cross(m2,field1),np.cross(m1,field2)


class CylinderWater:
    def __init__(self):
        r=C.R_IN*np.sqrt((np.arange(16)+.5)/16)
        a=(np.arange(64)+.5)*math.tau/64
        self.xy=np.column_stack([(r[:,None]*np.cos(a)).ravel(),(r[:,None]*np.sin(a)).ravel()])
        self.area=math.pi*C.R_IN**2/len(self.xy)
        self.capacity=math.pi*C.R_IN**2*(C.HEIGHT-C.FLOOR)

    def columns(self,p,R,h):
        zbase=p[2]+self.xy@R[2,:2];c=R[2,2]
        if abs(c)<1e-6:
            low=np.full(len(zbase),C.FLOOR);high=np.where(zbase<=h,C.HEIGHT,C.FLOOR)
        else:
            cut=np.clip((h-zbase)/c,C.FLOOR,C.HEIGHT)
            low=np.full(len(zbase),C.FLOOR) if c>0 else cut
            high=cut if c>0 else np.full(len(zbase),C.HEIGHT)
        return low,high

    def volume(self,p,R,h):
        lo,hi=self.columns(p,R,h);return float((hi-lo).sum()*self.area)

    def level(self,p,R,volume):
        spread=C.R_IN*float(np.linalg.norm(R[2,:2]))
        low=p[2]+min(R[2,2]*C.FLOOR,R[2,2]*C.HEIGHT)-spread-.00001
        high=p[2]+max(R[2,2]*C.FLOOR,R[2,2]*C.HEIGHT)+spread+.00001
        for _ in range(23):
            mid=(low+high)/2
            if self.volume(p,R,mid)<volume:low=mid
            else:high=mid
        return (low+high)/2

    def properties(self,p,R,volume):
        h=self.level(p,R,volume);lo,hi=self.columns(p,R,h)
        weights=(hi-lo)*self.area*1000
        z=(lo+hi)/2;z2=(lo*lo+lo*hi+hi*hi)/3
        points=np.column_stack([self.xy,z]);mass=C.DRY_MASS+float(weights.sum())
        center=(C.DRY_MASS*C.DRY_COM+(weights[:,None]*points).sum(0))/mass
        second=(points*weights[:,None]).T@points
        second[2,2]=float(weights@z2)
        iwater=np.eye(3)*np.trace(second)-second
        dryshift=C.DRY_MASS*(np.eye(3)*(C.DRY_COM@C.DRY_COM)-np.outer(C.DRY_COM,C.DRY_COM))
        inertia=C.DRY_INERTIA+dryshift+iwater-mass*(np.eye(3)*(center@center)-np.outer(center,center))
        return h,mass,center,inertia

    def lip(self,p,R):
        a=np.arange(96)*math.tau/96
        points=np.column_stack([C.R_IN*np.cos(a),C.R_IN*np.sin(a),np.full(96,C.HEIGHT)])
        points=np.vstack([points,[-C.R_OUT-.007,0,C.HEIGHT]])
        world=points@R.T+p
        return world[np.argmin(world[:,2])]


def bar_envelope(source,bar):
    """Full cylinder extent inside the source frame, irrespective of bar axis."""
    from yam_arena.urdf import quat_xyzw_to_matrix
    sr=quat_xyzw_to_matrix(np.asarray(source[3:]));br=sr.T@quat_xyzw_to_matrix(np.asarray(bar[3:]))
    b=sr.T@(np.asarray(bar[:3])-source[:3])
    z_extent=abs(br[2,0])*C.BAR_LENGTH/2+C.BAR_RADIUS*np.sqrt(max(0.,1-br[2,0]**2))
    angles=np.arange(16)*np.pi/8
    rings=np.array([[end*C.BAR_LENGTH/2,C.BAR_RADIUS*np.cos(a),C.BAR_RADIUS*np.sin(a)]
                    for end in (-1,1) for a in angles])@br.T+b
    return b,float(b[2]-z_extent),float(b[2]+z_extent),float(np.max(np.linalg.norm(rings[:,:2],axis=1)))


def capture(state):
    from yam_arena.urdf import quat_xyzw_to_matrix
    source=state['objects']['source']['pose'];bar=state['objects']['bar']['pose'];tool=state['objects']['restrainer']['pose']
    sr=quat_xyzw_to_matrix(np.asarray(source[3:]));tr=quat_xyzw_to_matrix(np.asarray(tool[3:]))
    b,low,high,radial=bar_envelope(source,bar);t=sr.T@(np.asarray(tool[:3])-source[:3])
    distance=float(np.linalg.norm(np.asarray(bar[:3])-(np.asarray(tool[:3])+tr@C.MAGNET_OFFSET)))
    # Observable centimetre-scale bar-at-wall/tool-outside relationship, not
    # a field-strength threshold. Actual retention still comes from forces.
    return bool(radial>=C.R_IN-.005 and low>=C.FLOOR-.002 and high<=C.HEIGHT+.003 and
                np.linalg.norm(t[:2])>=C.R_OUT-.002 and distance<.030)


class MaterialModel:
    def __init__(self):
        self.shape=CylinderWater();self.reset()

    def reset(self):
        self.volumes={'source':C.VOLUME,'receiver':0.};self.spilled=0.
        self.captured_delivered=0.;self.last_stream=None;self.clock=0.
        self.packets=[];self.segments=[];self.wet_robot=0.
        self.max_force=0.;self.last_force=np.zeros(3);self.levels={}

    def _first_hit(self,old,new,state):
        from yam_arena.urdf import quat_xyzw_to_matrix
        from .geometry import segment_robot_hit
        events=[]
        for name in ('source','receiver'):
            pose=state['objects'][name]['pose'];R=quat_xyzw_to_matrix(np.asarray(pose[3:]));p=np.asarray(pose[:3])
            a=R.T@(old-p);b=R.T@(new-p);delta=b-a
            if a[2]>C.HEIGHT and b[2]<=C.HEIGHT:
                f=(C.HEIGHT-a[2])/delta[2];point=a+f*delta
                if np.linalg.norm(point[:2])<C.R_IN-.001:
                    events.append((float(f),'collect',name))
            qa=float(delta[:2]@delta[:2]);qb=2*float(a[:2]@delta[:2]);qc=float(a[:2]@a[:2]-C.R_IN**2)
            roots=np.roots([qa,qb,qc]) if qa>1e-14 else []
            for root in roots:
                if abs(root.imag)<1e-8 and 0<root.real<=1:
                    f=float(root.real);z=a[2]+f*delta[2]
                    if C.FLOOR<z<C.HEIGHT:events.append((f,'spill',name))
            if a[2]>C.FLOOR and b[2]<=C.FLOOR and abs(delta[2])>1e-12:
                f=(C.FLOOR-a[2])/delta[2]
                if np.linalg.norm((a+f*delta)[:2])<C.R_OUT:events.append((float(f),'spill',name))
        hit=segment_robot_hit(old,new,state.get('hands',{}))
        if hit is not None:events.append((hit,'robot',None))
        if old[2]>C.TOP+.004 and new[2]<=C.TOP+.004:
            events.append(((C.TOP+.004-old[2])/(new[2]-old[2]),'spill',None))
        return min(events,key=lambda e:e[0]) if events else None

    def transport(self,state,dt):
        from yam_arena.urdf import quat_xyzw_to_matrix
        self.clock+=dt;self.segments=[];remaining=[]
        for packet in self.packets:
            old=packet['position'];new=old+packet['velocity']*dt+np.array([0,0,-4.905*dt*dt])
            hit=self._first_hit(old,new,state)
            end=new if hit is None else old+hit[0]*(new-old)
            self.segments.append((old,end,packet['volume']))
            if hit is None and packet['age']<2:
                packet['position']=new;packet['velocity']+=np.array([0,0,-9.81*dt]);packet['age']+=dt;remaining.append(packet)
            elif hit is not None and hit[1]=='collect':
                name=hit[2];received=min(packet['volume'],self.shape.capacity-self.volumes[name]);self.volumes[name]+=received
                self.spilled+=packet['volume']-received
                if name=='receiver':self.captured_delivered+=received*packet['eligible_fraction']
            else:
                self.spilled+=packet['volume']
                if hit is not None and hit[1]=='robot':self.wet_robot+=packet['volume']
        self.packets=remaining
        for name in ('source','receiver'):
            volume=self.volumes[name]
            if volume<1e-10:continue
            obj=state['objects'][name];p=np.asarray(obj['pose'][:3]);R=quat_xyzw_to_matrix(np.asarray(obj['pose'][3:]))
            h=self.shape.level(p,R,volume);self.levels[name]=h;lip=self.shape.lip(p,R);head=max(0.,h-lip[2])
            available=max(0.,volume-self.shape.volume(p,R,lip[2]))
            emitted=min(available,(2/3)*.65*.018*math.sqrt(2*9.81)*head**1.5*dt)
            if emitted<=1e-10:continue
            eligible=float(capture(state)) if name=='source' else min(1.,self.captured_delivered/max(volume,1e-12))
            self.volumes[name]-=emitted
            if name=='receiver':self.captured_delivered=max(0.,self.captured_delivered-emitted*eligible)
            axis=R[:,2];out=lip-(p+axis*C.HEIGHT);out/=max(np.linalg.norm(out),1e-9)
            direction=axis+out;direction/=max(np.linalg.norm(direction),1e-9)
            com=p+R@np.asarray(obj.get('com_local',C.DRY_COM))
            velocity=np.asarray(obj['velocity'][:3])+np.cross(obj['velocity'][3:],lip-com)+direction*math.sqrt(2*9.81*head)
            self.packets.append({'position':lip.copy(),'velocity':velocity,'volume':emitted,'eligible_fraction':eligible,'age':0.})
        return self.snapshot()

    def snapshot(self):
        return {'source_ml':self.volumes['source']*1e6,'receiver_ml':self.volumes['receiver']*1e6,
                'spilled_ml':self.spilled*1e6,'captured_delivered_ml':self.captured_delivered*1e6,
                'in_flight_ml':sum(p['volume'] for p in self.packets)*1e6,
                'wet_robot_ml':self.wet_robot*1e6,
                'mass_balance_ml':(sum(self.volumes.values())+self.spilled+sum(p['volume'] for p in self.packets))*1e6,
                'magnetic_force_N':self.last_force.tolist(),'maximum_magnetic_force_N':self.max_force}

    def water_mesh(self,p,R,volume):
        from scipy.spatial import ConvexHull
        if volume<1e-9:return [],[]
        h=self.shape.level(p,R,volume);n=48
        points=np.array([[C.R_IN*math.cos(k*math.tau/n),C.R_IN*math.sin(k*math.tau/n),z]
                         for z in (C.FLOOR,C.HEIGHT) for k in range(n)])
        z=points@R[2,:]+p[2]-h;inside=z<=0;clipped=points[inside].tolist()
        edges=[(k,(k+1)%n) for k in range(n)]+[(n+k,n+(k+1)%n) for k in range(n)]+[(k,n+k) for k in range(n)]
        for i,j in edges:
            if inside[i]!=inside[j]:clipped.append((points[i]+(points[j]-points[i])*(-z[i])/(z[j]-z[i])).tolist())
        pts=np.asarray(clipped)
        if len(pts)<4:return [],[]
        hull=ConvexHull(pts);faces=[];center=pts.mean(0)
        for f in hull.simplices:
            a,b,c=pts[f]
            if np.dot(np.cross(b-a,c-a),(a+b+c)/3-center)<0:f=f[::-1]
            faces.append(f.tolist())
        return pts.tolist(),faces
