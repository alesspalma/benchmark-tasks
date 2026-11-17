"""Native passive earmuff and anatomical printable manikin geometry.

The headband's two compliant bending regions are rotational springs. Cup height
slides, pivots and sixteen independent cushion sectors per side remain passive.
Nothing is attached to the head or kinematically driven during a trial.
"""
import math
import numpy as np
from . import config as C
from .usd_scene import Writer

SKIN=(.61,.42,.30)
DARK=(.035,.043,.049)
GREEN=(.26,.55,.10)


def mesh(w,s,path,points,faces,color,collide=True,convex=False):
    from pxr import Gf,UsdGeom,UsdPhysics
    m=UsdGeom.Mesh.Define(s,path)
    m.CreatePointsAttr([Gf.Vec3f(*map(float,p)) for p in points])
    m.CreateFaceVertexCountsAttr([len(f) for f in faces])
    m.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    m.CreateSubdivisionSchemeAttr('none')
    w._finish(s,m,color,collide)
    if convex and collide: UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr('convexHull')
    w.records.append({'asset':str(s.GetRootLayer().identifier).split('/')[-1], 'solid':path,'kind':'mesh','points':points,'faces':faces,'color':color,'collide':collide})
    return m


def oval_sector(w,s,path,outer,inner,depth,center,amin,amax,color):
    # Convex annular quadrilateral extruded normal to the side of the head.
    p=[]
    for dy in (-depth/2,depth/2):
        for rad,a in ((outer,amin),(outer,amax),(inner,amax),(inner,amin)):
            p.append([center[0]+rad[0]*math.cos(a),center[1]+dy,center[2]+rad[1]*math.sin(a)])
    faces=[[0,3,2,1],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]
    return mesh(w,s,path,p,faces,color,convex=True)


def ellipsoid(w,s,path,radii,center,color,clip_y=None,collide=True):
    p=[];faces=[];n=48;m=24
    for j in range(m+1):
        lat=-math.pi/2+math.pi*j/m
        for k in range(n):
            lon=math.tau*k/n
            q=np.array([radii[0]*math.cos(lat)*math.cos(lon),radii[1]*math.cos(lat)*math.sin(lon),radii[2]*math.sin(lat)])
            if clip_y is not None:q[1]=np.clip(q[1],-clip_y,clip_y)
            p.append((q+center).tolist())
    for j in range(m):
        for k in range(n):
            a=j*n+k;b=j*n+(k+1)%n;c=(j+1)*n+(k+1)%n;d=(j+1)*n+k
            faces.append([a,b,c,d])
    # The fixed manikin uses its exact triangle surface; convex cooking would
    # shrink its flat temporal zones and invalidate millimetre gap checks.
    return mesh(w,s,path,p,faces,color,collide=collide,convex=collide and not path.startswith('/Asset/Head/'))


def joint(w,s,path,parent,child,kind,axis,anchor0,anchor1=(0,0,0),limits=(-90,90),k=0,d=0,friction=0,reverse=False):
    from pxr import Gf,UsdPhysics,PhysxSchema
    cls=UsdPhysics.RevoluteJoint if kind=='angular' else UsdPhysics.PrismaticJoint
    j=cls.Define(s,path);j.CreateBody0Rel().SetTargets([parent]);j.CreateBody1Rel().SetTargets([child]);j.CreateAxisAttr(axis)
    j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,anchor0)));j.CreateLocalPos1Attr(Gf.Vec3f(*map(float,anchor1)))
    j.CreateLowerLimitAttr(limits[0]);j.CreateUpperLimitAttr(limits[1]);j.CreateCollisionEnabledAttr(False)
    if reverse:
        q=Gf.Quatf(0,Gf.Vec3f(1,0,0) if axis in ('Y','Z') else Gf.Vec3f(0,0,1));j.CreateLocalRot0Attr(q);j.CreateLocalRot1Attr(q)
    PhysxSchema.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(friction)
    if k or d:
        scale=math.pi/180 if kind=='angular' else 1.
        drive=UsdPhysics.DriveAPI.Apply(j.GetPrim(),kind);drive.CreateTypeAttr('force');drive.CreateStiffnessAttr(k*scale);drive.CreateDampingAttr(d*scale);drive.CreateTargetPositionAttr(0.)
    return j


def create_assets(directory):
    from pxr import Gf,UsdGeom,UsdPhysics,UsdShade
    w=Writer(directory);s=w.stage('hearing_protector')
    mat=UsdPhysics.MaterialAPI.Get(s,'/Asset/Physics');mat.GetStaticFrictionAttr().Set(.85);mat.GetDynamicFrictionAttr().Set(.65)
    root=w.body(s,'/Asset/Crown',.016,articulation=True)
    from pxr import PhysxSchema
    PhysxSchema.PhysxArticulationAPI.Get(s,root).GetEnabledSelfCollisionsAttr().Set(True)
    PhysxSchema.PhysxContactReportAPI.Apply(s.GetPrimAtPath(root)).CreateThresholdAttr(0.)
    w.box(s,root+'/Band',(0.024,.066,.006),(0,0,.003),DARK)
    for side,label in ((1,'Left'),(-1,'Right')):
        anchor=np.array([0,side*C.ARM_ANCHOR,0.])
        arm=w.body(s,'/Asset/'+label+'Band',.020,position=anchor)
        # Two coated spring-steel wire bands, represented by compound segments.
        for wire in (-1,1):
            for i in range(9):
                t0=i/9;t1=(i+1)/9
                a=np.array([wire*.008,side*(.026*math.sin(math.pi*t0/2)+.018*math.sin(math.pi*t0)),-.110*t0])
                b=np.array([wire*.008,side*(.026*math.sin(math.pi*t1/2)+.018*math.sin(math.pi*t1)),-.110*t1])
                v=b-a;length=np.linalg.norm(v)
                z=v/length;x=np.array([1.,0.,0.]);y=np.cross(z,x)
                w.box(s,arm+f'/Wire{wire+1}_{i}',(.004,.004,length+.001),(a+b)/2,DARK,rotation=np.column_stack([x,y,z]))
        joint(w,s,'/Asset/'+label+'Elastic',root,arm,'angular','X',anchor,limits=(-5,45) if side==1 else (-45,5),k=C.BAND_K,d=C.BAND_DAMPING)
        end=np.array([0,side*.026,-.110]);position=anchor+end
        yoke=w.body(s,'/Asset/'+label+'Yoke',.005,position=position)
        joint(w,s,'/Asset/'+label+'Height',arm,yoke,'linear','Z',end,limits=(0,C.SLIDE_TRAVEL),d=2.,friction=C.SLIDE_FRICTION,reverse=True)
        for wire in (-1,1):w.box(s,yoke+f'/Rail{wire+1}',(.004,.005,.090),(wire*.008,side*.034,.030),(.15,.17,.18))
        # Passive preloaded polymer guide pads supply the real bushing's
        # normal load, so native contact friction holds the height rail.
        for jaw in (-1,1):
            grip_position=position+np.array([jaw*.0125,side*.034,.025])
            guide=w.body(s,f'/Asset/{label}Bearing{jaw+1}',.001,position=grip_position)
            w.box(s,guide+'/Pad',(.005,.012,.025),(0,0,0),DARK)
            j=w.slide(s,f'/Asset/{label}BearingSpring{jaw+1}',arm,guide,.004,1000.,.0025,axis='X',reverse=jaw>0)
            j.CreateLocalPos0Attr(Gf.Vec3f(*map(float,end+[jaw*.0125,side*.034,.025])))
            j.CreateLocalPos1Attr(Gf.Vec3f(0,0,0))
        cup=w.body(s,'/Asset/'+label+'Cup',.049,position=position)
        joint(w,s,'/Asset/'+label+'Tilt',yoke,cup,'angular','X',(0,0,0),limits=(-50,50),k=.025,d=.003,friction=.002)
        # Rigid ABS cup: oval back and circumferential shell, actual cavity.
        ellipsoid(w,s,cup+'/Back',(.039,.004,.051),(0,side*.020,0),GREEN)
        for k in range(24):
            oval_sector(w,s,cup+f'/Shell{k:02}',C.CUP_OUTER,(.034,.046),.032,(0,side*.002,0),k*math.tau/24,(k+1)*math.tau/24,GREEN)
        for k in range(C.PAD_COUNT):
            pad=w.body(s,f'/Asset/{label}Pad{k:02}',.0005,position=position)
            # Pad compression travels outward, away from the head.
            joint(w,s,f'/Asset/{label}Foam{k:02}',cup,pad,'linear','Y',(0,0,0),limits=(0,C.PAD_TRAVEL),k=C.PAD_K,d=.7,reverse=side<0)
            oval_sector(w,s,pad+'/Cushion',C.CUSHION_OUTER,C.CUP_INNER,C.CUSHION_DEPTH,(0,-side*.018,0),k*math.tau/C.PAD_COUNT,(k+1)*math.tau/C.PAD_COUNT,DARK)
    # Cushion sectors represent one connected foam annulus. Filter their
    # internal boundaries and the shell/yoke embedded in that same assembly;
    # opposite cups and nonadjacent band pieces retain native collisions.
    for label in ('Left','Right'):
        pads=[f'/Asset/{label}Pad{k:02}' for k in range(C.PAD_COUNT)]
        family=pads+[f'/Asset/{label}Cup',f'/Asset/{label}Yoke',f'/Asset/{label}Band']
        for path in family:
            api=UsdPhysics.FilteredPairsAPI.Apply(s.GetPrimAtPath(path))
            api.CreateFilteredPairsRel().SetTargets([p for p in family if p!=path])
        guides=[f'/Asset/{label}Bearing{k}' for k in (0,2)]
        # The bushing is outside the earcup back; only its wire-rail contact
        # supplies sliding friction. Embedded cup/yoke connections are ideal.
        for path in guides:
            api=UsdPhysics.FilteredPairsAPI.Apply(s.GetPrimAtPath(path))
            api.CreateFilteredPairsRel().SetTargets([x for x in family if x!=f'/Asset/{label}Yoke'])
    # Finite rubber-finger contact patches resist rolling about the pinch
    # axis on the existing cup sidewalls; no added grasp geometry.
    for prim in s.Traverse():
        if 'Cup/Shell' in str(prim.GetPath()):
            contact=PhysxSchema.PhysxCollisionAPI.Apply(prim)
            contact.CreateTorsionalPatchRadiusAttr(.006)
            contact.CreateMinTorsionalPatchRadiusAttr(.004)
    assets=[{'name':'protector' ,'type':'articulation','file':w.save(s),'position':C.RESET.tolist()}]
    s=w.stage('headform')
    head=UsdGeom.Xform.Define(s,'/Asset/Head');head.AddTranslateOp().Set(Gf.Vec3d(*C.HEAD))
    w.box(s,'/Asset/Head/Foot',(.19,.20,.014),(0,0,.007),(.22,.24,.26))
    ellipsoid(w,s,'/Asset/Head/Neck',(.046,.044,.070),(0,0,.072),SKIN)
    # Adult skull, 190 mm front-to-back, 145 mm across the temporal contact
    # surfaces. The flat temporal fit zones follow standard earmuff test plates.
    ellipsoid(w,s,'/Asset/Head/Skull',(.095,.088,.120),(0,0,C.CROWN_HEIGHT-.120),SKIN,clip_y=C.HALF_HEAD_WIDTH)
    ellipsoid(w,s,'/Asset/Head/Jaw',(.073,.063,.068),(-.016,0,.157),SKIN)
    ellipsoid(w,s,'/Asset/Head/Nose',(.024,.016,.026),(-.097,0,.218),SKIN)
    for side,label in ((1,'Left'),(-1,'Right')):
        ear=np.array([.004,side*(C.HALF_HEAD_WIDTH+.005),C.EAR_HEIGHT])
        ellipsoid(w,s,'/Asset/Head/'+label+'Pinna',(.014,.009,.030),ear,SKIN)
        ellipsoid(w,s,'/Asset/Head/'+label+'Concha',(.007,.001,.016),ear+[0,side*.008,0],(.40,.25,.20),collide=False)
        ellipsoid(w,s,'/Asset/Head/'+label+'Eye',(.003,.010,.004),(-.087,side*.031,.245),(.08,.075,.070),collide=False)
        ellipsoid(w,s,'/Asset/Head/'+label+'Brow',(.003,.019,.003),(-.085,side*.031,.256),(.20,.15,.12),collide=False)
    ellipsoid(w,s,'/Asset/Head/Mouth',(.002,.022,.002),(-.086,0,.170),(.35,.22,.19),collide=False)
    w.box(s,'/Asset/StorageBase',(.13,.20,.012),(.26,0,C.TOP+.006),(.20,.24,.27))
    w.box(s,'/Asset/StoragePost',(.012,.012,.205),(.26,0,C.TOP+.1145),(.20,.24,.27))
    w.box(s,'/Asset/StorageCrossbar',(.060,.040,.010),(.26,0,C.TOP+.215),(.20,.24,.27))
    assets.append({'name':'headform','type':'base','file':w.save(s)})
    w.inventory()
    return {'assets':assets,'grains':[],'settle_actions':35}
