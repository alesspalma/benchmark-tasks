"""Original native hollow glassware, free bar and ordinary central-grip restrainer."""
import math
import numpy as np
from . import config as C
from .usd_scene import Writer


def material(w,s,name,color,opacity):
    from pxr import Sdf
    mat=w.Shade.Material.Define(s,'/Asset/'+name)
    sh=w.Shade.Shader.Define(s,'/Asset/'+name+'/Shader');sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(*color))
    sh.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(.15)
    sh.CreateInput('opacity',Sdf.ValueTypeNames.Float).Set(opacity)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(),'surface')
    return mat


def beaker(w,name):
    s=w.stage(name);p=w.body(s,'/Asset/Body',C.DRY_MASS,ccd=True)
    mass=w.Phy.MassAPI.Get(s,p);mass.CreateCenterOfMassAttr(w.Gf.Vec3f(*C.DRY_COM))
    mass.CreateDiagonalInertiaAttr(w.Gf.Vec3f(*np.diag(C.DRY_INERTIA)))
    glass=material(w,s,'Glass',(.70,.85,.90),.20)
    w.cylinder(s,p+'/Floor',C.R_OUT,C.FLOOR,(0,0,C.FLOOR/2),(.70,.85,.90))
    for k in range(64):
        a=(k+.5)*math.tau/64;radius=(C.R_OUT+C.R_IN)/2
        # Native thin tangential panels form a genuine hollow open vessel.
        r=np.array([[math.cos(a),-math.sin(a),0],[math.sin(a),math.cos(a),0],[0,0,1]])
        w.box(s,p+f'/Wall{k:02}',(C.R_OUT-C.R_IN,2*C.R_OUT*math.tan(math.pi/64)+.00012,C.HEIGHT-C.FLOOR),
              (radius*math.cos(a),radius*math.sin(a),(C.HEIGHT+C.FLOOR)/2),(.70,.85,.90),rotation=r)
    # A small outward pouring lip; the open cylinder remains the flow section.
    w.box(s,p+'/PourLip',(.010,.014,.002),(-C.R_OUT-.002,0,C.HEIGHT-.001),(.70,.85,.90))
    for prim in s.Traverse():
        if prim.HasAPI(w.Phy.CollisionAPI):w.Shade.MaterialBindingAPI.Apply(prim).Bind(glass)
    m=w.Geom.Mesh.Define(s,p+'/Liquid');m.CreateSubdivisionSchemeAttr('none')
    m.CreatePointsAttr([]);m.CreateFaceVertexCountsAttr([]);m.CreateFaceVertexIndicesAttr([])
    blue=material(w,s,'Water',(.02,.35,.78),.62);w.Shade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(blue)
    # Graduation ticks are cosmetic ordinary labware markings, not answers.
    for ml in (50,100,150,200,250):
        z=C.FLOOR+ml*1e-6/(math.pi*C.R_IN**2)
        w.box(s,p+f'/Tick{ml}',(.0003,.008,.0006),(-C.R_OUT-.00025,0,z),(.9,.93,.95),collide=False)
    return s


def create_assets(directory):
    w=Writer(directory);assets=[]
    for name,pos in [('source',C.SOURCE),('receiver',C.RECEIVER)]:
        s=beaker(w,name);mat=w.Phy.MaterialAPI.Get(s,'/Asset/Physics')
        mat.GetStaticFrictionAttr().Set(.65);mat.GetDynamicFrictionAttr().Set(.5)
        assets.append(dict(name=name,type='rigid',file=w.save(s),position=pos.tolist()))
    s=w.stage('bar');p=w.body(s,'/Asset/Body',C.BAR_MASS,ccd=True)
    w.cylinder(s,p+'/PTFE',C.BAR_RADIUS,C.BAR_LENGTH,(0,0,0),(.95,.96,.94),axis='X')
    mat=w.Phy.MaterialAPI.Get(s,'/Asset/Physics');mat.GetStaticFrictionAttr().Set(.10);mat.GetDynamicFrictionAttr().Set(.08)
    w.Px.PhysxMaterialAPI.Apply(mat.GetPrim()).CreateFrictionCombineModeAttr('min')
    assets.append(dict(name='bar',type='rigid',file=w.save(s),position=C.BAR.tolist()))
    s=w.stage('restrainer');p=w.body(s,'/Asset/Body',.070,ccd=True)
    blue=(.035,.20,.68)
    w.box(s,p+'/MagneticFace',(.080,.025,.010),(0,0,0),blue)
    w.cylinder(s,p+'/Stem',.008,.027,(0,0,.0185),blue)
    w.cylinder(s,p+'/ThumbDisk',.016,.007,(0,0,.0355),blue)
    assets.append(dict(name='restrainer',type='rigid',file=w.save(s),position=C.TOOL.tolist()))
    s=w.stage('fixtures')
    w.tray(s,'/Asset/CatchTray',(.55,.58),.025,.004,(.42,.47,.51),wall=.005)
    from pxr import UsdGeom,Gf
    xf=UsdGeom.Xform.Define(s,'/Asset/CatchTray');xf.AddTranslateOp().Set(Gf.Vec3d(.36,.03,C.TOP))
    for name,pos,dims in [('SourceMat',C.SOURCE,(.10,.10,.004)),('ReceiverMat',C.RECEIVER,(.10,.10,.004)),('ToolMat',C.TOOL,(.18,.16,.004))]:
        w.box(s,'/Asset/'+name,dims,(float(pos[0]),float(pos[1]),C.TOP+.006),(.16,.19,.21))
    stream=UsdGeom.Mesh.Define(s,'/Asset/Stream');stream.CreateSubdivisionSchemeAttr('none')
    stream.CreatePointsAttr([]);stream.CreateFaceVertexCountsAttr([]);stream.CreateFaceVertexIndicesAttr([])
    blue=material(w,s,'StreamWater',(.02,.35,.78),.80);w.Shade.MaterialBindingAPI.Apply(stream.GetPrim()).Bind(blue)
    assets.append(dict(name='fixtures',type='base',file=w.save(s),position=[0,0,0]))
    w.inventory()
    return dict(assets=assets,settle_actions=35)
