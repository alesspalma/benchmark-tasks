"""Native hinged cover, cord flexure and snap-position wall switch."""
import numpy as np
from yam_arena.urdf import axis_angle_matrix

from . import config as C
from .usd_scene import Writer

DECK = (.42, .31, .23)
RIM = (.27, .30, .29)
LID = (.36, .39, .37)
GASKET = (.10, .12, .11)
CABLE = (.94, .43, .068)
STEEL = (.48, .50, .50)
OUTLET = (.75, .75, .69)


def station(w):
    s = w.stage("station")
    w.box(s, "/Asset/Floor", (4., 4., .02), (.45, 0., -.01), (.32, .35, .33))
    for i, y in enumerate((-.305, .305)):
        w.box(s, f"/Asset/Pedestal{i}", (.19, .18, .75), (-.04, y, .375), (.28, .31, .29))
    for i, y in enumerate(np.arange(-.64, .65, .105)):
        if abs(y) > .13:
            w.box(s, f"/Asset/DeckBoard{i}", (.94, .103, .031),
                  (.686, y, C.FLOOR-.016), DECK if i % 2 else (.47, .35, .25))
        else:
            w.box(s, f"/Asset/DeckBoardBack{i}", (.678, .103, .031),
                  (.817, y, C.FLOOR-.016), DECK)
            w.box(s, f"/Asset/DeckBoardFront{i}", (.054, .103, .031),
                  (.233, y, C.FLOOR-.016), DECK)
    w.box(s, "/Asset/Wall", (.033, 1.40, 1.1), (.802, 0., 1.09), (.66, .64, .57), collide=False)
    for i in range(9):
        w.box(s, f"/Asset/Siding{i}", (.006, 1.38, .006), (.781, 0., .77 + i*.08),
              (.52, .51, .46), collide=False)
    return s


def hinge(w, s, path, root, child, p, axis, low, high, friction, stiffness=0., initial=0.):
    joint = w.Phy.RevoluteJoint.Define(s, path)
    joint.CreateBody0Rel().SetTargets([root])
    joint.CreateBody1Rel().SetTargets([child])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, p)))
    radians = np.radians(initial) / 2
    vector = np.zeros(3)
    vector["XYZ".index(axis)] = np.sin(radians)
    orient = w.Gf.Quatd(float(np.cos(radians)), w.Gf.Vec3d(*map(float, vector)))
    w.Geom.Xformable(s.GetPrimAtPath(child)).GetOrderedXformOps()[1].Set(orient)
    joint.CreateLocalRot0Attr(w.Gf.Quatf(orient))
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(low)
    joint.CreateUpperLimitAttr(high)
    w.Px.PhysxJointAPI.Apply(joint.GetPrim()).CreateJointFrictionAttr(friction)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateDampingAttr(.018)
    drive.CreateStiffnessAttr(stiffness)
    return joint


def cable_leg(w, s, path, a, b):
    a, b = np.array(a), np.array(b)
    vector = b-a
    length = np.linalg.norm(vector)
    direction = vector/length
    across = np.cross([0., 0., 1.], direction)
    across /= np.linalg.norm(across)
    rotation = np.column_stack((across, direction, np.cross(across, direction)))
    w.box(s, path, (.0086, length+.001, .0086), (a+b)/2, CABLE, rotation)


def outlet(w):
    s = w.stage("outlet")
    root = w.body(s, "/Asset/Box", 1., fixed=True, articulation=True)
    x = C.CENTER_X
    w.rounded_box(s, root+"/OuterFlange", (.211, .178, .008),
                  (x, 0., .722), RIM, .011, .0012, collide=False)
    w.box(s, root+"/RainPan", (.189,.133,.010), (x,0.,.688), RIM)
    for label, xx in (("Nose", .275), ("Rear", .461)):
        w.box(s, root+"/"+label, (.008,.143,.048), (xx,0.,.718), RIM)
        w.box(s, root+"/"+label+"Seal", (.009,.143,.003), (xx,0.,.743), GASKET)
    w.box(s, root+"/RightRim", (.188,.009,.043), (x,-.071,.719), RIM)
    w.box(s, root+"/RightSeal", (.188,.009,.003), (x,-.071,.742), GASKET)
    for i,(lo,hi) in enumerate(((.274,.372),(.399,.462))):
        w.box(s, root+f"/LeftRim{i}", (hi-lo,.009,.041), ((lo+hi)/2,.071,.720), RIM)
        w.box(s, root+f"/LeftSeal{i}", (hi-lo,.009,.003), ((lo+hi)/2,.071,.742), GASKET)
    # Cord-retention roof and outside drip-loop seat, left of the receptacle.
    w.box(s, root+"/CableHookStem", (.008,.024,.025), (.398,.049,.725), RIM)
    w.box(s, root+"/CableHook", (.021,.024,.005), (.3915,.049,.739), RIM)
    w.box(s, root+"/LoopFloor", (.029,.038,.008), (.385,.069,.714), RIM)
    w.box(s, root+"/Receptacle", (.064,.057,.009), (.388,-.031,.724), OUTLET)
    for i, dx in enumerate((-.019,.021)):
        for j, dy in enumerate((-.008,.008)):
            w.box(s, root+f"/Socket{i}_{j}", (.009,.003,.001), (.388+dx,-.031+dy,.729),
                  GASKET, collide=False)
    w.rounded_box(s, root+"/SeatedPlug", (.038,.028,.024),
                  (.385,-.034,.737), (.08,.09,.085), .004,.001, collide=False)
    for i,y in enumerate((-.079,.079)):
        w.cylinder(s, root+f"/HingePin{i}", .006,.023,(.467,y,.748), STEEL, "Y", False)
    p = C.HINGE
    lid = w.body(s, "/Asset/Lid", .145, p, ccd=True)
    w.rounded_box(s, lid+"/Cover", (.191,.151,.010), (-.093,0.,.006),
                  LID, .007,.0014)
    w.box(s, lid+"/FrontLip", (.008,.136,.009), (-.180,0.,-.002), LID)
    w.box(s, lid+"/RightSkirt", (.168,.007,.009), (-.094,-.069,-.001), LID)
    for i,(lo,hi) in enumerate(((-.178,-.095),(-.068,-.010))):
        w.box(s, lid+f"/LeftSkirt{i}", (hi-lo,.010,.011), ((lo+hi)/2,.069,-.001), LID)
    for i in range(8):
        w.box(s, lid+f"/MoldedTread{i}", (.005,.119,.001), (-.162+i*.019,0.,.0115),
              (.39,.42,.40), collide=False)
    w.box(s, lid+"/LiftLip", (.016,.050,.007), (-.185, .015, .004), LID)
    hinge(w,s,"/Asset/CoverHinge",root,lid,p,"Y",-C.LID_OPEN,0.,.024,initial=C.LID_OPEN)
    cord = w.body(s, "/Asset/Cord", .058, C.CABLE_PIVOT, ccd=True)
    for i, (a,b) in enumerate(zip(C.CABLE_POINTS, C.CABLE_POINTS[1:])):
        cable_leg(w,s,cord+f"/Rubber{i}",a,b)
    hinge(w,s,"/Asset/CordFlex",root,cord,C.CABLE_PIVOT,"Z",-33.-C.CABLE_OPEN,1.-C.CABLE_OPEN,.005,
          stiffness=.042,initial=C.CABLE_OPEN)
    return s


def light(w):
    s = w.stage("light")
    root = w.body(s, "/Asset/LightStand", 3., fixed=True, articulation=True)
    p = C.SWITCH_PIVOT
    w.box(s, root+"/PatioPost", (.077,.072,.63), (.504,-.207,1.035), (.40,.31,.23))
    w.box(s, root+"/SwitchPlate", (.013,.057,.098), p+[.017,0.,0.], (.68,.68,.59))
    lever = w.body(s, "/Asset/Toggle", .012, p)
    w.rounded_box(s, lever+"/Lever", (.015,.030,.025), (-.015,0.,0.),
                  (.31,.34,.32), .003,.001)
    joint = w.Phy.PrismaticJoint.Define(s, "/Asset/LightSwitch")
    joint.CreateBody0Rel().SetTargets([root])
    joint.CreateBody1Rel().SetTargets([lever])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, p)))
    joint.CreateAxisAttr("X")
    joint.CreateLowerLimitAttr(0.)
    joint.CreateUpperLimitAttr(.006)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateDampingAttr(3.)
    drive.CreateStiffnessAttr(230.)
    w.cylinder(s, root+"/LampCap", .045,.017,(.460,-.207,1.32), GASKET, collide=False)
    w.cylinder(s, root+"/LampGlass", .031,.051,(.460,-.207,1.285), (.39,.41,.39), collide=False)
    w.cylinder(s, root+"/LampFloor", .039,.014,(.460,-.207,1.255), GASKET, collide=False)
    return s


def create_assets(directory):
    w = Writer(directory)
    data = [
        {"name": "station", "type": "base", "file": w.save(station(w))},
        {"name": "outlet", "type": "articulation", "file": w.save(outlet(w)),
         "joints": {"CoverHinge": 0., "CordFlex": 0.}},
        {"name": "lamp_post", "type": "articulation", "file": w.save(light(w)),
         "joints": {"LightSwitch": 0.}},
    ]
    w.inventory()
    return {"assets": data, "settle_actions": 14}
