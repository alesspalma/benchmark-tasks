"""Full-sized worktop, hinged sealer, working rotary keeper, and a compliant pouch mouth."""

import numpy as np

from . import config as C
from .motion import rotation
from .usd_scene import Writer

DARK = (.129, .148, .160)
BLACK = (.049, .057, .062)
GRAY = (.317, .343, .359)
GASKET = (.080, .097, .094)
FILM = (.52, .70, .67)
FILM_RIM = (.63, .77, .73)
FOOD = (.64, .392, .166)


def joint(w, stage, kind, name, parent, child, pivot, axis, travel, *,
          friction, damping, stiffness=0., target=0.):
    schema = w.Phy.RevoluteJoint if kind == "angular" else w.Phy.PrismaticJoint
    j = schema.Define(stage, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([parent])
    j.CreateBody1Rel().SetTargets([child])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, pivot)))
    j.CreateAxisAttr(axis)
    j.CreateLowerLimitAttr(float(travel[0]))
    j.CreateUpperLimitAttr(float(travel[1]))
    w.Px.PhysxJointAPI.Apply(j.GetPrim()).CreateJointFrictionAttr(friction)
    drive = w.Phy.DriveAPI.Apply(j.GetPrim(), kind)
    drive.CreateTypeAttr("force")
    drive.CreateDampingAttr(damping)
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateTargetPositionAttr(target)


def station(w):
    stage = w.stage("counter")
    w.box(stage, "/Asset/Counter", (1.10, 1.48, .054),
          (.395, 0., C.TABLE-.027), (.63, .57, .45))
    w.box(stage, "/Asset/Backsplash", (.020, 1.75, .312), (.744, 0., .900),
          (.76, .78, .76), collide=False)
    w.box(stage, "/Asset/Floor", (6., 6., .024), (0., 0., -.012),
          (.35, .38, .37))
    return stage


def keeper_heel(w, stage, parent):
    """The cam's underside rises onto the lid flange, then bears on its flat."""
    # Profile in the locked orientation, measured from the cam axis. The
    # heel at x=10–12 mm carries the back of the closed lid's catch.
    profile = ((.002,.046), (.010,.0414), (.012,.0414),
               (.015,.047), (.015,.052), (.002,.052))
    rotate = rotation("Y", -np.radians(C.CAM_TRAVEL))
    points = [rotate @ [x, y, z] for y in (.002, .019) for x, z in profile]
    count = len(profile)
    faces = [list(range(count)), list(reversed(range(count, 2*count)))]
    faces.extend([count+i, count+(i+1)%count, (i+1)%count, i] for i in range(count))
    path = parent + "/Heel"
    mesh = w.Geom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([w.Gf.Vec3f(*p) for p in points])
    mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
    mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
    w._finish(stage, mesh, GRAY, True)
    w.records.append({"asset": "sealer", "solid": path, "kind": "prism",
                      "local_vertices": [p.tolist() for p in points]})


def sealer(w):
    stage = w.stage("sealer")
    root = w.body(stage, "/Asset/Chassis", 2.03, fixed=True, articulation=True)
    # The thin front ledge rises continuously from the counter to the heater strip.
    # A real open vacuum channel sits behind that strip, with a gasket around it.
    w.rounded_box(stage, root+"/Base", (.191, .386, .030),
                  (.493, 0., .7645), DARK, .019, .004)
    w.box(stage, root+"/FrontDeck", (.046, .351, .013), (.417, 0., .782),
          GRAY)
    w.box(stage, root+"/RearDeck", (.099, .367, .013), (.541, 0., .7835),
          DARK)
    w.box(stage, root+"/ChannelWell", (.054, .302, .003), (.465, 0., .7822),
          BLACK)
    for side in (-1., 1.):
        w.box(stage, root+f"/EndCap{side:g}", (.152, .043, .024),
              (.493, side*.172, .781), DARK)
        w.box(stage, root+f"/GasketEnd{side:g}", (.052, .008, .0040),
              (.466, side*.132, .7905), GASKET)
    w.box(stage, root+"/HeatingStrip", (.008, .266, .002), (.423, 0., .789),
          (.58, .388, .253))
    w.box(stage, root+"/GasketFront", (.004, .263, .003), (.438, 0., .7905),
          GASKET)
    w.box(stage, root+"/GasketRear", (.005, .264, .003), (.492, 0., .7905),
          GASKET)
    w.cylinder(stage, root+"/SuctionPort", .008, .001, (.481, .090, .784),
               GRAY, collide=False)
    lid = w.body(stage, "/Asset/Lid", .37, C.LID, ccd=True)
    w.rounded_box(stage, lid+"/Top", (.193, .386, .031), (-.080, 0., .011),
                  (.199, .240, .256), .022, .004)
    w.box(stage, lid+"/Face", (.011, .322, .018), (-.163, 0., .010), DARK)
    w.box(stage, lid+"/PressBar", (.009, .267, .002), (-.148, 0., -.007),
          GASKET)
    w.box(stage, lid+"/TroughSeal", (.005, .267, .002), (-.133, 0., -.006),
          GASKET)
    w.box(stage, lid+"/ControlEdge", (.031, .065, .006), (-.059, .122, .025),
          GRAY, collide=False)
    # Rear panel stays under the lid; use a front-accessible lid lip.
    w.box(stage, lid+"/LiftLip", (.015, .205, .009), (-.177, 0., .005), DARK)
    w.box(stage, lid+"/Catch", (.016, .037, .008), (-.164, -.206, .002), GRAY)
    for side in (-1., 1.):
        w.cylinder(stage, root+f"/HingeCover{side:g}", .016, .032,
                   C.LID+[0.,side*.166,0.], DARK, "Y", collide=False)
    joint(w, stage, "angular", "LidPivot", root, lid, C.LID, "Y", (0, C.LID_OPEN),
          friction=.28, damping=.0035, stiffness=.0085, target=95.)
    # Native side cam holds the lid flange against the heating strip.
    w.cylinder(stage, root+"/KeeperBearing", .014, .033, C.CAM+[0.,.0165,0.],
               DARK, "Y", collide=False)
    cam = w.body(stage, "/Asset/Keeper", .037, C.CAM, ccd=True)
    w.Px.PhysxRigidBodyAPI.Apply(stage.GetPrimAtPath(cam)).CreateDisableGravityAttr(True)
    w.cylinder(stage, cam+"/Spindle", .016, .024, (0., -.007, 0.), DARK, "Y")
    w.box(stage, cam+"/Arm", (.031, .020, .012), C.LATCH_ARM*.52,
          (.23, .27, .28))
    w.cylinder(stage, cam+"/Thumb", .010, .016, C.LATCH_ARM+[0.,-.007,0.],
               (.54,.63,.57), "Y")
    keeper_heel(w, stage, cam)
    joint(w, stage, "angular", "KeeperPivot", root, cam, C.CAM, "Y",
          (0., C.CAM_TRAVEL), friction=.11, damping=.0015)
    button = w.body(stage, "/Asset/CycleButton", .012, C.BUTTON, ccd=True)
    w.rounded_box(stage, button+"/Pad", (.022, .042, .006), (0.,0.,0.),
                  (.151, .392, .299), .007, .001)
    for i, y in enumerate((-.007, .007)):
        w.cylinder(stage, button+f"/Print{i}", .003, .0004, (-.004,y,.0032),
                   (.69, .88, .77), collide=False)
    joint(w, stage, "linear", "CycleTravel", lid, button, C.BUTTON-C.LID, "Z",
          (-.004,0), friction=.014, damping=1.8, stiffness=310.)
    return stage


def translucent(w, stage, obj, color, opacity):
    from pxr import Sdf
    material = w.Shade.Material.Define(stage, obj+"_Film")
    shader = w.Shade.Shader.Define(stage, obj+"_Film/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(w.Gf.Vec3f(*color))
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.42)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    w.Shade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(obj)).Bind(material)


def pouch(w):
    stage = w.stage("pouch")
    body = w.body(stage, "/Asset/Pouch", .205, articulation=True, ccd=True)
    # The filled area is short of the smooth grippable film around its mouth.
    w.rounded_box(stage, body+"/FilmBack", (.145,.190,.008), (-.049,0,0),
                  FILM_RIM, .018,.002)
    w.rounded_box(stage, body+"/DryLoad", (.091,.132,.024), (-.046,0,.016),
                  FOOD,.012,.003)
    for i in range(6):
        x, y = (-.071+.032*(i//3), -.040+.040*(i%3))
        w.rounded_box(stage, body+f"/Cracker{i}", (.029,.036,.006),
                      (x,y,.031), (.78,.52,.235), .004,.002, collide=False)
    w.rounded_box(stage, body+"/SealedFilm", (.147,.192,.037), (-.048,0,.018),
                  FILM, .023,.006, collide=False)
    translucent(w, stage, body+"/SealedFilm", FILM, .46)
    w.Geom.Imageable(stage.GetPrimAtPath(body+"/SealedFilm")).MakeInvisible()
    w.rounded_box(stage, body+"/LooseFilm", (.154,.196,.063), (-.050,0,.025),
                  FILM,.030,.007, collide=False)
    translucent(w, stage, body+"/LooseFilm", FILM, .34)
    neck = w.body(stage, "/Asset/Neck", .017, (C.MOUTH_PANEL_START,0,C.MOUTH_Z),
                  ccd=True)
    end = C.MOUTH_TIP - C.MOUTH_PANEL_START
    width = 2*(C.MOUTH_Y-C.WING)
    w.box(stage, neck+"/CentreFilm", (end, width, .004), (end/2,0.,0.), FILM)
    translucent(w, stage, neck+"/CentreFilm", FILM, .65)
    w.box(stage, neck+"/CentreEdge", (.006,width,.0015), (end-.003,0.,.0024),
          FILM_RIM, collide=False)
    w.box(stage, neck+"/HeatWeld", (.0027,width,.0006),
                 (.067,0.,.00255), (.72,.81,.78), collide=False)
    w.Geom.Imageable(stage.GetPrimAtPath(neck+"/HeatWeld")).MakeInvisible()
    joint(w, stage, "angular", "NeckPitch", body, neck, (C.MOUTH_PANEL_START,0,C.MOUTH_Z),
          "Y", (-17.,24.), friction=.035, damping=.0006, stiffness=.40, target=0.)
    for suffix, side in (("Left",1.),("Right",-1.)):
        edge_y = side*(C.MOUTH_Y-C.WING)
        wing = w.body(stage, "/Asset/"+suffix, .012,
                      (C.MOUTH_PANEL_START,edge_y,C.MOUTH_Z), ccd=True)
        y = side*C.WING/2.
        w.box(stage, wing+"/Panel", (end,C.WING,.004), (end/2,y,0.), FILM)
        translucent(w, stage, wing+"/Panel", FILM, .65)
        w.box(stage, wing+"/Edge", (.006,C.WING,.0014), (end-.003,y,.00245),
              FILM_RIM, collide=False)
        w.box(stage, wing+"/SideWeld", (end,.005,.002), (end/2,side*C.WING-.0025*side,.0025),
              FILM_RIM, collide=False)
        w.box(stage, wing+"/HeatWeld", (.0027,C.WING,.0006),
                     (.067,y,.00255), (.72,.81,.78), collide=False)
        w.Geom.Imageable(stage.GetPrimAtPath(wing+"/HeatWeld")).MakeInvisible()
        joint(w, stage, "angular", "Fold"+suffix, neck, wing,
              (0,edge_y,0), "X", (-38.,38.), friction=.018, damping=.00033,
              stiffness=.0008, target=side*C.INITIAL_WING)
    return stage


def create_assets(directory):
    w = Writer(directory)
    assets = [
        {"name":"counter", "type":"base", "file":w.save(station(w))},
        {"name":"sealer", "type":"articulation", "file":w.save(sealer(w)),
         "initial":{"joint_pos":{"LidPivot":np.radians(C.LID_OPEN),"KeeperPivot":0.,"CycleTravel":0.}}},
        {"name":"pouch", "type":"articulation", "file":w.save(pouch(w)),
         "position":C.POUCH_START,
         "initial":{"joint_pos":{"NeckPitch":0., "FoldLeft":np.radians(C.INITIAL_WING),
                                 "FoldRight":-np.radians(C.INITIAL_WING)}}},
    ]
    w.inventory()
    return {"assets":assets, "settle_actions":16}
