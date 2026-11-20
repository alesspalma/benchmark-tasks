"""Generate tabletop, native contact surfaces and material-state geometry."""

from math import cos, pi, sin

from . import config as C
from .motion import rotation
from .usd_scene import Writer

FILM = (.83, .75, .87)
FOLD = (.72, .57, .75)
STEEL = (.57, .60, .64)
CUP = (.12, .31, .37)
ICING = (.91, .44, .47)


def prismatic(w, s, name, body0, body1, axis, anchor, limits, stiffness, damping):
    joint = w.Phy.PrismaticJoint.Define(s, "/Asset/" + name)
    joint.CreateBody0Rel().SetTargets([body0])
    joint.CreateBody1Rel().SetTargets([body1])
    joint.CreateLocalPos0Attr(w.Gf.Vec3f(*anchor))
    joint.CreateAxisAttr(axis)
    joint.CreateLowerLimitAttr(limits[0])
    joint.CreateUpperLimitAttr(limits[1])
    joint.CreateCollisionEnabledAttr(False)
    drive = w.Phy.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(stiffness)
    drive.CreateDampingAttr(damping)
    drive.CreateTargetPositionAttr(0.)
    drive.CreateMaxForceAttr(36.)


def cone_wall(w, s, path, bottom, top, dimensions, color, collide=True):
    """Shell from independently convex sectors, leaving the center open."""
    outer, inner = dimensions
    for n in range(12):
        theta = 2 * pi * n / 12
        angles = (theta, theta + 2 * pi / 12)
        points = [
            w.Gf.Vec3f(rx * cos(a), ry * sin(a), height)
            for height, j in ((bottom, 0), (top, 1))
            for rad in (outer, inner)
            for a in angles
            for rx, ry in [rad[j]]
        ]
        faces = [[0, 1, 3, 2], [4, 6, 7, 5], [0, 4, 5, 1],
                 [2, 3, 7, 6], [0, 2, 6, 4], [1, 5, 7, 3]]
        mesh = w.Geom.Mesh.Define(s, f"{path}/Panel{n:02d}")
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexCountsAttr([4] * len(faces))
        mesh.CreateFaceVertexIndicesAttr([i for face in faces for i in face])
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.CreateDoubleSidedAttr(True)
        if collide:
            w.Phy.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr("convexHull")
        w._finish(s, mesh, color, collide)


def scene(w):
    s = w.stage("table")
    w.box(s, "/Asset/Bench", (1.11, 1.25, .06), (.32, 0., C.TABLE - .03), (.54, .43, .30))
    w.box(s, "/Asset/Floor", (8., 8., .02), (0., 0., -.01), (.33, .36, .35))
    for x in (0., .70):
        for y in (-.54, .54):
            w.box(s, f"/Asset/Leg_{int(x*100)}_{int((y+1)*100)}",
                  (.06, .06, C.TABLE-.06), (x, y, (C.TABLE-.06)/2), (.19, .20, .19))
    position = C.SPATULA + C.TOOL_START_R @ [-.028, 0., 0.]
    position[2] = C.TABLE + .002
    w.box(s, "/Asset/RestBed", (.117, .072, .004), position, (.18, .43, .34),
          C.TOOL_START_R)
    return s


def cup(w):
    s = w.stage("cup")
    root = w.body(s, "/Asset/Cup", .34, ccd=True)
    w.cylinder(s, root + "/Base", .050, .010, (0., 0., .005), CUP)
    cone_wall(w, s, root, .009, C.CUP_HEIGHT,
              (((.050, .050), (.057, .057)), ((.046, .046), (.053, .053))), CUP)
    return s


def bag(w):
    s = w.stage("bag")
    root = w.body(s, "/Asset/Bagshell", .045, articulation=True, ccd=True)
    # The folded bag has a tapered catch pocket open at the top. The surfaces
    # below the live cuff approximate the folds of the supported film.
    cone_wall(w, s, root, -.150, -.028,
              (((.012, .014), (.057, .036)), ((.010, .012), (.054, .033))), FILM)
    w.cylinder(s, root + "/Coupler", .0135, .021, (0., 0., -.154), (.83, .83, .80))
    w.cylinder(s, root + "/Tip", .006, .015, (0., 0., -.170), STEEL)
    for name, direction, limits in (("Near", -1, (0., .029)), ("Far", 1, (-.029, 0.))):
        anchor = (0., direction * C.CUFF_HALF_GAP, 0.)
        link = w.body(s, f"/Asset/{name}", .010, anchor, ccd=True)
        w.rounded_box(s, link+"/Fold", (C.CUFF_HALF_LENGTH*2, C.CUFF_THICKNESS, C.CUFF_HEIGHT),
                      (0., 0., 0.), FOLD, .002, .0004)
        w.rounded_box(s, link+"/UpperLip", (.107, .0068, .0065),
                      (0., 0., .015), FILM, .0024, .0005)
        for k in range(5):
            w.box(s, link+f"/Crease{k}", (.002, .0011, .022),
                  (-.045+.0225*k, direction*.0027, -.001), (.77, .64, .80),
                  collide=False)
        prismatic(w, s, name+"Flex", root, link, "Y", anchor, limits, C.CUFF_K, 6.5)
    for i in range(C.SEGMENTS):
        # Slivers expose the deposited volume from above. These render-only
        # states do not influence the native grasp, cuff or spatula.
        w.rounded_box(s, root + f"/Deposit{i}", (.026, .021, .006),
                      ((i%2-.5)*.017, (i//2-1.5)*.014, -.064+(.006 if i%2 else 0.)),
                      ICING, .007, .001, collide=False)
    return s


def spatula(w):
    s = w.stage("spatula")
    root = w.body(s, "/Asset/Handle", .070, articulation=True, ccd=True)
    w.rounded_box(s, root+"/Handle", (.117, .022, .020), (-.039, 0., .023),
                  (.105, .11, .13), .009, .003)
    w.box(s, root+"/Shank", (.036, .012, .005), (.034, 0., .017), STEEL,
          rotation("Y", .52))
    blade = w.body(s, "/Asset/Blade", .019, ccd=True)
    w.rounded_box(s, blade+"/Steel", (C.BLADE_X1-C.BLADE_X0, C.BLADE_WIDTH, .0018),
                  ((C.BLADE_X1+C.BLADE_X0)*.5, 0., C.BLADE_Z), STEEL, .008, .00035)
    for i, X in enumerate(C.ICING_XS):
        w.rounded_box(s, blade + f"/Icing{i}", (.015, .024, .016),
                      (float(X), 0., C.BLADE_Z+.008), ICING, .005, .002, collide=False)
    prismatic(w, s, "BladeFlex", root, blade, "Z", (0., 0., 0.), (-.007, .007), C.FLEX_K, 12.)
    return s


def create_assets(directory):
    w = Writer(directory)
    rows = (
        ("table", "base", scene(w), (0., 0., 0.)),
        ("cup", "rigid", cup(w), (C.BAG[0], C.BAG[1], C.TABLE)),
        ("bag", "articulation", bag(w), C.BAG),
        ("spatula", "articulation", spatula(w), C.SPATULA),
    )
    assets = [
        {"name": name, "type": kind, "file": str(w.save(stage)), "position": list(position)}
        for name, kind, stage, position in rows
    ]
    assets[-1]["quaternion"] = [0., 0., sin(C.START_YAW/2), cos(C.START_YAW/2)]
    assets[-2]["quaternion"] = [0., 0., sin(C.BAG_YAW/2), cos(C.BAG_YAW/2)]
    w.inventory()
    return {"assets": assets, "settle_actions": 24}
