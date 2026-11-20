"""Full-size cold filament printer, native wheel and carriage joints, loose paper."""

import math

import numpy as np

from . import config as C
from .motion import rotation
from .usd_scene import Writer

OAK = (.55, .43, .31)
BLACK = (.072, .080, .085)
RAIL = (.17, .18, .19)
STEEL = (.49, .52, .54)
BED = (.13, .17, .17)
BRASS = (.72, .44, .17)
YELLOW = (.91, .57, .11)
PAPER = (.91, .90, .80)
BLUE = (.16, .34, .57)
AT = C.BED_CENTER.copy()


def contact(w, geom, offset):
    w.Px.PhysxCollisionAPI.Get(geom.GetPrim().GetStage(), geom.GetPath()).CreateContactOffsetAttr(offset)


def room(w):
    s = w.stage("room")
    w.box(s, "/Asset/Worktop", (1.10, 1.25, .060), (.32, 0, C.TABLE - .030), OAK)
    for i, x in enumerate((-.16, .80)):
        for j, y in enumerate((-.57, .57)):
            w.box(s, f"/Asset/Leg{i}{j}", (.058, .058, C.TABLE - .06),
                  (x, y, (C.TABLE - .06) / 2), BLACK)
    w.box(s, "/Asset/Floor", (8., 8., .02), (0, 0, -.01), (.35, .36, .35))
    return s


def joint(w, s, name, a, b, local_a, axis, limits, drive=None, linear=False):
    cls = w.Phy.PrismaticJoint if linear else w.Phy.RevoluteJoint
    j = cls.Define(s, "/Asset/" + name)
    j.CreateBody0Rel().SetTargets([a])
    j.CreateBody1Rel().SetTargets([b])
    j.CreateLocalPos0Attr(w.Gf.Vec3f(*map(float, local_a)))
    j.CreateLocalPos1Attr(w.Gf.Vec3f(0, 0, 0))
    j.CreateAxisAttr(axis)
    j.CreateCollisionEnabledAttr(False)
    j.CreateLowerLimitAttr(limits[0])
    j.CreateUpperLimitAttr(limits[1])
    if drive:
        stiffness, damping, force = drive
        d = w.Phy.DriveAPI.Apply(j.GetPrim(), "linear" if linear else "angular")
        d.CreateStiffnessAttr(stiffness)
        d.CreateDampingAttr(damping)
        d.CreateMaxForceAttr(force)
    return j


def printer(w):
    s = w.stage("printer")
    frame = w.body(s, "/Asset/Frame", 5.1, fixed=True, articulation=True)
    w.Px.PhysxArticulationAPI.Get(s, frame).CreateEnabledSelfCollisionsAttr(False)

    def box(name, size, at, color, collide=True):
        return w.box(s, frame + "/" + name, size, at, color, collide=collide)

    # Open-frame printer with full-height extrusion uprights, Y carriage,
    # opposed guide rollers, Z lead screw and the X hot-end carriage.
    for sign in (-1, 1):
        side = "L" if sign > 0 else "R"
        y = sign * .174
        box(side + "BaseRail", (.40, .04, .04), (.450, y, .778), RAIL)
        box(side + "Upright", (.040, .040, .409), (.495, y, .993), RAIL)
        for wall in (-1, 1):
            box(f"{side}VSlot{int(wall > 0)}", (.002, .007, .39),
                (.495 + wall * .0204, y, .997), BLACK, collide=False)
        for x in (.283, .622):
            box(f"{side}RubberFoot{int(x > .4)}", (.045, .041, .0105),
                (x, y, C.TABLE + .00525), BLACK)
        for j, z in enumerate((.914, .952)):
            w.cylinder(s, f"{frame}/{side}GantryRoller{j}", .011, .009,
                       (.466, y, z), BLACK, axis="X", collide=False)

    box("TSlotCrossBar", (.020, .388, .020), (.495, .0, 1.196), RAIL)
    box("XExtrusion", (.020, .355, .020), (.466, .0, .946), RAIL)
    box("XDrive", (.047, .044, .043), (.470, .153, .946), BLACK)
    box("YExtrusion", (.433, .040, .020), (.453, 0, .774), RAIL)
    box("YCarrier", (.157, .174, .005), (.453, 0, .814), STEEL, collide=False)
    for k, x in enumerate((.402, .508)):
        for sign in (-1, 1):
            w.cylinder(s, f"{frame}/YWheel{k}{int(sign > 0)}", .012, .008,
                       (x, sign*.025, .800), BLACK, axis="Z", collide=False)
    box("BaseCrossMember", (.04, .379, .02), (.608, 0, .788), RAIL)
    box("ZDrive", (.043, .044, .039), (.520, .139, .833), BLACK)
    w.cylinder(s, frame+"/ZLeadScrew", .004, .354, (.520, .136, 1.019), STEEL)
    for j in range(58):
        w.cylinder(s, f"{frame}/LeadThread{j}", .00455, .0015,
                   (.520, .136, .85 + .0059*j), STEEL, collide=False)
    box("PowerSupply", (.048, .102, .197), (.528, -.145, .903), BLACK)
    box("ControlPanel", (.077, .087, .021), (.317, -.216, .774), BLACK)
    box("LCD", (.037, .057, .001), (.320, -.215, .7848), BLUE, collide=False)
    w.cylinder(s, frame+"/LCDWheel", .009, .008, (.295, -.212, .789), BLACK, collide=False)
    x, y, z = C.NOZZLE
    box("HotEndBackplate", (.012, .071, .072), (.450, y, .922), BLACK)
    box("CoolingFan", (.034, .047, .041), (.408, y+.002, .924), BLACK)
    w.cylinder(s, frame+"/FanHub", .014, .004, (.389, y, .929), RAIL, axis="X", collide=False)
    for i in range(6):
        a = math.tau*i/6
        box(f"FanBlade{i}", (.0018, .008, .003), (.387, y+.010*math.sin(a), .929+.010*math.cos(a)), BLACK, collide=False)
    for i, shift in enumerate((-.014, .014)):
        w.cylinder(s, f"{frame}/HeadGuide{i}", .011, .009,
                   (.453, y+shift, .950), BLACK, axis="X", collide=False)
    box("HotEndBridge", (.081, .036, .014), (.410, y, .904), BLACK)
    w.cylinder(s, frame+"/HeatSink", .009, .030, (x, y, .893), STEEL)
    for j in range(8):
        w.cylinder(s, f"{frame}/Fin{j}", .012, .0015, (x, y, .880+j*.0038), STEEL, collide=False)
    box("HeaterBlock", (.020, .016, .012), (x, y, z + .0115), STEEL)
    w.cylinder(s, frame+"/PTFETube", .002, .047, (x, y, .935), PAPER, collide=False)

    # Four threaded wheels turn freely under tangential pad loads.
    # The real springs' neutral heights are calculated from wheel angle.
    for i, offset in enumerate(C.SUPPORTS):
        pos = np.r_[AT[:2]+offset, C.WHEEL[2]]
        b = w.body(s, f"/Asset/WheelBody{i}", .023, pos, ccd=True)
        w.cylinder(s, b + "/Hub", .015, .012, (0, 0, 0), BLACK)
        w.cylinder(s, b + "/Rim", C.WHEEL_RADIUS-.001, C.WHEEL_THICKNESS,
                   (0, 0, 0), BLACK)
        for k in range(24):
            a = math.tau*k/24
            w.box(s, f"{b}/Flute{k}", (.003, .005, C.WHEEL_THICKNESS),
                  ((C.WHEEL_RADIUS-.0015)*math.cos(a), (C.WHEEL_RADIUS-.0015)*math.sin(a), 0.),
                  BLACK, rotation("Z", a))
        w.box(s, b+"/IndexRim", (.0035, .0022, .007),
              (-C.WHEEL_RADIUS+.0003, 0, 0), YELLOW, collide=False)
        joint(w, s, f"Screw{i}", frame, b, pos, "Z", (-160., 160.), (0., .001, .14))
        spring_x, spring_y = pos[:2]
        for k in range(6):
            w.cylinder(s, f"{frame}/Spring{i}Coil{k}", .0055, .0012,
                       (spring_x, spring_y, .8335+k*.0028), YELLOW, collide=False)
        w.cylinder(s, f"{frame}/Stud{i}", .0019, .036,
                   (spring_x, spring_y, .834), STEEL, collide=False)

    # The three short constrained DOFs let the plate translate, pitch and roll
    # in response to the support springs instead of being teleported by a score.
    carriage = w.body(s, "/Asset/CarriageAxis", .04, AT)
    roll = w.body(s, "/Asset/RollAxis", .04, AT)
    deck = w.body(s, "/Asset/BuildPlate", .54, AT, ccd=True)
    for b in (carriage, roll):
        w.Px.PhysxRigidBodyAPI.Get(s, b).CreateDisableGravityAttr(True)
    joint(w, s, "Heave", frame, carriage, AT, "Z", (-.002, .002), (32000., 65., 65.), linear=True)
    joint(w, s, "Roll", carriage, roll, (0.,0.,0.), "X", (-2.2,2.2), (3.8,.23, 12.))
    joint(w, s, "Pitch", roll, deck, (0.,0.,0.), "Y", (-2.2,2.2), (3.8,.23, 12.))
    surface = w.box(s, deck+"/SpringSteel", (.235, .235, .0052), (0.,0.,-.0026), BED)
    contact(w, surface, .000031)
    w.box(s, deck+"/HeaterFoil", (.218, .218, .0012), (0.,0.,-.0057), STEEL, collide=False)
    for sign in (-1, 1):
        for i in range(9):
            w.box(s, f"{deck}/Grid{int(sign>0)}{i}", ((.209 if sign>0 else .00035),
                  (.209 if sign<0 else .00035), .00008),
                  ((i-4)*.024 if sign<0 else 0., (i-4)*.024 if sign>0 else 0., .00004),
                  (.23,.29,.28), collide=False)

    # The axial slider represents aggregate head, carriage and plate deflection
    # at first physical contact. All visible nozzle surfaces move with it.
    nozzle = w.body(s, "/Asset/BrassNozzle", .016, C.NOZZLE)
    w.Px.PhysxRigidBodyAPI.Get(s, nozzle).CreateDisableGravityAttr(True)
    w.cylinder(s, nozzle+"/Hex", .004, .003, (0,0,.0056), BRASS, collide=False)
    tip = w.cylinder(s, nozzle+"/Tip", .0009, .00045, (0,0,.000225), BRASS)
    contact(w, tip, .000029)
    w.cylinder(s, nozzle+"/Shoulder", .0022, .0015, (0,0,.0013), BRASS, collide=False)
    joint(w, s, "NozzleFlex", frame, nozzle, C.NOZZLE, "Z", (-.000006,.0015),
          (C.NOZZLE_STIFFNESS, 2.3, 25.), linear=True)
    return s


def paper(w):
    # A flat working leaf and a folded free end share a passive flexural
    # hinge. The real edge remains thin at the nozzle and can stay on the
    # bed while the free end bends slightly inside the gripper.
    s = w.stage("paper")
    p = w.body(s, "/Asset/Strip", .00045, articulation=True, ccd=True)
    w.Px.PhysxArticulationAPI.Get(s, p).CreateEnabledSelfCollisionsAttr(False)
    lead = -C.PAPER_SIZE[0]/2 + .018
    folded = w.body(s, "/Asset/FoldedEdge", .00004, (lead, 0., 0.), ccd=True)
    body = w.box(s, p+"/Leaf", (C.PAPER_SIZE[0]-.018, C.PAPER_SIZE[1], C.PAPER_SIZE[2]),
                 (.009, 0., 0.), PAPER)
    curl = w.box(s, folded+"/BowedEdge", (.0184, C.PAPER_SIZE[1], 4*C.PAPER_SIZE[2]),
                 (-.009, 0., .0029), PAPER, rotation("Y", .314))
    contact(w, body, .000028)
    contact(w, curl, .000028)
    joint(w, s, "PaperBend", p, folded, (lead, 0., 0.), "Y", (-22.,22.), (.00004,.000014,.0015))
    return s


def create_assets(directory):
    w = Writer(directory)
    assets = [
        dict(name="table", type="base", file=w.save(room(w)), position=[0.,0.,0.]),
        dict(name="printer", type="articulation", file=w.save(printer(w)), position=[0.,0.,0.]),
        dict(name="paper", type="articulation", file=w.save(paper(w)), position=C.PAPER_CENTER.tolist(),
             quaternion=[0., 0., math.sin(C.PAPER_YAW/2), math.cos(C.PAPER_YAW/2)]),
    ]
    w.inventory()
    return {"assets": assets, "settle_actions": 23}
