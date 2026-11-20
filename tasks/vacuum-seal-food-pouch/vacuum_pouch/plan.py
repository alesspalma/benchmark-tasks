"""Ordinary Cartesian approaches and paired gripper motions for the reference."""

import numpy as np

from . import config as C
from .motion import Program, rotation

FRONT = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
PINCH = FRONT @ rotation("Z", np.pi / 2)
TILT = rotation("Y", np.radians(25.))
FLIP = rotation("Z", np.pi)


def pinch(side, shift=(0., 0., 0.), gap=.035, degrees=0.):
    turn = rotation("Z", np.radians(degrees))
    pos = C.POUCH_START + np.asarray(shift) + turn @ [
        C.GRASP_X, side*C.GRASP_Y, C.MOUTH_Z+.0024
    ]
    approach = rotation("Z", np.radians(-side*42.)) @ TILT @ PINCH
    if side < 0:
        approach = approach @ FLIP
    return pos, turn @ approach, gap


def lid(degrees, front=0., high=0., grip=.065, side=1):
    turn = rotation("Y", np.radians(degrees))
    face = rotation("Y", np.radians(63.-.07*degrees)) @ PINCH
    point = C.LID + turn @ [-.167, side*.080, .0265]
    pos = point - .012*face[:, 2] + [front, 0., high]
    return pos, face, grip

def cam(degrees, reach=0., high=0., grip=.024):
    turn = rotation("Y", np.radians(degrees))
    pos = C.CAM + turn @ C.LATCH_ARM + [reach, -.009, high]
    orientation = rotation("Z", np.radians(12.)) @ rotation("Y", np.radians(24.+degrees*.40)) @ PINCH @ FLIP
    return pos, orientation, grip


def button(high):
    return C.BUTTON + [-.003, 0., high], rotation("Y", np.radians(62.)) @ PINCH, .06


def waypoints(mode):
    if mode == "hold":
        return {}
    path = Program()
    short = mode == "short-mouth"
    dx, dz = .089, .008
    yaw = 22. if short else 0.
    if short:
        dx = .080
    path.add(1.6, "Reach above the counter outside the pouch",
             left=pinch(1, (-.006,.043,.044), .31),
             right=pinch(-1, (-.006,-.043,.044), .31))
    path.add(1., "Lower at either side of the film",
             left=pinch(1, (0.,.042, -.003), .30),
             right=pinch(-1, (0.,-.042, -.003), .30))
    path.add(1.1, "Straddle the two film edges from the sides",
             left=pinch(1, (0.,0.,-.003), .29),
             right=pinch(-1, (0.,0.,-.003), .29))
    path.add(.65, "Pinch the two side welds to control the film",
             left=pinch(1,(0.,0.,-.003),.022), right=pinch(-1,(0.,0.,-.003),.022))
    path.add(.75, "Raise the dry pouch enough to clear the front deck",
             left=pinch(1, (0.,0.,.062),.022), right=pinch(-1, (0.,0.,.062),.022))
    path.add(1.4, "Spread and feed the whole open end across the gasket",
             left=pinch(1, (dx,.0004,.063),degrees=yaw), right=pinch(-1, (dx,-.0004,.063),degrees=yaw))
    path.add(.85, "Set the open end down into the channel",
             left=pinch(1, (dx,.0004,dz),degrees=yaw), right=pinch(-1,(dx,-.0004,dz),degrees=yaw))
    path.add(.65, "Release the supported film edge slowly",
             left=pinch(1, (dx,0.,dz),.36,yaw), right=pinch(-1, (dx,0.,dz),.36,yaw))
    path.add(1.0, "Lift away from the two edges without dragging them",
             left=pinch(1,(dx,.065,dz+.085),.30),
             right=pinch(-1,(dx,-.065,dz+.085),.30))
    path.add(1.35, "Approach the top of the lid",
             left=lid(C.LID_OPEN,-.042,.012),
             right=pinch(-1,(dx-.09,-.023,dz+.065),.35))
    path.add(1.1, "Contact the upper face of the lid", left=lid(C.LID_OPEN, 0., -.008))
    for degrees in np.linspace(C.LID_OPEN, 0., 13)[1:]:
        path.add(.24, "Lower the lid along its hinge", left=lid(degrees, 0., -.012))
    path.add(1.3, "Hold the heater lid down and approach the rotary keeper",
             left=lid(0., 0., -.010), right=cam(0., -.058, .020, .38))
    path.add(.93, "Touch the side keeper", right=cam(0., .001, .001,.33))
    path.add(.45, "Grip the side keeper", right=cam(0., .001, .001,.05))
    for degrees in np.linspace(0., C.CAM_TRAVEL+30, 16)[1:]:
        path.add(.24, "Turn the side keeper to Operate", right=cam(degrees,.001,.001,.055))
    path.add(.6, "Open both fingers before letting go",
             left=lid(0., 0., -.010),
             right=cam(C.CAM_TRAVEL+23, .002,.001,.38))
    path.add(.85, "Lift clear of the gasket lid", left=lid(0., -.020, .060),
             right=cam(C.CAM_TRAVEL, -.10, .073,.36))
    path.add(1.35, "Move above Vacuum & Seal", left=button(.062))
    path.add(1.0, "Press Vacuum & Seal", left=button(.008))
    path.add(.32, "Keep the pad depressed briefly", left=button(.008))
    path.add(3.4, "Let the programmed vacuum and seal cycle finish",
             left=button(.062))
    path.add(1.25, "Approach the closed keeper and the clamped lid",
             right=cam(C.CAM_TRAVEL,.0,.001,.33), left=lid(0.,-.014,.030))
    path.add(.72, "Take the load off the keeper and grip its thumb lever",
             right=cam(C.CAM_TRAVEL,.0,.001,.055), left=lid(0.,0.,-.012))
    for degrees in np.linspace(C.CAM_TRAVEL, -17., 15)[1:]:
        path.add(.24, "Turn the unloaded keeper back against the Open stop",
                 right=cam(degrees,.001,.0005,.055))
    path.add(.45, "Release the keeper at the open stop",
             right=cam(-17.,.001,.0005,.33))
    path.add(1.15, "Withdraw while the lid swings up",
             right=cam(-15.,-.065,.090,.40), left=lid(0.,-.010,.110))
    path.add(1.15, "Approach above the two film edges",
             right=pinch(-1,(dx,-.063,dz+.054),.32),
             left=pinch(1,(dx,.063,dz+.054),.32))
    path.add(1.10, "Lower to either side of the sealed pouch",
             left=pinch(1,(dx,.047,dz+.003),.31),right=pinch(-1,(dx,-.047,dz+.003),.31))
    path.add(1.05, "Settle the fingertips outside the film",
             left=pinch(1,(dx,.047,dz+.003),.31),right=pinch(-1,(dx,-.047,dz+.003),.31))
    path.add(1.10, "Straddle the film from its two outside edges",
             left=pinch(1,(dx,0.,dz+.003),.29),right=pinch(-1,(dx,0.,dz+.003),.29))
    path.add(.69, "Grasp both sides of the sealed pouch",
             left=pinch(1,(dx,0.,dz+.003),.022),right=pinch(-1,(dx,0.,dz+.003),.022))
    path.add(.85, "Lift the sealed mouth clear of the heating lip",
             left=pinch(1,(dx,0.,.052)),right=pinch(-1,(dx,0.,.052)))
    path.add(1.45, "Draw the sealed pouch clear of the appliance",
             left=pinch(1,(-.083,0.,.045)),right=pinch(-1,(-.083,0.,.045)))
    path.add(.95, "Set the pouch down flat on the counter",
             left=pinch(1,(-.083,0.,0.)),right=pinch(-1,(-.083,0.,0.)))
    path.add(.65, "Leave the sealed pouch on the counter",
             left=pinch(1,(-.083,0.,0.),.36),right=pinch(-1,(-.083,0.,0.),.36))
    path.add(1.1, "Withdraw both arms",
             left=pinch(1,(-.09,.074,.105),.38),right=pinch(-1,(-.09,-.074,.105),.38))
    return path.moves
