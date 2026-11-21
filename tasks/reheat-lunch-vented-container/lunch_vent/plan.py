"""Open the steam valve, seat the physical door, then press Start."""

import numpy as np
from scipy.spatial.transform import Rotation

from . import config as C
from .motion import Program, rotation

FRONT = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
PANEL = rotation("Y", np.radians(20.)) @ FRONT
DOWN = rotation("Y", np.radians(34.)) @ FRONT
VENT_APPROACH = rotation("Y", np.radians(15.)) @ FRONT


def brace(before=0., lift=0.):
    # Load the handle against the glass while the other hand works the rocker.
    yaw = rotation("Z", np.radians(C.HANDLE_YAW))
    p = C.BOWL_RESET + yaw @ [0., C.HANDLE_RADIUS, C.HANDLE_HEIGHT + .004]
    return p + [-before, 0., lift], DOWN, .018


def rocker(angle, before=0., lift=0.):
    motion = rotation("Y", np.radians(angle))
    p = C.BOWL_RESET + C.VENT_PIVOT + motion @ (C.VENT_TAIL + [0., 0., .001])
    return p + [-.012 - before, 0., lift], VENT_APPROACH, .018


def measured_rocker(state, progress):
    """Follow the actual moving thumb pad using only physical robot commands."""
    pose = state["objects"]["lunch_bowl"]["links"]["Vent"]
    point = Rotation.from_quat(pose[3:]).apply(C.VENT_TAIL) + pose[:3]
    open_angle = np.degrees(state["objects"]["lunch_bowl"]["joints"]["SteamVent"])
    forward = -.006 if open_angle >= C.VENT_OPEN_DEG - .4 else -.016 + .051 * progress
    desired_tip = point + [forward, 0., -.013 * progress]
    tip = np.asarray(state["hands"]["right_robot"]["points"][1])
    correction = np.clip(.42 * (desired_tip - tip), -.022, .022)
    return desired_tip - .013 * VENT_APPROACH[:, 2] + correction


def door(angle, offset=0.):
    arc = rotation("Z", np.radians(angle))
    orientation = rotation("Z", np.radians(.31 * angle)) @ rotation("Y", np.radians(25.)) @ FRONT
    return C.DOOR_PIVOT + arc @ [-.019 - offset, -C.DOOR_RADIUS, .022], orientation, .02


def button(offset):
    return C.START_POSITION + [offset, 0., 0.], PANEL, .018


def ready(side, grip=.96):
    return [.300, .272 * side, .967], rotation("Y", np.radians(24.)) @ FRONT, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    p.add(1.45, "Lift the left hand clear of the open door",
          left=([.308, .285, 1.085], rotation("Y", np.radians(48.)) @ FRONT, .10))
    if mode == "demo":
        p.add(1.6, "Move the hands into the space ahead of the cavity",
              left=([.324, .138, 1.010], DOWN, .018),
              right=([.342, -.154, .939], DOWN, .018))
        p.add(1.45, "Align outside the two sides of the bowl",
              left=brace(.132, .015), right=rocker(0., .150, .027))
        p.add(.75, "Let both wrists settle before entry")
        p.add(1.5, "Bring the left fingertips over the handle",
              left=brace(0., .019))
        p.add(1.2, "Brace the bowl against the glass",
              left=brace(0., -.015))
        p.add(.60, "Hold contact with the bowl handle")
        p.add(1.4, "Reach the open-door vent",
              right=rocker(0., -.008, .018))
        p.add(.85, "Align above the rocker")
        p.add(1.20, "Lower onto the back of the steam vent rocker", right=rocker(0., .004, .006))
        p.add(3.0, "Press the rocker through its travel",
              right=rocker(C.VENT_OPEN_DEG, -.008, .002))
        p.add(1.10, "Withdraw the right gripper while bracing the bowl",
              right=rocker(C.VENT_OPEN_DEG, .150, .020))
        p.add(1.45, "Retract the supporting fingers straight back",
              left=brace(.155, -.006), right=rocker(C.VENT_OPEN_DEG, .175, .035))
    preclose_p, preclose_r, _ = door(-C.DOOR_OPEN_DEG, .075)
    p.add(.95, "Raise the left hand ahead of the door",
          left=([.282, .102, 1.070], rotation("Y", np.radians(38.)) @ FRONT, .15),
          right=ready(-1.))
    p.add(1.15, "Move around the outer face of the door",
          left=(preclose_p + [0., 0., .159], rotation("Y", np.radians(35.)) @ FRONT, .05))
    p.add(1.10, "Lower onto the open door's approach", left=(preclose_p, preclose_r, .02))
    p.add(.65, "Contact the microwave door", left=door(-C.DOOR_OPEN_DEG))
    for angle in np.linspace(-C.DOOR_OPEN_DEG, 0., 23)[1:]:
        p.add(.21, "Swing the door closed", left=door(angle, -.004))
    p.add(.70, "Seat the closed door", left=door(0., -.016))
    closed_p, closed_r, closed_g = door(0., -.016)
    p.add(1.05, "Retract straight off the closed door",
          left=(closed_p + [-.125, 0., 0.], closed_r, closed_g))
    p.add(.95, "Move off the door and approach Start", left=ready(1.), right=button(-.081))
    p.add(.80, "Physically press Start", right=button(.007))
    p.add(.47, "Allow the press to register")
    p.add(.92, "Withdraw and leave the microwave running", right=ready(-1.))
    p.add(1.25, "Finish hands clear")
    return p.moves
