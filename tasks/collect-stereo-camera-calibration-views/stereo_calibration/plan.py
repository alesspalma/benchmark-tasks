"""Bimanual carrying waypoints for the reference and repeated-view control."""

from math import pi, radians

import numpy as np
from scipy.spatial.transform import Rotation
from yam_arena import ik
from yam_arena.demo import grip_rotation

from . import config as C
from .motion import Program, rotation

# Starting from opposite sides keeps the fingers clear of both optical fields.
GRIP = {
    side: rotation("X", radians(-54 if side == "left" else 54)) @ grip_rotation(pi/2)
    for side in ("left", "right")
}
VIEWS = (
    (.370, .000, .921, 0.0, 0.0, 0.0),
    (.3383, -.0693, .9296, -13.8, -2.1, -7.4),
    (.3205, -.0540, .9026, 2.7, 7.7, -6.1),
    (.3963, -.0688, .9477, -8.9, -18.3, -4.3),
    (.4398, .0600, .9644, -1.8, -12.7, 2.2),
    (.3940, .0759, .9675, 6.0, -22.8, 19.2),
    (.413, .054, .921, 0., -18., 12.),
    (.4178, -.0002, .9259, 12.1, -18.3, -11.2),
    (.4082, -.0371, .9672, 3.8, -21.7, -1.9),
)


def arms(position, board_rot, opening, standoff=0.):
    poses = {}
    for side, sign in (("left", 1), ("right", -1)):
        orientation = board_rot @ GRIP[side]
        local = np.array([0., .135*sign, .052])
        poses[side] = (
            position + board_rot@local - standoff*orientation[:, 2],
            orientation, opening,
        )
    return poses


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    start = C.BOARD
    current, current_r = start, np.eye(3)
    grip = .035
    p.add(1.8, "Approach the blank side margins", **arms(start, current_r, .29, .065))
    p.add(1.0, "Align jaws across the board", **arms(start, current_r, .29))
    p.add(.55, "Grip both margins", **arms(start, current_r, grip))

    def carry(position, board_rot, duration, label):
        nonlocal current, current_r
        start, start_r = current, current_r
        for i in range(1, 5):
            t = i/4
            point = start+t*(position-start)
            matrix = ik.slerp_matrix(start_r, board_rot, t)
            p.add(duration/4, label, **arms(point, matrix, grip))
        current, current_r = position, board_rot

    first = np.array(VIEWS[0][:3])
    carry(start+[0., 0., .055], np.eye(3), 1.0, "Lift out of the stand")
    plans = VIEWS if mode == "demo" else (VIEWS[0], (.375, .012, .923, 0., 0., 3.))*4
    for n, view in enumerate(plans, start=1):
        target = np.array(view[:3])
        board_rot = Rotation.from_euler("xyz", view[3:], degrees=True).as_matrix()
        carry(target, board_rot, 1.35, f"Present view {n}")
        p.add(1.15, f"Hold matched view {n}", **arms(target, board_rot, grip))
    carry(first, np.eye(3), 1.38, "Align with the return stand")
    carry(start+[.003, 0., .065], np.eye(3), .8, "Center the lower edge")
    carry(start+[.003, 0., -.003], np.eye(3), 1.4, "Lower the board into the stand")
    p.add(.7, "Release in the stand", **arms(start+[.003, 0., 0.], np.eye(3), .30))
    p.add(.85, "Withdraw both grippers", **arms(start, np.eye(3), .30, .075))
    return p.moves
