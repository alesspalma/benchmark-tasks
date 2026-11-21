"""Cartesian reference positions for supporting, loading, and testing the cups."""

import numpy as np

from . import config as C
from .motion import Program


def tool_orientation(yaw, tilt):
    yaw, tilt = np.deg2rad([yaw, tilt])
    forward = np.array([np.cos(yaw)*np.sin(tilt),
                        np.sin(yaw)*np.sin(tilt), -np.cos(tilt)])
    closing = np.array([np.sin(yaw), -np.cos(yaw), 0.])
    return np.column_stack([closing, np.cross(forward, closing), forward])


FRONT = tool_orientation(0, 60)
TOP = tool_orientation(-60, 70)


def pose(xy, z, orientation, grip):
    return (np.r_[xy, z], orientation, grip)


def waypoints(mode):
    program = Program()
    if mode == "hold":
        return program.moves

    stock_base = C.TABLE_Z+C.STOCK_ELEVATION
    top_offset = .068
    top_grip_z = stock_base+(C.NUM_CUPS-1)*.010+top_offset
    top_grip = .49
    back = -.099*TOP[:2, 2]
    carry = .047
    guide_depth = .044
    pickup_z = stock_base+.009
    lower = .69
    test_height = C.FINGER_Z+C.CUP_HEIGHT-.011
    extraction_center = C.DISPENSER - [.009, 0.]

    program.add(3.15, "Approach the loose stack from above and from the front",
                left=pose(C.STOCK+back, top_grip_z+.012, TOP, .97),
                right=pose(C.STOCK-.117*FRONT[:2, 2], pickup_z, FRONT, 1.))
    program.add(.60, "Steady the open jaws outside of the stack")
    program.add(1.20, "Bring the right jaws around the lowest rolled rim",
                right=pose(C.STOCK, pickup_z, FRONT, .985))
    program.add(.72, "Grip the lowest cup under the nested stack",
                right=pose(C.STOCK, pickup_z, FRONT, lower))
    program.add(1.12, "Bring the left jaws around the top cup",
                left=pose(C.STOCK, top_grip_z, TOP, .95))
    program.add(.63, "Steady the top of the stack with the left jaws",
                left=pose(C.STOCK, top_grip_z, TOP, top_grip))
    program.add(1.18, "Lift the six cups together",
                left=pose(C.STOCK, top_grip_z+carry, TOP, top_grip),
                right=pose(C.STOCK, pickup_z+carry, FRONT, lower))
    program.add(2.05, "Carry the nested stack to the dispenser opening",
                left=pose(C.DISPENSER, top_grip_z+carry, TOP, top_grip),
                right=pose(C.DISPENSER, pickup_z+carry, FRONT, lower))
    program.add(.89, "Guide the edges down into the guard",
                left=pose(C.DISPENSER, top_grip_z+carry-guide_depth, TOP, top_grip),
                right=pose(C.DISPENSER, pickup_z+carry-guide_depth, FRONT, lower))
    program.add(.72, "Preload the top of the stack before releasing the lower grasp",
                left=pose(C.DISPENSER, top_grip_z+carry-guide_depth-.011, TOP, top_grip))
    program.add(.52, "Open the lower hand over the center of the guard",
                right=pose(C.DISPENSER, pickup_z+carry-guide_depth, FRONT, .985))
    program.add(.90, "Clear the lower hand from the opening",
                right=pose(C.DISPENSER+[-.102, 0.], pickup_z+carry-guide_depth, FRONT, .985),
                left=pose(C.DISPENSER, top_grip_z+carry-guide_depth-.016, TOP, top_grip))
    load_z = C.FINGER_Z+top_offset-.024
    program.add(2.05, "Push the cups past the retaining tips against the follower",
                left=pose(C.DISPENSER, load_z, TOP, top_grip),
                right=pose(C.DISPENSER+[-.145, -.025], pickup_z+carry-guide_depth+.052, FRONT, .985))
    program.add(.75, "Hold briefly at insertion depth")
    program.add(.55, "Release the follower to seat the top cup",
                left=pose(C.DISPENSER, load_z, TOP, .99))
    program.add(.92, "Withdraw the open upper hand",
                left=pose(C.DISPENSER+back, load_z+.013, TOP, .99))
    program.add(.68, "Raise the upper hand clear of the dispenser",
                left=pose(C.DISPENSER+back+[0., .040], load_z+.064, TOP, .99))
    program.add(.8, "Check the reloaded cups with both hands clear")

    if mode == "reload_only":
        return program.moves

    program.add(1.08, "Approach the exposed base of one cup",
                left=pose(C.DISPENSER+[.013, .165], C.FINGER_Z+C.CUP_HEIGHT+.070, TOP, .97),
                right=pose(C.DISPENSER+[-.102, 0.], test_height, FRONT, .96))
    program.add(1.04, "Position the two jaws around the outside wall",
                right=pose(extraction_center, test_height, FRONT, .96))
    program.add(.75, "Grip the exposed cup",
                right=pose(extraction_center, test_height, FRONT, .44))
    program.add(1.90, "Pull one cup out of the guard",
                right=pose(extraction_center, test_height+.131, FRONT, .44))
    program.add(1.72, "Move the dispensed cup over the blue mat",
                right=pose(C.MAT, test_height+.131, FRONT, .44))
    on_mat = test_height-C.FINGER_Z+C.TABLE_Z+.002
    program.add(1.13, "Rest the dispensed cup mouth down on the mat",
                right=pose(C.MAT, on_mat, FRONT, .44))
    program.add(.48, "Open the jaws",
                right=pose(C.MAT, on_mat, FRONT, .84))
    program.add(1.05, "Lift the open hand and leave the cup on the mat",
                right=pose(C.MAT, on_mat+.105, FRONT, .92))
    return program.moves
