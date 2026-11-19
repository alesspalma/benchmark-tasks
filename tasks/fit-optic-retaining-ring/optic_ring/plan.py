"""Cartesian reference with ordinary proprioceptive feedback."""

import math

import numpy as np

from . import config as C
from .motion import Program, rotation

DOWN = rotation("X", np.pi)
LEFT = rotation("X", -1.12) @ DOWN
LEFT_OFFSET = np.array([0., .012, .025])
TOOL_OFFSET = np.array([0., 0., C.TOOL_GRIP_Z])
REST_LIFT = [0, 0, .075]
PRE_ALIGN = [0, 0, .021]
SEAT_TIP = C.TUBE + [0., 0., C.RING_BOTTOM + C.SLOT_BOTTOM + .00008]
TURN = math.radians(-86.)


def tube(gap, raise_by=0.):
    return C.TUBE + LEFT_OFFSET + [0., 0., raise_by], LEFT, gap


def tool(tip, gap, angle=0.):
    return tip + TOOL_OFFSET, rotation("Z", angle) @ DOWN, gap


def waypoints(mode):
    plan = Program()
    if mode == "hold":
        return plan.moves
    if mode not in ("demo", "unseated"):
        raise ValueError(f"Unknown policy {mode!r}")

    plan.add(2.6, "approach the loose tube and parked spanner",
             left=tube(.95, .045), right=tool(C.TOOL + REST_LIFT, .72))
    plan.add(1.7, "lower both grippers to their grasp points",
             left=tube(.95), right=tool(C.TOOL, .72))
    plan.add(.7, "grasp the tube and the spanner",
             left=tube(.38), right=tool(C.TOOL, .14))
    plan.add(1.1, "lift the spanner straight out of its stand",
             right=tool(C.TOOL + REST_LIFT, .14))
    plan.add(1.9, "align both spanner teeth above the recessed ring",
             right=tool(SEAT_TIP + PRE_ALIGN, .14))
    plan.add(1.35, "center both teeth over the bore",
             right=tool(SEAT_TIP + PRE_ALIGN, .14))
    if mode == "demo":
        plan.add(2.75, "engage both teeth in the recessed slots",
                 right=tool(SEAT_TIP, .14))
        plan.add(1.1, "apply gentle axial engagement", right=tool(SEAT_TIP, .14))
        plan.add(3.0, "turn the spanner while counterholding the loose tube",
                 right=tool(SEAT_TIP + [0., 0., -.000152], .14, TURN))
        plan.add(.7, "settle the installed ring against the optic rim",
                 right=tool(SEAT_TIP + [0., 0., -.000152], .14, TURN))
    else:
        # A wrench turned while hovering can look busy without clamping the lens.
        plan.add(2.75, "lower near the slots but stop short",
                 right=tool(SEAT_TIP + [0., 0., .012], .14))
        plan.add(1.1, "hover above the ring",
                 right=tool(SEAT_TIP + [0., 0., .012], .14))
        plan.add(3., "turn the wrench above the ring",
                 right=tool(SEAT_TIP + [0., 0., .012], .14, TURN))
        plan.add(.7, "pause over the loose ring",
                 right=tool(SEAT_TIP + [0., 0., .012], .14, TURN))
    plan.add(1.25, "withdraw the spanner above the tube",
             right=tool(SEAT_TIP + [0, 0, .020], .14, TURN))
    plan.add(1.35, "align the tool with its storage rest",
             right=tool(C.TOOL + REST_LIFT, .14))
    plan.add(1.9, "return the spanner to its stand", right=tool(C.TOOL, .14))
    plan.add(.55, "settle the spanner into the storage rest", right=tool(C.TOOL, .14))
    plan.add(.65, "release the spanner and the lens tube",
             left=tube(.95), right=tool(C.TOOL, .75))
    plan.add(1.10, "leave the optical assembly clear",
             left=tube(.95, .095), right=tool(C.TOOL + REST_LIFT, .75))
    plan.add(1.10, "release and settle")
    return plan.moves
