"""Two native actions: squeeze while aimed, then drain the recovered sample."""

import numpy as np

from . import config as C
from .motion import Program, rotation

RIGHT_SIDE = rotation("Z", np.deg2rad(80.0)) @ np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
)
LEFT_SIDE = rotation("X", np.deg2rad(-62.0)) @ np.diag([1.0, -1.0, -1.0])
BEAKER_CLOSE = 0.20
BOTTLE_SQUEEZE = 0.365
BEAKER_START = C.BEAKER + [0, 0, C.BEAKER_GRIP]
BOTTLE_START = C.BOTTLE + [0, 0, C.BOTTLE_GRIP]
UPRIGHT = np.eye(3)


def grasp_at(center, R=UPRIGHT, grip=BEAKER_CLOSE):
    return center + R @ [0, 0, C.BEAKER_GRIP], R @ LEFT_SIDE, grip


def receive(angle):
    R = rotation("X", np.deg2rad(angle))
    center = C.FUNNEL + [0, 0.002, 0.025] - R @ C.SPOUT
    return grasp_at(center, R)


def waypoints(mode="demo"):
    action = Program()
    if mode == "hold":
        return action.moves
    r_far = BOTTLE_START - 0.060 * RIGHT_SIDE[:, 2] + [0, 0, 0.018]
    action.add(2.8, "approach the aimed wash bottle", right=(r_far, RIGHT_SIDE, 1.0))
    action.add(
        1.6,
        "place fingertips across the LDPE walls",
        right=(BOTTLE_START, RIGHT_SIDE, 1.0),
    )
    squeeze = BOTTLE_SQUEEZE if mode != "dry" else 0.54
    action.add(
        2.3,
        "squeeze water against the beaker wall",
        right=(BOTTLE_START, RIGHT_SIDE, squeeze),
    )
    action.add(1.2, "let the directed rinse settle around the lower wall")
    action.add(
        0.75, "release the flexible bottle", right=(BOTTLE_START, RIGHT_SIDE, 1.0)
    )
    action.add(
        1.4,
        "withdraw from the bench",
        right=(r_far, RIGHT_SIDE, 1.0),
        left=(BEAKER_START + [0, 0, 0.082], LEFT_SIDE, 1.0),
    )
    action.add(
        1.25, "center fingertips on the beaker", left=(BEAKER_START, LEFT_SIDE, 1.0)
    )
    action.add(
        1.0, "grasp the rinsed beaker", left=(BEAKER_START, LEFT_SIDE, BEAKER_CLOSE)
    )
    lift = C.BEAKER + [0, 0, 0.118]
    action.add(1.65, "lift the rinse above the funnel", left=grasp_at(lift))
    action.add(
        0.85,
        "wet the remaining inner wall",
        left=grasp_at(lift, rotation("X", np.deg2rad(-17))),
    )
    action.add(1.15, "guide the pouring lip above the funnel", left=receive(33))
    action.add(2.3, "drain the rinsings into the flask", left=receive(86))
    action.add(2.0, "let the flask collect the rinse")
    pour_tcp, pour_rotation, pour_grip = receive(86)
    action.add(
        1.30,
        "retreat from the funnel without rotating the beaker",
        left=(pour_tcp + [0.0, 0.075, 0.0], pour_rotation, pour_grip),
    )
    action.add(
        1.40,
        "carry the tilted beaker around the far edge of the funnel",
        left=(pour_tcp + [0.120, 0.075, -0.026], pour_rotation, pour_grip),
    )
    action.add(
        2.30,
        "level the beaker on the clear side of the funnel",
        left=grasp_at(lift + [0.044, 0.021, 0.0]),
    )
    action.add(1.15, "carry the upright beaker over its mark", left=grasp_at(lift))
    action.add(
        1.5, "set down the emptied beaker", left=(BEAKER_START, LEFT_SIDE, BEAKER_CLOSE)
    )
    action.add(
        0.70, "release the supported beaker", left=(BEAKER_START, LEFT_SIDE, 1.0)
    )
    action.add(
        1.35,
        "withdraw from both vessels",
        left=(BEAKER_START + [0, 0, 0.078], LEFT_SIDE, 1.0),
        right=(r_far, RIGHT_SIDE, 1.0),
    )
    action.add(1.05, "leave stable glassware")
    return action.moves
