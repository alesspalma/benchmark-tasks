"""Reference waypoints for the load-bearing workcell and two pad assemblies."""

import numpy as np

from . import config as C
from .motion import Program, rotation

DOWN = rotation("Y", np.radians(-55.0)) @ np.diag([1.0, -1.0, -1.0])
FRONT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
EDGE = FRONT @ rotation("Z", -np.pi / 2)
HUB = rotation("Y", np.radians(-50.0)) @ np.diag([1.0, -1.0, -1.0])


def grip(angle, side, lift=0.0, opening=0.09):
    r = rotation("Y", np.radians(angle))
    center = C.PIVOT + r @ [-C.GRIP_RADIUS, side * C.GRIP_Y, 0.0]
    return center + [0.0, 0.0, 0.003 + lift], DOWN, opening


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    p.add(
        1.5,
        "Approach the two turnover handles",
        left=grip(0, 1, 0.08, 0.84),
        right=grip(0, -1, 0.08, 0.84),
    )
    p.add(
        0.9,
        "Go down to the turnover handles",
        left=grip(0, 1, 0, 0.84),
        right=grip(0, -1, 0, 0.84),
    )
    p.add(0.65, "Take both rubber grips", left=grip(0, 1), right=grip(0, -1))
    for angle in np.linspace(0.0, 180.0, 25)[1:]:
        p.add(0.19, "Turn the cleaner over", left=grip(angle, 1), right=grip(angle, -1))
    p.add(0.5, "Settle at the inverted stop")
    p.add(
        0.65,
        "Release both handles",
        left=grip(180, 1, 0, 0.95),
        right=grip(180, -1, 0, 0.95),
    )
    p.add(
        0.85,
        "Lift free of the turnover cradle",
        left=grip(180, 1, 0.095, 0.95),
        right=grip(180, -1, 0.095, 0.95),
    )
    if mode == "flip-only":
        return p.moves
    p.add(
        1.8,
        "Clear the workpiece",
        left=([0.33, 0.286, 1.133], DOWN, 1.0),
        right=([0.33, -0.286, 1.133], DOWN, 1.0),
    )
    for arm, side in (("left", 1), ("right", -1)):
        remove(p, arm, side)
        peel(p, side)
        sort(p, arm, side)
    return p.moves


def remove(p, arm, side):
    hub = np.array([0.463, 0.075 * side, 1.068])
    grasp = HUB @ rotation("Z", np.pi / 2)

    def move(duration, label, xyz, opening, orientation=grasp):
        p.add(duration, label, **{arm: (xyz, orientation, opening)})

    move(1.5, "Approach above the central grip", hub + [-0.032, 0.0, 0.031], 0.48)
    move(1.0, "Engage the carrier hub", hub, 0.48)
    move(0.9, "Grip the carrier hub", hub, 0.06)
    move(
        1.15,
        "Pull the carrier clear of its magnetic mount",
        hub + [0.0, 0.0, 0.033],
        0.06,
    )
    move(
        1.3, "Carry the assembly clear of the shell", [0.379, 0.265 * side, 1.102], 0.06
    )
    presentation = [0.149, 0.186 * side, 1.044]
    move(1.6, "Present the carrier for peeling", presentation, 0.06)
    move(1.0, "Hold the detached assembly steady", presentation, 0.06)


def peel(p, side):
    arm = "right" if side > 0 else "left"
    face = rotation("Z", np.radians(64.0 * side)) @ EDGE
    fringe = np.array([0.148, 0.120 * side, 1.018])

    def move(duration, label, d, opening):
        p.add(duration, label, **{arm: (fringe + d, face, opening)})

    move(1.7, "Approach the exposed cloth edge", [0.0, -0.126 * side, 0.008], 0.44)
    move(
        1.5,
        "Align the open fingers with the cloth edge",
        [0.0, -0.043 * side, 0.0],
        0.40,
    )
    move(1.7, "Engage the cloth edge from the side", [0.0, 0.0, 0.0], 0.35)
    move(0.7, "Settle the fingers around the cloth edge", [0.0, 0.0, 0.0], 0.35)
    move(0.95, "Pinch the soft fringe", [0.0, 0.0, 0.0], 0.04)
    move(
        1.65,
        "Lift the cloth edge while the other arm holds the carrier",
        [0.006, 0.0, 0.057],
        0.04,
    )
    move(
        2.6,
        "Peel the remainder of the cloth clear",
        [-0.024, -0.100 * side, 0.056],
        0.04,
    )
    move(
        0.6,
        "Hold the separated cloth pad and carrier",
        [-0.024, -0.100 * side, 0.056],
        0.04,
    )


def sort(p, holder, side):
    fabric_arm = "right" if side > 0 else "left"
    plastic_tool = rotation("Y", np.radians(27.0)) @ HUB @ rotation("Z", np.pi / 2)
    cloth_tool = rotation("Z", np.radians(64.0 * side)) @ EDGE
    plastic = np.array([C.CARRIER_TRAY_X - 0.017, C.SORT_Y * side, 0.842])
    cloth = np.array([0.141, 0.078 * side, 0.925])
    retreat = [0.139, 0.024 * side, 1.075]

    def move(duration, label, plastic_xyz, cloth_xyz, opening):
        p.add(
            duration,
            label,
            **{
                holder: (plastic_xyz, plastic_tool, opening),
                fabric_arm: (cloth_xyz, cloth_tool, opening),
            },
        )

    move(
        1.4,
        "Move the plastic carrier away from the cloth tray",
        [0.290, 0.280 * side, 1.073],
        retreat,
        0.055,
    )
    move(
        1.4,
        "Lower the cloth and carrier into their own trays",
        plastic + [0.0, 0.0, 0.065],
        cloth + [0.0, 0.0, 0.055],
        0.055,
    )
    move(1.15, "Place both pieces in the trays", plastic, cloth, 0.055)
    move(0.72, "Release both pieces", plastic, cloth, 0.74)
    move(
        1.4,
        "Lift both grippers free",
        plastic + [-0.07, 0.0, 0.13],
        cloth + [0.0, 0.015 * side, 0.13],
        0.92,
    )
    p.add(
        1.4,
        "Clear the workpiece",
        left=([0.33, 0.286, 1.133], DOWN, 1.0),
        right=([0.33, -0.286, 1.133], DOWN, 1.0),
    )
