"""Blue side intake at depth; separate orange bottom release above the jar."""

import numpy as np

from . import config as C
from .motion import SIDE_X, Program


def waypoints(mode="demo"):
    if mode == "hold":
        return {}
    p = Program()
    left_grip = 0.06
    right_grip = 0.06
    grasp = C.PROBE + C.SUPPORT_GRASP
    p.add(3, "approach support handle", left=(grasp + [-0.08, 0, 0.03], SIDE_X, 1))
    p.add(2, "reach support handle", left=(grasp, SIDE_X, 1))
    p.add(2, "grasp support handle", left=(grasp, SIDE_X, left_grip))
    p.add(
        3,
        "lift closed probe from cradle",
        left=(grasp + [0, 0, 0.19], SIDE_X, left_grip),
    )
    source = C.TUB + [-0.012, 0, C.SAMPLING_BASE_HEIGHT]
    support = source + C.SUPPORT_GRASP
    p.add(
        4,
        "closed probe above granules",
        left=(support + [0, 0, 0.19], SIDE_X, left_grip),
    )
    p.add(4, "insert closed sample chamber", left=(support, SIDE_X, left_grip))
    p.add(1, "settle inserted probe")
    if mode == "insert":
        return p.moves

    def operate(origin, stage):
        handle = origin + C.GATE_GRASP
        p.add(
            3,
            "approach " + stage + " gate handle",
            right=(handle + [-0.08, 0, 0.08], SIDE_X, 1),
        )
        p.add(2, "reach " + stage + " gate handle", right=(handle, SIDE_X, 1))
        p.add(1, "settle " + stage + " handle reach")
        p.add(2, "grasp " + stage + " gate handle", right=(handle, SIDE_X, right_grip))
        p.add(
            3,
            "pull " + stage + " gate open",
            right=(handle + [0, 0, C.GATE_TRAVEL + 0.001], SIDE_X, right_grip),
        )
        p.add(
            4 if stage == "sampling" else 5,
            "fill chamber" if stage == "sampling" else "drain sample inside jar",
        )
        if stage == "sampling":
            for dz in (0.008, 0, 0.008, 0):
                p.add(
                    1,
                    "settle material through open port",
                    left=(origin + C.SUPPORT_GRASP + [0, 0, dz], SIDE_X, left_grip),
                    right=(
                        handle + [0, 0, C.GATE_TRAVEL + 0.001 + dz],
                        SIDE_X,
                        right_grip,
                    ),
                )
        if stage == "sampling":
            # Sweep the open port through fresh material, rather than relying
            # on gravity to fill an uphill side chamber.
            delta = np.array([0.024, 0, 0])
            p.add(
                4,
                "fill open chamber by a short buried sweep",
                left=(origin + C.SUPPORT_GRASP + delta, SIDE_X, left_grip),
                right=(
                    handle + [0, 0, C.GATE_TRAVEL + 0.001] + delta,
                    SIDE_X,
                    right_grip,
                ),
            )
            origin = origin + delta
            handle = origin + C.GATE_GRASP
        if mode == "open" and stage == "sampling":
            return None
        p.add(
            3,
            "return " + stage + " gate to closed stop",
            right=(handle, SIDE_X, right_grip),
        )
        p.add(2, "settle closed " + stage + " gate")
        p.add(2, "release " + stage + " gate handle", right=(handle, SIDE_X, 1))
        p.add(
            2,
            "clear " + stage + " gate handle",
            right=(handle + [-0.08, 0, 0], SIDE_X, 1),
        )
        p.add(1, "open clear right gripper", right=(handle + [-0.08, 0, 0], SIDE_X, 1))
        return origin

    source = operate(source, "sampling")
    if source is None:
        return p.moves
    support = source + C.SUPPORT_GRASP
    p.add(
        2,
        "clear right hand laterally from lower handle",
        right=(source + C.GATE_GRASP + [-0.08, -0.14, 0], SIDE_X, 1),
    )
    p.add(
        2,
        "park right arm before extracting sampler",
        right=(np.array([0.32, -0.30, 1.08]), SIDE_X, 1),
    )
    p.add(
        3,
        "withdraw closed sample over source",
        left=(support + [0, 0, 0.19], SIDE_X, left_grip),
    )
    for dx in (-0.008, 0.008, 0):
        p.add(
            1,
            "shed exterior grains over source",
            left=(support + [dx, 0, 0.19], SIDE_X, left_grip),
        )
    p.add(2, "shed exterior grains into source")
    if mode == "retain":
        return p.moves
    discharge = C.JAR + [0, 0, C.DISCHARGE_HEIGHT]
    sample = discharge + C.SUPPORT_GRASP
    p.add(
        4, "closed sample above jar", left=(sample + [0, 0, 0.090], SIDE_X, left_grip)
    )
    p.add(3, "position bottom outlet above jar rim", left=(sample, SIDE_X, left_grip))
    p.add(1, "settle closed sampler above jar")
    # This is a distinct, horizontal bottom-discharge stroke. The blue
    # intake shutter stays at its closed stop throughout this operation.
    handle = discharge + C.DUMP_GRASP
    p.add(
        3,
        "approach orange bottom-slide handle",
        right=(handle + [-0.08, 0, 0.06], SIDE_X, 1),
    )
    p.add(2, "reach orange bottom-slide handle", right=(handle, SIDE_X, 1))
    p.add(1, "settle bottom-slide reach")
    p.add(2, "grasp orange bottom-slide handle", right=(handle, SIDE_X, right_grip))
    opened = handle + [-C.DUMP_TRAVEL - 0.001, 0, 0]
    p.add(
        4, "pull floor aside for gravity discharge", right=(opened, SIDE_X, right_grip)
    )
    p.add(5, "sample falls through bottom into jar")
    p.add(
        4,
        "return empty bottom plate to closed stop",
        right=(handle, SIDE_X, right_grip),
    )
    p.add(2, "settle closed bottom plate")
    p.add(2, "release orange discharge handle", right=(handle, SIDE_X, 1))
    p.add(
        2, "clear orange handle horizontally", right=(handle + [-0.08, 0, 0], SIDE_X, 1)
    )
    if mode == "deliver":
        return p.moves
    p.add(
        3,
        "park right arm clear of empty sampler",
        right=(np.array([0.32, -0.30, 1.08]), SIDE_X, 1),
    )
    p.add(
        3, "lift empty closed sampler", left=(sample + [0, 0, 0.09], SIDE_X, left_grip)
    )
    p.add(
        4,
        "return closed probe above wide cradle",
        left=(grasp + [0, 0, 0.19], SIDE_X, left_grip),
    )
    p.add(3, "lower probe onto cradle foot", left=(grasp, SIDE_X, left_grip))
    p.add(2, "release supported sampler", left=(grasp, SIDE_X, 1))
    p.add(2, "settle probe with fingers fully open")
    p.add(2, "withdraw left fingers", left=(grasp + [-0.08, 0, 0], SIDE_X, 1))
    p.add(1, "open clear left hand", left=(grasp + [-0.08, 0, 0], SIDE_X, 1))
    p.add(3, "park left arm", left=(np.array([0.27, 0.35, 1.05]), SIDE_X, 1))
    p.add(2, "settle retained jar sample")
    return p.moves
