"""Routine trip-lever service with the intact native handle and loose clip."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def hand(approach, closing):
    axis = np.asarray(approach, dtype=float)
    axis /= np.linalg.norm(axis)
    jaws = np.asarray(closing, dtype=float)
    jaws -= np.dot(jaws, axis) * axis
    jaws /= np.linalg.norm(jaws)
    return np.column_stack((jaws, np.cross(axis, jaws), axis))


LEFT = hand([0.40, -0.14, -0.91], [-1.0, 0, 0])
RIGHT = hand([0.62, 0.25, -0.75], [0.0, -1.0, 0])


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    h = C.LEVER + C.HANDLE
    standby = h + [-0.055, 0.055, 0.077]
    pickup = C.CLIP_HOME + C.CLIP_GRIP + [-0.001, 0, 0.005]
    eye = C.LEVER + C.PINS[C.CORRECT] + C.HOOK_REST + C.CLIP_GRIP + [0, 0, -0.003]
    high_eye = eye + [0, 0, 0.012]

    def l(pos, grip=0.15):
        return pos, LEFT, grip

    def r(pos, grip=0.10):
        return pos, RIGHT, grip

    def pull(degrees):
        rot = rotation("X", np.deg2rad(degrees))
        return (
            C.LEVER
            + rot @ C.HANDLE
            + [-0.003, 0, 0.002 - 0.012 * min(1, -degrees / 29)],
            rot @ LEFT,
            0.13,
        )

    p.add(
        2.2,
        "Reach toward the front handle and the free chain clip",
        left=l(standby, 0.68),
        right=r(pickup + [-0.066, -0.124, 0.045], 0.54) if mode != "no_clip" else None,
    )
    p.add(
        1.40,
        "Approach the detached spring clip from above",
        right=r(pickup + [0, 0, 0.030], 0.54) if mode != "no_clip" else None,
    )
    p.add(
        1.25,
        "Bring the left jaws around the trip handle",
        left=l(h + [-0.003, 0, 0.002], 0.72),
        right=r(pickup, 0.46) if mode != "no_clip" else None,
    )
    p.add(
        0.75,
        "Counterhold the lever and take the spring clip",
        left=l(h + [-0.003, 0, 0.002]),
        right=r(pickup) if mode != "no_clip" else None,
    )
    if mode != "no_clip":
        p.add(
            1.05,
            "Lift the loose clip from the sill",
            right=r(pickup + [-0.006, 0.004, 0.009]),
        )
        p.add(1.45, "Bring the clip directly over the lever eye", right=r(high_eye))
        p.add(
            1.05,
            "Hook the clip down over the eye while supporting the handle",
            right=r(eye),
        )
        p.add(0.45, "Let the spring hook seat on the lever eye")
        p.add(0.75, "Open the fingers without pulling the clip out", right=r(eye, 0.72))
        p.add(
            1.10,
            "Withdraw the clip hand above the cistern",
            right=r(pickup + [-0.048, -0.107, 0.040], 0.62),
        )
    for n in (-8, -16, -23, -29):
        p.add(0.47, "Press the outside handle for the test flush", left=pull(n))
    p.add(0.75, "Hold a full external handle stroke")
    for n in (-20, -10, 0):
        p.add(0.42, "Let the trip handle return to rest", left=pull(n))
    p.add(0.63, "Release the outside handle", left=l(h + [-0.003, 0, 0.002], 0.72))
    p.add(0.95, "Clear the handle so the flapper can reseat", left=l(standby, 0.74))
    p.add(5.50, "Observe the gravity flush and completed seal")
    return p.moves
