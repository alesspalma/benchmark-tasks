"""Ordinary two-arm handling of a vacuum gauge test fixture."""

import numpy as np
from scipy.spatial.transform import Rotation

from . import config as C
from .motion import Program


def frame(forward, closing):
    forward = np.array(forward, dtype=float)
    forward /= np.linalg.norm(forward)
    closing = np.array(closing, dtype=float)
    closing -= np.dot(closing, forward) * forward
    closing /= np.linalg.norm(closing)
    return np.column_stack((closing, np.cross(forward, closing), forward))


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    left_orientation = frame([0.59, -0.12, -0.798], [0, -1, 0])
    seat_orientation = (
        Rotation.from_rotvec([0.073, -0.056, 0.070]).as_matrix() @ left_orientation
    )
    right_orientation = frame([0.41, 0.11, -0.905], [0, -1, 0])
    pickup = C.PICK + C.HANDLE_LOCAL + [0, 0, 0.002]
    seat = C.TARGET + C.HANDLE_LOCAL + [0, 0, 0.002]
    seal_pose = seat + [0, 0, 0.003]
    regrasp = seat + [-0.001, 0, -0.004]
    spool = C.SPOOL + [0, 0, 0.0024]
    slider_end = spool + [C.SPOOL_TRAVEL, 0, 0]
    vent = C.VENT + [0, 0, 0.010]

    def l(pos, grip, in_test=False):
        return (pos, seat_orientation if in_test else left_orientation, grip)

    def r(pos, grip):
        return (pos, right_orientation, grip)

    p.add(
        2.0,
        "Approach the plate and supply handles",
        left=None if mode == "no_seal" else l(pickup + [-0.035, 0.020, 0.083], 0.80),
        right=r(spool + [-0.037, -0.030, 0.067], 0.80),
    )
    p.add(
        1.4,
        "Align on the handles",
        left=None if mode == "no_seal" else l(pickup, 0.80),
        right=r(spool, 0.80),
    )
    p.add(
        0.65,
        "Grasp the handles",
        left=None if mode == "no_seal" else l(pickup, 0.10),
        right=r(spool, 0.14),
    )
    p.add(
        1.25,
        "Lift the plate off the rest and open supply",
        left=None if mode == "no_seal" else l(pickup + [0, 0, 0.063], 0.10),
        right=r(slider_end + [0.002, 0, 0], 0.14),
    )
    if mode != "no_seal":
        p.add(
            1.8,
            "Carry the plate below the cup",
            left=l(seat + [0, 0, -0.037], 0.10, True),
        )
        p.add(
            1.65, "Press the plate squarely into the bellows", left=l(seat, 0.10, True)
        )
    p.add(0.85, "Let the pressure rise")
    if mode != "no_isolation":
        p.add(
            1.05,
            "Close the supply spool to isolate the cup",
            right=r(spool + [-0.002, 0, 0], 0.14),
        )
    if mode == "no_seal":
        p.add(0.55, "Release the isolated supply spool", right=r(spool, 0.80))
        p.add(
            1.1,
            "Withdraw from the spool",
            right=r(spool + [-0.04, -0.036, 0.071], 0.80),
        )
        return p.moves
    current = slider_end if mode == "no_isolation" else spool
    p.add(
        0.55,
        "Release the spool and the supported test plate",
        left=l(seal_pose, 0.85, True),
        right=r(current, 0.80),
    )
    p.add(
        1.35,
        "Withdraw the hands without disturbing the seal",
        left=l(seal_pose + [-0.065, 0.084, 0.004], 0.85, True),
        right=r(current + [-0.028, -0.038, 0.069], 0.80),
    )
    p.add(2.35, "Observe the isolated gauge and unsupported plate")
    if mode == "no_isolation":
        return p.moves
    p.add(
        1.45,
        "Return to support the test plate",
        left=l(regrasp, 0.80, True),
        right=r(vent + [-0.032, 0.010, 0.076], 0.05),
    )
    p.add(
        0.65, "Take the load at the handle before venting", left=l(regrasp, 0.10, True)
    )
    p.add(1.15, "Press the orange vent", right=r(vent - [0, 0, 0.006], 0.05))
    p.add(0.55, "Let the circuit reach atmospheric pressure")
    p.add(
        0.88,
        "Lower clear of the relaxed cup",
        left=l(seat + [0, 0, -0.030], 0.10, True),
        right=r(vent + [-0.033, -0.040, 0.073], 0.70),
    )
    p.add(1.55, "Return to the padded rest", left=l(pickup + [0, 0, 0.054], 0.10))
    p.add(
        1.30,
        "Set the test plate flat on the rest",
        left=l(pickup + [0, 0, -0.001], 0.10),
    )
    p.add(0.55, "Open the jaws and leave the plate on its rest", left=l(pickup, 0.80))
    p.add(1.25, "Withdraw the left arm", left=l(pickup + [-0.031, 0.082, 0.079], 0.80))
    p.add(0.70, "Leave the gauge vented and the plate undisturbed")
    return p.moves
