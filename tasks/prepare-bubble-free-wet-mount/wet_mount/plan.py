"""Taught bimanual manipulation and the deliberate flat-lowering control."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def down(yaw=0.0, tilt=0.0):
    return rotation("Z", yaw) @ np.diag([1.0, -1.0, -1.0]) @ rotation("X", tilt)


DROP_R = down(C.DROPPER_YAW, 0.61)
SLIP_R = down(-np.pi / 2, 1.15)
OPEN = 0.62
DROP_HOLD = 0.293
DROP_PRESS = 0.178
SLIP_CLAMP = 0.200
PICK_OFFSET = np.array([-0.004, 0, 0.002])
DROP_LOW = C.SLIDE + C.DOSE_OFFSET - C.DROPPER_TIP + [0, 0, 0.024]
DOSE_PHASES = ("align the dropper tip", "deliver one drop", "relax the bulb")
CONTACT_PHASES = ("set the lower edge", "lower the coverslip", "release the coverslip")


def cover_pose(
    tilt_deg, clearance, extra_height=0.0, grip=SLIP_CLAMP, center_trim=(0, 0, 0)
):
    turn = rotation("Y", np.radians(tilt_deg))
    offset = np.array([C.SLIP_SIZE / 2, 0, -C.SLIP_THICK / 2])
    edge = (
        C.SLIDE
        + offset
        + [-0.0025, -0.0035, C.SLIDE_SIZE[2] / 2 + clearance + extra_height]
    )
    return (
        edge + np.asarray(center_trim) + turn @ (PICK_OFFSET - offset),
        turn @ SLIP_R,
        grip,
    )


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    pick_slip = C.SLIP_REST + PICK_OFFSET
    p.add(
        2.2,
        "approach both loose tools",
        left=(C.DROPPER + [0, 0, 0.059], DROP_R, OPEN),
        right=(pick_slip + [0, 0, 0.057], SLIP_R, OPEN),
    )
    p.add(
        1.05,
        "place the jaws around the bulb and the slip edges",
        left=(C.DROPPER, DROP_R, OPEN),
        right=(pick_slip, SLIP_R, OPEN),
    )
    p.add(
        1.15,
        "grip the dropper and coverslip by its edges",
        left=(C.DROPPER, DROP_R, DROP_HOLD),
        right=(pick_slip, SLIP_R, SLIP_CLAMP),
    )
    p.add(
        0.95,
        "lift them from their rests",
        left=(C.DROPPER + [0, 0, 0.079], DROP_R, DROP_HOLD),
        right=(pick_slip + [0, 0, 0.073], SLIP_R, SLIP_CLAMP),
    )
    p.add(
        1.12,
        "bring the bulb over the microscope slide",
        left=(DROP_LOW + [0, 0, 0.046], DROP_R, DROP_HOLD),
    )
    p.add(0.98, DOSE_PHASES[0], left=(DROP_LOW, DROP_R, DROP_HOLD))
    squeeze = DROP_HOLD if mode == "dry" else DROP_PRESS
    p.add(1.10, DOSE_PHASES[1], left=(DROP_LOW, DROP_R, squeeze))
    p.add(0.65, DOSE_PHASES[2], left=(DROP_LOW, DROP_R, DROP_HOLD))
    p.add(
        0.80,
        "raise the dropper clear",
        left=(DROP_LOW + [0, 0, 0.057], DROP_R, DROP_HOLD),
    )
    p.add(
        1.05,
        "return the dropper toward its stand",
        left=(C.DROPPER + [0, 0, 0.084], DROP_R, DROP_HOLD),
    )
    p.add(0.94, "set the dropper in its stand", left=(C.DROPPER, DROP_R, DROP_HOLD))
    p.add(0.65, "release the dropper", left=(C.DROPPER, DROP_R, OPEN))
    p.add(
        1.75,
        "clear the dropper and bring the coverslip over the slide",
        left=(C.DROPPER + [-0.042, 0.046, 0.104], DROP_R, OPEN),
        right=cover_pose(0 if mode == "flat" else 45, 0.005, 0.044),
    )
    p.add(0.50, "steady the held coverslip above the slide")
    angle = 0 if mode == "flat" else 45
    p.add(
        1.40,
        CONTACT_PHASES[0],
        right=cover_pose(
            angle, -0.0032, center_trim=(0, 0, 0) if mode == "flat" else (-0.0015, 0, 0)
        ),
    )
    p.add(0.40, CONTACT_PHASES[0])
    release_angle = 0 if mode == "flat" else 37
    release_trim = (-0.00045, 0.0012, 0) if mode == "flat" else (0.0018, 0.0035, 0)
    release_clearance = -0.0015 if mode == "flat" else -0.0025
    p.add(
        1.45,
        CONTACT_PHASES[1],
        right=cover_pose(release_angle, release_clearance, center_trim=release_trim),
    )
    p.add(0.80, CONTACT_PHASES[1])
    p.add(
        0.45,
        CONTACT_PHASES[2],
        right=cover_pose(
            release_angle, release_clearance, grip=0.262, center_trim=release_trim
        ),
    )
    p.add(
        0.75,
        CONTACT_PHASES[2],
        right=cover_pose(
            release_angle, release_clearance, grip=0.63, center_trim=release_trim
        ),
    )
    p.add(
        0.92,
        "clear the new wet mount",
        right=(np.array([0.340, -0.010, 0.874]), down(np.radians(-65), 0.82), OPEN),
    )
    p.add(0.55, "let the wet mount settle")
    return p.moves
