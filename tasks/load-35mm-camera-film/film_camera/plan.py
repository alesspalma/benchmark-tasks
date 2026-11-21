"""Taught contacts from the fixed 35 mm camera setup."""

from math import cos, radians, sin

import numpy as np

from . import config as C
from .motion import Program


def down(tilt, close_y=True):
    a = radians(tilt)
    forward = np.array([sin(a), 0, -cos(a)])
    closing = np.array([0, -1, 0]) if close_y else np.array([-cos(a), 0, -sin(a)])
    return np.column_stack((closing, np.cross(forward, closing), forward))


def almost_vertical_strip():
    forward = np.array([0.25, 0.46, -0.852])
    forward /= np.linalg.norm(forward)
    closing = np.array([1.0, 0.0, 0.0])
    closing -= np.dot(closing, forward) * forward
    closing /= np.linalg.norm(closing)
    return np.column_stack((closing, np.cross(forward, closing), forward))


def waypoints(mode):
    program = Program()
    if mode == "hold":
        return program.moves
    cassette_grip = down(36)
    strip_grip = almost_vertical_strip()
    door_grip = down(34)
    lever_grip = down(16)

    def cassette(y, z, x=C.X - 0.011):
        return ((x, y, z), cassette_grip, 0.05)

    def strip(y, z, grip, x=C.X):
        return (x, y, z), strip_grip, grip

    def door(deg, extra_y=0, extra_z=0):
        a = radians(deg)
        return (
            (
                C.DOOR[0] - 0.025,
                C.DOOR[1] - C.DOOR_REACH * cos(a) + extra_y,
                C.DOOR[2] + C.DOOR_REACH * sin(a) + extra_z,
            ),
            door_grip,
            0.04,
        )

    def lever(deg, z_offset=0, back=0):
        a = radians(deg)
        return (
            (
                C.LEVER[0] + C.LEVER_REACH * sin(a) - back,
                C.LEVER[1] - C.LEVER_REACH * cos(a),
                C.LEVER[2] + 0.011 + z_offset,
            ),
            lever_grip,
            0.07,
        )

    start_y = C.SEAT_Y + C.LEADER_DY - 0.014
    end_y = start_y - (C.PULL - 0.011 if mode != "short_leader" else 0.010)
    top = C.FILM_Z + 0.005

    program.add(
        1.70,
        "Approach the exposed film and seated cartridge",
        right=strip(-0.112, 0.960, 0.53, x=0.344),
    )
    program.add(
        1.45,
        "Center over the film and reach around the open back",
        right=strip(start_y, 0.902, 0.52),
    )
    program.add(
        1.7,
        "Reach down to the free edge of the film",
        right=strip(start_y, top, 0.52),
    )
    program.add(0.8, "Pinch the film leader", right=strip(start_y, top, 0.01))
    program.add(
        2.30,
        "Draw the leader straight across the film gate",
        right=strip(end_y, top, 0.01),
    )
    program.add(
        0.80, "Keep the film leader level at the index", right=strip(end_y, top, 0.01)
    )
    program.add(0.45, "Release at the index", right=strip(end_y, top, 0.53))
    program.add(
        1.12,
        "Clear the loading rails and camera cover",
        right=strip(end_y, 0.949, 0.55, x=0.343),
    )
    program.add(
        1.25,
        "Position at the rear door's molded lip",
        left=door(C.DOOR_OPEN, extra_y=0.047, extra_z=0.010),
    )
    program.add(
        1.20,
        "Bring the fingertips against the open rear cover",
        left=door(C.DOOR_OPEN, extra_y=0.006, extra_z=-0.012),
    )
    for angle in (87, 75, 55):
        program.add(
            0.65,
            "Press the hinged cover through its detent",
            left=door(angle, extra_z=-0.002),
        )
    program.add(
        0.85, "Let the pressure plate latch over the film", left=door(61, extra_z=0.029)
    )
    program.add(0.8, "Clear the latched rear cover", left=cassette(0.115, 0.967))
    if mode != "no_advance":
        program.add(
            0.95,
            "Approach the resting advance lever",
            right=lever(0, z_offset=0.061, back=0.015),
        )
        program.add(
            0.86, "Set the fingertips against the lever", right=lever(0, back=0.016)
        )
        for angle in (8, 24, 40, 56, 64):
            program.add(
                0.56, "Stroke the advance lever and move the film", right=lever(angle)
            )
        program.add(
            0.90, "Clear the lever and let it return", right=lever(65, z_offset=0.080)
        )
    program.add(
        1.00,
        "Clear both wrists so the rewind witness can be inspected",
        left=cassette(0.170, 0.971),
        right=strip(-0.170, 0.968, 0.73, x=0.362),
    )
    return program.moves
