"""Install the spring guard while the skate rests on its side."""

import numpy as np

from . import config as C
from .motion import Program, rotation

DOWN = rotation("X", np.pi)
BOOT_GRASP = rotation("X", np.deg2rad(28.)) @ rotation("Y", np.deg2rad(-55.)) @ DOWN
GUARD_GRASP = rotation("X", np.deg2rad(28.)) @ rotation("Y", np.deg2rad(-42.)) @ DOWN
SKEWED = C.SIDE @ rotation("X", np.deg2rad(15.))
SHORT = .037


def boot(origin, grip=.06):
    return np.asarray(origin) + C.SIDE @ C.GRIP_BOOT, BOOT_GRASP, grip


def cup(origin, orientation=C.SIDE, extension=0., grip=.10):
    local_grip = C.GRIP_GUARD + [0., -extension, .001]
    return np.asarray(origin) + orientation @ local_grip, orientation @ C.SIDE.T @ GUARD_GRASP, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    boot_start = C.BOOT_START - [0, 0, .002]
    guard_start = C.GUARD_START - [0, 0, .002]
    toe = boot_start + C.SIDE @ [0, C.RUNNER_TOE, .001]
    guard_toe = np.array([0, C.GUARD_TOE_INNER - .002, C.GUARD_SEAT_Z])
    angled = toe - SKEWED @ guard_toe
    aligned = toe - C.SIDE @ guard_toe
    carry = np.array([.077, 0, .088])
    bagged = C.BAG + [.077, 0, C.BAG_FLOOR + .043]
    movement = bagged - boot_start
    if mode == "bare":
        moving_heel = boot_start + C.SIDE @ [0, -.083, .052]

        def right(origin, grip=.02):
            return moving_heel + origin - boot_start, GUARD_GRASP, grip
    else:
        def right(origin, grip=.10):
            return cup(aligned + origin - boot_start, extension=.017, grip=grip)

    p.add(2.4, "approach the boot and the loose guard",
          left=boot(boot_start + [0, 0, .052], grip=1.),
          right=(right(boot_start + [0, 0, .060], grip=1.) if mode == "bare"
                 else cup(guard_start + [0, 0, .060], grip=1.)))
    p.add(1.4, "come down onto the toe box and heel grasp surfaces",
          left=boot(boot_start, grip=1.),
          right=(right(boot_start, grip=1.) if mode == "bare" else cup(guard_start, grip=.85)))
    p.add(1.0, "counterhold the dry boot and grip the heel cup",
          left=boot(boot_start), right=right(boot_start) if mode == "bare" else cup(guard_start))
    if mode != "bare":
        p.add(.80, "lift the spring guard above its wooden rest",
              right=cup(guard_start + [0, 0, .030]))
        p.add(1.45, "bring the guard toe to the runner tip at an angle",
              right=cup(angled + [.014, 0, 0], SKEWED))
        p.add(1.25, "seat the toe under the end roof",
              right=cup(angled, SKEWED))
        p.add(1.25, "extend the heel cup against both springs",
              right=cup(angled, SKEWED, SHORT))
        p.add(1.15, "align the opened heel cup with the runner",
              right=cup(aligned, extension=SHORT))
        p.add(.72, "let the springs capture the runner heel",
              right=cup(aligned, extension=SHORT, grip=.95))
        p.add(.9, "reposition outside the engaged heel cup",
              right=right(boot_start, grip=.95))
        p.add(.80, "grasp the protected heel for the lift",
              right=right(boot_start))
    p.add(1.35, "lift together above the bag rim",
          left=boot(boot_start + [0, 0, carry[2]]),
          right=right(boot_start + [0, 0, carry[2]]))
    p.add(2.7, "carry the skate above the bag compartment",
          left=boot(boot_start + [movement[0], movement[1], carry[2]]),
          right=right(boot_start + [movement[0], movement[1], carry[2]]))
    p.add(1.55, "lower the skate onto the bag floor",
          left=boot(bagged), right=right(bagged))
    p.add(.85, "release the skate on the bag floor",
          left=boot(bagged, grip=1.), right=right(bagged, grip=1.))
    p.add(1.25, "withdraw both hands",
          left=boot(bagged + [0, 0, .109], grip=1.),
          right=right(bagged + [0, 0, .105], grip=1.))
    return p.moves
