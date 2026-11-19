"""The left hand releases the lock while the right hand guides the end hook."""

import numpy as np

from . import config as C
from .motion import Program, rotation

DOWN = rotation("Y", np.deg2rad(-12.)) @ rotation("X", np.pi)
BUTTON_APPROACH = rotation("X", np.deg2rad(-37.)) @ DOWN
HOOK_APPROACH = rotation("X", np.deg2rad(33.)) @ DOWN
HOOK_GRIP = .19


def hook_pose(length, grip, dz=0.):
    xyz = C.CASE + C.APERTURE + [0., -length - .0035, -.004 + dz]
    return xyz, HOOK_APPROACH, grip


def button_pose(depression, dz=0.):
    surface = C.CASE + C.BUTTON + [0., 0., C.BUTTON_HEIGHT / 2. - depression + dz]
    return surface - .013 * BUTTON_APPROACH[:, 2], BUTTON_APPROACH, 0.


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    ready_hook = hook_pose(C.EXPOSED_AT_RESET, .75, dz=.079)
    p.add(2.6, "reach above the tape and case",
          left=button_pose(0., dz=.071), right=ready_hook if mode != "unguided" else None)
    if mode != "unguided":
        p.add(1.15, "bring the pads around the end hook",
              right=hook_pose(C.EXPOSED_AT_RESET, .75), left=button_pose(0., dz=.022))
        p.add(.75, "grasp the steel hook",
              right=hook_pose(C.EXPOSED_AT_RESET, HOOK_GRIP))
    if mode == "lock-held":
        return p.moves
    p.add(1.05, "press the release button", left=button_pose(.0125))
    if mode != "unguided":
        p.add(.65, "hold the hook against the spring")
        p.add(3.55, "guide the return under spring load",
              right=hook_pose(C.RELEASE_GAP, HOOK_GRIP))
        p.add(.40, "let the hook settle the final distance",
              right=hook_pose(C.RELEASE_GAP, .67))
    p.add(1., "clear the parked tape",
          left=button_pose(0., dz=.11),
          right=hook_pose(.070, .82, dz=.10) if mode != "unguided" else None)
    p.add(1., "observe the parked condition")
    return p.moves
