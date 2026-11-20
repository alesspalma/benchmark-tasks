"""Adjust the cold wheel, withdraw, then gauge the bed with real paper motion."""

import numpy as np
from yam_arena.urdf import quat_xyzw_to_matrix

from . import config as C
from .motion import Program, rotation


def pinch(axis, advance):
    z = np.asarray(advance) / np.linalg.norm(advance)
    x = np.asarray(axis) - np.dot(axis, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


WHEEL_APPROACH = np.array([1., .10, -.14])
RIGHT = pinch([0., 0., -1.], WHEEL_APPROACH)
FIRST_STROKE_END = 11.25
GAUGE_START = 19.1


def wheel_rim(angle, *, away=0., up=0., grip=.33):
    radial = np.array([np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle)), 0.])
    contact = C.WHEEL + .027 * radial
    turn = rotation("Z", .47 * max(0., 1. - away/.045) * np.deg2rad(angle + 145.))
    return contact + [-away, 0., up], turn @ RIGHT, grip


def waypoints(mode, remaining_degrees=None):
    if mode == "hold":
        return {}
    p = Program()
    entry = -145
    p.add(2.7, "align above the free side of the wheel", right=wheel_rim(entry, away=.075, up=.045))
    p.add(.75, "lower beside the wheel", right=wheel_rim(entry, away=.075))
    for index in range(1 if mode == "undertravel" else 2):
        sweep = 44.
        if index == 1 and remaining_degrees is not None:
            sweep = float(np.clip(remaining_degrees / .565, 10., 44.))
        end = entry - sweep
        p.add(1.85, "place pads above and below the rim", right=wheel_rim(entry))
        p.add(.75, "grip the wheel rim", right=wheel_rim(entry, grip=0.))
        for angle in np.linspace(entry, end, 7)[1:]:
            p.add(.45, "roll the wheel between the pads", right=wheel_rim(angle, grip=0.))
        p.add(.6, "release both pads", right=wheel_rim(end))
        p.add(1.25, "withdraw horizontally through the released gap", right=wheel_rim(end, away=.075))
        p.add(.65, "recenter beyond the wheel", right=wheel_rim(entry, away=.075))
    p.add(.65, "lift outside the printer", right=wheel_rim(entry, away=.075, up=.060))
    p.add(.4, "allow the bed to settle")
    return p.moves

def paper_waypoints(paper_state):
    """Choose the actual fold as the initial pickup and recenter the leaf."""
    if paper_state:
        fold = np.asarray(paper_state["links"]["FoldedEdge"])
        root = np.asarray(paper_state["pose"])
        rotation_matrix = quat_xyzw_to_matrix(root[3:])
        fold_matrix = quat_xyzw_to_matrix(fold[3:])
        grip = fold[:3] + fold_matrix @ [-.007, 0., .0023]
        advance = np.r_[rotation_matrix[:2, 0], 0.]
        center = np.r_[root[:2], 0.]
    else:
        grip = C.PAPER_GRIP
        advance = C.PAPER_AXIS
        center = np.r_[C.PAPER_CENTER[:2], 0.]
    advance /= np.linalg.norm(advance)
    grip -= .007 * advance
    r = pinch([0., 0., 1.], advance + [0.,0.,-.13])
    lateral = np.cross([0.,0.,1.], advance)
    sideways = float((np.r_[C.NOZZLE[:2], 0.] - center) @ lateral)
    # A small translation into alignment leaves the last third of the actual
    # working end at the nozzle. The fingers never approach the nozzle itself.
    align = np.clip(sideways, -.029, .029) * lateral
    def close(opening, delta=(0.,0.,0.)):
        return grip + delta, r, opening
    retreat = - .062 * advance + [0.,0.,.048]
    p = Program()
    p.add(2.4, "approach free paper edge", left=close(.50, retreat))
    p.add(1.25, "place fingers around paper", left=close(.58))
    p.add(.95, "grasp the paper", left=close(0.))
    # The press of the actual nozzle deflects it passively against the sheet.
    for i, move in enumerate((-.004, .011, -.010, .011, -.009, .008)):
        p.add(.75, "draw paper through the light drag", left=close(0., align + move*advance))
    p.add(.53, "release paper", left=close(.58, align + .008*advance))
    p.add(1.25, "clear paper", left=close(.58, retreat))
    p.add(1.15, "verify the settled gap")
    return p.moves["left"]
