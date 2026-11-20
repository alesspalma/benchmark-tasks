"""Bimanual cold cutting reference through ordinary gripper actions."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def pinch(lateral, approach):
    z = np.asarray(approach, dtype=float)
    z /= np.linalg.norm(z)
    x = np.asarray(lateral) - np.dot(lateral, z) * z
    x /= np.linalg.norm(x)
    return np.column_stack((x, np.cross(z, x), z))


PART_PINCH = pinch([0., 0., -1.], [1., .20, -.15])
TOOL_PINCH = pinch([0., -1., 0.], [1., .12, -.52])
PART_YAW = np.eye(3)
TOOL_TILT = rotation("Y", np.deg2rad(-9.))


def plate_at(pos, attitude=np.eye(3), grip=.01):
    return pos + attitude @ C.PLATE_GRASP, attitude @ PART_PINCH, grip


def pickup_at(h, grip=.02):
    return C.TOOL_START + C.TOOL_GRASP + [0., 0., h], TOOL_PINCH, grip


def tool_at(xy, h, grip=.025):
    plate_relative = np.r_[xy, C.PLATE_THICKNESS / 2 + h]
    tip = C.PLATE_START + PART_YAW @ plate_relative
    return tip + TOOL_TILT @ (C.TOOL_GRASP - C.KNIFE), TOOL_TILT @ TOOL_PINCH, grip


def waypoints(mode):
    if mode == "hold":
        return {}
    p = Program()
    p.add(2.0, "Approach loose plate and tool", left=plate_at(C.PLATE_START + [-.10, .018, .025], grip=.68),
          right=pickup_at(.103, .62))
    p.add(1.2, "Place jaws around the loose plate and the tool handle", left=plate_at(C.PLATE_START, grip=.61),
          right=pickup_at(0., .60))
    p.add(.65, "Grip the plate by the outside flank and the cutter by its handle",
          left=plate_at(C.PLATE_START), right=pickup_at(0.))
    p.add(1.65, "Brace the loose plate and free the swivel tool from its rest",
          left=plate_at(C.PLATE_START), right=pickup_at(.113))

    path = C.outline()
    ordered = np.roll(path, -24, axis=0)
    centered = (ordered + np.roll(ordered, -1, axis=0)) * .5
    tangent = np.roll(ordered, -1, axis=0) - ordered
    tangent /= np.linalg.norm(tangent, axis=1)[:, None]
    # Advance barely onto the solid edge, with the cutting toe inside the
    # aperture and its working face bearing on the top lip.
    outward = np.column_stack((-tangent[:, 1], tangent[:, 0]))
    path = centered + .0018 * outward
    # Keep the curved-to-straight transition centered under the cutting nose.
    trim = np.array([.0008, .0014, .0023, .0018, .0015, .0007])
    path[23:29] -= trim[:, None] * outward[23:29]
    count = len(path)
    starts = range(0, count, 7) if mode != "partial" else range(0, 14, 7)
    strokes = [path[np.arange(start - 2, min(start + 7, count) + 2) % count]
               for start in starts]
    if mode == "demo":
        finishing = path[:13].copy()
        # Take a lighter bite with an inward transition on the long straight.
        finishing[3:7, 1] += [.0010, .0022, .0029, .0012]
        rounded_end = path[23:29].copy()
        rounded_end[0] += .0014 * outward[23]
        straight_entry = path[25:31].copy()
        straight_entry[2:5] -= np.array([.0007, .00165, .0012])[:, None] * outward[27:30]
        strokes.extend((finishing, rounded_end, straight_entry, path[33:48]))
    depth = .020 if mode == "air-pass" else .00035
    for i, points in enumerate(strokes):
        # Use overlapping draws with unloaded approaches, as when re-seating
        # the swivel at different parts of a non-circular slot by hand.
        p.add(2.65 if i == 0 else .85, "Bring the cutting toe over the held opening",
              right=tool_at(points[0], .035))
        finish = mode == "demo" and i >= len(starts)
        rework = mode == "demo" and i in (len(starts) + 1, len(starts) + 2)
        label = " for a finishing cut" if rework else ""
        engage = "Contour: engage the cutting toe against the rim" + label
        draw = "Contour: draw the swivel blade around the opening" + label
        p.add(.95, engage, right=tool_at(points[0], depth))
        p.add(1.65 if finish else .85, engage)
        for target in points[1:]:
            p.add(.425 if finish else .340, draw, right=tool_at(target, depth))
        p.add(.85, "Contour: withdraw vertically", right=tool_at(points[-1], .039))
    p.add(2.5, "Return the tool to its rest from above", right=pickup_at(.106))
    p.add(.6, "Return the tool to its rest from above")
    p.add(2.5, "Return the tool to its rest and retain the plate",
          left=plate_at(C.PLATE_START + [0., .0, .001]), right=pickup_at(.001))
    p.add(2.5, "Square the plate on both parallels", left=plate_at(C.PLATE_START + [0., .0, .001]))
    p.add(2.5, "Seat the deburring tool in its guard", right=pickup_at(.001))
    p.add(.7, "Release the supported plate and tool",
          left=plate_at(C.PLATE_START + [0., 0., .001], grip=.62), right=pickup_at(.001, .62))
    p.add(1.1, "Clear the stock edge before lifting",
          left=plate_at(C.PLATE_START + [0., .062, .001], grip=.62),
          right=pickup_at(.115, .62))
    p.add(1.6, "Withdraw both grippers",
          left=plate_at(C.PLATE_START + [-.09, .062, .045], grip=.62),
          right=pickup_at(.115, .62))
    return p.moves
