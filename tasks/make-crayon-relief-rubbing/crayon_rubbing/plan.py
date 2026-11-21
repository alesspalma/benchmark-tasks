"""Teach an ordinary paper hold and overlapping strokes with the wax block."""

from math import radians

from yam_arena.demo import grip_rotation

from . import config as C
from .motion import Program, rotation


WAX_STROKE_Z = C.PAPER_Z + C.TCP_FROM_WAX_BASE + .0047


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    down = grip_rotation()
    outward = rotation("X", radians(-21)) @ grip_rotation(radians(90))
    wax_angle = rotation("Y", radians(-12)) @ down
    paper_x, paper_y = C.PAPER_CENTER
    crayon_x, crayon_y = C.CRAYON_START
    pick_z = C.TABLE + C.CRAYON_REST + C.TCP_FROM_WAX_BASE
    stroke_z = WAX_STROKE_Z + (.017 if mode == "air-pass" else 0.)
    wax_grip = .08
    pin_x, pin_y = paper_x + .012, paper_y + .091
    pin_pose = ((pin_x, pin_y, C.PAPER_Z + .004), outward, .48)
    pin_above = ((pin_x, pin_y, .825), outward, .48)
    pin_near = ((pin_x, pin_y, C.PAPER_Z + .027), outward, .48)
    p.add(2.30, "Approach the loose sheet and wax block",
          left=pin_above if mode != "no-pin" else None,
          right=((crayon_x, crayon_y, .84), down, .73))
    p.add(1.05, "Bring the fingers close above the sheet",
          left=pin_near if mode != "no-pin" else None,
          right=((crayon_x, crayon_y, pick_z), down, .73))
    p.add(1.4, "Press the sheet straight downward and grasp the wax block",
          left=pin_pose if mode != "no-pin" else None,
          right=((crayon_x, crayon_y, pick_z), down, wax_grip))
    p.add(.72, "Lift the block", right=((crayon_x, crayon_y, .825), down, wax_grip))
    for i, lane in enumerate((-.036, -.018, 0., .018, .036)):
        start, end = (-.043, .070) if i % 2 == 0 else (.070, -.043)
        y = paper_y + lane
        hover = .822 if i == 0 else stroke_z + .021
        p.add(1.65 if i == 0 else .72, "Advance to the next track",
              right=((paper_x + start, y, hover), wax_angle, wax_grip))
        p.add(.98 if i == 0 else .53, "Lower the broad wax face onto the paper",
              right=((paper_x + start, y, stroke_z), wax_angle, wax_grip))
        p.add(1.42, "Rub across the raised texture",
              right=((paper_x + end, y, stroke_z), wax_angle, wax_grip))
        p.add(.45, "Lift the wax from the paper",
              right=((paper_x + end, y, stroke_z + .021), wax_angle, wax_grip))
    p.add(.55, "Lift vertically above the last stroke",
          right=((paper_x + .061, paper_y + .036, .833), wax_angle, wax_grip))
    p.add(1.05, "Move the wax block to the right margin",
          right=((paper_x + .044, paper_y - .089, .833), wax_angle, wax_grip))
    p.add(1.20, "Withdraw the wax block beyond the finished sheet",
          right=((crayon_x, crayon_y, .82), down, wax_grip))
    p.add(.82, "Unpin and lift the supporting fingers",
          left=pin_above if mode != "no-pin" else None)
    p.add(.82, "Set the wax block down", right=((crayon_x, crayon_y, pick_z - .002), down, wax_grip))
    p.add(.55, "Release the wax block", right=((crayon_x, crayon_y, pick_z - .002), down, .76))
    p.add(.80, "Clear the finished rubbing", right=((crayon_x, crayon_y, .846), down, .76))
    return p.moves
