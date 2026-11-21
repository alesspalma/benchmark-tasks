"""Reference trajectories in the known, fixed restocking layout."""

import numpy as np

from . import config as C
from .motion import Program, rotation


def orientation(direction, closing):
    forward = np.asarray(direction, dtype=float)
    forward /= np.linalg.norm(forward)
    across = np.asarray(closing, dtype=float)
    across -= forward * np.dot(across, forward)
    across /= np.linalg.norm(across)
    return np.column_stack([across, np.cross(forward, across), forward])


CARRY = {
    "left": orientation((1, 0, -0.30), (0, -1, 0)),
    "right": orientation((1, 0, -0.30), (0, -1, 0)),
}
GUIDE = orientation((1, 0, -0.25), (0, -1, 0))
PRESS = orientation((1, 0, -0.40), (0, -1, 0))
TUCK_ENTRY = np.array([0.423, 0.020, 0.815])
TUCK_FINISH = np.array([0.470, 0.020, 0.805])
PADDLE_TOOL = orientation((1, 0.36, -0.40), (0, -1, 0))


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves

    def grip_pair(center, side_offset, x_offset, z_offset, grip):
        return {
            name: (
                center + np.array([x_offset, sign * side_offset, z_offset]),
                CARRY[name],
                grip,
            )
            for name, sign in (("left", 1), ("right", -1))
        }

    p.add(
        2.3,
        "Approach the two ends of the stock roll from above",
        **grip_pair(C.STOCK, 0.144, -0.075, 0.083, 0.10),
    )
    p.add(
        1.55,
        "Lower the jaws alongside the ends",
        **grip_pair(C.STOCK, 0.135, -0.048, -0.015, 0.13),
    )
    p.add(
        1.2,
        "Brace the paper roll between both hands",
        **grip_pair(C.STOCK, 0.087, -0.048, -0.015, 0.13),
    )
    p.add(
        4.20,
        "Slide the roll off the service cart and into the empty cradle",
        **grip_pair(C.SEAT, 0.072, -0.055, -0.005, 0.13),
    )
    p.add(0.65, "Let the loaded roll come to rest")
    p.add(
        0.90,
        "Release the roll and clear the ends at the loading height",
        **grip_pair(C.SEAT, 0.147, -0.083, 0.025, 0.10),
    )
    p.add(
        0.85,
        "Withdraw clear of the loaded roll",
        left=(np.array([0.340, 0.133, 0.977]), GUIDE, 0.20),
        right=(np.array([0.321, -0.242, 0.927]), PRESS, 0.12),
    )
    p.add(
        0.95,
        "Approach the outside paddle face",
        left=(np.array([0.323, 0.049, 0.861]), GUIDE, 0.20),
        right=(np.array([0.341, -0.197, 0.840]), PADDLE_TOOL, 0.10),
    )
    p.add(
        1.30,
        "Line up the palm with the right end of the paddle",
        right=(C.PADDLE + [-0.066, -0.134, 0.018], PADDLE_TOOL, 0.10),
    )
    lead = TUCK_ENTRY
    if mode != "no_thread":
        p.add(
            1.35,
            "Tuck the free paper edge into the roller mouth",
            left=(lead, GUIDE, 0.20),
        )
        p.add(
            0.95,
            "Hold the tucked paper while it engages",
            left=(TUCK_FINISH, GUIDE, 0.20),
        )
    if mode != "no_feed":
        p.add(
            1.5,
            "Push the paddle through its feed stroke",
            right=(C.PADDLE + [0.078, -0.134, 0.009], PADDLE_TOOL, 0.10),
        )
        p.add(0.65, "Let the rollers advance the new paper tail")
        p.add(
            1.1,
            "Release the spring-returned paddle",
            right=(C.PADDLE + [-0.079, -0.145, 0.049], PADDLE_TOOL, 0.10),
        )
    p.add(
        1.0,
        "Withdraw the paper hand after threading",
        left=(np.array([0.299, 0.145, 0.841]), GUIDE, 0.20),
    )
    if mode == "demo":
        p.add(
            1.3,
            "Complete the feed with a second paddle stroke",
            left=(np.array([0.224, 0.275, 1.066]), GUIDE, 0.21),
            right=(C.PADDLE + [0.078, -0.134, 0.009], PADDLE_TOOL, 0.10),
        )
        p.add(0.46, "Let the tail emerge below the dispenser")
        p.add(
            1.1,
            "Release the paddle and inspect the exposed tail",
            right=(C.PADDLE + [-0.079, -0.145, 0.049], PADDLE_TOOL, 0.10),
        )
    if mode != "demo":
        return p.moves

    def cover_target(angle, reach, grip):
        hinge = rotation("Y", angle)
        approach = hinge @ np.array([0.0, 0.23, -1.0])
        facing = orientation(approach, hinge[:, 0])
        pos = C.COVER_HINGE + hinge @ np.array([-0.053, -0.073, C.COVER_HEIGHT - 0.064])
        return (pos + reach * facing[:, 2], facing, grip)

    p.add(
        0.80,
        "Clear the working arm around the outside of the cover",
        right=(np.array([0.318, -0.192, 0.885]), PRESS, 0.35),
    )
    p.add(
        1.80,
        "Approach the open cover pull",
        right=cover_target(C.COVER_OPEN, -0.042, 0.68),
    )
    p.add(
        0.9,
        "Surround the pull before closing the jaws",
        right=cover_target(C.COVER_OPEN, 0.0, 0.68),
    )
    p.add(0.48, "Settle the open jaws around the cover pull")
    p.add(0.80, "Grip the cover pull", right=cover_target(C.COVER_OPEN, 0.0, 0.17))
    for a in (-1.19, -1.05, -0.80, -0.55, -0.33, -0.26):
        p.add(0.60, "Swing the dispenser cover shut", right=cover_target(a, 0.0, 0.17))
    p.add(
        0.60,
        "Open the jaws after the sprung latch engages",
        right=cover_target(-0.26, 0.0, 0.72),
    )
    p.add(
        0.85,
        "Approach the raised cover",
        right=(np.array([0.304, -0.100, 1.038]), PRESS, 0.15),
    )
    p.add(
        0.70,
        "Press the cover latch home",
        right=(np.array([0.410, -0.100, 1.038]), PRESS, 0.15),
    )
    p.add(
        1.2,
        "Release the latch and clear the dispenser",
        right=(np.array([0.245, -0.249, 0.940]), PRESS, 0.15),
    )
    p.add(0.7, "Check the protruding towel and latched cover")
    return p.moves
