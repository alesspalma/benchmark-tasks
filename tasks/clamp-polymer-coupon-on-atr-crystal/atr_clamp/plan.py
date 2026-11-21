"""Pick the coupon, maintain its placement and turn the pressure knob."""

from math import cos, radians, sin

import numpy as np

from . import config as C
from .motion import Program


def frame(forward, closing):
    forward = np.array(forward, dtype=float)
    forward /= np.linalg.norm(forward)
    closing = np.array(closing, dtype=float)
    closing -= np.dot(closing, forward) * forward
    closing /= np.linalg.norm(closing)
    return np.column_stack((closing, np.cross(forward, closing), forward))


def yaw(angle):
    c, s = cos(angle), sin(angle)
    return np.asarray([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves

    strip = frame([0.04, -0.23, -0.97], [1, 0, 0])
    knob = frame([0.36, 0.22, -0.897], [1, 0, 0])
    edge = np.array([0.0, 0.027, 0.0])
    pickup = C.PICK + edge + [0.0, -0.003, 0.006]
    target = C.CRYSTAL + edge + [0, 0.0, 0.00455]

    def sample(pos, grip):
        return pos, strip, grip

    def wheel(deg, grip, offset=(0, 0, 0)):
        return C.KNOB + np.asarray(offset), yaw(radians(deg)) @ knob, grip

    p.add(
        2.2,
        "Approach the exposed coupon edge",
        left=sample(pickup + [-0.052, 0.038, 0.062], 0.78),
    )
    p.add(
        1.2,
        "Bring the jaws above the coupon edge",
        left=sample(pickup + [0.0, 0.0, 0.038], 0.78),
    )
    p.add(1.2, "Align the jaws on the clean outer edge", left=sample(pickup, 0.70))
    p.add(0.7, "Pinch the polymer coupon", left=sample(pickup, 0.45))
    p.add(
        1.1,
        "Lift the polymer on its edge grip",
        left=sample(pickup + [0, 0, 0.042], 0.45),
    )
    p.add(
        2.1,
        "Bring the coupon alongside the diamond plate",
        left=sample(target + [-0.003, 0.070, 0.003], 0.45),
    )
    p.add(1.5, "Slide the coupon flat across the diamond", left=sample(target, 0.45))
    p.add(
        0.85, "Keep the coupon flat under the raised anvil", left=sample(target, 0.45)
    )
    if mode != "unclamped":
        p.add(
            1.35,
            "Approach the knurled pressure crown",
            right=wheel(20, 0.88, [-0.037, -0.068, 0.062]),
        )
        p.add(1.3, "Centre the jaws around the pressure knob", right=wheel(20, 0.88))
        p.add(0.65, "Grasp the crown", right=wheel(20, 0.38))
        for stroke in range(2):
            for angle in (4, -17, -31, -48):
                p.add(
                    0.52,
                    "Turn the knob down and load the centred coupon",
                    right=wheel(angle, 0.38),
                )
            p.add(0.55, "Let the pressure clutch settle", right=wheel(-48, 0.38))
            p.add(0.6, "Open the knob grasp", right=wheel(-48, 0.88))
            p.add(
                1.22,
                "Lift straight clear of the crown",
                right=wheel(-48, 0.94, [-0.008, -0.004, 0.050]),
            )
            if stroke == 0:
                p.add(
                    1.02,
                    "Reset the wrist for the next tightening stroke",
                    right=wheel(20, 0.94, [-0.008, -0.042, 0.050]),
                )
                p.add(1.04, "Regrasp the pressure crown", right=wheel(20, 0.88))
                p.add(0.6, "Close around the crown again", right=wheel(20, 0.38))
        p.add(
            1.0,
            "Retract from the pressure tower",
            right=wheel(20, 0.94, [-0.036, -0.115, 0.066]),
        )
    p.add(0.65, "Release the edge after clamping", left=sample(target, 0.75))
    p.add(
        1.05,
        "Withdraw sideways from the pressure tip",
        left=sample(target + [-0.027, 0.089, 0.004], 0.75),
    )
    p.add(
        0.80,
        "Lift away to verify the pressure witness",
        left=sample(target + [-0.063, 0.112, 0.068], 0.75),
    )
    p.add(0.8, "Leave the specimen undisturbed")
    return p.moves
