"""Repeatable withdrawal of the independently moving stocker."""

import math

import numpy as np


HIP = np.array((.362, -.565, .925))
SHOULDERS = tuple(np.array((x, -.351, .937)) for x in (.274, .402))
ELBOWS = tuple(np.array((x, -.196, .828)) for x in (.274, .402))
WRISTS = tuple(np.array((x - .005, -.066, .704)) for x in (.274, .402))
ANKLES = tuple(np.array((x, -.59, .010)) for x in (.281, .453))
LEG_TOPS = tuple(np.array((x, -.57, .918)) for x in (.281, .453))
BODY_NAMES = (
    "MotionTorso", "MotionUpper0", "MotionForearm0",
    "MotionUpper1", "MotionForearm1", "MotionLeg0", "MotionLeg1",
)
ANCHORS = dict(zip(BODY_NAMES, (HIP, SHOULDERS[0], ELBOWS[0], SHOULDERS[1], ELBOWS[1], *ANKLES), strict=True))
DURATION = 2.4


def blend(t, start, end):
    u = min(max((t - start) / (end - start), 0.), 1.)
    return u * u * (3. - 2. * u)


def rotation(angle):
    """Rotation around the longitudinal axis of the cabinet."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array(((1., 0., 0.), (0., c, -s), (0., s, c)))


def body_transform(target, angle):
    return (*target,
            math.sin(angle / 2.), 0., 0., math.cos(angle / 2.))


def direction(vector):
    return math.atan2(vector[2], vector[1])


def targets(t):
    """World transforms for rigid segments using shared joint anchors."""
    rise = math.radians(69. * blend(t, 1.55, DURATION))
    hip = HIP + (0., -.20 * blend(t, .61, 1.55), 0.)
    result = [body_transform(hip, rise)]
    lifted = blend(t, 0., .61)
    retrieved = blend(t, 1.16, 1.55)
    relaxed = blend(t, 1.55, DURATION)
    for shoulder, elbow, wrist in zip(SHOULDERS, ELBOWS, WRISTS, strict=True):
        moved_shoulder = hip + rotation(rise) @ (shoulder - HIP)
        upper, lower = elbow - shoulder, wrist - elbow
        # Lift the hands while keeping the elbows below the lid. Fold the
        # elbows down only after they have passed outside the cabinet wall.
        upper_angle = math.radians(-45. * retrieved - 7. * relaxed)
        lower_angle = (math.radians(15.) - direction(lower)) * lifted
        lower_angle += math.radians(32. * retrieved - 132. * relaxed)
        moved_elbow = moved_shoulder + rotation(upper_angle) @ upper
        result.extend((body_transform(moved_shoulder, upper_angle),
                       body_transform(moved_elbow, lower_angle)))
    for index, (ankle, leg_top) in enumerate(zip(ANKLES, LEG_TOPS, strict=True)):
        lift = blend(t, .72 + index * .28, 1.37 + index * .28)
        moved_ankle = ankle + (0., -.20 * lift, .04 * math.sin(math.pi * lift))
        moved_top = leg_top + (hip - HIP)
        angle = direction(moved_top - moved_ankle) - direction(leg_top - ankle)
        result.append(body_transform(moved_ankle, angle))
    return np.asarray(result, dtype=np.float32)
