"""Reference demonstration through native presses, grasps and releases."""

from math import cos, degrees, radians, sin

import numpy as np

from . import config as C
from .motion import Program


def hand(approach, pinch):
    forward = np.asarray(approach, dtype=float)
    forward /= np.linalg.norm(forward)
    close = np.asarray(pinch, dtype=float) - np.dot(forward, pinch) * forward
    close /= np.linalg.norm(close)
    return np.column_stack((close, np.cross(forward, close), forward))


LEFT = hand([0.45, -0.85, -0.30], [-1.0, 0.0, 0.0])
PRESS_GRIP = hand([0.50, 0.00, -0.866], [1.0, 0.0, 0.0])


def waypoints(mode):
    p = Program()
    if mode == "hold":
        return p.moves
    lower = C.PRESS + C.BASE_TAB + [-0.004, 0.009, -0.0010]
    upper = C.LID_GRIP

    def catch(angle, jaw, offset=(0.0, 0.0, 0.0)):
        a = radians(angle)
        point = C.PRESS + [upper[0], upper[1] * cos(a) + upper[2] * sin(a),
                          -upper[1] * sin(a) + upper[2] * cos(a)]
        return (point + np.asarray(offset), PRESS_GRIP, jaw)

    rest = degrees(C.READY_FOLD) - 7.0
    p.add(1.75, "Approach the lower tab and the loaded press",
          left=(lower + [-0.020, 0.055, 0.075], LEFT, 0.90),
          right=catch(rest, 0.75, [-0.030, -0.052, 0.108]))
    p.add(1.45, "Brace the lower tab and position above the upper tab",
          left=(lower, LEFT, 0.73), right=catch(rest, 0.75, [-0.006, 0.000, 0.045]))
    p.add(0.80, "Grip the lower tab", left=(lower, LEFT, 0.12))
    p.add(1.15, "Set the jaws across the upper tab", right=catch(rest, 0.75, [-0.005, 0.000, 0.005]))
    p.add(0.72, "Grip the upper tab", right=catch(rest, 0.15, [-0.005, 0.000, 0.005]))
    for angle in (139, 151, 163, 172, 180):
        if mode == "no_crimp" and angle > 163:
            break
        p.add(0.95, "Finish folding and squeeze the crimp ring", right=catch(angle, 0.15, [-0.005, 0.008, -0.009]))
    p.add(1.0, "Hold the squeeze while the dough rim takes a set" if mode == "demo" else "Pause without applying the final crimp", right=catch(180 if mode == "demo" else 161, 0.15, [-0.004, 0.008, -0.016]))
    p.add(4.35, "Open the mold without pulling the folded dough apart",
          right=catch(0, 0.15, [-0.005, -0.003, 0.008]))
    p.add(0.75, "Release the empty upper die", right=catch(0, 0.89, [0, -0.009, 0.031]))
    p.add(1.25, "Release the lower tab and raise the left gripper",
          left=(lower + [0.000, 0.038, 0.130], LEFT, 0.95),
          right=catch(0, 0.90, [-0.025, -0.087, 0.077]))
    return p.moves
