"""Place a fingertip against each rider, slide it to its index and withdraw."""
import numpy as np

from . import config as C
from .motion import Program, rotation

TOP_DOWN = rotation("Y", np.deg2rad(-45)) @ rotation("Z", np.pi) @ np.diag([-1., 1., -1.])


def waypoints(mode="demo"):
    action = Program()
    if mode == "hold":
        return action.moves
    for r in C.RIDERS:
        travel = r.target
        if mode == "coarse" and r.name == "Grams":
            break
        plane = C.PIVOT[2] + r.z + r.size[2] / 2
        rise = -r.start_y * np.sin(np.deg2rad(C.LIMIT_DEG))
        # The pads touch above the stationary rail. The low torque near balance
        # lets the beam settle back as the last rider approaches its position.
        y_face = C.PIVOT[1] + r.start_y + r.size[1] / 2
        start = np.array([C.PIVOT[0] + r.x, y_face + .030, plane + rise + .003])
        finish = start + [0., travel - (.026 if r.name == "Tens" else .013), 0.]
        for dt, label, target in [
            (1.55, "approach " + r.name, start + [0., 0., .095]),
            (1.0, "lower fingertip by the " + r.name + " poise", start),
            (2.45, "slide " + r.name + " poise on its beam", finish),
            (.35, "hold near the scale index", finish),
            (.36, "back away from the " + r.name + " poise", finish + [0., .018, .012]),
            (.7, "clear the " + r.name + " poise", finish + [0., .018, .09]),
        ]:
            action.add(dt, label, right=(target, TOP_DOWN, 0.))
    action.add(1., "clear the scale and let the pointer settle",
               right=(np.array([.30, -.29, 1.02]), TOP_DOWN, 1.))
    action.add(2.4, "read the unsupported pointer")
    return action.moves
