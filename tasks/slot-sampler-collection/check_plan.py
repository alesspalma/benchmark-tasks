#!/usr/bin/env python3
"""Screen every reference command with URDF IK and standard action bounds."""

import json

import numpy as np
from slot_sampler.config import MAX_STEPS
from slot_sampler.controller import ScriptedPolicy

p = ScriptedPolicy()
previous = None
max_delta = 0.0
for k in range(int(np.ceil(p.duration * 20)) + 1):
    action = p.action_at(k / 20)
    if not np.isfinite(action).all():
        raise RuntimeError("Nonfinite command")
    if np.any(action < p.info.action_space.low - 1e-8) or np.any(
        action > p.info.action_space.high + 1e-8
    ):
        raise RuntimeError("Joint limit exceeded")
    if previous is not None:
        max_delta = max(max_delta, float(np.max(abs(action - previous))))
    previous = action
if p.duration * 20 > MAX_STEPS:
    raise RuntimeError("Program exceeds task action limit")
if p.worst_ik_m > 0.002 or p.worst_ik_rad > 0.03:
    raise RuntimeError(f"Unreachable command: {p.worst_ik_m}, {p.worst_ik_rad}")
if max_delta > 0.040001:
    raise RuntimeError("Default Inspect delta guard exceeded")
print(
    json.dumps(
        {
            "scope": "Kinematics and command guards only; native feasibility is separate",
            "steps": k + 1,
            "duration_s": p.duration,
            "worst_ik_m": p.worst_ik_m,
            "worst_ik_rad": p.worst_ik_rad,
            "max_joint_step": max_delta,
        },
        indent=2,
    )
)
