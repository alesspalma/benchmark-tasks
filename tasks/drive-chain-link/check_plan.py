"""Screen nominal IK and Inspect guards; native execution proves feasibility."""

import argparse
import json

import numpy as np
from chain_link.controller import ScriptedPolicy

parser = argparse.ArgumentParser()
parser.add_argument("--mode", default="demo")
a = parser.parse_args()
p = ScriptedPolicy(mode=a.mode)
actions = np.array([p.action_at(i / 20) for i in range(1, round(p.duration * 20) + 1)])
print(
    json.dumps(
        {
            "scope": "offline nominal IK and robot command screen only",
            "mode": a.mode,
            "actions": len(actions),
            "duration_s": p.duration,
            "worst_ik_m": p.worst_ik_m,
            "worst_ik_rad": p.worst_ik_rad,
            "maximum_step_change": np.abs(np.diff(actions, axis=0))
            .max(axis=0)
            .tolist(),
            "program_sha256": p.program_sha256,
            "runtime_inputs": ["elapsed time", "measured robot joints"],
        },
        indent=2,
    )
)
