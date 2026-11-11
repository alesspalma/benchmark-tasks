"""Offline IK only, never a substitute for actual native feasibility."""

import argparse, json
from chain_link.controller import ScriptedPolicy

p = argparse.ArgumentParser()
p.add_argument("--mode", default="demo")
a = p.parse_args()
s = ScriptedPolicy(mode=a.mode)
s.reset(None)
for i in range(1, int(s.duration * 20) + 1):
    s.action_at(i / 20)
print(
    json.dumps(
        dict(
            scope="offline IK only",
            mode=a.mode,
            duration_s=s.duration,
            worst_position_m=s.worst_position_error,
            worst_rotation_rad=s.worst_rotation_error,
        )
    )
)
