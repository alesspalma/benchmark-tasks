#!/usr/bin/env python3
"""Author/check USD with installed native schema libraries, without starting Isaac Sim."""

import argparse
import hashlib
import json
import os
import socket
import sys
from importlib.metadata import distribution
from pathlib import Path


def bootstrap():
    cache = Path(distribution("isaacsim-kernel").locate_file("isaacsim/extscache"))
    (usd,) = cache.glob("omni.usd.libs-*")
    (physx,) = cache.glob("omni.usd.schema.physx-*")
    env = os.environ.copy()
    additions = {
        "PYTHONPATH": [usd, physx],
        "LD_LIBRARY_PATH": [usd / "bin", physx / "bin"],
        "PXR_PLUGINPATH_NAME": [
            physx / "plugins" / name / "resources"
            for name in ("PhysxSchema", "PhysxSchemaAddition")
        ],
    }
    for name, paths in additions.items():
        for path in paths:
            if not path.is_dir():
                raise FileNotFoundError(path)
        env[name] = ":".join(map(str, paths)) + (
            ":" + env[name] if env.get(name) else ""
        )
    os.execve(
        sys.executable,
        [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:], "--worker"],
        env,
    )


def execute(output):
    from pxr import Usd, UsdPhysics, UsdUtils

    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    from capsule_recovery.builder import create_assets

    specification = create_assets(output)
    paths = {x["name"]: x["file"] for x in specification["assets"]}
    records = []
    for path in paths.values():
        stage = Usd.Stage.Open(str(path))
        checker = UsdUtils.ComplianceChecker()
        checker.CheckCompliance(str(path))
        errors, failed, joints = (
            list(checker.GetErrors()),
            list(checker.GetFailedChecks()),
            [],
        )
        for prim in stage.Traverse():
            if prim.IsA(UsdPhysics.Joint):
                joint = UsdPhysics.Joint(prim)
                targets = [
                    *joint.GetBody0Rel().GetTargets(),
                    *joint.GetBody1Rel().GetTargets(),
                ]
                errors += [
                    f"Unresolved joint target {target}"
                    for target in targets
                    if not stage.GetPrimAtPath(target)
                ]
                joints.append(
                    {"path": str(prim.GetPath()), "bodies": list(map(str, targets))}
                )
        records.append(
            {
                "file": path.name,
                "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "errors": errors,
                "failed_checks": failed,
                "joints": joints,
                "structure_passed": not errors and not failed,
            }
        )
    receipt = {
        "scope": "Offline USD/schema structure only; no physics or feasibility claim",
        "hostname": socket.gethostname(),
        "assets": records,
    }
    (output / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))
    if not all(record["structure_passed"] for record in records):
        raise SystemExit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not args.worker:
        bootstrap()
    execute(args.output)
