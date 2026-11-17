"""Export printable OpenSCAD unions from the exact native solid inventory."""

import argparse
import json
import re
from pathlib import Path

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("--inventory", type=Path, default=Path("scene-inventory.json"))
p.add_argument("--output", type=Path, default=Path("reconstruction"))
a = p.parse_args()
records = json.loads(a.inventory.read_text())
groups = {}
if isinstance(records, dict):
    records = records.get("geometry", records.get("records", records))
for r in records:
    bits = r["solid"].split("/")
    if "Grains" in bits:
        continue
    group = bits[2]
    groups.setdefault(group, []).append(r)
a.output.mkdir(parents=True, exist_ok=True)
lines = [
    "// Original task reconstruction,MIT license. Units:mm.",
    "// Select part; F6 render; exportSTL. Standard hardware specified inREADME.",
    'part = "Outer";',
    "$fn=64;",
]
for group, items in groups.items():
    name = re.sub(r"[^A-Za-z0-9_]", "_", group)
    lines.append(f"module {name}() {{ union() {{")
    for r in items:
        xyz = [round(float(x) * 1000, 6) for x in r.get("position", [0, 0, 0])]
        prefix = f"translate({json.dumps(xyz)}) "
        if r["kind"] == "box":
            shape = f"cube({json.dumps([round(float(x) * 1000, 6) for x in r['size']])},center=true);"
        elif r["kind"] == "cylinder":
            axis = r.get("axis", "Z")
            prefix += {"X": "rotate([0,90,0]) ", "Y": "rotate([90,0,0]) ", "Z": ""}[
                axis
            ]
            shape = f"cylinder(r={r['radius'] * 1000:g},h={r['length'] * 1000:g},center=true);"
        elif r["kind"] == "convex_mesh":
            prefix = ""
            pts = [[round(float(x) * 1000, 6) for x in v] for v in r["vertices"]]
            faces = [t[::-1] for t in r["triangles"]]
            shape = f"polyhedron(points={json.dumps(pts)},faces={json.dumps(faces)},convexity=10);"
        else:
            raise ValueError(f"Unsupported reconstruction solid:{r['kind']}")
        lines.append("  " + prefix + shape)
    lines.append("} }")
    lines.append(f'if(part=="{name}") {name}();')
lines += ['if(part=="granule") sphere(r=3.4);']
(a.output / "sampler.scad").write_text("\n".join(lines) + "\n")
(a.output / "parts.json").write_text(
    json.dumps(
        {
            k: {"solids": len(v), "units": "mm", "source": "scene-inventory.json"}
            for k, v in groups.items()
        },
        indent=2,
    )
    + "\n"
)
print(
    "Printable union models:",
    ", ".join(groups),
    "and granule; exact native primitives,mm units.",
)
