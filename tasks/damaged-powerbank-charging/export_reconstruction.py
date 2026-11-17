"""Export actual native geometry as millimetre OpenSCAD parts and inventory."""

import argparse
import json
from pathlib import Path


def number(v):
    return json.dumps(v, separators=(",", ":"))


def solid(row):
    if row["kind"] == "box":
        return "translate("+number([1000*x for x in row["position"]])+") cube("+number([1000*x for x in row["size"]])+",center=true);"
    if row["kind"] == "convex_mesh":
        return "polyhedron(points="+number([[1000*x for x in p] for p in row["vertices"]])+",faces="+number([list(reversed(f)) for f in row["triangles"]])+");"
    if row["kind"] == "cylinder":
        axis = row.get("axis", "Z")
        rot = "rotate([0,90,0]) " if axis == "X" else ("rotate([90,0,0]) " if axis == "Y" else "")
        return "translate("+number([1000*x for x in row["position"]])+") "+rot+"cylinder(r="+str(1000*row["radius"])+",h="+str(1000*row["length"])+",center=true,$fn=40);"
    raise ValueError(row["kind"])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--inventory", type=Path, required=True)
    p.add_argument("--output", type=Path, default=Path("reconstruction"))
    a = p.parse_args()
    rows = json.loads(a.inventory.read_text())
    a.output.mkdir(parents=True, exist_ok=True)
    (a.output/"native-geometry.json").write_text(json.dumps(rows, separators=(",", ":"))+"\n")
    groups = {}
    for row in rows:
        path = row["solid"].split("/")
        body = path[2] if len(path) > 2 else path[-1]
        name = row["asset"]+"_"+body
        if "ACBlade" in row["solid"]:
            name += "_"+path[-1]
        # Cosmetic detail does not become a printed obstruction to an opening.
        if any(x in row["solid"] for x in ("LED", "USBPort", "USBContact", "StatusButton", "GroundOpening")):
            continue
        groups.setdefault(name, []).append(row)
    text = ["// Millimetres. Original native task geometry. MIT.",
            'part="intact_rear";', "$fn=40;"]
    for name, members in groups.items():
        text.append("module "+name+"(){ union(){")
        text.extend(solid(row) for row in members)
        text.append("}}")
    text += [
        "module shell(name){",
        'if(name=="intact") intact_Body(); else damaged_Body();',
        "}",
        "module rear(name){difference(){intersection(){shell(name);translate([50,0,0])cube([104,100,100],center=true);}cube([18,45,50],center=true);",
        "for(y=[-6.35,6.35]) translate([15.0,y,0])cube([14,1.65,6.5],center=true);",
        "for(y=[-25,25],z=[-26,26])translate([0,y,z])rotate([0,90,0])cylinder(d=2.4,h=50,center=true);}}",
        "module front(name){difference(){intersection(){shell(name);translate([-52,0,0])cube([100,100,100],center=true);}cube([18,45,50],center=true);",
        "for(y=[-25,25],z=[-26,26])translate([0,y,z])rotate([0,90,0])cylinder(d=2.4,h=50,center=true);}}",
        'if(part=="intact_rear")rear("intact");',
        'if(part=="intact_front")front("intact");',
        'if(part=="damaged_rear")rear("damaged");',
        'if(part=="damaged_front")front("damaged");',
        'if(part=="intact")intact_Body();',
        'if(part=="damaged")damaged_Body();',
        'if(part=="socket_body")socket_Socket();',
        'if(part=="contact_leaf")socket_Leaf0_0_P();',
        'if(part=="blade")translate([-15.494,6.350,0])intact_Body_ACBlade0();',
        'if(part=="ceramic_pad")cube([110,120,8],center=true);',
    ]
    (a.output/"parts.scad").write_text("\n".join(text)+"\n")
    (a.output/"parts-index.json").write_text(json.dumps({
        "units":"millimetres",
        "parts":list(groups),
        "source_inventory_sha256":__import__("hashlib").sha256(a.inventory.read_bytes()).hexdigest(),
        "build_note":"External models follow native geometry; hollow split shells and blade recesses support inert ballast/LED assembly. Use ordinary metal plug blades and a disconnected Type-A receptacle or specified sprung contacts. Cosmetic port/lens openings must be drilled/fitted during assembly."
    },indent=2)+"\n")
    print("Exported",len(rows),"native solids and",len(groups),"body groups")


if __name__ == "__main__":
    main()
