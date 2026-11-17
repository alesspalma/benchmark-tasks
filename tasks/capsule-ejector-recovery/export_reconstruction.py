#!/usr/bin/env python3
"""Generate millimetre OpenSCAD parts from the native USD geometry and transforms."""
import argparse
import json
import os
import sys
from importlib.metadata import distribution
from pathlib import Path


def bootstrap():
    cache = Path(distribution('isaacsim-kernel').locate_file('isaacsim/extscache'))
    usd, = cache.glob('omni.usd.libs-*')
    physx, = cache.glob('omni.usd.schema.physx-*')
    env = os.environ.copy()
    for name, paths in {
        'PYTHONPATH': [usd, physx],
        'LD_LIBRARY_PATH': [usd / 'bin', physx / 'bin'],
        'PXR_PLUGINPATH_NAME': [physx / 'plugins' / n / 'resources'
                               for n in ('PhysxSchema', 'PhysxSchemaAddition')],
    }.items():
        env[name] = ':'.join(map(str, paths)) + (':' + env[name] if env.get(name) else '')
    os.execve(sys.executable, [sys.executable, str(Path(__file__).resolve()),
                             *sys.argv[1:], '--worker'], env)


def number(value):
    return round(float(value), 7)


def export(args):
    import numpy as np
    from capsule_recovery import config as C
    from pxr import Usd, UsdGeom

    args.output.mkdir(parents=True, exist_ok=True)
    origin = np.array([0.50, 0.0, C.TOP])
    groups = {}
    counts = {}
    inventory = []
    for filename in ('machine.usda', 'bin.usda', 'fresh.usda', 'fixtures.usda'):
        stage = Usd.Stage.Open(str(args.assets / filename))
        cache = UsdGeom.XformCache()
        for prim in stage.Traverse():
            kind = ('box' if prim.IsA(UsdGeom.Cube) else 'cylinder'
                    if prim.IsA(UsdGeom.Cylinder) else 'mesh'
                    if prim.IsA(UsdGeom.Mesh) else None)
            if kind is None:
                continue
            path = str(prim.GetPath())
            if filename == 'machine.usda':
                part = 'Head' if path.startswith('/Asset/Head/') else 'Base'
                offset = origin
            elif filename == 'bin.usda':
                part, offset = 'Collector', np.zeros(3)
            elif filename == 'fresh.usda':
                part, offset = 'Capsule', np.zeros(3)
            elif path.startswith('/Asset/Recycling/'):
                part, offset = 'RecyclingTray', C.WASTE
            else:
                part, offset = 'FreshStand', np.array([*C.FRESH[:2], C.TOP])
            matrix = np.asarray(cache.GetLocalToWorldTransform(prim), dtype=float)
            transform = matrix.T.copy()
            transform[:3, 3] = (transform[:3, 3] - offset) * 1000
            if kind == 'box':
                size = float(UsdGeom.Cube(prim).GetSizeAttr().Get()) * 1000
                solid = f'cube([{size},{size},{size}],center=true);'
            elif kind == 'cylinder':
                obj = UsdGeom.Cylinder(prim)
                radius = number(float(obj.GetRadiusAttr().Get()) * 1000)
                height = number(float(obj.GetHeightAttr().Get()) * 1000)
                axis = obj.GetAxisAttr().Get()
                rotate = {'X': 'rotate([0,90,0]) ', 'Y': 'rotate([90,0,0]) ', 'Z': ''}[axis]
                solid = f'{rotate}cylinder(r={radius},h={height},center=true);'
            else:
                obj = UsdGeom.Mesh(prim)
                points = (np.asarray(obj.GetPointsAttr().Get(), dtype=float) * 1000).tolist()
                sizes = list(obj.GetFaceVertexCountsAttr().Get())
                indices = list(obj.GetFaceVertexIndicesAttr().Get())
                faces, at = [], 0
                for size in sizes:
                    # OpenSCAD uses clockwise exterior winding; USD uses right-handed.
                    faces.append([int(i) for i in indices[at:at + size]][::-1])
                    at += size
                solid = f'polyhedron(points={json.dumps(points)},faces={json.dumps(faces)},convexity=12);'
            # Scale of USD cubes is already in the matrix. Translation changes to
            # millimetres; primitive dimensions change to millimetres once only.
            command = 'multmatrix(' + json.dumps([[number(v) for v in row] for row in transform]) + ') ' + solid
            if prim.GetName() == 'HingeShaft':
                # Hardware shaft is separate; model the internal bearing as bores.
                part = 'HingeShaft'
            groups.setdefault(part, []).append(command)
            counts[part] = counts.get(part, 0) + 1
            inventory.append({'file': filename, 'prim': path, 'part': part,
                              'kind': kind, 'transform_mm': transform.tolist()})
    head_pivot = ((C.HINGE - origin) * 1000).tolist()
    lines = [
        '// Original reconstruction generated from native task USD; MIT. Units:mm.',
        'part="Collector";', '$fn=64;',
    ]
    for name, solids in groups.items():
        lines.append(f'module {name}Raw(){{union(){{\n' + '\n'.join(solids) + '\n}}')
    lines += [
        f'hinge={json.dumps(head_pivot)};',
        'module HingeBore(){translate(hinge) rotate([90,0,0]) cylinder(d=6.2,h=240,center=true);}',
        'module StopMountBores(){for(x=[-16,16]) translate(hinge+[x,0,20]) rotate([90,0,0]) cylinder(d=3.4,h=240,center=true);}',
        'module ArmMountBores(){for(x=[-5,5]) translate(hinge+[x,0,6]) rotate([90,0,0]) cylinder(d=3.4,h=180,center=true);}',
        'module Base(){difference(){BaseRaw();HingeBore();StopMountBores();}}',
        'module Head(){difference(){HeadRaw();HingeBore();ArmMountBores();}}',
    ]
    for name in groups:
        if name not in ('Base', 'Head'):
            lines.append(f'module {name}(){{{name}Raw();}}')
    lines += [
        'module MachineAssembly(open_angle=0){Base();translate(hinge) rotate([0,open_angle,0]) translate(-hinge) Head();HingeShaft();}',
        'module BaseFront(){intersection(){Base();translate([-1000,-1000,-1000]) cube([1000,2000,2000]);}}',
        'module BaseRear(){intersection(){Base();translate([0,-1000,-1000]) cube([1000,2000,2000]);}}',
        'module HeadLeft(){intersection(){Head();translate([-1000,0,-1000]) cube([2000,1000,2000]);}}',
        'module HeadRight(){intersection(){Head();translate([-1000,-1000,-1000]) cube([2000,1000,2000]);}}',
    ]
    for name in groups:
        lines.append(f'if(part=="{name}") {name}();')
    for name in ('BaseFront', 'BaseRear', 'HeadLeft', 'HeadRight'):
        lines.append(f'if(part=="{name}") {name}();')
    lines.append('if(part=="MachineAssembly") MachineAssembly();')
    (args.output / 'coffee_machine.scad').write_text('\n'.join(lines) + '\n')
    # A separate ordinary bearing/limit kit makes the native revolute constraint
    # physically reproducible. It does not help the robot perform any task step.
    hinge_kit = '''// M6 bearing and 0..85 degree stop kit. Units:mm; MIT.
part="StopPlate";
$fn=64;
module ArcSlot(){for(a=[0:1:84]) hull(){
translate([15*cos(a),0,-15*sin(a)]) rotate([90,0,0]) cylinder(d=3.4,h=8,center=true);
translate([15*cos(a+1),0,-15*sin(a+1)]) rotate([90,0,0]) cylinder(d=3.4,h=8,center=true);}}
module StopPlate(){difference(){
union(){rotate([90,0,0]) cylinder(d=44,h=4,center=true);translate([0,0,18]) cube([44,4,12],center=true);}
rotate([90,0,0]) cylinder(d=6.2,h=8,center=true);ArcSlot();
for(x=[-16,16]) translate([x,0,20]) rotate([90,0,0]) cylinder(d=3.4,h=8,center=true);}}
module StopArm(){difference(){union(){rotate([90,0,0]) cylinder(d=24,h=4,center=true);translate([7.5,0,0]) cube([15,4,8],center=true);translate([15,0,0]) rotate([90,0,0]) cylinder(d=8,h=4,center=true);}
rotate([90,0,0]) cylinder(d=6.2,h=8,center=true);translate([15,0,0]) rotate([90,0,0]) cylinder(d=3.1,h=8,center=true);
for(x=[-5,5]) translate([x,0,6]) rotate([90,0,0]) cylinder(d=3.4,h=8,center=true);}}
module Spacer(){difference(){rotate([90,0,0]) cylinder(d=10,h=4,center=true);rotate([90,0,0]) cylinder(d=6.2,h=8,center=true);}}
if(part=="StopPlate") StopPlate();
if(part=="StopArm") StopArm();
if(part=="Spacer") Spacer();
'''
    (args.output / 'hinge_kit.scad').write_text(hinge_kit)
    report = {'units': 'mm', 'native_asset_directory': args.assets.name,
              'generated_from': 'Native USD primitives and complete transforms',
              'native_primitive_counts': counts, 'hinge_axis': 'Y',
              'hinge_origin_relative_to_base_mm': head_pivot,
              'hinge_limits_degrees': [0, 85], 'shaft_diameter_mm': 6,
              'bearing_bore_mm': 6.2, 'source_inventory': inventory}
    (args.output / 'parts.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'source_inventory'}, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--assets', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--worker', action='store_true', help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not args.worker:
        bootstrap()
    export(args)
