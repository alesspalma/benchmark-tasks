# Construction inventory and CAD

The task supplies the dimensions, material assignments, initial transforms and
joint definitions for its custom service fixture, guided applicator and two
retaining clips. The files describe the same geometry used by the submitted
scripted and Dream-1 runs.

| File | Contents |
| --- | --- |
| [../scene-inventory.json](../scene-inventory.json) | Compact construction inventory: 22 rigid bodies, all 18 joints, all 120 applicator/fixture solids, world starting poses, materials and clip dimensions. |
| [../setup-layout.json](../setup-layout.json) | Standard station, layout, component specifications and reset. |
| [native-solids.json.gz](native-solids.json.gz) | Exact original 2,782-record native inventory, including every convex sector vertex. |
| [parts.scad](parts.scad) | OpenSCAD models generated from those exact native solid records. Units are millimetres. |

The source code remains the authoritative executable construction:
`python check_assets.py --output /tmp/chain-assets` writes
`/tmp/chain-assets/geometry-inventory.json`. Its bytes match the decompressed
inventory, SHA256
`bae4e5a4c390f6c0e9c9cea5112a2ffafbc14385d48da7f6d6ef12f1823c0e0b`.

## Frames and fixed layout

Inventory lengths are in metres; mass is kilograms. Quaternions are XYZW.
`asset_world_starts_position_and_quaternion` gives each asset's world pose
before the 0.5-second settling period. Each body has an asset-local position
and yaw; each solid's centre or mesh vertices are body-local. All movable
joints start at zero. The `fixtures` asset has the identity world transform,
so its listed centres are already world positions.

| Component | Starting world position, metres | Starting quaternion XYZW |
| --- | --- | --- |
| Yellow connecting link | (0.400, −0.350, 0.8975) | (0, 1, 0, 0) |
| Blue closed-hole cover | (0.500, 0.350, 0.8975) | (0, 0, 0, 1) |
| Applicator mount | (0.310, 0.030, 0.9249) | (0, 0, 1, 0) |
| Clip 0 | (0.310, 0.080, 0.9249) | (0, 0, −0.70710678, 0.70710678) |
| Clip 1 | (0.310, −0.020, 0.9249) | (0, 0, −0.70710678, 0.70710678) |

The root pose and all eleven local body poses of the immobilized chain are
listed explicitly in the inventory. They correspond to the twelve world
joint-node positions in `setup-layout.json`. Tightened end nests and five
hold-down straps retain this service layout throughout a trial.

## Material and mechanism specification

The chain, connecting pins, blue cover and locating fixture are nominal steel.
The applicator has a hollow polymer grip, aluminium frame/carriage and steel
clip channels. Its fixed pedestal and linear guide can be assembled from
printed rigid parts and ordinary linear bearings or sliding rails with the
listed travel stops. Preserve the channel clearance and pin/cover geometry.
Keep metal edges touching the gripper pads blunt or covered.

| Mechanism | Exact construction parameters |
| --- | --- |
| Connecting link | 100 mm pin pitch; 12 mm pin diameter; 56 mm tip height from backing; 2 mm × 45° leads ending in an 8 mm flat; 10.6 mm groove diameter, 1.4 mm groove width, centres 47.8 mm from the backing. |
| Cover | 35 × 135 × 5 mm envelope; two 12.2 mm closed bores 100 mm apart; 0.5 mm entry chamfers on both faces. |
| Each clip | 1 mm spring-steel sheet; 14.8 mm outer envelope; back ring radii 5.4/7.4 mm over 125–235°; leaves radii 5.4/6.1 mm over 55–125° and 235–305°; bearing ears radii 6.1/7.4 mm over 55–75° and 285–305°. |
| Applicator channels | Loose clips rest in open-front channels. Listed floor/side/0.2 mm guide-cap solids prevent out-of-plane escape and leave the front open for withdrawal after fitting. Every channel solid has its exact dimensions, centre and yaw in the inventory. |
| Slide X | `/Tool/Mount` to `/Tool/XSlide`, local X translation 0–45 mm, passive bearing, joint friction 0.02, no drive. |
| Slide Y | `/Tool/XSlide` to `/Tool/Body`, local Y translation 0–30.6 mm, passive bearing, joint friction 0.02, no drive. |
| Clip compliance | Two native Z hinges per clip, anchored at radius 6 mm and angles 125°/235°; limits ±25°; constant spring 0.8 N·m/rad and damping 0.003 N·m·s/rad, closed rest angle. These represent the leaf compliance of the continuous steel outline. |
| Fixed connections | Ten chain-brace joints, one chain-to-bench joint and one applicator-to-bench joint. The native mount is a mass/constraint anchor; the visible pedestal, guide and rails are separate fixture solids. |

All 18 joint records include their parent/child relationships, local frame
positions/rotations, axis, limits and authored drive/friction attributes.
Native angular-drive stiffness/damping values are per degree; the engineering
values in the table are per radian. The guide constrains applicator height and
orientation; ordinary robot contact supplies both horizontal fitting strokes.

## Affordable fabrication route

Use `parts.scad` in OpenSCAD. Select an asset for a closed-position model, or
an individual body/fixture solid for fabrication. Example exports:

```bash
openscad -o applicator-body.stl -D 'part="Tool_Body"' parts.scad
openscad -o carriage.stl -D 'part="Tool_XSlide"' parts.scad
openscad -o connector.stl -D 'part="Connector"' parts.scad
openscad -o cover.stl -D 'part="Keeper"' parts.scad
openscad -o clip-outline.stl -D 'part="Clip0"' parts.scad
openscad -o nest-floor.stl -D 'part="Fixtures_rightNestFloor"' parts.scad
```

Individual fixture primitives export centred and unrotated; assemble them at
the centres/yaws in the inventory. Asset/body exports preserve their local
geometry. Large fixed fixtures can be built from separately printed pieces
on an ordinary board; they do not require a full-size industrial conveyor.

The clip's CAD model supplies the continuous closed outline for fabrication
from spring-steel sheet. Its three simulated bodies are a bending model, not
three loose hardware pieces to connect with powered hinges. A plastic print
checks the outline and clearances; use a repeatable compliant retainer that
closes into the specified groove for the physical trial. Bench-test seating,
retention and guided withdrawal before a robot trial. Nominal component
masses may be matched with fixed ballast clear of contact paths. CAD and
drawings specify the apparatus; no built hardware trial is claimed.

All supplied models and geometry are original and MIT licensed. Manufacturer
references document the joining procedure and dimensional context; no
manufacturer CAD is redistributed.
