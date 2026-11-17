# Response to TK-QAH7UF revision feedback

The main mechanism is supplied as a reproducible physical construction:

- `reconstruction/BUILD.md` specifies the collector dimensions/guides, passive lifting fork, carrier supports, M6 bearing shaft, bearing bores, stops, materials, hardware, assembly, reset and capsule stand-ins.
- `reconstruction/coffee_machine.scad` provides native-USD-derived Base, Head, Collector, Capsule, tray and stand parts, with smaller-printer splits.
- `reconstruction/hinge_kit.scad` provides a 6 mm bearing / 0–85 degree stop kit. `physical_housing.scad` includes the stop pin's clearance through the fixed right panel.
- `reconstruction/mechanism.svg` is a dimensioned side/collector/hinge drawing; `parts.json` records native primitive transforms.

The title, instruction, context and README explicitly identify a coffee machine. The native setup includes a rear reservoir, front coffee outlet, drip grille and dark appliance housing. Fixed carrier beams, hinge ears and fork supports connect the visible mechanism physically.

The revised full-score native demonstration uses the exact new scene and instruction:1.0,all7spentcapsulescollected,collectorrefitted,freshseated,head3.51degreesclosed,bothhandsreleased,in2332/2400actions. A matching single Dream-1 episode completed0.0/2400without runtimeerror. Both recordings preserve all native initial/action frames and use the requested native camera dimensions and composite formats.

The controller, motion plan, scorer, environment and geometric measurements are unchanged. Geometry changes make the appliance and construction explicit; no scoring conditions were relaxed and no additional robot handles were added.
