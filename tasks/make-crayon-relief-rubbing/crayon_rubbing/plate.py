"""The printable fern plate and the coordinates shared with the native scene."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np

from . import config


def fern():
    """Convex raised silhouettes in plate-local (depth, width) coordinates."""
    parts = [np.array([[-.053, -.002], [.049, -.0015], [.054, 0],
                       [.049, .0015], [-.053, .002]])]
    for root, reach in ((-.040, .042), (-.018, .039), (.004, .033), (.023, .024)):
        leaf = np.array([[root, .003], [root - .002, .016],
                         [root + .010, reach], [root + .017, .020],
                         [root + .013, .003]])
        parts.extend((leaf[::-1], leaf * [1., -1.]))
    parts.append(np.array([[.037, -.004], [.050, -.008], [.058, 0.],
                           [.050, .008], [.037, .004]]))
    return parts


def margins():
    """Raised margins support the bracing hand without confining the paper."""
    lip = np.array([[-.059, .079], [.059, .079], [.059, .098], [-.059, .098]])
    return [lip, (lip * [1., -1.])[::-1]]


def relief_at(points, *, include_margins=False):
    """Project the actual relief outline onto a set of plate-local XY points."""
    inside = np.zeros(len(points), dtype=bool)
    for poly in fern() + (margins() if include_margins else []):
        edge = np.roll(poly, -1, axis=0) - poly
        delta = points[:, None, :] - poly[None, :, :]
        side = edge[:, 0] * delta[:, :, 1] - edge[:, 1] * delta[:, :, 0]
        inside |= np.all(side >= -1e-10, axis=1) | np.all(side <= 1e-10, axis=1)
    return inside


def openscad():
    """Generate the physical plate in millimetres."""
    width, length = 1000 * np.array(config.PLATE_SIZE)
    base, rise = 1000 * np.array([config.PLATE_BASE, config.RELIEF])
    lines = [
        "// Fern rubbing plate. Print flat, relief up, at 0.20 mm layer height.",
        f"translate([-{width / 2:g}, -{length / 2:g}, 0]) cube([{width:g}, {length:g}, {base:g}]);",
    ]
    for part in fern():
        points = ", ".join(f"[{x:g}, {y:g}]" for x, y in 1000 * part)
        lines.append(f"translate([0,0,{base - .05:g}]) linear_extrude({rise + .05:g}) polygon([{points}]);")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output", type=Path)
    parser.add_argument("--overlay", type=Path, help="write a square-on 5 px/mm A5 guide")
    args = parser.parse_args()
    args.output.write_text(openscad())
    if args.overlay:
        from PIL import Image, ImageDraw

        pixels_per_m = 5000
        depth, width = np.round(config.PAPER_SIZE * pixels_per_m).astype(int)
        canvas = Image.new("RGB", (int(width), int(depth)), "white")
        draw = ImageDraw.Draw(canvas)
        for part in fern():
            path = [(width / 2 - y * pixels_per_m, depth / 2 - x * pixels_per_m)
                    for x, y in part]
            draw.polygon(path, fill=(190, 204, 214))
        for v in range(0, int(depth), 10):
            draw.line((0, v, width - 1, v), fill=(223, 223, 223))
        for u in range(0, int(width), 10):
            draw.line((u, 0, u, depth - 1), fill=(223, 223, 223))
        canvas.save(args.overlay, dpi=(127, 127))


if __name__ == "__main__":
    main()
