"""Print-ready clipboard sheet and its measurable black-ink areas."""

from pathlib import Path

from . import config as C

# Each digit is a five-pixel row, most significant bit on the left.
GLYPHS = {
    "A": (14, 17, 17, 31, 17, 17, 17), "C": (14, 17, 16, 16, 16, 17, 14),
    "D": (30, 17, 17, 17, 17, 17, 30), "G": (14, 17, 16, 23, 17, 17, 14),
    "H": (17, 17, 17, 31, 17, 17, 17), "I": (14, 4, 4, 4, 4, 4, 14),
    "K": (17, 18, 20, 24, 20, 18, 17), "L": (16, 16, 16, 16, 16, 16, 31),
    "O": (14, 17, 17, 17, 17, 17, 14), "P": (30, 17, 17, 30, 16, 16, 16),
    "S": (15, 16, 16, 14, 1, 1, 30), "T": (31, 4, 4, 4, 4, 4, 4),
    "F": (31, 16, 16, 30, 16, 16, 16), "M": (17, 27, 21, 21, 17, 17, 17),
    "W": (17, 17, 17, 21, 21, 27, 17), "U": (17, 17, 17, 17, 17, 17, 14),
    "E": (31, 16, 16, 30, 16, 16, 31), "N": (17, 25, 21, 19, 17, 17, 17),
    "R": (30, 17, 17, 30, 20, 18, 17), " ": (0, 0, 0, 0, 0, 0, 0),
}
PAGE = (.79, .85, .91)
INK = (.029, .060, .111)
RULE = (.33, .44, .59)
MARK = (.92, .34, .17)
COATING = (.98, .98, .96)


def word(text, x, y, pitch):
    width = (6 * len(text) - 1) * pitch
    return [
        (x - width / 2 + (6 * letter + col) * pitch,
         y + (2.5 - row) * pitch, pitch, pitch)
        for letter, char in enumerate(text)
        for row, bits in enumerate(GLYPHS[char])
        for col in range(5)
        if bits & (16 >> col)
    ]


def entries():
    return [word(text, x, C.ROW_Y, .0004)
            for text, x in zip(C.ENTRY_NAMES, C.ENTRY_CENTERS)]


def artwork():
    highlight = [(-.006, C.ROW_Y - .00425, .012, .00055)]
    rules = [(-.067, y, .134, .00028) for y in (-.034, -.023, -.012, .002, .015, .026)]
    ink = word("STOCK LOG", 0., .045, .00083)
    for name, x in zip(("MON", "TUE", "WED"), C.ENTRY_CENTERS):
        ink.extend(word(name, x, .021, .00052))
    return ((RULE, rules), (MARK, highlight), (INK, ink + sum(entries(), [])))


def write_svg(path):
    # User-space metres are converted to physical millimetres for 1:1 printing.
    lx, ly = (1000 * dim for dim in C.PAPER_SIZE)
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{lx:g}mm" '
           f'height="{ly:g}mm" viewBox="0 0 {lx:g} {ly:g}">']
    groups = ((PAGE, [(-C.PAPER_SIZE[0] / 2, -C.PAPER_SIZE[1] / 2, *C.PAPER_SIZE)]),) + artwork()
    for color, rects in groups:
        color = "#" + "".join(f"{round(255*c):02x}" for c in color)
        for x, y, w, h in rects:
            svg.append(f'<rect x="{lx/2+1000*x:.3f}" y="{ly/2-1000*(y+h):.3f}" '
                       f'width="{1000*w:.3f}" height="{1000*h:.3f}" fill="{color}"/>')
    svg.append("</svg>")
    Path(path).write_text("\n".join(svg) + "\n")
