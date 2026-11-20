"""Original printable recipe pages and their native USD face materials."""

from math import cos, radians, sin

from PIL import Image, ImageDraw, ImageFont


PAPER = (245, 232, 201)
OLIVE = (36, 74, 44)
INK = (38, 40, 31)
YELLOW = (240, 191, 66)


def art(page_number):
    page = Image.new("RGB", (500, 710), PAPER)
    draw = ImageDraw.Draw(page)

    def label(x, y, words, size, color=INK):
        draw.text((x, y), words, font=ImageFont.load_default(size), fill=color)

    label(44, 31, "FROM THE OVEN", 26, OLIVE)
    draw.line((44, 71, 456, 71), fill=OLIVE, width=3)
    label(42, 92, "Lemon", 63, OLIVE)
    label(42, 159, "Potatoes", 63, OLIVE)
    if page_number == 24:
        draw.rounded_rectangle((43, 263, 458, 313), radius=11, fill=OLIVE)
        label(62, 272, "INGREDIENTS  /  SERVES 4", 27, PAPER)
        for i, ingredient in enumerate((
            "800 g small yellow potatoes", "3 tbsp olive oil",
            "1 lemon, zest and juice", "2 rosemary sprigs",
            "1 tsp flaky sea salt"
        )):
            label(48, 347 + i * 46, ingredient, 27)
    else:
        draw.ellipse((105, 237, 397, 520), fill=(209, 179, 135), outline=OLIVE, width=4)
        draw.ellipse((118, 249, 383, 505), fill=(232, 221, 190), outline=(139, 129, 80), width=4)
        for i in range(15):
            a = 2.39996 * i
            r = 89 * (i / 14) ** .5
            x, y = 249 + r * cos(a), 377 + r * sin(a)
            draw.ellipse((x - 28, y - 18, x + 30, y + 17), fill=YELLOW,
                         outline=(145, 82, 31), width=3)
        for i in range(11):
            a = radians(i * 83)
            x, y = 250 + 69 * cos(a), 377 + 67 * sin(a)
            draw.line((x, y, x + 15 * cos(a), y + 15 * sin(a)), fill=OLIVE, width=4)
        label(47, 544, "Toss with the oil and herbs.", 25)
        label(47, 581, "Roast 35 minutes at 220 C.", 25)
    draw.line((44, 641, 456, 641), fill=OLIVE, width=2)
    label(225, 660, str(page_number), 24, OLIVE)
    return page


def print_face(w, stage, body, *, center_y, width, height, front, page_number):
    from pxr import Sdf

    name = f"Page{page_number}"
    path = w.directory / (name + ".png")
    art(page_number).save(path)
    geom = w.Geom.Mesh.Define(stage, body + "/Print")
    y0, y1 = center_y + width / 2, center_y - width / 2
    geom.CreatePointsAttr([(front, y0, .002), (front, y1, .002),
                           (front, y1, height), (front, y0, height)])
    geom.CreateFaceVertexCountsAttr([4])
    geom.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    geom.CreateSubdivisionSchemeAttr("none")
    uv = w.Geom.PrimvarsAPI(geom).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                              w.Geom.Tokens.vertex)
    uv.Set([(0., 0.), (1., 0.), (1., 1.), (0., 1.)])
    root = "/Asset/Print" + name
    mat = w.Shade.Material.Define(stage, root)
    st = w.Shade.Shader.Define(stage, root + "/coordinates")
    st.CreateIdAttr("UsdPrimvarReader_float2")
    st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    img = w.Shade.Shader.Define(stage, root + "/page")
    img.CreateIdAttr("UsdUVTexture")
    img.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(path))
    img.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    img.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
    rgb = img.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
    surf = w.Shade.Shader.Define(stage, root + "/paper")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(rgb)
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(.8)
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    w.Shade.MaterialBindingAPI.Apply(geom.GetPrim()).Bind(mat)
