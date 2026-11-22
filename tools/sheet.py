#!/usr/bin/env python3
"""Render the final presentation sheet in the layout of docs/reference.png:
callout close-ups on the left, FRONT / LEFT SIDE / BACK in the middle, and on the right the
checks, measured polycount, topology legend and an exploded coat panel breakdown.

    python3 tools/sheet.py                      # -> renders/topology_sheet.png
    python3 tools/sheet.py --samples 8 -o x.png # quicker draft
"""
import argparse
import math
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

import bpy  # noqa: E402
from mathutils import Vector  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

import preview  # noqa: E402
from infantry import lib, spec  # noqa: E402
from infantry.parts import PARTS  # noqa: E402

BG = (22, 22, 26)
GOLD = (205, 175, 115)
TEXT = (226, 226, 230)
DIM = (150, 150, 158)
FONT_DIR = "/usr/share/fonts/truetype"


def font(size, bold=False, serif=False):
    name = ("dejavu/DejaVuSerif-Bold.ttf" if bold else "liberation/LiberationSerif-Regular.ttf") if serif else \
        ("dejavu/DejaVuSans-Bold.ttf" if bold else "dejavu/DejaVuSans.ttf")
    try:
        return ImageFont.truetype(os.path.join(FONT_DIR, name), size)
    except OSError:
        return ImageFont.load_default()


def cleanup():
    """Drop overlays, cameras and lights left by a previous render_views call."""
    for o in list(bpy.data.objects):
        if o.get("topo") == "wire" or o.type in {"CAMERA", "LIGHT"}:
            bpy.data.objects.remove(o)


def render(objs, views, res, samples, full=False, zoom=None, wire=True):
    cleanup()
    tiles = preview.render_views(objs, None, full=full, zoom=zoom, res=res, samples=samples, views=views,
                                 wire=wire, return_tiles=True)
    return [t[1] for t in tiles]


def on_bg(im):
    bg = Image.new("RGBA", im.size, BG + (255,))
    bg.alpha_composite(im)
    return bg.convert("RGB")


def frame(draw, box, color=(70, 66, 58), width=2):
    draw.rectangle(box, outline=color, width=width)


# (title, caption lines, zoom cx,cy,cz,half_h, view) -- positions from spec v2, left side of the body
def callouts():
    S = spec
    return [
        ("SHOULDER / ARMPIT FLOW", ["16-vert seam shared by coat", "and sleeve; plate + guard"],
         (S.SHOULDER.x + 0.02, -0.02, S.SHOULDER.z - 0.05, 0.16), "3/4"),
        ("ELBOW LOOPS", ["3 tight support loops,", "fold rolls above"],
         (S.ELBOW.x, S.ELBOW.y, S.ELBOW.z + 0.05, 0.11), "LEFT SIDE"),
        ("HAND TOPOLOGY", ["fingers welded to palm,", "loop pairs per knuckle"],
         (S.WRIST.x + 0.01, -0.05, S.WRIST.z - 0.10, 0.10), "LEFT SIDE"),
        ("NECK & COLLAR FLOW", ["gaiter rolls inside an open", "flared collar"],
         (0.0, -0.02, 1.62, 0.13), "3/4"),
        ("HAT TOPOLOGY", ["32 around, concentric brim", "loops, creased crown"],
         (0.0, -0.02, 1.80, 0.15), "3/4"),
    ]


def explode_coat(objs):
    """Keep coat torso + skirt pieces, push each one outward from the body axis."""
    keep = []
    for o in objs:
        if o.get("part") in ("torso_coat", "skirt"):
            keep.append(o)
        else:
            bpy.data.objects.remove(o)
    for o in keep:
        c = sum((Vector(v.co) for v in o.data.vertices), Vector()) / max(len(o.data.vertices), 1)
        d = Vector((c.x, c.y, 0))
        k = 0.16 if o.get("part") == "skirt" else 0.0
        if d.length > 1e-3:
            o.location = d.normalized() * k
        if "under" in o.name:
            o.location = Vector((0, 0, -0.05))
    return keep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", default=os.path.join(ROOT, "renders", "topology_sheet.png"))
    ap.add_argument("--samples", type=int, default=16)
    a = ap.parse_args()

    lib.reset_scene()
    objs, failed = preview.build_parts(PARTS)
    if failed:
        sys.exit("parts failed: %s" % failed)
    rows = preview.report(objs)
    quads = sum(r["quads"] for r in rows)
    tris = sum(r["triangles"] for r in rows)
    trif = sum(r["tris_faces"] for r in rows)
    ngons = sum(r["ngons"] for r in rows)
    nonman = sum(r["nonmanifold"] for r in rows)

    W, H = 2700, 1950
    sheet = Image.new("RGB", (W, H), BG)
    d = ImageDraw.Draw(sheet)

    # ---------------------------------------------------------------- main views
    view_w, view_h = 600, 1560
    x0 = 400
    for i, (label, im) in enumerate(zip(["FRONT", "LEFT SIDE", "BACK"],
                                        render(objs, ["FRONT", "LEFT SIDE", "BACK"], (view_w, view_h),
                                               a.samples, full=True))):
        x = x0 + i * (view_w + 20)
        sheet.paste(on_bg(im), (x, 150))
        d.line((x + 60, 1740, x + view_w - 60, 1740), fill=(90, 84, 72), width=2)
        f = font(40, serif=True)
        tw = d.textlength(label, font=f)
        d.text((x + (view_w - tw) / 2, 1755), label, font=f, fill=TEXT)

    # ---------------------------------------------------------------- callouts
    y = 210
    for title, cap, zoom, view in callouts():
        im = on_bg(render(objs, [view], (300, 250), a.samples, zoom=zoom)[0])
        sheet.paste(im, (40, y))
        frame(d, (40, y, 340, y + 250))
        d.text((40, y + 258), title, font=font(19, bold=True), fill=TEXT)
        for k, line in enumerate(cap):
            d.text((40, y + 284 + 20 * k), line.upper(), font=font(14), fill=DIM)
        y += 345

    # ---------------------------------------------------------------- title block
    d.text((40, 30), "INFANTRY", font=font(64, bold=True, serif=True), fill=TEXT)
    d.text((44, 105), "RANGED TROOPER", font=font(26, serif=True), fill=GOLD)
    d.text((44, 145), "CHARACTER TOPOLOGY TURNAROUND", font=font(18), fill=TEXT)

    # ---------------------------------------------------------------- right column
    rx = 2300
    ry = 40

    def head(t):
        nonlocal ry
        d.text((rx, ry), t, font=font(24, serif=True), fill=GOLD)
        ry += 44

    head("KEY IMPROVEMENTS")
    checks = [
        ("QUAD-DOMINANT TOPOLOGY", f"{100 * quads / max(quads + trif + ngons, 1):.1f}% quads, {ngons} n-gons"),
        ("CLEAN SHOULDER LOOPS", "shared 16-vert armhole seam"),
        ("SEPARATE COAT PANELS", "torso, front, side, back, under"),
        ("RIGID ACCESSORIES", "armour, belt, buckles, gear"),
        ("ANIMATION-READY DETAIL", "elbow, wrist, finger, knee loops"),
    ]
    for t, sub in checks:
        d.rectangle((rx, ry + 3, rx + 18, ry + 21), outline=GOLD, width=2)
        d.line((rx + 4, ry + 12, rx + 8, ry + 17, rx + 15, ry + 6), fill=GOLD, width=2)
        d.text((rx + 30, ry), t, font=font(15, bold=True), fill=TEXT)
        d.text((rx + 30, ry + 22), sub.upper(), font=font(12), fill=DIM)
        ry += 58
    ry += 10

    d.rectangle((rx - 6, ry - 6, W - 30, ry + 150), outline=(110, 96, 70), width=2)
    head("POLYCOUNT (ACTUAL)")
    for t, v in [("TRIANGLES", f"{tris:,}"), ("QUADS", f"{quads:,}"), ("NON-MANIFOLD EDGES", f"{nonman}")]:
        d.text((rx, ry), t, font=font(15), fill=TEXT)
        d.text((W - 50 - d.textlength(v, font=font(15)), ry), v, font=font(15), fill=TEXT)
        ry += 26
    d.text((rx, ry), "TARGET 48-62K TRIS / 24-31K QUADS", font=font(12), fill=DIM)
    ry += 50

    head("TOPOLOGY LEGEND")
    for kind, t in [("main", "MAIN TOPOLOGY (QUADS)"), ("support", "SUPPORT LOOPS (DEFORMATION)"),
                    ("rigid", "HARD EDGES / RIGID PARTS")]:
        c = tuple(int(255 * min(1, x * 1.6)) for x in preview.TINT[kind])
        d.rectangle((rx, ry, rx + 26, ry + 20), fill=c, outline=TEXT)
        d.text((rx + 40, ry + 2), t, font=font(14), fill=TEXT)
        ry += 34
    ry += 20

    head("COAT PANEL BREAKDOWN")
    names = [("1", "TORSO (CORE)", "coat_torso"), ("2", "SHOULDER ARMOR (RIGID)", "pauldron"),
             ("3", "FRONT PANELS (L/R)", "skirt_front"), ("4", "SIDE OVERLAPS (L/R)", "skirt_side"),
             ("5", "BACK PANELS (L/R)", "skirt_back"), ("6", "UNDER SKIRT (INNER)", "skirt_under")]
    coat = explode_coat(objs)
    im = on_bg(render(coat, ["3/4"], (330, 380), a.samples, wire=False)[0])
    sheet.paste(im, (rx - 10, ry))
    ly = ry + 395
    for n, t, key in names:
        q = sum(r["quads"] for r in rows if r["name"].startswith(key) and "trim" not in r["name"]
                and "emblem" not in r["name"])
        d.ellipse((rx, ly, rx + 22, ly + 22), outline=TEXT, width=2)
        d.text((rx + 7, ly + 2), n, font=font(13), fill=TEXT)
        d.text((rx + 34, ly + 3), t, font=font(13), fill=TEXT)
        d.text((W - 50 - d.textlength(f"{q}q", font=font(13)), ly + 3), f"{q}q", font=font(13), fill=DIM)
        ly += 30

    frame(d, (12, 12, W - 12, H - 12), color=(90, 80, 62), width=2)
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    sheet.save(a.out)
    print("saved", a.out)


if __name__ == "__main__":
    main()
