#!/usr/bin/env python3
"""Put our render next to the matching crop of the reference sheet, at the same scale, and
measure how far the silhouette is off.

    python3 tools/compare.py --view FRONT                      # whole figure, all parts
    python3 tools/compare.py --view SIDE --region head -o x.png
    python3 tools/compare.py --view BACK --region skirt --solid

Regions: full head torso arms waist skirt legs.   Views: FRONT SIDE BACK.
The reference "LEFT SIDE" drawing faces the other way from our +X camera, so it is mirrored here.

The printed table gives, at several heights, the half-width of the silhouette left and right of the
figure centre line as a fraction of total figure height (hat top -> sole), reference vs ours.
'ratio' > 1 means we are too wide there.  Aim for 0.95 - 1.05 everywhere.
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFilter  # noqa: E402

REF_BOX = {"FRONT": (190, 98, 530, 862), "SIDE": (546, 98, 738, 862), "BACK": (742, 98, 1040, 862)}
REGION = {  # y0, y1 (fractions of figure height from the top), half width (fraction of height)
    "full": (-0.01, 1.01, 0.27), "head": (-0.01, 0.20, 0.14), "torso": (0.10, 0.46, 0.24),
    "arms": (0.14, 0.62, 0.27), "waist": (0.30, 0.52, 0.22), "skirt": (0.38, 0.86, 0.26),
    "legs": (0.66, 1.01, 0.22),
}
OUR_VIEW = {"FRONT": "FRONT", "SIDE": "LEFT SIDE", "BACK": "BACK"}


def figure_frame(mask):
    """-> top, height, centre_x of a boolean silhouette mask (centre from the hat crown rows)."""
    rows = np.where(mask.sum(1) > 2)[0]
    top, bot = rows[0], rows[-1]
    H = bot - top
    cs = []
    for y in range(int(top + 0.01 * H), int(top + 0.06 * H)):
        c = np.where(mask[y])[0]
        if len(c):
            cs.append((c[0] + c[-1]) / 2)
    return top, H, float(np.median(cs))


def ref_image(view):
    im = Image.open(os.path.join(ROOT, "docs", "reference.png")).convert("RGB").crop(REF_BOX[view])
    if view == "SIDE":
        im = im.transpose(Image.FLIP_LEFT_RIGHT)
    g = im.convert("L").filter(ImageFilter.MaxFilter(5)).filter(ImageFilter.MinFilter(5))
    mask = np.asarray(g) > 78
    # neighbouring figures on the sheet poke into the crop: keep only the blob connected to the hat
    rows = np.where(mask.sum(1) > 2)[0]
    y = rows[0] + int(0.05 * (rows[-1] - rows[0]))
    c = np.where(mask[y])[0]
    m = Image.fromarray((mask * 255).astype("uint8"))
    ImageDraw.floodfill(m, (int((c[0] + c[-1]) / 2), int(y)), 128)
    keep = np.asarray(m) == 128
    if not keep[y].any():       # seed fell on a dark pixel: fall back to the raw mask
        keep = mask
    arr = np.asarray(im).copy()
    arr[~np.asarray(Image.fromarray(keep).filter(ImageFilter.MaxFilter(9)))] //= 4
    return Image.fromarray(arr), keep


def our_image(view, parts, solid):
    import preview
    from infantry import lib
    from infantry.parts import PARTS
    lib.reset_scene()
    objs, failed = preview.build_parts(parts or PARTS)
    preview.report(objs)
    tiles = preview.render_views(objs, None, full=True, res=(1100, 1500), samples=12,
                                 views=[OUR_VIEW[view]], wire=not solid, return_tiles=True)
    rgba = tiles[0][1]
    bg = Image.new("RGBA", rgba.size, (24, 24, 28, 255))
    bg.alpha_composite(rgba)
    return bg.convert("RGB"), np.asarray(rgba)[..., 3] > 40


def our_frame(view):
    """Frame from the spec (hat top -> ground, centre line x=0), so it is right even when only
    some parts are built.  Must match render_views(full=True, res=(1100, 1500))."""
    from infantry import spec
    ppm = 1500 / 2.08
    top = 750 - (spec.CLOTH["hat_crown_top_z"] - 0.98) * ppm
    cx = 550 + (spec.HEAD_CENTER.y * ppm if view == "SIDE" else 0.0)
    return top, spec.CLOTH["hat_crown_top_z"] * ppm, cx


def crop_region(im, frame, region, out_h):
    top, H, cx = frame
    y0, y1, hw = REGION[region]
    box = (int(cx - hw * H), int(top + y0 * H), int(cx + hw * H), int(top + y1 * H))
    c = im.crop(box)
    return c.resize((int(c.width * out_h / c.height), out_h), Image.LANCZOS)


def extents(mask, frame, f):
    top, H, cx = frame
    y = int(top + f * H)
    band = mask[max(y - 1, 0):y + 2].any(0)
    c = np.where(band)[0]
    if not len(c):
        return None
    return (cx - c[0]) / H, (c[-1] - cx) / H


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("parts", nargs="*")
    ap.add_argument("--view", default="FRONT", choices=list(REF_BOX))
    ap.add_argument("--region", default="full", choices=list(REGION))
    ap.add_argument("--solid", action="store_true")
    ap.add_argument("-o", "--out", default=None)
    a = ap.parse_args()
    rim, rmask = ref_image(a.view)
    oim, omask = our_image(a.view, a.parts, a.solid)
    rf, of = figure_frame(rmask), our_frame(a.view)
    y0, y1, _ = REGION[a.region]
    print(f"\nSILHOUETTE {a.view} / {a.region}   (half-widths as fraction of figure height; image-left, image-right)")
    print(f"{'height':>7s} {'ref L':>7s} {'ours L':>7s} {'ratio':>6s}   {'ref R':>7s} {'ours R':>7s} {'ratio':>6s}")
    errs = []
    for i in range(15):
        f = max(0.01, y0) + (min(0.99, y1) - max(0.01, y0)) * i / 14
        r, o = extents(rmask, rf, f), extents(omask, of, f)
        if not r or not o:
            continue
        ql, qr = o[0] / max(r[0], 1e-4), o[1] / max(r[1], 1e-4)
        errs += [abs(ql - 1), abs(qr - 1)]
        print(f"{f:7.2f} {r[0]:7.3f} {o[0]:7.3f} {ql:6.2f}   {r[1]:7.3f} {o[1]:7.3f} {qr:6.2f}")
    if errs:
        print(f"mean silhouette error {100 * sum(errs) / len(errs):.1f}%   (target < 5%)")
    print("note: SIDE numbers on the back half between heights 0.35-0.60 are polluted by a neighbouring figure on"
          " the sheet; trust the picture there.\n")
    h = 1100
    A, B = crop_region(rim, rf, a.region, h), crop_region(oim, of, a.region, h)
    sheet = Image.new("RGB", (A.width + B.width + 30, h + 36), (24, 24, 28))
    sheet.paste(A, (10, 30))
    sheet.paste(B, (A.width + 20, 30))
    d = ImageDraw.Draw(sheet)
    d.text((14, 8), f"REFERENCE  {a.view} / {a.region}", fill=(230, 210, 150))
    d.text((A.width + 24, 8), "OURS", fill=(230, 230, 235))
    out = a.out or os.path.join(ROOT, "renders", f"preview_compare_{a.view}_{a.region}.png")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    sheet.save(out)
    print("saved", out)


if __name__ == "__main__":
    main()
