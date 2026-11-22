#!/usr/bin/env python3
"""Overlay the model on the reference sheet, view by view, at the reference's own scale.

    python3 tools/overlay.py                       # -> renders/overlay_reference.png
    python3 tools/overlay.py --alpha 0.6 -o x.png

Row 1: the model's wireframe render blended over the reference drawing (alignment: hat top,
       ground line and centre line of each figure).
Row 2: silhouette comparison (small enclosed holes filled in both masks, so wire lines and shading
       inside the drawing don't count as background) - grey where both cover, ORANGE where only the reference has
       material (the model is missing it), CYAN where only the model has material (too big),
       with the reference drawing faintly underneath.
"""
import argparse
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

import compare  # noqa: E402
import preview  # noqa: E402
from character import lib  # noqa: E402
from character.parts import PARTS  # noqa: E402

VIEWS = ["FRONT", "SIDE", "BACK"]
BG = (22, 22, 26)
K = 1.6                       # display upscale of the reference figure


def font(size):
    try:
        return ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", size)
    except OSError:
        return ImageFont.load_default()


def fill_small_holes(mask, max_area):
    """Fill background islands smaller than max_area px that do not touch the image border.
    The reference drawing's dark wire lines and shading otherwise punch holes in its silhouette."""
    h, w = mask.shape
    out = mask.copy()
    seen = mask.copy()
    for y0 in range(h):
        row = seen[y0]
        if row.all():
            continue
        for x0 in np.where(~row)[0]:
            if seen[y0, x0]:
                continue
            stack = [(y0, x0)]
            seen[y0, x0] = True
            comp, border = [], False
            while stack:
                y, x = stack.pop()
                comp.append((y, x))
                if y == 0 or x == 0 or y == h - 1 or x == w - 1:
                    border = True
                for ny, nx in ((y + 1, x), (y - 1, x), (y, x + 1), (y, x - 1)):
                    if 0 <= ny < h and 0 <= nx < w and not seen[ny, nx]:
                        seen[ny, nx] = True
                        stack.append((ny, nx))
            if not border and len(comp) <= max_area:
                ys, xs = zip(*comp)
                out[list(ys), list(xs)] = True
    return out


def aligned(our_rgba, our_mask, of, rf, size):
    """Scale + move our render so its frame (top, height, centre) lands on the reference frame."""
    otop, oH, ocx = of
    rtop, rH, rcx = rf
    s = rH / oH * K
    w, h = our_rgba.size
    big = our_rgba.resize((int(w * s), int(h * s)), Image.LANCZOS)
    m = Image.fromarray((our_mask * 255).astype("uint8")).resize(big.size, Image.NEAREST)
    dx = int(round(rcx * K - ocx * s))
    dy = int(round(rtop * K - otop * s))
    canvas = Image.new("RGBA", size, (0, 0, 0, 0))
    canvas.alpha_composite(big, (max(dx, 0), max(dy, 0)), (max(-dx, 0), max(-dy, 0)))
    mc = Image.new("L", size, 0)
    mc.paste(m.crop((max(-dx, 0), max(-dy, 0), m.width, m.height)), (max(dx, 0), max(dy, 0)))
    return canvas, np.asarray(mc) > 127


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-o", "--out", default=os.path.join(ROOT, "renders", "overlay_reference.png"))
    ap.add_argument("--alpha", type=float, default=0.55, help="opacity of the model in row 1")
    ap.add_argument("--samples", type=int, default=12)
    a = ap.parse_args()

    lib.reset_scene()
    objs, failed = preview.build_parts(PARTS)
    if failed:
        sys.exit("parts failed: %s" % failed)
    tiles = preview.render_views(objs, None, full=True, res=(1100, 1500), samples=a.samples,
                                 views=[compare.OUR_VIEW[v] for v in VIEWS], wire=True, return_tiles=True)

    panels = []
    errs = {}
    for view, (_, rgba) in zip(VIEWS, tiles):
        rim, rmask = compare.ref_image(view)
        rf = compare.figure_frame(rmask)
        of = compare.our_frame(view)
        size = (int(rim.width * K), int(rim.height * K))
        ref_big = rim.resize(size, Image.LANCZOS).convert("RGBA")
        rmask_big = np.asarray(Image.fromarray((rmask * 255).astype("uint8")).resize(size, Image.NEAREST)) > 127
        ours, omask = aligned(rgba, np.asarray(rgba)[..., 3] > 40, of, rf, size)
        hole = int(0.004 * rmask_big.sum())           # islands under 0.4% of the figure are line/shade noise
        rmask_big = fill_small_holes(rmask_big, hole)
        omask = fill_small_holes(omask, hole)

        # row 1: blend
        tint = np.asarray(ours).astype(float)
        tint[..., :3] = tint[..., :3] * np.array([0.75, 0.95, 1.10])          # cool tint so it separates
        tint = np.clip(tint, 0, 255)
        tint[..., 3] *= a.alpha
        blend = ref_big.copy()
        blend.alpha_composite(Image.fromarray(tint.astype("uint8")))

        # row 2: silhouette difference
        base = np.asarray(ref_big.convert("L")).astype(float) * 0.35
        out = np.stack([base] * 3, -1)
        both = rmask_big & omask
        ref_only = rmask_big & ~omask
        our_only = omask & ~rmask_big
        out[both] = out[both] * 0.5 + np.array([150, 150, 158]) * 0.5
        out[ref_only] = np.array([240, 140, 40])
        out[our_only] = np.array([40, 200, 235])
        diff = Image.fromarray(out.astype("uint8")).convert("RGBA")
        union = (rmask_big | omask).sum()
        errs[view] = (ref_only.sum() / union, our_only.sum() / union, both.sum() / union)
        panels.append((view, blend, diff))

    pw = max(p[1].width for p in panels)
    ph = max(p[1].height for p in panels)
    head = 60
    sheet = Image.new("RGB", (pw * 3 + 40, (ph + head) * 2 + 120), BG)
    d = ImageDraw.Draw(sheet)
    for i, (view, blend, diff) in enumerate(panels):
        x = 10 + i * (pw + 10)
        for r, (im, label) in enumerate([(blend, "model over reference"), (diff, "silhouette difference")]):
            y = r * (ph + head)
            sheet.paste(im.convert("RGB"), (x + (pw - im.width) // 2, y + head))
            d.text((x + 8, y + 14), f"{view}  -  {label}", font=font(24), fill=(230, 230, 235))
        ro, oo, bo = errs[view]
        d.text((x + 8, 2 * (ph + head) + 10),
               f"{view}: overlap {100 * bo:.0f}%   missing {100 * ro:.0f}%   extra {100 * oo:.0f}%",
               font=font(17), fill=(230, 230, 235))
    y = 2 * (ph + head) + 55
    x = 14
    for c, t in [((240, 140, 40), "only in reference (missing)"), ((40, 200, 235), "only in model (extra)"),
                 ((150, 150, 158), "both")]:
        d.rectangle((x, y, x + 26, y + 24), fill=c)
        d.text((x + 36, y), t, font=font(20), fill=(220, 220, 225))
        x += 36 + int(d.textlength(t, font=font(20))) + 40
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    sheet.save(a.out)
    for v, (ro, oo, bo) in errs.items():
        print(f"{v}: overlap {100 * bo:.1f}%  missing {100 * ro:.1f}%  extra {100 * oo:.1f}%")
    print("saved", a.out)


if __name__ == "__main__":
    main()
