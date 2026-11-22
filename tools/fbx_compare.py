#!/usr/bin/env python3
"""Render the user's FBX game model (docs/ref/fbx/infantry_model.fbx) and our model with the same
camera framing, and stack them: FBX on top, ours below.  The FBX is scaled uniformly so its top
(hat) matches our figure top (1.895 m); feet stay on z = 0.  It faces -Y like ours, but it is in a
T-POSE, so arm/hand framings only show the FBX.

    python3 tools/fbx_compare.py --zoom 0,-0.01,1.72,0.15 -o out.png               # head, solid
    python3 tools/fbx_compare.py --zoom 0.14,-0.03,0.13,0.15 --wire -o feet.png    # boot, wireframe
    python3 tools/fbx_compare.py --zoom ... --parts legs --fbx-objects Legs -o x.png
    python3 tools/fbx_compare.py --pose tpose -o full_tpose.png                  # ours T-posed, full body
    python3 tools/fbx_compare.py --only-fbx --zoom 0.45,0,1.47,0.17 --views FRONT,BACK -o arm.png

Runs each side in its own Blender process (bpy can only be initialised once per process).
FBX object names: Retopology (mask/face plate), Neck (head + ears + neck), Hair, Hat, JacketMain,
JacketMain.002/.003, Jacket wave .. .005 (skirt panels), Legs (trousers + boots), Hands,
Arm Cuffs, Arm Pads.
"""
import argparse
import os
import subprocess
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FBX = os.path.join(ROOT, "docs", "ref", "fbx", "infantry_model.fbx")
FBX_TOP = 1.01            # FBX model height (hat top) in its own units


def render_fbx(out, zoom, views, wire, objects):
    import bpy
    import mathutils
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import preview
    bpy.ops.wm.read_factory_settings(use_empty=True)
    bpy.ops.import_scene.fbx(filepath=FBX)
    for o in list(bpy.data.objects):
        if o.type != "MESH" or (objects and o.name not in objects):
            bpy.data.objects.remove(o)
    obs = list(bpy.data.objects)
    s = 1.895 / FBX_TOP
    for o in obs:
        o.data.transform(o.matrix_world)
        o.matrix_world = mathutils.Matrix.Identity(4)
        o.data.transform(mathutils.Matrix.Scale(s, 4))
        o["topo"] = "rigid" if o.name == "Retopology" else "main"
    preview.render_views(obs, out, full=zoom is None, zoom=zoom, res=(600, 600) if zoom else (700, 1000),
                         samples=12, views=views, wire=wire)


def render_ours(out, zoom, views, wire, parts, pose=None):
    sys.path.insert(0, ROOT)
    sys.path.insert(0, os.path.join(ROOT, "tools"))
    import preview
    from infantry import lib
    from infantry.parts import PARTS
    lib.reset_scene()
    objs, _ = preview.build_parts(parts or PARTS)
    if pose:
        from infantry import pose as posing
        posing.apply_pose(objs, pose)
    preview.render_views(objs, out, full=zoom is None, zoom=zoom, res=(600, 600) if zoom else (700, 1000),
                         samples=12, views=views, wire=wire)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--zoom", default=None, help="cx,cy,cz,half_height (metres, our scale); omit for full body")
    ap.add_argument("--views", default="FRONT,LEFT SIDE,BACK,3/4")
    ap.add_argument("--wire", action="store_true")
    ap.add_argument("--parts", default="", help="comma list of our part modules (default all)")
    ap.add_argument("--fbx-objects", default="", help="comma list of FBX object names (default all)")
    ap.add_argument("--only-fbx", action="store_true")
    ap.add_argument("--pose", default="", help="pose OUR model first (e.g. tpose) to match the FBX's T-pose")
    ap.add_argument("--_side", default=None, help=argparse.SUPPRESS)
    ap.add_argument("--_out", default=None, help=argparse.SUPPRESS)
    ap.add_argument("-o", "--out", default=os.path.join(ROOT, "renders", "preview_fbx_compare.png"))
    a = ap.parse_args()
    zoom = [float(x) for x in a.zoom.split(",")] if a.zoom else None
    views = a.views.split(",")
    if a._side == "fbx":
        return render_fbx(a._out, zoom, views, a.wire, [x for x in a.fbx_objects.split(",") if x])
    if a._side == "ours":
        return render_ours(a._out, zoom, views, a.wire, [x for x in a.parts.split(",") if x], a.pose or None)
    tmp = tempfile.mkdtemp(prefix="fbxcmp_")
    sides = ["fbx"] if a.only_fbx else ["fbx", "ours"]
    for side in sides:
        cmd = [sys.executable, os.path.abspath(__file__), "--_side", side, "--_out", os.path.join(tmp, side + ".png"),
               "--views", a.views, "--parts", a.parts, "--fbx-objects", a.fbx_objects, "--pose", a.pose]
        if a.zoom:
            cmd += ["--zoom", a.zoom]
        if a.wire:
            cmd += ["--wire"]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode:
            print(r.stdout[-2000:], r.stderr[-2000:])
            sys.exit(1)
    from PIL import Image, ImageDraw
    ims = [Image.open(os.path.join(tmp, s + ".png")) for s in sides]
    W = max(i.width for i in ims)
    sheet = Image.new("RGB", (W, sum(i.height for i in ims) + 30 * len(ims)), (20, 20, 24))
    d = ImageDraw.Draw(sheet)
    y = 0
    for s, im in zip(sides, ims):
        d.text((10, y + 8), "FBX GAME MODEL" if s == "fbx" else "OURS", fill=(230, 200, 140) if s == "fbx" else (230, 230, 235))
        sheet.paste(im, (0, y + 30))
        y += im.height + 30
    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    sheet.save(a.out)
    print("saved", a.out)


if __name__ == "__main__":
    main()
