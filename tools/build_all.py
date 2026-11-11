#!/usr/bin/env python3
"""Assemble every part -> out/infantry_trooper.blend + .glb, stats JSON, turnaround renders."""
import json
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

import bpy  # noqa: E402

import preview  # noqa: E402
from infantry import lib  # noqa: E402
from infantry.parts import PARTS  # noqa: E402


def main():
    lib.reset_scene()
    objs, failed = preview.build_parts(PARTS)
    rows = preview.report(objs)
    out = os.path.join(ROOT, "out")
    os.makedirs(out, exist_ok=True)
    for part in PARTS:                       # one collection per part, like the panel breakdown
        col = bpy.data.collections.new(part)
        bpy.context.scene.collection.children.link(col)
        for o in objs:
            if o.get("part") == part:
                bpy.context.scene.collection.objects.unlink(o)
                col.objects.link(o)
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(out, "infantry_trooper.blend"), compress=True)
    try:
        bpy.ops.export_scene.gltf(filepath=os.path.join(out, "infantry_trooper.glb"), export_apply=True)
    except Exception as e:                   # exporter add-on can be unavailable in bpy wheels
        print("glb export skipped:", e)
    json.dump(dict(objects=rows,
                   total_quads=sum(r["quads"] for r in rows),
                   total_triangles=sum(r["triangles"] for r in rows),
                   tri_faces=sum(r["tris_faces"] for r in rows),
                   ngons=sum(r["ngons"] for r in rows)),
              open(os.path.join(out, "stats.json"), "w"), indent=1)
    if "--no-render" not in sys.argv:
        r = os.path.join(ROOT, "renders")
        preview.render_views(objs, os.path.join(r, "turnaround_wire.png"), full=True, res=(900, 1300),
                             samples=24, views=["FRONT", "LEFT SIDE", "BACK"])
        for o in [o for o in bpy.data.objects if o.get("topo") == "wire" or o.type in {"CAMERA", "LIGHT"}]:
            bpy.data.objects.remove(o)
        preview.render_views(objs, os.path.join(r, "turnaround_solid.png"), full=True, res=(900, 1300),
                             samples=24, views=["FRONT", "LEFT SIDE", "BACK", "3/4"], wire=False)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
