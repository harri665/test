#!/usr/bin/env python3
"""Assemble every part -> out/infantry_trooper.blend + .glb, stats JSON, turnaround renders.

    python3 tools/build_all.py                  # A-pose (as modelled): out/infantry_trooper.*, out/stats.json,
                                                #   renders/turnaround_wire.png + turnaround_solid.png
    python3 tools/build_all.py --pose tpose     # posed by infantry/pose.py, with an armature (rest = T-pose):
                                                #   out/infantry_trooper_tpose.*, out/stats_tpose.json,
                                                #   renders/turnaround_tpose_wire.png + turnaround_tpose_solid.png
    --no-render                                 # skip the turnaround renders
"""
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


def _arg(flag, default=None):
    if flag in sys.argv[:-1]:
        return sys.argv[sys.argv.index(flag) + 1]
    return default


def main():
    pose_name = _arg("--pose")
    sfx = f"_{pose_name}" if pose_name else ""
    lib.reset_scene()
    objs, failed = preview.build_parts(PARTS)
    rig = None
    if pose_name:
        from infantry import pose
        rig = pose.apply_pose(objs, pose_name)
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
    if rig is not None:
        col = bpy.data.collections.new("rig")
        bpy.context.scene.collection.children.link(col)
        bpy.context.scene.collection.objects.unlink(rig)
        col.objects.link(rig)
    bpy.ops.wm.save_as_mainfile(filepath=os.path.join(out, f"infantry_trooper{sfx}.blend"), compress=True)
    try:
        # export_apply bakes modifiers except Armature ones -> the T-pose glb carries the skin + armature
        bpy.ops.export_scene.gltf(filepath=os.path.join(out, f"infantry_trooper{sfx}.glb"), export_apply=True)
    except Exception as e:                   # exporter add-on can be unavailable in bpy wheels
        print("glb export skipped:", e)
    json.dump(dict(objects=rows,
                   total_quads=sum(r["quads"] for r in rows),
                   total_triangles=sum(r["triangles"] for r in rows),
                   tri_faces=sum(r["tris_faces"] for r in rows),
                   ngons=sum(r["ngons"] for r in rows)),
              open(os.path.join(out, f"stats{sfx}.json"), "w"), indent=1)
    if "--no-render" not in sys.argv:
        r = os.path.join(ROOT, "renders")
        res = (1200, 1100) if pose_name else (900, 1300)      # T-pose: wide enough for the arm span
        preview.render_views(objs, os.path.join(r, f"turnaround{sfx}_wire.png"), full=True, res=res,
                             samples=24, views=["FRONT", "LEFT SIDE", "BACK"])
        for o in [o for o in bpy.data.objects if o.get("topo") == "wire" or o.type in {"CAMERA", "LIGHT"}]:
            bpy.data.objects.remove(o)
        preview.render_views(objs, os.path.join(r, f"turnaround{sfx}_solid.png"), full=True, res=res,
                             samples=24, views=["FRONT", "LEFT SIDE", "BACK", "3/4"], wire=False)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
