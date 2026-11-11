#!/usr/bin/env python3
"""Build one or more parts and render a wireframe contact sheet (front / left side / back / 3-4).

    python3 tools/preview.py head_hat                 # just that part, framed tight
    python3 tools/preview.py arms legs --ghost        # with the mannequin as a faint ghost
    python3 tools/preview.py all --full -o renders/turnaround.png
    python3 tools/preview.py skirt --zoom 0.15,-0.3,0.3,0.25   # close-up: centre x,y,z + half height

Prints per-object topology stats and a budget check.  Exit code 1 if a part fails to build.
"""
import argparse
import importlib
import math
import os
import sys
import tempfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

import bpy  # noqa: E402
from mathutils import Vector  # noqa: E402
from PIL import Image, ImageDraw  # noqa: E402

from infantry import lib, spec  # noqa: E402
from infantry.parts import PARTS  # noqa: E402

TINT = {"main": (0.42, 0.43, 0.46), "support": (0.30, 0.38, 0.55), "rigid": (0.55, 0.50, 0.36),
        "ghost": (0.16, 0.16, 0.18)}
VIEWS = {"FRONT": (0, -1, 0), "LEFT SIDE": (1, 0, 0), "BACK": (0, 1, 0), "3/4": (0.65, -0.76, 0.25)}


def material(kind):
    m = bpy.data.materials.new("topo_" + kind)
    m.use_nodes = True
    nt = m.node_tree
    nt.nodes.clear()
    out = nt.nodes.new("ShaderNodeOutputMaterial")
    if kind == "wire":
        sh = nt.nodes.new("ShaderNodeEmission")
        sh.inputs["Color"].default_value = (0.90, 0.90, 0.93, 1)
        sh.inputs["Strength"].default_value = 0.85
    else:
        sh = nt.nodes.new("ShaderNodeBsdfDiffuse")
        sh.inputs["Color"].default_value = (*TINT[kind], 1)
    nt.links.new(sh.outputs[0], out.inputs[0])
    return m


def add_wire_overlay(objs, thickness, mat):
    """Real polygon edges (quads stay quads) as thin emissive struts -- Cycles' own wireframe
    node would show the render triangulation instead."""
    for o in objs:
        w = bpy.data.objects.new(o.name + "_wire", o.data.copy())
        bpy.context.scene.collection.objects.link(w)
        w["topo"] = "wire"
        mod = w.modifiers.new("w", "WIREFRAME")
        mod.thickness = thickness
        mod.use_replace = True
        mod.use_even_offset = False
        mod.use_boundary = True
        w.data.materials.clear()
        w.data.materials.append(mat)


def build_ghost():
    """Bare mannequin from spec, so a part can be judged against the body it sits on."""
    S = spec
    objs = []
    zs = [0.86 + (1.60 - 0.86) * i / 14 for i in range(15)]
    rings = []
    for z in zs:
        cy, rx, ry = S.torso_section(z)
        rings.append(lib.ring_z(z, rx, ry, 16, cy=cy))
    objs.append(lib.lofted("ghost_torso", rings, cap_start=True, cap_end=True))
    hc, hr = S.HEAD_CENTER, S.HEAD_RADII
    hr_rings = []
    for i in range(1, 8):
        a = math.pi * i / 8
        hr_rings.append(lib.ring_z(hc.z - hr.z * math.cos(a), hr.x * math.sin(a), hr.y * math.sin(a), 12, cy=hc.y))
    objs.append(lib.lofted("ghost_head", hr_rings, cap_start=True, cap_end=True))
    arm = lib.lofted("ghost_arm_L", lib.tube_rings([S.arm_point(t / 4) for t in range(13)],
                                                  [S.arm_radius(t / 4) for t in range(9)] + [0.03, 0.028, 0.022, 0.012],
                                                  n=8), cap_start=True, cap_end=True)
    leg = lib.lofted("ghost_leg_L", lib.tube_rings([S.leg_point(t / 4) for t in range(9)],
                                                  [S.leg_radius(t / 4) for t in range(9)], n=8),
                     cap_start=True, cap_end=True)
    foot = lib.box("ghost_foot_L", (0.09, abs(S.TOE.y - S.HEEL.y), 0.07),
                   loc=((S.TOE.x + S.HEEL.x) / 2, (S.TOE.y + S.HEEL.y) / 2, 0.035))
    objs += [arm, leg, foot, lib.mirror_copy(arm), lib.mirror_copy(leg), lib.mirror_copy(foot)]
    for o in objs:
        o["topo"] = "ghost"
    return objs


def build_parts(names):
    objs, failed = [], []
    for n in names:
        try:
            mod = importlib.import_module(f"infantry.parts.{n}")
            got = list(mod.build())
            for o in got:
                o["part"] = n
            objs += got
        except Exception:
            import traceback
            traceback.print_exc()
            failed.append(n)
    return objs, failed


def report(objs):
    rows = [lib.stats(o) | {"part": o.get("part", "?")} for o in objs]
    print(f"\n{'object':34s} {'part':11s} {'topo':8s} {'quads':>6s} {'tris':>5s} {'ngon':>5s} {'nonman':>6s} {'open':>5s}")
    for r in rows:
        print(f"{r['name'][:34]:34s} {r['part']:11s} {r['topo']:8s} {r['quads']:6d} {r['tris_faces']:5d} "
              f"{r['ngons']:5d} {r['nonmanifold']:6d} {r['boundary_edges']:5d}")
    print()
    total_q = total_t = 0
    for p in sorted({r["part"] for r in rows}):
        pr = [r for r in rows if r["part"] == p]
        q, t3, ng = sum(r["quads"] for r in pr), sum(r["tris_faces"] for r in pr), sum(r["ngons"] for r in pr)
        tri = sum(r["triangles"] for r in pr)
        total_q += q
        total_t += tri
        b = spec.BUDGET.get(p)
        pct = 100.0 * q / max(q + t3 + ng, 1)
        flag = "" if not b else ("  OVER BUDGET" if q > b * 1.1 else ("  (light - room for more detail)" if q < b * 0.5 else ""))
        print(f"PART {p:11s} quads {q:6d} / budget {b}  tri-faces {t3}  ngons {ng}  quad% {pct:5.1f}  triangles {tri}{flag}")
    print(f"TOTAL quads {total_q}  triangles {total_t}   (target 24k-31k quads / 48k-62k tris)\n")
    return rows


def render_views(objs, out, full=False, zoom=None, res=(720, 960), samples=12, views=None, wire=True):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    sc.cycles.device = "CPU"
    sc.cycles.samples = samples
    sc.cycles.use_denoising = False
    sc.cycles.max_bounces = 1
    sc.render.film_transparent = True
    sc.render.resolution_x, sc.render.resolution_y = res
    sc.render.image_settings.file_format = "PNG"
    sc.view_settings.view_transform = "Standard"
    world = bpy.data.worlds.new("w")
    sc.world = world
    world.use_nodes = True
    world.node_tree.nodes["Background"].inputs[0].default_value = (1, 1, 1, 1)
    world.node_tree.nodes["Background"].inputs[1].default_value = 0.55
    mats = {k: material(k) for k in TINT}
    for o in objs:
        o.data.materials.clear()
        o.data.materials.append(mats.get(o.get("topo", "main"), mats["main"]))
    real = [o for o in objs if o.get("topo") != "ghost"] or objs
    if zoom:
        centre, half_h = Vector(zoom[:3]), zoom[3]
    elif full:
        centre, half_h = Vector((0, 0, 0.98)), 1.04
    else:
        pts = [o.matrix_world @ Vector(c) for o in real for c in o.bound_box]
        lo = Vector([min(p[i] for p in pts) for i in range(3)])
        hi = Vector([max(p[i] for p in pts) for i in range(3)])
        centre = (lo + hi) / 2
        ext = hi - lo
        half_h = max(ext.z, max(ext.x, ext.y) * res[1] / res[0]) / 2 * 1.12 + 0.01
    if wire:
        add_wire_overlay(real, half_h * 2 / res[1] * 1.5, material("wire"))
    cam_data = bpy.data.cameras.new("cam")
    cam_data.type = "ORTHO"
    cam_data.sensor_fit = "VERTICAL"
    cam_data.ortho_scale = half_h * 2
    cam_data.clip_end = 50
    cam = bpy.data.objects.new("cam", cam_data)
    sc.collection.objects.link(cam)
    sc.camera = cam
    sun_d = bpy.data.lights.new("sun", "SUN")
    sun_d.energy = 2.2
    sun = bpy.data.objects.new("sun", sun_d)
    sc.collection.objects.link(sun)
    tiles = []
    tmp = tempfile.mkdtemp(prefix="infantry_")
    for label in (views or list(VIEWS)):
        d = Vector(VIEWS[label]).normalized()
        cam.location = centre + d * 10
        cam.rotation_euler = (-d).to_track_quat("-Z", "Y").to_euler()
        ld = (d + Vector((0.0, 0.0, 0.8)) + d.cross(Vector((0, 0, 1))) * 0.5).normalized()
        sun.rotation_euler = (-ld).to_track_quat("-Z", "Y").to_euler()
        sc.render.filepath = os.path.join(tmp, label.replace("/", "_") + ".png")
        bpy.ops.render.render(write_still=True)
        tiles.append((label, Image.open(sc.render.filepath).convert("RGBA")))
    W, H = res
    sheet = Image.new("RGBA", (W * len(tiles), H + 40), (24, 24, 28, 255))
    dr = ImageDraw.Draw(sheet)
    for i, (label, im) in enumerate(tiles):
        sheet.alpha_composite(im, (i * W, 0))
        dr.text((i * W + W // 2 - 4 * len(label), H + 12), label, fill=(225, 225, 230, 255))
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    sheet.convert("RGB").save(out)
    print("saved", out)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("parts", nargs="+", help="part module names, or 'all'")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("--ghost", action="store_true", help="also show the bare mannequin")
    ap.add_argument("--full", action="store_true", help="frame the whole character")
    ap.add_argument("--zoom", default=None, help="cx,cy,cz,half_height close-up")
    ap.add_argument("--views", default=None, help="comma list of FRONT,LEFT SIDE,BACK,3/4")
    ap.add_argument("--no-render", action="store_true")
    ap.add_argument("--solid", action="store_true", help="no wireframe overlay (judge silhouette / shading)")
    ap.add_argument("--res", default="720x960")
    a = ap.parse_args()
    names = PARTS if a.parts == ["all"] else a.parts
    lib.reset_scene()
    objs, failed = build_parts(names)
    report(objs)
    if failed:
        print("FAILED PARTS:", failed)
    if a.ghost:
        objs += build_ghost()
    if not a.no_render and objs:
        out = a.out or os.path.join(ROOT, "renders", "preview_" + "_".join(names if len(names) < 4 else ["all"]) + ".png")
        render_views(objs, out, a.full, [float(x) for x in a.zoom.split(",")] if a.zoom else None,
                     tuple(int(x) for x in a.res.split("x")), views=a.views.split(",") if a.views else None,
                     wire=not a.solid)
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
