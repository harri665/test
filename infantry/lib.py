"""Shared quad-modelling helpers.  Everything here works headless (no bpy.ops on context).

Core idea: build rings / rows of points with identical counts and `loft` them into
quad grids.  That is how we get clean, countable edge loops.

topo tags (drive the render colour, matching the reference sheet legend):
    "main"    main deforming topology (quads)
    "support" extra support / deformation loops (elbows, knees, armpit ...)
    "rigid"   hard-surface parts (armour, buckles, pouches, boots soles ...)
"""
import math

import bmesh
import bpy
from mathutils import Matrix, Vector

V = Vector
TAU = math.tau


# ------------------------------------------------------------------ point generators
def ring_z(z, rx, ry=None, n=16, cx=0.0, cy=0.0, phase=0.0, power=2.0):
    """Horizontal (super)ellipse ring, CCW seen from above.
    Angle 0 = +X (character's left); FRONT (-Y) is at angle 270 deg, i.e. index 3n/4 when phase=0.
    power 2 = ellipse, >2 = boxier."""
    ry = rx if ry is None else ry
    out = []
    for i in range(n):
        a = phase + TAU * i / n
        c, s = math.cos(a), math.sin(a)
        e = 2.0 / power
        out.append(V((cx + rx * math.copysign(abs(c) ** e, c), cy + ry * math.copysign(abs(s) ** e, s), z)))
    return out


def ring_frame(center, x_axis, y_axis, rx, ry=None, n=16, phase=0.0):
    """Ellipse ring in an arbitrary plane spanned by x_axis / y_axis."""
    ry = rx if ry is None else ry
    center, x_axis, y_axis = V(center), V(x_axis).normalized(), V(y_axis).normalized()
    return [center + x_axis * (rx * math.cos(phase + TAU * i / n)) + y_axis * (ry * math.sin(phase + TAU * i / n))
            for i in range(n)]


def frames_along(path, x_hint=(1, 0, 0)):
    """Parallel-transport frames for a poly line -> list of (tangent, x_axis, y_axis)."""
    path = [V(p) for p in path]
    tans = []
    for i in range(len(path)):
        a = path[max(i - 1, 0)]
        b = path[min(i + 1, len(path) - 1)]
        tans.append((b - a).normalized())
    x = V(x_hint)
    x = (x - tans[0] * x.dot(tans[0]))
    if x.length < 1e-6:
        x = tans[0].orthogonal()
    x.normalize()
    frames = []
    for t in tans:
        x = (x - t * x.dot(t)).normalized()
        frames.append((t, x.copy(), t.cross(x).normalized()))
    return frames


def tube_rings(path, radii, n=12, x_hint=(1, 0, 0), phase=0.0):
    """Rings swept along a path.  radii: one value, or per-point values, each a float or (rx, ry)."""
    path = [V(p) for p in path]
    if not isinstance(radii, (list, tuple)) or (len(radii) == 2 and len(path) != 2 and not isinstance(radii[0], (list, tuple))):
        radii = [radii] * len(path)
    rings = []
    for p, r, (t, x, y) in zip(path, radii, frames_along(path, x_hint)):
        rx, ry = (r if isinstance(r, (list, tuple)) else (r, r))
        rings.append(ring_frame(p, x, y, rx, ry, n, phase))
    return rings


def smooth_path(points, subdiv=4):
    """Catmull-Rom resample of a poly line (keeps the original points)."""
    P = [V(p) for p in points]
    out = []
    for i in range(len(P) - 1):
        p0, p1, p2, p3 = P[max(i - 1, 0)], P[i], P[i + 1], P[min(i + 2, len(P) - 1)]
        for k in range(subdiv):
            t = k / subdiv
            t2, t3 = t * t, t * t * t
            out.append(0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t2
                              + (-p0 + 3 * p1 - 3 * p2 + p3) * t3))
    out.append(P[-1])
    return out


# ------------------------------------------------------------------ quad builders
def _cap(ring_idx, verts):
    """All-quad cap for a closed ring given as vertex indices.  Coons grid when n % 4 == 0, else strip."""
    n = len(ring_idx)
    faces = []
    if n % 4 == 0 and n >= 8:
        m = n // 4
        P = lambda k: verts[ring_idx[k % n]]
        G = [[None] * (m + 1) for _ in range(m + 1)]
        for i in range(m + 1):
            G[0][i] = ring_idx[i]
            G[i][m] = ring_idx[m + i]
            G[m][i] = ring_idx[(3 * m - i) % n]
            G[i][0] = ring_idx[(4 * m - i) % n]
        c00, c10, c11, c01 = P(0), P(m), P(2 * m), P(3 * m)
        for j in range(1, m):
            for i in range(1, m):
                u, v = i / m, j / m
                p = ((1 - v) * P(i) + v * P(3 * m - i) + (1 - u) * P(4 * m - j) + u * P(m + j)
                     - ((1 - u) * (1 - v) * c00 + u * (1 - v) * c10 + u * v * c11 + (1 - u) * v * c01))
                verts.append(p)
                G[j][i] = len(verts) - 1
        for j in range(m):
            for i in range(m):
                faces.append((G[j][i], G[j][i + 1], G[j + 1][i + 1], G[j + 1][i]))
    else:
        k = n // 2
        for i in range(k - 1):
            faces.append((ring_idx[i], ring_idx[i + 1], ring_idx[n - 2 - i], ring_idx[n - 1 - i]))
        if n % 2:
            faces.append((ring_idx[k - 1], ring_idx[k], ring_idx[k + 1]))
    return faces


def loft(rings, closed=True, cap_start=False, cap_end=False):
    """Bridge consecutive rings (all the same point count) with quads -> (verts, faces).
    closed=False gives an open sheet (rows of a grid).  Caps are all-quad (see _cap)."""
    n = len(rings[0])
    assert all(len(r) == n for r in rings), "all rings need the same vertex count"
    verts = [V(p) for r in rings for p in r]
    faces = []
    span = n if closed else n - 1
    for i in range(len(rings) - 1):
        for j in range(span):
            a, b = i * n + j, i * n + (j + 1) % n
            faces.append((a, b, b + n, a + n))
    if closed and cap_start:
        faces += [tuple(reversed(f)) for f in _cap(list(range(n)), verts)]
    if closed and cap_end:
        faces += _cap(list(range((len(rings) - 1) * n, len(rings) * n)), verts)
    return verts, faces


# ------------------------------------------------------------------ objects
def mesh_object(name, verts, faces, topo="main", merge=1e-5, sharp_angle=40.0):
    """Create + link a finished mesh object.  Recalculates normals, merges doubles."""
    me = bpy.data.meshes.new(name)
    me.from_pydata([tuple(v) for v in verts], [], [tuple(f) for f in faces])
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    ob["topo"] = topo
    finalize(ob, merge, sharp_angle)
    return ob


def lofted(name, rings, closed=True, cap_start=False, cap_end=False, topo="main", **kw):
    return mesh_object(name, *loft(rings, closed, cap_start, cap_end), topo=topo, **kw)


def finalize(ob, merge=1e-5, sharp_angle=40.0):
    me = ob.data
    bm = bmesh.new()
    bm.from_mesh(me)
    if merge:
        bmesh.ops.remove_doubles(bm, verts=bm.verts, dist=merge)
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    lim = math.radians(sharp_angle)
    for f in bm.faces:
        f.smooth = True
    for e in bm.edges:
        e.smooth = not (len(e.link_faces) == 2 and e.calc_face_angle(0.0) > lim)
    bm.to_mesh(me)
    bm.free()
    me.update()
    return ob


def apply_modifier(ob, mod_type, **props):
    """Add a modifier, evaluate it, bake the result back into the mesh (headless-safe)."""
    mod = ob.modifiers.new(mod_type.title(), mod_type)
    for k, v in props.items():
        setattr(mod, k, v)
    dg = bpy.context.evaluated_depsgraph_get()
    new = bpy.data.meshes.new_from_object(ob.evaluated_get(dg), depsgraph=dg)
    ob.modifiers.clear()
    old = ob.data
    new.name = old.name
    ob.data = new
    bpy.data.meshes.remove(old)
    return ob


def solidify(ob, thickness=0.008, offset=1.0, even=True):
    """Give an open sheet real thickness (offset +1 grows along normals, -1 inward)."""
    apply_modifier(ob, "SOLIDIFY", thickness=thickness, offset=offset, use_even_offset=even)
    return finalize(ob)


def bevel(ob, width=0.003, segments=1, angle=35.0):
    """Bevel sharp edges of a hard-surface part (keeps quads when segments=1 or 2)."""
    apply_modifier(ob, "BEVEL", width=width, segments=segments, limit_method="ANGLE",
                   angle_limit=math.radians(angle), miter_outer="MITER_ARC", harden_normals=False)
    return finalize(ob)


def subdivide(ob, levels=1):
    """Catmull-Clark subdivide.  x4 quads per level -- mind the budget."""
    return apply_modifier(ob, "SUBSURF", levels=levels, render_levels=levels)


def transform(ob, loc=(0, 0, 0), rot=(0, 0, 0), scale=(1, 1, 1)):
    """Bake a transform into the mesh data (object matrix stays identity).  rot = XYZ euler radians."""
    from mathutils import Euler
    s = scale if isinstance(scale, (tuple, list, Vector)) else (scale,) * 3
    m = Matrix.Translation(V(loc)) @ Euler(rot, "XYZ").to_matrix().to_4x4() @ Matrix.Diagonal((*s, 1.0))
    ob.data.transform(m)
    if m.determinant() < 0:
        ob.data.flip_normals()
    ob.data.update()
    return ob


def align_z_to(ob, normal, loc=(0, 0, 0)):
    """Rotate so local +Z points along `normal`, then move to loc.  Handy for studs / buckles / emblems."""
    q = V((0, 0, 1)).rotation_difference(V(normal).normalized())
    ob.data.transform(Matrix.Translation(V(loc)) @ q.to_matrix().to_4x4())
    ob.data.update()
    return ob


def mirror_copy(ob, name=None):
    """Mirrored duplicate across X=0 (left -> right)."""
    me = ob.data.copy()
    new = bpy.data.objects.new(name or ob.name.replace("_L", "_R") if "_L" in ob.name else (name or ob.name + "_R"), me)
    me.name = new.name
    bpy.context.scene.collection.objects.link(new)
    for k in ob.keys():
        new[k] = ob[k]
    me.transform(Matrix.Diagonal((-1, 1, 1, 1)))
    me.flip_normals()
    me.update()
    return new


def join(objs, name, topo=None):
    """Merge several objects into one mesh object (no welding between them)."""
    bm = bmesh.new()
    for o in objs:
        tmp = o.data.copy()
        tmp.transform(o.matrix_world)
        bm.from_mesh(tmp)
        bpy.data.meshes.remove(tmp)
    me = bpy.data.meshes.new(name)
    bm.to_mesh(me)
    bm.free()
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    ob["topo"] = topo or objs[0].get("topo", "main")
    for o in objs:
        m = o.data
        bpy.data.objects.remove(o)
        bpy.data.meshes.remove(m)
    return ob


# ------------------------------------------------------------------ hard-surface primitives
def box(name, size, loc=(0, 0, 0), rot=(0, 0, 0), bevel_w=0.0, segs=(1, 1, 1), topo="rigid"):
    """Axis box of full size (sx, sy, sz), optional edge bevel and per-axis segments."""
    sx, sy, sz = [s / 2 for s in size]
    nx, ny, nz = segs
    bm = bmesh.new()
    bmesh.ops.create_grid(bm, x_segments=nx, y_segments=ny, size=1.0)
    bm.free()
    verts, faces, idx = [], [], {}

    def vid(i, j, k):
        key = (i, j, k)
        if key not in idx:
            idx[key] = len(verts)
            verts.append(V((-sx + 2 * sx * i / nx, -sy + 2 * sy * j / ny, -sz + 2 * sz * k / nz)))
        return idx[key]
    for k in (0, nz):
        for i in range(nx):
            for j in range(ny):
                faces.append((vid(i, j, k), vid(i + 1, j, k), vid(i + 1, j + 1, k), vid(i, j + 1, k)))
    for j in (0, ny):
        for i in range(nx):
            for k in range(nz):
                faces.append((vid(i, j, k), vid(i + 1, j, k), vid(i + 1, j, k + 1), vid(i, j, k + 1)))
    for i in (0, nx):
        for j in range(ny):
            for k in range(nz):
                faces.append((vid(i, j, k), vid(i, j + 1, k), vid(i, j + 1, k + 1), vid(i, j, k + 1)))
    ob = mesh_object(name, verts, faces, topo=topo)
    if bevel_w:
        bevel(ob, bevel_w)
    return transform(ob, loc, rot)


def cylinder(name, radius, depth, n=12, loc=(0, 0, 0), rot=(0, 0, 0), bevel_w=0.0, rings=1, topo="rigid"):
    """Capped (all-quad caps) cylinder along local Z, centred."""
    rs = [ring_z(-depth / 2 + depth * i / rings, radius, n=n) for i in range(rings + 1)]
    ob = lofted(name, rs, cap_start=True, cap_end=True, topo=topo)
    if bevel_w:
        bevel(ob, bevel_w)
    return transform(ob, loc, rot)


def stud(name, radius=0.008, height=0.004, n=8, loc=(0, 0, 0), normal=(0, 0, 1), topo="rigid"):
    """Small domed rivet / button, base on local Z=0."""
    rs = [ring_z(0, radius, n=n), ring_z(height * 0.6, radius * 0.85, n=n), ring_z(height, radius * 0.45, n=n)]
    ob = lofted(name, rs, cap_end=True, topo=topo)
    return align_z_to(ob, normal, loc)


def emblem_cross(name, size=0.06, thickness=0.004, loc=(0, 0, 0), normal=(0, -1, 0), up=(0, 0, 1),
                 long_tail=1.6, topo="rigid"):
    """The faction emblem: 4-pointed star / cross with a longer lower point.  All quads.
    `size` = half width (side points).  Lies in the plane perpendicular to `normal`, tail towards -up."""
    tips = [V((size, 0, 0)), V((0, size, 0)), V((-size, 0, 0)), V((0, -size * long_tail, 0))]
    k = size * 0.20
    notches = [V((k, k, 0)), V((-k, k, 0)), V((-k, -k, 0)), V((k, -k, 0))]
    verts, faces = [], []
    for zf in (0.0, thickness):
        base = len(verts)
        verts.append(V((0, 0, zf + (thickness * 0.5 if zf else 0))))       # raised centre on front
        for i in range(4):
            verts += [tips[i] + V((0, 0, zf * 0.4)), notches[i] + V((0, 0, zf))]
        for i in range(4):
            tip = base + 1 + 2 * i
            n_after = base + 2 + 2 * i
            n_before = base + 2 + 2 * ((i - 1) % 4)
            f = (base, n_before, tip, n_after)
            faces.append(f if zf else tuple(reversed(f)))
    off = 9
    outline = []
    for i in range(4):
        outline += [1 + 2 * i, 2 + 2 * i]
    for i in range(8):
        a, b = outline[i], outline[(i + 1) % 8]
        faces.append((a, b, b + off, a + off))
    ob = mesh_object(name, verts, faces, topo=topo, sharp_angle=25)
    n = V(normal).normalized()
    u = V(up)
    u = (u - n * u.dot(n)).normalized()
    x = u.cross(n).normalized()
    m = Matrix((x, u, n)).transposed().to_4x4()
    ob.data.transform(Matrix.Translation(V(loc)) @ m)
    ob.data.update()
    return finalize(ob, sharp_angle=25)


# ------------------------------------------------------------------ inspection
def stats(ob):
    me = ob.data
    tris = sum(len(p.vertices) - 2 for p in me.polygons)
    quads = sum(1 for p in me.polygons if len(p.vertices) == 4)
    t3 = sum(1 for p in me.polygons if len(p.vertices) == 3)
    ngons = sum(1 for p in me.polygons if len(p.vertices) > 4)
    bm = bmesh.new()
    bm.from_mesh(me)
    nonman = sum(1 for e in bm.edges if len(e.link_faces) > 2)
    boundary = sum(1 for e in bm.edges if len(e.link_faces) == 1)
    bm.free()
    return dict(name=ob.name, topo=ob.get("topo", "main"), verts=len(me.vertices), faces=len(me.polygons),
                quads=quads, tris_faces=t3, ngons=ngons, triangles=tris, nonmanifold=nonman, boundary_edges=boundary)


def reset_scene():
    bpy.ops.wm.read_factory_settings(use_empty=True)
