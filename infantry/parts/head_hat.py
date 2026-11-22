"""Head + ears + neck, short clumped hair, faceted face mask with glowing slats, flat-brim fedora.

Round 5: matched to the user's FBX game model (docs/ref/fbx/infantry_model.fbx, objects Retopology /
Neck / Hair / Hat; docs/FBX_COMPARISON.md H1-H7).  The FBX design is adopted at OUR head size: the
FBX head is ~15 % wider, so its lateral / depth offsets are scaled by FBX_XY (0.87) about our
HEAD_CENTER and its heights by FBX_Z (0.93) about the chin (FBX chin 1.633 -> ours 1.630).  The hat is
taken from the FBX directly at our scale (both models are aligned on the hat top, z 1.895).

Topology summary
----------------
head        one 24-around tube: flared neck bottom ring (z 1.530, meets the coat's neck hem), neck,
            three non-planar jaw loops (throat / under-jaw / jawline: low at the chin, rising to the
            jaw angle under the ear, nape at the back) -> real jaw and chin; planar skull loops up
            into the hat crown, closed by a 6x6 grid (hidden).  The face under the mask stays coarse.
ears        two closed-off ear shells (12 around): root ring sunk into the skull, back of the ear,
            rolled rim (helix), bowl (concha) closed by a 3x3 grid; solid lobe, low front rim.
hair        one 24 x 5 quad sheet over the back and sides of the skull from just under the hat's
            sweatband down to a zig-zag edge of pointed, swept clumps (V point at the nape, a
            sideburn tuft in front of each ear, a straight edge over the ears); its whole boundary
            tucks into the skull (no visible edge thickness gap, no open edge in view).
mask        rigid faceted plate: a Coons quad grid between the straight top edge, the two short
            rear side edges and the long V edge to the chin point.  Columns run brow -> chin and
            converge on the chin (they carry the centre ridge with its V channel, the light grooves
            and the side ridges); rows are nested chevrons parallel to the V edge.  Relief is a
            per-vertex offset along the plate normal; a raised rim runs all round and the outer
            edge rolls back into a wall that ends inside the head (ray-cast), so no gap shows.
mask_lights the FBX "Lights" material slats as separate thin rigid strips sunk into the plate: the two
            long grooves, the top of the centre channel and a short strip under the brow on each
            side facet.  Own object so it can get an emissive material.
hat         one 40-around shell: sweatband (ends inside the head), thin flat oval brim with a wavy
            edge, crown with two deep front pinch dents and a centre crease, 10x10 top grid.
hat_band    thin plain closed ribbon round the base of the crown.
"""
import math

import bpy  # noqa: F401
from mathutils import Vector as V
from mathutils.bvhtree import BVHTree

from .. import lib, spec

TAU = math.tau
HC = spec.HEAD_CENTER
HR = spec.HEAD_RADII
C = spec.CLOTH

FBX_XY = 0.87                      # FBX head (x +-0.090 incl. ears) -> ours (+-0.078)
FBX_Z = 1.00                       # heights: FBX hat == ours, so FBX z - 0.003 (chin 1.633 -> 1.630)

# ------------------------------------------------------------------ hat (FBX "Hat" at our scale)
N_HAT = 40
HAT_CY = HC.y
HAT_TOP = C["hat_crown_top_z"]     # 1.895: figure top, every comparison aligns on it
BRIM_Z = 1.800                     # FBX: brim top where it meets the crown ...
RIM_Z = 1.7745                     # ... slopes gently down to the edge (mean height) ...
RIM_WAVE = 0.0060                  # ... which is wavy: up at the four diagonals, down front/back/sides
BRIM_R = (0.158, 0.178)            # FBX: oval brim, deeper front-to-back
BRIM_TH = 0.0050                   # FBX: thin brim
BRIM_ROLL = 0.0
BAND_Z = (BRIM_Z + 0.001, 1.827)   # FBX: thin plain band
BAND_OFF = 0.0040
# crown wall (inside the band, then up to the top edge): (z, rx, ry) -- FBX sections
_CROWN_KEYS = [(BRIM_Z, 0.0790, 0.0975), (1.827, 0.0770, 0.0950), (1.845, 0.0740, 0.0905),
               (1.865, 0.0705, 0.0870), (HAT_TOP, 0.0675, 0.0835)]
CROWN_ROUND = 0.011                # rounded top edge
CROWN_FRONT_NARROW = 0.07          # FBX top view: crown is egg-shaped, narrower at the front
CREASE_DEPTH = 0.0105              # FBX: centre crease dips the top (deepest toward the front)
CREASE_W = 0.024
PINCH_A = math.radians(40)         # FBX: two deep pinch dents this far either side of the front
PINCH_Z = (1.831, 1.889)           # ... spanning this height
PINCH_HW = math.radians(25)        # ... and this half width
PINCH_DEPTH = 0.0210


def _crown_r(z):
    rx, ry = lib_cr(_CROWN_KEYS, z)
    return rx, ry


# ------------------------------------------------------------------ head / neck
N_HEAD = 24
Z_NECK_END = 1.530                 # neck bottom ring: coat neck hem meets it (shared seam)
# neck rings (z, rx, ry, cy, front sag): bottom two flare out over the coat's turned-in neck hem
# (which runs in to spec.torso_section(1.63) = rx 0.050 / ry 0.054 up to z ~1.58 at the sides)
_NECK = [(Z_NECK_END, 0.0710, 0.0750, 0.005, 0.0040),
         (1.542, 0.0632, 0.0672, 0.005, 0.0032),
         (1.562, 0.0568, 0.0608, 0.004, 0.0015),
         (1.585, 0.0540, 0.0578, 0.003, 0.0),
         (1.604, 0.0460, 0.0500, 0.000, 0.0)]     # slim neck above the hem (FBX neck ~ +-0.041)
# jaw loops: z at front / side / back, rx, cy, front depth, back depth, front exponent
_JAW = [dict(zf=1.618, zs=1.624, zb=1.628, rx=0.0425, cy=-0.002, df=0.0500, db=0.0500, ef=2.0),   # throat / nape
        dict(zf=1.630, zs=1.652, zb=1.645, rx=0.0415, cy=-0.008, df=0.0720, db=0.0600, ef=1.3),   # under the jaw
        dict(zf=1.638, zs=1.673, zb=1.664, rx=0.0525, cy=-0.010, df=0.0790, db=0.0680, ef=1.25)]  # jawline + V chin
# planar skull sections (z, half width, front depth, back depth) about HEAD_CENTER
_HEAD_KEYS = [(1.664, 0.0510, 0.0770, 0.0680),     # guide only (matches the jawline loop at the back)
              (1.690, 0.0548, 0.0800, 0.0775),     # cheeks
              (1.712, 0.0588, 0.0820, 0.0840),
              (1.735, 0.0615, 0.0830, 0.0890),
              (1.758, 0.0635, 0.0830, 0.0910),
              (1.780, 0.0640, 0.0820, 0.0910),     # widest skull, just under the brim
              (1.800, 0.0615, 0.0770, 0.0870),     # inside the hat from here (sweatband 1.795)
              (1.822, 0.0550, 0.0680, 0.0770)]
Z_SKULL_TOP = 1.845                # inside the crown
_HEAD_POW = 2.05

# ------------------------------------------------------------------ ears (FBX "Neck" ears, scaled)
EAR_ZC = 1.728                     # FBX ear z 1.699..1.764
EAR_FRONT_Y = -0.0105              # front edge just behind the mask's rear edge / sideburn
EAR_W, EAR_H = 0.035, 0.062
EAR_FLARE = math.radians(25)       # back of the ear stands out from the skull (FBX front view)
EAR_LEAN = math.radians(12)        # top leans back
N_EAR = 12

# ------------------------------------------------------------------ mask (FBX "Retopology", scaled)
MASK_TOP_Z = 1.776                 # straight top edge, tucked under the brim (underside 1.795)
MASK_SIDE_Z = 1.733                # short rear side edges end here, the V edge runs to the chin
MASK_TIP_Z = 1.630
# front-view outline half width (FBX Retopology sections, scaled)
_MASK_W = [(1.630, 0.0), (1.637, 0.0087), (1.647, 0.0174), (1.657, 0.0252), (1.667, 0.0330),
           (1.677, 0.0426), (1.687, 0.0487), (1.697, 0.0530), (1.717, 0.0600), (1.737, 0.0653),
           (1.757, 0.0687), (1.776, 0.0715)]
MASK_XC = 0.0715
_MASK_YF = [(1.630, -0.0945), (1.700, -0.0985), (1.776, -0.0995)]   # front centre line (base)
# plan section: depth back from the front = W * (p u^2 + (c - p) u^8), u = |x| / W: pointed prow
# low down, flat front + steep side facets sweeping back toward the ears higher up
_MASK_P = [(1.697, 0.94, 0.97), (1.727, 0.55, 0.99), (1.757, 0.33, 1.03), (1.776, 0.31, 1.05)]
# columns (half): x on the top edge, fraction along the V edge (tip 0 .. rear side edge 1)
_MASK_COLS = [(0.0, 0.0),
              (0.0040, 0.010),     # 1 channel lip (V channel in the centre ridge)
              (0.0090, 0.026),     # 2 centre ridge outer top edge
              (0.0108, 0.034),     # 3 ridge foot / light groove
              (0.0215, 0.085),     # 4 groove floor outer
              (0.0233, 0.095),     # 5 side ridge top
              (0.0305, 0.135),     # 6 side ridge top
              (0.0323, 0.145),     # 7 side ridge foot / side facet
              (0.0460, 0.340),     # 8 facet
              (0.0600, 0.620),     # 9 rim foot
              (0.0680, 0.820),     # 10 rim inner edge
              (MASK_XC, 1.0)]      # 11 outline (rear side edge)
_ROWS_TOP = [0.0, 0.0035, 0.0055, 0.0085, 0.0135, 0.0165]   # metres down from the top edge
_ROWS_BOT = [0.0055, 0.0035, 0.0]                             # metres up from the V edge
N_MID = 6
WALL_MAX = 0.014                   # rim wall runs back into the head (ray-cast), at most this far
H_RIM, H_RIDGE, H_CAP, H_SLOT = 0.0040, 0.0100, 0.0112, 0.0030      # FBX: ridges ~1 cm proud
H_SIDE, H_FLOOR, H_LIGHT, H_CHEV = 0.0085, -0.0005, -0.0012, 0.0030

# ------------------------------------------------------------------ hair (FBX "Hair", scaled)
# left half columns from the front end to the back centre: (plan angle deg, bottom z, kind)
_HAIR_COLS = [(-9.0, 1.752, "end"), (-5.0, 1.690, "burn"), (0.0, 1.771, "ear"), (8.0, 1.772, "ear"),
              (16.0, 1.772, "ear"), (24.0, 1.765, "ear"), (31.0, 1.744, "notch"), (39.0, 1.708, "tip"),
              (48.0, 1.733, "notch"), (57.0, 1.701, "mid"), (65.0, 1.680, "tip"), (75.0, 1.707, "notch"),
              (83.0, 1.680, "mid"), (90.0, 1.656, "tip")]
HAIR_TOP = 1.7925                  # top edge tucked into the skull just under the sweatband (1.795)
HAIR_Z1 = 1.7895
HAIR_ROWS = (0.30, 0.60, 0.85, 1.0)
HAIR_OFF = (0.0085, 0.0110, 0.0110, 0.0095)    # thickness over the skull per row (last = edge)


# ------------------------------------------------------------------ small helpers
def _cr(keys, x):
    """Non-uniform Catmull-Rom through keys [(x, v0, v1, ...)] -> tuple of values."""
    if x <= keys[0][0]:
        return tuple(keys[0][1:])
    if x >= keys[-1][0]:
        return tuple(keys[-1][1:])
    for i in range(len(keys) - 1):
        if keys[i][0] <= x <= keys[i + 1][0]:
            p0, p1, p2, p3 = keys[max(i - 1, 0)], keys[i], keys[i + 1], keys[min(i + 2, len(keys) - 1)]
            h = p2[0] - p1[0]
            t = (x - p1[0]) / h
            t2, t3 = t * t, t * t * t
            out = []
            for k in range(1, len(p1)):
                m1 = (p2[k] - p0[k]) / max(p2[0] - p0[0], 1e-9) * h
                m2 = (p3[k] - p1[k]) / max(p3[0] - p1[0], 1e-9) * h
                out.append((2 * t3 - 3 * t2 + 1) * p1[k] + (t3 - 2 * t2 + t) * m1
                           + (-2 * t3 + 3 * t2) * p2[k] + (t3 - t2) * m2)
            return tuple(out)


lib_cr = _cr


def _spow(c, e):
    return math.copysign(abs(c) ** e, c)


def _smooth(e0, e1, x):
    t = min(max((x - e0) / (e1 - e0), 0.0), 1.0)
    return t * t * (3 - 2 * t)


def _coons(ring, verts, rot=0, shape=None):
    """All-quad grid cap for a closed ring of vertex indices (len % 4 == 0).
    rot shifts where the four grid corners (valence-3 poles) land.  shape(p) may reposition
    the new interior points.  Returns faces."""
    n = len(ring)
    ring = ring[rot:] + ring[:rot]
    m = n // 4
    P = lambda k: verts[ring[k % n]]
    G = [[None] * (m + 1) for _ in range(m + 1)]
    for i in range(m + 1):
        G[0][i] = ring[i]
        G[i][m] = ring[m + i]
        G[m][i] = ring[(3 * m - i) % n]
        G[i][0] = ring[(4 * m - i) % n]
    c00, c10, c11, c01 = P(0), P(m), P(2 * m), P(3 * m)
    for j in range(1, m):
        for i in range(1, m):
            u, v = i / m, j / m
            p = ((1 - v) * P(i) + v * P(3 * m - i) + (1 - u) * P(4 * m - j) + u * P(m + j)
                 - ((1 - u) * (1 - v) * c00 + u * (1 - v) * c10 + u * v * c11 + (1 - u) * v * c01))
            verts.append(shape(p) if shape else p)
            G[j][i] = len(verts) - 1
    return [(G[j][i], G[j][i + 1], G[j + 1][i + 1], G[j + 1][i]) for j in range(m) for i in range(m)]


def _bridge(a, b, closed=True):
    """Quads between two index rings of equal length."""
    n = len(a)
    return [(a[i], a[(i + 1) % n], b[(i + 1) % n], b[i]) for i in range(n if closed else n - 1)]


def _add_ring(verts, pts):
    i0 = len(verts)
    verts.extend(V(p) for p in pts)
    return list(range(i0, i0 + len(pts)))


def _strip(rings, verts, faces):
    """Closed 4-sided tube through rings of 4 points + quad end caps (a thin glowing slat)."""
    idx = [_add_ring(verts, r) for r in rings]
    for a, b in zip(idx, idx[1:]):
        faces += _bridge(a, b)
    faces.append(tuple(reversed(idx[0])))
    faces.append(tuple(idx[-1]))


# ------------------------------------------------------------------ head surface
def head_section(z):
    """(half width, front depth, back depth) of the skull at height z (planar part of the head)."""
    return _cr(_HEAD_KEYS, z)


def head_pt(a, z, grow=0.0):
    """Point on the skull at plan angle a (0 = +X, 270 deg = front) and height z."""
    rx, df, db = head_section(z)
    c, s = math.cos(a), math.sin(a)
    e = 2.0 / (_HEAD_POW if s > 0 else 2.0)
    d = db if s > 0 else df
    return V(((rx + grow) * _spow(c, e), HC.y + (d + grow) * _spow(s, e), z))


def _jaw_pt(r, a):
    c, s = math.cos(a), math.sin(a)
    z = r["zs"] + (r["zf"] - r["zs"]) * max(-s, 0.0) ** 2 + (r["zb"] - r["zs"]) * max(s, 0.0) ** 2
    e = 2.0 / (r["ef"] if s < 0 else 2.0)
    d = r["df"] if s < 0 else r["db"]
    return V((r["rx"] * _spow(c, e), r["cy"] + d * _spow(s, e), z))


def _head_mesh():
    n = N_HEAD
    rings = []
    for z, rx, ry, cy, sag in _NECK:
        rings.append([V((rx * math.cos(TAU * i / n), cy + ry * math.sin(TAU * i / n),
                         z + sag * math.sin(TAU * i / n))) for i in range(n)])
    for r in _JAW:
        rings.append([_jaw_pt(r, TAU * i / n) for i in range(n)])
    for z, *_ in _HEAD_KEYS[1:]:
        rings.append([head_pt(TAU * i / n, z) for i in range(n)])
    verts, faces = [], []
    idx = [_add_ring(verts, r) for r in rings]
    for a, b in zip(idx, idx[1:]):
        faces += _bridge(a, b)
    zc = _HEAD_KEYS[-1][0]
    rx, df, db = head_section(zc)

    def dome(p):
        d = db if p.y > HC.y else df
        rho2 = min((p.x / rx) ** 2 + ((p.y - HC.y) / d) ** 2, 1.0)
        return V((p.x, p.y, zc + (Z_SKULL_TOP - zc) * (1.0 - rho2) ** 0.6))
    faces += _coons(idx[-1], verts, rot=3, shape=dome)
    return verts, faces


class _Surf:
    """Ray-cast access to the built head mesh: exact surface point + outward normal at (angle, z)."""

    def __init__(self, verts, faces):
        self.bvh = BVHTree.FromPolygons([tuple(v) for v in verts], [tuple(f) for f in faces])

    def at(self, a, z):
        o = V((0.0, HC.y, z))
        d = V((math.cos(a), math.sin(a), 0.0))
        hit, nrm, _, _ = self.bvh.ray_cast(o, d, 0.5)
        if hit is None:
            return head_pt(a, z), d
        if nrm.dot(d) < 0:
            nrm = -nrm
        return hit, nrm

    def cast(self, o, d, dist=0.05):
        hit, _, _, _ = self.bvh.ray_cast(o, d, dist)
        return hit


def build_head(verts, faces):
    return lib.mesh_object("head", verts, faces, topo="main")


# ------------------------------------------------------------------ ears
def _ear_mesh(surf):
    # front attachment point: skull point at y = EAR_FRONT_Y on the left side at EAR_ZC
    lo, hi = -math.pi / 2, 0.0
    for _ in range(40):
        mid = (lo + hi) / 2
        if head_pt(mid, EAR_ZC).y < EAR_FRONT_Y:
            lo = mid
        else:
            hi = mid
    a0 = (lo + hi) / 2
    A, _ = surf.at(a0, EAR_ZC)
    wd = V((0.0, math.sin(EAR_LEAN), math.cos(EAR_LEAN)))
    s0 = V((math.sin(EAR_FLARE), math.cos(EAR_FLARE), 0.0))
    sd = (s0 - wd * s0.dot(wd)).normalized()
    nd = sd.cross(wd).normalized()
    if nd.x < 0:
        nd = -nd
    cen = A + sd * (EAR_W / 2)
    n = N_EAR

    def outline(k):
        phi = TAU * k / n
        c, s = math.cos(phi), math.sin(phi)
        su = 0.5 * EAR_W * c * (1.0 - 0.28 * max(-s, 0.0)) + 0.003 * max(s, 0.0) - 0.002 * max(-s, 0.0)
        return su, 0.5 * EAR_H * s, c, s

    # (outline scale, height over the ear plane): back, rim edge, helix crest, helix inner, bowl
    prof = [(1.00, 0.0), (0.97, 0.0050), (0.85, 0.0063), (0.70, 0.0043), (0.47, 0.0012)]
    verts, rings = [], []
    root = []
    for k in range(n):
        su, w, c, s = outline(k)
        q = cen + sd * (su * 0.78) + wd * (w * 0.78)
        a = math.atan2(q.y - HC.y, q.x)
        p, nrm = surf.at(a, q.z)
        root.append(p - nrm * 0.003)
    rings.append(root)
    for j, (ks, h) in enumerate(prof):
        ring = []
        for k in range(n):
            su, w, c, s = outline(k)
            lobe = max(-s, 0.0) ** 2 * max(-c + 0.3, 0.0) / 1.3   # lower front: solid lobe
            front = max(-c, 0.0) ** 2                               # front: low rim (tragus side)
            hh = h
            if j == 2:
                hh = h - 0.0022 * front
            if j == 3:
                hh = h + (0.0060 - h) * min(lobe * 2.0, 1.0)
            if j == 4:
                hh = h + (0.0050 - h) * min(lobe * 2.0, 1.0)
            ring.append(cen + sd * (su * ks) + wd * (w * ks) + nd * hh)
        rings.append(ring)
    idx = [_add_ring(verts, r) for r in rings]
    faces = []
    for a, b in zip(idx, idx[1:]):
        faces += _bridge(a, b)
    faces += _coons(idx[-1], verts, rot=1, shape=lambda p: p - nd * 0.0006)
    return verts, faces


def build_ears(surf):
    verts, faces = _ear_mesh(surf)
    m = len(verts)
    verts = verts + [V((-v.x, v.y, v.z)) for v in verts]
    faces = faces + [tuple(i + m for i in reversed(f)) for f in faces]
    return lib.mesh_object("ears", verts, faces, topo="main")


# ------------------------------------------------------------------ hair
def build_hair(surf):
    left = _HAIR_COLS
    cols = left + [(180.0 - a, zb, kind) for a, zb, kind in reversed(left[:-1])]
    verts, grid, par = [], [], []
    for a_deg, zb, kind in cols:
        col, cpar = [], []
        back = 1.0 if a_deg < 90.0 else -1.0                 # toward the back centre
        for k in range(len(HAIR_ROWS) + 2):
            if k == 0:
                z, off, sweep = HAIR_TOP, -0.002, 0.0
            elif k == 1:
                z, off, sweep = HAIR_Z1, HAIR_OFF[0], 0.0
            else:
                f = HAIR_ROWS[k - 2]
                z = HAIR_Z1 + (zb - HAIR_Z1) * f
                off = HAIR_OFF[min(k - 1, 3)]
                sweep = f ** 1.5
            if k == len(HAIR_ROWS) + 1:                       # edge: thin pointed tips
                off = 0.0025 if kind in ("tip", "burn", "mid") else 0.0035
            elif k >= 2:
                off += {"tip": 0.0030, "mid": 0.0018, "notch": -0.0020, "burn": 0.0006}.get(kind, 0.0)
            da = 0.0
            if kind in ("tip", "mid"):
                da = 4.5 * sweep * back
            elif kind == "notch":
                da = 2.5 * sweep * back
            elif kind == "burn":
                da = -3.0 * sweep * back                      # sideburn sweeps forward
            a = math.radians(a_deg + da)
            p, nrm = surf.at(a, z)
            verts.append(p + nrm * off)
            col.append(len(verts) - 1)
            cpar.append((a, z))
        grid.append(col)
        par.append(cpar)
    nc, nr = len(grid), len(grid[0])
    faces = []
    for i in range(nc - 1):
        for k in range(nr - 1):
            faces.append((grid[i][k], grid[i + 1][k], grid[i + 1][k + 1], grid[i][k + 1]))
    # tuck: the whole boundary turns into the skull
    loop = ([(i, 0) for i in range(nc)] + [(nc - 1, k) for k in range(1, nr)]
            + [(i, nr - 1) for i in range(nc - 2, -1, -1)] + [(0, k) for k in range(nr - 2, 0, -1)])
    outer, inner = [], []
    for i, k in loop:
        a, z = par[i][k]
        p, nrm = surf.at(a, z)
        verts.append(p - nrm * (0.0035 if k == 0 else 0.0015))
        outer.append(grid[i][k])
        inner.append(len(verts) - 1)
    faces += _bridge(outer, inner)
    return lib.mesh_object("hair", verts, faces, topo="main")


# ------------------------------------------------------------------ hat
def _brim_r(t):
    c0 = _crown_r(BRIM_Z)
    return (c0[0] + (BRIM_R[0] - c0[0]) * t, c0[1] + (BRIM_R[1] - c0[1]) * t)


def rim_z(a):
    """Brim edge height at plan angle a: flat with a gentle four-lobed wave."""
    return RIM_Z - RIM_WAVE * math.cos(4.0 * a)


def brim_z(t, a):
    """Top surface of the brim.  t: 0 at the crown, 1 at the rim."""
    return BRIM_Z + (rim_z(a) - BRIM_Z) * t ** 1.1 + BRIM_ROLL * max(0.0, 1.0 - t / 0.2) ** 2


def crown_r(z, a=None):
    rx, ry = _crown_r(z)
    if a is not None:
        up = _smooth(BAND_Z[1], BAND_Z[1] + 0.03, z)
        rx *= 1.0 - CROWN_FRONT_NARROW * up * max(-math.sin(a), 0.0) ** 2
    return rx, ry


def _hat_ring(rfun, zf, n=N_HAT):
    out = []
    for i in range(n):
        a = TAU * i / n
        rx, ry = rfun(a)
        out.append(V((rx * math.cos(a), HAT_CY + ry * math.sin(a), zf(a) if callable(zf) else zf)))
    return out


def _crown_shape(p):
    """Fedora deformation of the crown: centre crease along the top and two deep pinch dents."""
    q = p.copy()
    a = math.atan2(p.y - HAT_CY, p.x)
    # centre crease: a front-back trough dipping the top, deepest toward the front
    top = _smooth(HAT_TOP - 0.030, HAT_TOP - 0.004, p.z)
    if top > 0:
        g = math.exp(-(p.x / CREASE_W) ** 2)
        fb = 1.0 if p.y < HAT_CY else 0.65
        q.z -= CREASE_DEPTH * fb * g * top
    # pinches: bowl-shaped dents with a fairly crisp rim
    zc, hz = 0.5 * (PINCH_Z[0] + PINCH_Z[1]), 0.5 * (PINCH_Z[1] - PINCH_Z[0])
    for ap in (-math.pi / 2 + PINCH_A, -math.pi / 2 - PINCH_A):
        d = (a - ap + math.pi) % TAU - math.pi
        r = math.sqrt((d / PINCH_HW) ** 2 + ((p.z - zc) / hz) ** 2)
        if r < 1.0:
            w = 1.0 - _smooth(0.42, 1.0, r)        # flat-bottomed bowl, rim ramp > 1 grid step
            r = V((p.x, p.y - HAT_CY, 0.0))
            if r.length > 1e-6:
                q -= r.normalized() * (PINCH_DEPTH * w)
                q.z += 0.002 * w                           # dent floor tilts up a little
    return q


def build_hat():
    n = N_HAT
    verts, faces = [], []
    rings = []
    under0 = brim_z(0.0, 0.0) - BRIM_TH
    # sweatband: ends just inside the head (open there; the head fills it)
    rings.append([head_pt(TAU * i / n, under0 + 0.0005, grow=-0.002) for i in range(n)])
    # brim underside (crown -> rim), rim edge, brim top (rim -> crown)
    for t in (0.0, 0.5, 0.92):
        rx, ry = _brim_r(t)
        if t == 0.0:
            rx, ry = rx - 0.002, ry - 0.002
        rings.append(_hat_ring(lambda a, rx=rx, ry=ry: (rx, ry), lambda a, t=t: brim_z(t, a) - BRIM_TH))
    rxo, ryo = _brim_r(1.0)
    for dz in (0.80, 0.20):
        rings.append(_hat_ring(lambda a: (rxo, ryo), lambda a, dz=dz: brim_z(1.0, a) - BRIM_TH * dz))
    for t in (0.92, 0.5, 0.15, 0.0):
        rx, ry = _brim_r(t)
        rings.append(_hat_ring(lambda a, rx=rx, ry=ry: (rx, ry), lambda a, t=t: brim_z(t, a)))
    # crown wall: hidden under the band, then ~1.1 cm rows up to the rounded top edge
    zs = [BAND_Z[0] + 0.013, 1.831, 1.840, 1.849, 1.858, 1.867, 1.876, HAT_TOP - CROWN_ROUND]
    for z in zs:
        rings.append(_hat_ring(lambda a, z=z: crown_r(z, a), z))
    c = CROWN_ROUND
    rings.append(_hat_ring(lambda a: tuple(r - 0.29 * c for r in crown_r(HAT_TOP, a)), HAT_TOP - 0.29 * c))
    rings.append(_hat_ring(lambda a: tuple(r - c for r in crown_r(HAT_TOP, a)), HAT_TOP))
    idx = [_add_ring(verts, r) for r in rings]
    for a, b in zip(idx, idx[1:]):
        faces += _bridge(a, b)
    faces += _coons(idx[-1], verts, rot=5)
    verts = [_crown_shape(v) if v.z > BAND_Z[1] else v for v in verts]
    return lib.mesh_object("hat", verts, faces, topo="main", sharp_angle=55)


def build_band():
    """Thin plain closed ribbon around the base of the crown."""
    z0, z1 = BAND_Z
    sect = [(-0.003, z0), (0.002, z0), (BAND_OFF, z0 + 0.002), (BAND_OFF, z1 - 0.002),
            (0.002, z1), (-0.003, z1)]
    verts, faces, idx = [], [], []
    for off, z in sect:
        idx.append(_add_ring(verts, _hat_ring(lambda a, z=z, off=off: tuple(r + off for r in crown_r(z, a)), z)))
    for k in range(len(sect)):
        faces += _bridge(idx[k], idx[(k + 1) % len(sect)])
    return lib.mesh_object("hat_band", verts, faces, topo="main")


# ------------------------------------------------------------------ face mask
def mask_w(z):
    return max(_cr(_MASK_W, z)[0], 1e-4)


def mask_y(x, z):
    """Plate base surface (front-view coordinates x, z -> y)."""
    w = mask_w(z)
    u = min(abs(x) / w, 1.1)
    p, c = _cr(_MASK_P, z)
    return _cr(_MASK_YF, z)[0] + w * (p * u * u + (c - p) * u ** 8)


def mask_n(x, z):
    e = 1e-4
    dx = (mask_y(x + e, z) - mask_y(x - e, z)) / (2 * e)
    dz = (mask_y(x, z + e) - mask_y(x, z - e)) / (2 * e)
    return V((dx, -1.0, dz)).normalized()


def mask_pt(x, z, h=0.0):
    return V((x, mask_y(x, z), z)) + mask_n(x, z) * h


class _VEdge:
    """The V edge (front view) from the left rear side edge through the chin point to the right one,
    by arc length: s in [-1, 1], 0 = chin point."""

    def __init__(self, m=240):
        zs = [MASK_TIP_Z + (MASK_SIDE_Z - MASK_TIP_Z) * i / m for i in range(m + 1)]
        pts = [(mask_w(z) if i else 0.0, z) for i, z in enumerate(zs)]
        acc = [0.0]
        for a, b in zip(pts, pts[1:]):
            acc.append(acc[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
        self.pts, self.acc = pts, acc

    def __call__(self, s):
        target = abs(s) * self.acc[-1]
        k = 0
        while k < len(self.acc) - 2 and self.acc[k + 1] < target:
            k += 1
        t = (target - self.acc[k]) / max(self.acc[k + 1] - self.acc[k], 1e-12)
        (x0, z0), (x1, z1) = self.pts[k], self.pts[k + 1]
        return V((math.copysign(x0 + (x1 - x0) * t, s), z0 + (z1 - z0) * t, 0.0))


def _mask_grid():
    """Front-view (x, z) grid of the plate: Coons patch between top edge, rear side edges and V."""
    vedge = _VEdge()
    cols = [(-x, -s, j) for j, (x, s) in reversed(list(enumerate(_MASK_COLS))) if j] + \
           [(x, s, j) for j, (x, s) in enumerate(_MASK_COLS)]
    TL, TR = V((-MASK_XC, MASK_TOP_Z, 0)), V((MASK_XC, MASK_TOP_Z, 0))
    SL, SR = vedge(-1.0), vedge(1.0)

    def side(v, sg):
        z = MASK_TOP_Z + (MASK_SIDE_Z - MASK_TOP_Z) * v
        return V((sg * mask_w(z), z, 0))
    grid, cls = [], []
    for x, s, j in cols:
        u = (x + MASK_XC) / (2 * MASK_XC)
        top, bot = V((x, MASK_TOP_Z, 0)), vedge(s)
        L = (top - bot).length
        vt = [d / L for d in _ROWS_TOP]
        vb = [1.0 - d / L for d in _ROWS_BOT]
        vm = [vt[-1] + (vb[0] - vt[-1]) * (k + 1) / (N_MID + 1) for k in range(N_MID)]
        col = []
        for v in vt + vm + vb:
            p = ((1 - v) * top + v * bot + (1 - u) * side(v, -1) + u * side(v, 1)
                 - ((1 - u) * (1 - v) * TL + u * (1 - v) * TR + (1 - u) * v * SL + u * v * SR))
            col.append((p.x, p.y))
        grid.append(col)
        cls.append(j)
    return grid, cls


def _relief(j, r, nr):
    """Offset of grid vertex (column class j = 0 centre .. 11 outline, row r of 0..nr)."""
    if r <= 1 or r >= nr - 1 or j >= 10:
        return H_RIM
    if r == 2 or r == nr - 2 or j == 9:
        return 0.0                                        # rim foot
    if j <= 2:                                            # centre ridge, T cap, V channel
        if r in (3, 4):
            return H_CAP
        if j == 0 and 5 <= r <= 7:
            return H_SLOT
        return H_RIDGE
    if j <= 4:                                            # light grooves, sloped end near the chin
        return H_FLOOR if r <= nr - 4 else H_SIDE
    if j <= 6:
        return H_SIDE                                     # side ridges
    if r in (3, 4):
        return H_LIGHT                                    # recess for the brow light
    if r == nr - 4:
        return H_CHEV                                     # chevron line parallel to the V edge
    return 0.0


def build_mask(surf):
    grid, cls = _mask_grid()
    nc, nr = len(grid), len(grid[0]) - 1
    verts, faces = [], []
    G = [[None] * (nr + 1) for _ in range(nc)]
    for c in range(nc):
        for r in range(nr + 1):
            x, z = grid[c][r]
            verts.append(mask_pt(x, z, _relief(cls[c], r, nr)))
            G[c][r] = len(verts) - 1
    for c in range(nc - 1):
        for r in range(nr):
            faces.append((G[c][r], G[c + 1][r], G[c + 1][r + 1], G[c][r + 1]))
    # rolled outer edge + wall back into the head all round the outline
    loop = ([(c, 0) for c in range(nc)] + [(nc - 1, r) for r in range(1, nr + 1)]
            + [(c, nr) for c in range(nc - 2, -1, -1)] + [(0, r) for r in range(nr - 1, 0, -1)])
    m = len(loop)
    pts = [verts[G[c][r]] for c, r in loop]
    cen = sum(pts, V()) / m
    roll, wall = [], []
    for k, (c, r) in enumerate(loop):
        p = pts[k]
        nrm = mask_n(*grid[c][r])
        perp = []
        for e in (p - pts[k - 1], pts[(k + 1) % m] - p):
            q = e.cross(nrm).normalized()
            perp.append(q if q.dot(p - cen) > 0 else -q)
        mit = (perp[0] + perp[1]).normalized()
        a = p + mit * 0.0010 - nrm * 0.0016
        hit = surf.cast(a, -nrm, WALL_MAX)
        b = hit - nrm * 0.0012 if hit is not None else a - nrm * WALL_MAX
        verts.append(a)
        roll.append(len(verts) - 1)
        verts.append(b)
        wall.append(len(verts) - 1)
    faces += _bridge([G[c][r] for c, r in loop], roll)
    faces += _bridge(roll, wall)
    mask = lib.mesh_object("mask", verts, faces, topo="rigid", sharp_angle=35)
    return mask, grid, cls


def build_lights(grid, cls, nr):
    """Glowing slats (FBX material "Lights") as thin strips sunk into the plate's recesses."""
    col = {j * (1 if k >= len(cls) // 2 else -1): k for k, j in enumerate(cls)}
    up, down = 0.0006, 0.0009

    def section(pa, pb, h, inset=0.0005):
        (xa, za), (xb, zb) = pa, pb
        d = math.hypot(xb - xa, zb - za)
        t = min(inset / max(d, 1e-9), 0.3)
        a = (xa + (xb - xa) * t, za + (zb - za) * t)
        b = (xb + (xa - xb) * t, zb + (za - zb) * t)
        return [mask_pt(*a, h + up), mask_pt(*b, h + up), mask_pt(*b, h - down), mask_pt(*a, h - down)]

    def lerp2(p, q, t):
        return (p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t)
    verts, faces = [], []
    for sg in (1, -1):
        c3, c4 = col[3 * sg], col[4 * sg]
        _strip([section(grid[c3][r], grid[c4][r], H_FLOOR) for r in range(3, nr - 3)], verts, faces)
        c7, c8, c9 = col[7 * sg], col[8 * sg], col[9 * sg]
        _strip([section(grid[c][3], grid[c][4], H_LIGHT) for c in (c7, c8)]
               + [section(lerp2(grid[c8][3], grid[c9][3], 0.55), lerp2(grid[c8][4], grid[c9][4], 0.55), H_LIGHT)],
               verts, faces)
    c0, cl, cr = col[0], col[-1], col[1]
    hs = 0.5 * (H_SLOT + H_RIDGE)
    _strip([section(lerp2(grid[c0][r], grid[cl][r], 0.55), lerp2(grid[c0][r], grid[cr][r], 0.55), hs - 0.0003,
                    inset=0.0) for r in range(5, 8)], verts, faces)
    return lib.mesh_object("mask_lights", verts, faces, topo="rigid", sharp_angle=30)


# ------------------------------------------------------------------ entry point
def build():
    hv, hf = _head_mesh()
    surf = _Surf(hv, hf)
    head = build_head(hv, hf)
    mask, grid, cls = build_mask(surf)
    lights = build_lights(grid, cls, len(grid[0]) - 1)
    return [head, build_ears(surf), build_hair(surf), build_hat(), build_band(), mask, lights]
