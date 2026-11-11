"""Head + neck, faceted face mask with vertical slits, wide-brim hat, hat band + emblem.

Everything is derived from spec.HEAD_CENTER / HEAD_RADII / TORSO / CLOTH["hat_*"].

Topology summary
----------------
head        32 around the skull, 16 around the neck.  The skull is a 32-ring tube closed by an
            8x8 all-quad grid on top (hidden in the hat) and by an 8x8 grid underneath whose
            central 4x4 block is removed: that 16-vertex hole IS the top neck loop, so the neck
            loops run on concentrically (16 -> 24 -> 32) into the jaw line.  Only poles: 4 x
            valence 3 on the jaw line, 4 x valence 5 on the neck (plus 4 x valence 3 under the hat).
hat         one closed 32-around shell: quad top cap, crown, brim top, rim loop, brim underside,
            sweatband cone that meets the head at brow level (closed off inside the skull).
hat_band    closed 32-around ribbon, + rigid slide buckle / keeper / studs, front plaque + emblem.
mask        rigid plate: regular column/row grid, each slit is an inset (recessed) groove,
            plain back shell + rim so the plate is a closed solid, bevelled edge, rivets.
"""
import math

import bpy  # noqa: F401
from mathutils import Vector as V

from .. import lib, spec

TAU = math.tau
HC = spec.HEAD_CENTER
HR = spec.HEAD_RADII
C = spec.CLOTH

N_HEAD = 32
N_NECK = 16
N_HAT = 32

HOOD = 0.003                      # balaclava cloth over the bare neck
Z_SKULL_TOP = HC.z + HR.z         # 1.81
Z_CAP_RING = HC.z + 0.625 * HR.z  # 1.765  last planar skull ring (brow level, under the hat)
Z_NECK_END = 1.50                 # PLAN: neck runs down to here inside the collar

BRIM_Z = C["hat_brim_z"]
BRIM_R = C["hat_brim_r"]
CROWN_R = C["hat_crown_r"]
CROWN_TOP = C["hat_crown_top_z"]
BRIM_TH = 0.007
HAT_CY = HC.y


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


def _lin(keys, x):
    """Piecewise linear through [(x, v)]."""
    if x <= keys[0][0]:
        return keys[0][1]
    for (x0, v0), (x1, v1) in zip(keys, keys[1:]):
        if x <= x1:
            return v0 + (v1 - v0) * (x - x0) / (x1 - x0)
    return keys[-1][1]


def _spow(c, e):
    return math.copysign(abs(c) ** e, c)


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


# ------------------------------------------------------------------ head surface
# h = (z - centre) / half height ; factors of HEAD_RADII: half width, front depth, back depth
_HEAD_KEYS = [
    (-1.10, 0.40, 0.66, 0.40),
    (-0.92, 0.56, 0.81, 0.50),   # chin
    (-0.67, 0.76, 0.93, 0.66),   # jaw
    (-0.33, 0.925, 0.985, 0.88),  # mouth / nape
    (0.00, 0.995, 1.00, 0.97),   # cheek bones
    (0.33, 1.01, 0.98, 1.00),    # widest part of the cranium
    (0.625, 0.965, 0.925, 0.96),  # brow (cap ring)
    (0.85, 0.78, 0.74, 0.78),
    (1.00, 0.35, 0.33, 0.35),
]
_HEAD_POW = 2.2                  # slightly boxier than an ellipse -> flatter face / temples


def head_section(z):
    """(half width, front depth, back depth) of the hooded head at height z."""
    fx, ff, fb = _cr(_HEAD_KEYS, (z - HC.z) / HR.z)
    return fx * HR.x, ff * HR.y, fb * HR.y


def head_pt(a, z, grow=0.0):
    """Point on the head surface at plan angle a (0 = +X, 270 deg = front) and height z."""
    rx, df, db = head_section(z)
    c, s = math.cos(a), math.sin(a)
    e = 2.0 / _HEAD_POW
    d = db if s > 0 else df
    return V(((rx + grow) * _spow(c, e), HC.y + (d + grow) * _spow(s, e), z))


def _jaw_z(a):
    """Height of the lowest skull ring: chin at the front, jaw angle at the sides, skull base behind."""
    s = math.sin(a)
    zs = HC.z - 0.65 * HR.z
    return zs + (0.036 * s ** 1.5 if s > 0 else -0.032 * (-s) ** 1.5)


def _neck_ring(z, tilt=0.0, n=N_NECK):
    cy, rx, ry = spec.torso_section(min(max(z, Z_NECK_END), 1.60))
    out = []
    for k in range(n):
        a = TAU * k / n
        out.append(V(((rx + HOOD) * math.cos(a), cy + (ry + HOOD) * math.sin(a), z + tilt * math.sin(a))))
    return out


def build_head():
    n = N_HEAD
    verts, faces = [], []
    # --- skull tube: 9 rings of 32, lowest one follows the jaw line
    rows = 8
    tube = []
    for j in range(rows + 1):
        t = j / rows
        pts = []
        for i in range(n):
            a = TAU * i / n
            z0 = _jaw_z(a)
            pts.append(head_pt(a, z0 + (Z_CAP_RING - z0) * t))
        tube.append(_add_ring(verts, pts))
    for j in range(rows):
        faces += _bridge(tube[j], tube[j + 1])

    # --- top of the skull: 8x8 grid, domed
    rx, df, db = head_section(Z_CAP_RING)

    def dome(p):
        d = db if p.y > HC.y else df
        rho2 = min((p.x / rx) ** 2 + ((p.y - HC.y) / d) ** 2, 1.0)
        return V((p.x, p.y, Z_CAP_RING + (Z_SKULL_TOP - Z_CAP_RING) * (1.0 - rho2) ** 0.62))
    faces += _coons(tube[rows], verts, rot=4, shape=dome)

    # --- underside: 8x8 grid minus the central 4x4 = neck hole (16 verts)
    neck0 = _add_ring(verts, _neck_ring(1.593, tilt=0.010))
    G = [[None] * 9 for _ in range(9)]
    b = lambda k: tube[0][(k + 4) % n]
    h = lambda k: neck0[(k + 2) % N_NECK]
    for i in range(9):
        G[0][i] = b(i)
        G[i][8] = b(8 + i)
        G[8][8 - i] = b(16 + i)
        G[8 - i][0] = b(24 + i)
    for i in range(5):
        G[2][2 + i] = h(i)
        G[2 + i][6] = h(4 + i)
        G[6][6 - i] = h(8 + i)
        G[6 - i][2] = h(12 + i)
    cl = lambda v: min(max(v, 2), 6)
    edge = lambda v: 0 if v == 1 else (8 if v == 7 else v)
    for j in range(1, 8):
        for i in range(1, 8):
            if G[j][i] is None and (j in (1, 7) or i in (1, 7)):
                po, pi = verts[G[edge(j)][edge(i)]], verts[G[cl(j)][cl(i)]]
                p = po.lerp(pi, 0.5)
                out = V((p.x, p.y - HC.y, 0.0))
                if out.length > 1e-6:
                    out.normalize()
                p = p + out * 0.002 + V((0, 0, -0.003))      # rounded under-jaw
                verts.append(p)
                G[j][i] = len(verts) - 1
    for j in range(8):
        for i in range(8):
            if 2 <= i <= 5 and 2 <= j <= 5:
                continue
            faces.append((G[j][i], G[j][i + 1], G[j + 1][i + 1], G[j + 1][i]))

    # --- neck: 16 around, down into the collar
    prev = neck0
    for z, tilt in ((1.571, 0.005), (1.548, 0.002), (1.524, 0.0), (Z_NECK_END, 0.0)):
        cur = _add_ring(verts, _neck_ring(z, tilt))
        faces += _bridge(prev, cur)
        prev = cur
    return lib.mesh_object("head", verts, faces, topo="main")


# ------------------------------------------------------------------ hat
def _brim_r(t):
    return (CROWN_R[0] + (BRIM_R[0] - CROWN_R[0]) * t, CROWN_R[1] + (BRIM_R[1] - CROWN_R[1]) * t)


def brim_z(t, a):
    """Top surface of the brim.  t: 0 at the crown, 1 at the rim.  Gentle saddle: droops at
    front / back, lifts at the sides, plus a slight downward slope and forward tilt."""
    return BRIM_Z - 0.028 * t ** 1.2 + 0.009 * t ** 1.5 * math.cos(2 * a) + 0.007 * t * math.sin(a)


def crown_r(z):
    """Crown ellipse radii at height z (slight taper towards the top)."""
    f = 1.0 - 0.08 * (z - BRIM_Z) / (CROWN_TOP - BRIM_Z)
    return CROWN_R[0] * f, CROWN_R[1] * f


def _hat_ring(rx, ry, zf, n=N_HAT):
    return [V((rx * math.cos(TAU * i / n), HAT_CY + ry * math.sin(TAU * i / n),
               zf(TAU * i / n) if callable(zf) else zf)) for i in range(n)]


def build_hat():
    n = N_HAT
    verts, faces = [], []
    rings = []
    # hidden closure inside the skull, then the sweatband ring that meets the head at brow level
    rings.append([head_pt(TAU * i / n, BRIM_Z + 0.004, grow=-0.022) + V((0, 0, 0.014)) for i in range(n)])
    rings.append([head_pt(TAU * i / n, BRIM_Z - 0.001, grow=-0.003) for i in range(n)])
    # brim underside (inner -> rim)
    T = (0.06, 0.25, 0.44, 0.63, 0.82, 0.955)
    rings.append(_hat_ring(CROWN_R[0] - 0.002, CROWN_R[1] - 0.002, BRIM_Z - BRIM_TH))
    for t in T[1:]:
        rx, ry = _brim_r(t)
        th = BRIM_TH - 0.001 * t
        rings.append(_hat_ring(rx, ry, lambda a, t=t, th=th: brim_z(t, a) - th))
    # rim loop (rounded edge)
    rx, ry = _brim_r(1.0)
    rings.append(_hat_ring(rx, ry, lambda a: brim_z(1.0, a) - 0.003))
    # brim top (rim -> crown)
    for t in reversed(T):
        rx, ry = _brim_r(t)
        rings.append(_hat_ring(rx, ry, lambda a, t=t: brim_z(t, a)))
    # crown wall
    z0, z1 = BRIM_Z + 0.008, CROWN_TOP - 0.011
    for k in range(6):
        z = z0 + (z1 - z0) * k / 5
        rings.append(_hat_ring(*crown_r(z), z))
    # rounded top edge
    rx, ry = crown_r(CROWN_TOP)
    rings.append(_hat_ring(rx - 0.0035, ry - 0.0035, CROWN_TOP - 0.003))
    rings.append(_hat_ring(rx - 0.012, ry - 0.012, CROWN_TOP))
    idx = [_add_ring(verts, r) for r in rings]
    for a, b in zip(idx, idx[1:]):
        faces += _bridge(a, b)
    faces += _coons(idx[0], verts, rot=4)

    def sink(p):                       # very slight telescope dent in the flat top
        rho2 = min((p.x / (rx - 0.012)) ** 2 + ((p.y - HAT_CY) / (ry - 0.012)) ** 2, 1.0)
        return V((p.x, p.y, p.z - 0.004 * (1 - rho2 ** 2)))
    faces += _coons(idx[-1], verts, rot=4, shape=sink)
    return lib.mesh_object("hat", verts, faces, topo="main")


BAND_Z = (BRIM_Z + 0.004, BRIM_Z + 0.040)
BAND_OFF = 0.0045


def build_band():
    """Closed ribbon around the base of the crown."""
    n = N_HAT
    z0, z1 = BAND_Z
    zm = (z0 + z1) / 2
    sect = [(-0.004, z0), (0.003, z0), (BAND_OFF, z0 + 0.003), (BAND_OFF, zm), (BAND_OFF, z1 - 0.003),
            (0.003, z1), (-0.004, z1)]
    verts, faces = [], []
    idx = []
    for off, z in sect:
        rx, ry = crown_r(z)
        idx.append(_add_ring(verts, _hat_ring(rx + off, ry + off, z)))
    for k in range(len(sect)):
        faces += _bridge(idx[k], idx[(k + 1) % len(sect)])
    return lib.mesh_object("hat_band", verts, faces, topo="main")


def _band_point(a, z=None, lift=0.0):
    """Point on the outer face of the hat band at plan angle a, and its outward normal."""
    z = (BAND_Z[0] + BAND_Z[1]) / 2 if z is None else z
    rx, ry = crown_r(z)
    rx, ry = rx + BAND_OFF, ry + BAND_OFF
    nrm = V((math.cos(a) / rx, math.sin(a) / ry, 0.0)).normalized()
    return V((rx * math.cos(a), HAT_CY + ry * math.sin(a), z)) + nrm * lift, nrm


def _on_band(ob, a, z=None, lift=0.0):
    """Move an object modelled at the origin facing -Y onto the band at plan angle a."""
    p, nrm = _band_point(a, z, lift)
    return lib.transform(ob, loc=p, rot=(0, 0, math.atan2(nrm.y, nrm.x) + math.pi / 2))


def _chamfer_rect(w, h, c, y, x0=0.0):
    hw, hh = w / 2, h / 2
    xz = [(hw, hh - c), (hw - c, hh), (-hw + c, hh), (-hw, hh - c),
          (-hw, -hh + c), (-hw + c, -hh), (hw - c, -hh), (hw, -hh + c)]
    return [V((x + x0, y, z)) for x, z in xz]


def _plaque(name, w, h, th, bev, x0=0.0):
    """Small chamfered rectangular plate at the origin facing -Y, back on y=0.  All quads
    (12-vertex rings, grid caps)."""
    def ring(inset, y):
        hw, hh, c = w / 2 - inset, h / 2 - inset, min(w, h) * 0.16
        xz = [(hw, 0), (hw, hh - c), (hw - c, hh), (0, hh), (-hw + c, hh), (-hw, hh - c),
              (-hw, 0), (-hw, -hh + c), (-hw + c, -hh), (0, -hh), (hw - c, -hh), (hw, -hh + c)]
        return [V((x + x0, y, z)) for x, z in xz]
    verts, faces = [], []
    idx = [_add_ring(verts, r) for r in (ring(0, 0.0), ring(0, -th + bev), ring(bev, -th))]
    faces += _bridge(idx[0], idx[1]) + _bridge(idx[1], idx[2])
    faces += _coons(idx[0], verts) + _coons(idx[2], verts)
    return lib.mesh_object(name, verts, faces, topo="rigid", sharp_angle=30)


def _frame(name, w, h, bar, depth):
    """Rectangular buckle frame at the origin facing -Y (closed, all quads)."""
    loops = [_chamfer_rect(w + bar, h + bar, bar * 0.7, -depth), _chamfer_rect(w + bar, h + bar, bar * 0.7, 0.0),
             _chamfer_rect(w - bar, h - bar, bar * 0.25, 0.0), _chamfer_rect(w - bar, h - bar, bar * 0.25, -depth)]
    verts, faces = [], []
    idx = [_add_ring(verts, lp) for lp in loops]
    for k in range(4):
        faces += _bridge(idx[k], idx[(k + 1) % 4])
    return lib.mesh_object(name, verts, faces, topo="rigid", sharp_angle=30)


def _rivet(name, radius, height, loc, normal, n=8):
    """Closed domed rivet (lib.stud leaves the base open)."""
    rs = [lib.ring_z(-0.0012, radius, n=n), lib.ring_z(height * 0.55, radius * 0.88, n=n),
          lib.ring_z(height, radius * 0.5, n=n)]
    ob = lib.lofted(name, rs, cap_start=True, cap_end=True, topo="rigid")
    return lib.align_z_to(ob, normal, loc)


BUCKLE_A = math.radians(28.0)     # slide buckle on the wearer's left, slightly behind the ear


def build_band_details():
    objs = []
    band_h = BAND_Z[1] - BAND_Z[0]
    # front plaque carrying the faction emblem
    plaque = _on_band(_plaque("hat_plaque", 0.040, band_h + 0.010, 0.006, 0.0015), -math.pi / 2, lift=-0.001)
    p, nrm = _band_point(-math.pi / 2, lift=0.005)
    emblem = lib.emblem_cross("hat_emblem", size=0.0125, thickness=0.003,
                              loc=p + V((0, 0, 0.0035)), normal=nrm, up=(0, 0, 1))
    objs += [plaque, emblem]
    # slide buckle + keeper where the band fastens
    frame = _on_band(_frame("b_frame", 0.024, band_h + 0.004, 0.0042, 0.004), BUCKLE_A, lift=0.0)
    prong = _on_band(lib.box("b_prong", (0.026, 0.003, 0.0034), loc=(0, -0.0035, 0)), BUCKLE_A)
    keeper = _on_band(_plaque("b_keeper", 0.008, band_h + 0.003, 0.003, 0.001, x0=-0.027), BUCKLE_A, lift=-0.0005)
    tail = _on_band(_plaque("b_tail", 0.030, band_h - 0.010, 0.0022, 0.0008, x0=-0.030), BUCKLE_A, lift=-0.0005)
    objs.append(lib.join([frame, prong, keeper, tail], "hat_buckle", topo="rigid"))
    # studs round the band
    studs = []
    for k, deg in enumerate((-90 - 52, -90 + 52, -90 - 104, 180 - 28, 90, 90 - 32, -14)):
        p, nrm = _band_point(math.radians(deg))
        studs.append(_rivet(f"stud{k}", 0.0048, 0.0032, p, nrm))
    objs.append(lib.join(studs, "hat_band_studs", topo="rigid"))
    return objs


# ------------------------------------------------------------------ face mask
MASK_Z_TOP = BRIM_Z - 0.011       # tucked under the brim / sweatband
MASK_Z_TIP = HC.z - HR.z - 0.016  # pointed chin, a little below the real chin
MASK_Z_SIDE = HC.z - 0.055        # bottom corner of the cheek edge
MASK_NR = 12                      # rows
MASK_CREASE_ROW = 7               # chevron crease: the plate is straight above and below it
MASK_XE = 0.095                   # half width at the top (temple)
MASK_WF = 0.045                   # half width of the front (grille) panel
MASK_PROUD = 0.010                # outer face in front of the head surface (centre line)
MASK_TH = 0.005
MASK_BEVEL = 0.0018
GROOVE_PITCH = 0.0165
GROOVE_HW = 0.0042
GROOVE_FLOOR_HW = 0.0028
GROOVE_DEPTH = 0.0036
_MASK_DEP = [(MASK_WF, 0.004), (0.058, 0.013), (0.071, 0.026), (0.084, 0.044), (MASK_XE, 0.067)]
_VC = MASK_CREASE_ROW / MASK_NR
_Y_TOP = HC.y - head_section(MASK_Z_TOP)[1] - MASK_PROUD
_Y_MID = HC.y - HR.y - MASK_PROUD - 0.001
_Y_TIP = HC.y - head_section(HC.z - 0.92 * HR.z)[1] - MASK_PROUD + 0.004
_ZC0 = MASK_Z_TOP + (MASK_Z_TIP - MASK_Z_TOP) * _VC
_K_UP = (_Y_MID - _Y_TOP) / (MASK_Z_TOP - _ZC0)      # upper facet: y per metre of drop
_K_LOW = (_Y_TIP - _Y_MID) / (_ZC0 - MASK_Z_TIP)     # lower facet (recedes to the chin tip)


def _mask_dep(x):
    x = abs(x)
    if x <= MASK_WF:
        return 0.004 * (x / MASK_WF) ** 2
    return _lin(_MASK_DEP, x)


def mask_surf(X, v):
    """Outer surface of the plate.  X = plan position at the top edge, v = 0 top .. 1 bottom.
    Rows are horizontal at the top and turn into a chevron towards the pointed chin; the plate
    is two flat-ish facets (in side view) that meet along the chevron row MASK_CREASE_ROW."""
    X = max(-MASK_XE, min(MASK_XE, X))
    u = abs(X) / MASK_XE
    zbot = MASK_Z_TIP + (MASK_Z_SIDE - MASK_Z_TIP) * u ** 1.1
    z = MASK_Z_TOP + (zbot - MASK_Z_TOP) * v
    zc = MASK_Z_TOP + (zbot - MASK_Z_TOP) * _VC
    y = _Y_TOP + _K_UP * (MASK_Z_TOP - max(z, zc)) + _K_LOW * max(zc - z, 0.0)
    s = 1.0 - 0.08 * v ** 1.6
    return V((X * s, y + _mask_dep(X) * (1.0 - 0.30 * v), z))


def _mask_normal_in(X, v):
    e = 1e-3
    du = mask_surf(X + e, v) - mask_surf(X - e, v)
    dv = mask_surf(X, min(v + e, 1.0)) - mask_surf(X, max(v - e, 0.0))
    nrm = du.cross(dv).normalized()
    return nrm if nrm.y > 0 else -nrm


def build_mask():
    NR = MASK_NR
    groove_c = [GROOVE_PITCH * k for k in (-2, -1, 0, 1, 2)]
    half = []
    for gc in groove_c:
        half += [gc - GROOVE_HW, gc + GROOVE_HW]
    cheek = [x for x, _ in _MASK_DEP]
    cols = [-x for x in reversed(cheek)] + half + cheek
    NC = len(cols)
    verts, faces = [], []
    outer = [[None] * NC for _ in range(NR + 1)]
    mid = [[None] * NC for _ in range(NR + 1)]      # bevel loop (border only)
    inner = [[None] * NC for _ in range(NR + 1)]
    for r in range(NR + 1):
        for c, X in enumerate(cols):
            v = r / NR
            border = r in (0, NR) or c in (0, NC - 1)
            # the visible face stops MASK_BEVEL short of the outline -> chamfered edge
            Xo = math.copysign(abs(X) - MASK_BEVEL, X) if c in (0, NC - 1) else X
            vo = 0.012 if r == 0 else (1 - 0.012 if r == NR else v)
            p = mask_surf(X, v)
            nin = _mask_normal_in(X, v)
            verts.append(mask_surf(Xo, vo))
            outer[r][c] = len(verts) - 1
            verts.append(p + nin * MASK_TH)
            inner[r][c] = len(verts) - 1
            if border:
                verts.append(p + nin * MASK_BEVEL)
                mid[r][c] = len(verts) - 1
    # grooves: column index -> (first row, last row)
    first_gc = len(cheek)
    groove_cols = {first_gc + 2 * k: (1, 10 if abs(k - 2) < 2 else 9) for k in range(5)}
    for r in range(NR):
        for c in range(NC - 1):
            faces.append((inner[r][c], inner[r][c + 1], inner[r + 1][c + 1], inner[r + 1][c]))
            if c in groove_cols and groove_cols[c][0] <= r < groove_cols[c][1]:
                continue
            faces.append((outer[r][c], outer[r][c + 1], outer[r + 1][c + 1], outer[r + 1][c]))
    back = V((0, 1, 0))
    for c, (r1, r2) in groove_cols.items():
        gc = (cols[c] + cols[c + 1]) / 2
        L, R = [], []
        for r in range(r1, r2 + 1):
            rr = r + (0.45 if r == r1 else (-0.45 if r == r2 else 0.0))
            for lst, X in ((L, gc - GROOVE_FLOOR_HW), (R, gc + GROOVE_FLOOR_HW)):
                verts.append(mask_surf(X, rr / NR) + back * GROOVE_DEPTH)
                lst.append(len(verts) - 1)
        for k in range(r2 - r1):
            r = r1 + k
            faces.append((outer[r][c], L[k], L[k + 1], outer[r + 1][c]))
            faces.append((L[k], R[k], R[k + 1], L[k + 1]))
            faces.append((R[k], outer[r][c + 1], outer[r + 1][c + 1], R[k + 1]))
        faces.append((outer[r1][c], outer[r1][c + 1], R[0], L[0]))
        faces.append((L[-1], R[-1], outer[r2][c + 1], outer[r2][c]))
    # bevel + rim
    loop = ([(0, c) for c in range(NC)] + [(r, NC - 1) for r in range(1, NR + 1)]
            + [(NR, c) for c in range(NC - 2, -1, -1)] + [(r, 0) for r in range(NR - 1, 0, -1)])
    for (ra, ca), (rb, cb) in zip(loop, loop[1:] + loop[:1]):
        faces.append((outer[ra][ca], outer[rb][cb], mid[rb][cb], mid[ra][ca]))
        faces.append((mid[ra][ca], mid[rb][cb], inner[rb][cb], inner[ra][ca]))
    return lib.mesh_object("mask", verts, faces, topo="rigid", sharp_angle=32)


MASK_RIVETS = [(0.0895, 0.10), (0.0895, 0.47), (0.0895, 0.86)]     # (X, v) on the left cheek
STRAP_Z, STRAP_W, STRAP_TH = HC.z + 0.014, 0.016, 0.0028


def build_mask_details():
    objs = []
    rivets = []
    for k, (X, v) in enumerate(MASK_RIVETS):
        for sgn in (1, -1):
            rivets.append(_rivet(f"mr{k}{sgn}", 0.0042, 0.003, mask_surf(sgn * X, v), -_mask_normal_in(sgn * X, v)))
    objs.append(lib.join(rivets, "mask_rivets", topo="rigid"))
    # retaining strap: from under one cheek edge round the back of the head to the other.
    # follows the head loops (same 32-around spacing), closed box section, capped ends.
    verts, faces = [], []
    sect = []
    for i in range(-3, 20):
        a = TAU * i / N_HEAD
        zc = STRAP_Z - 0.006 * math.sin(a)
        ring = [head_pt(a, zc - STRAP_W / 2, grow=0.0004), head_pt(a, zc - STRAP_W / 2, grow=STRAP_TH),
                head_pt(a, zc + STRAP_W / 2, grow=STRAP_TH), head_pt(a, zc + STRAP_W / 2, grow=0.0004)]
        sect.append(_add_ring(verts, ring))
    for s0, s1 in zip(sect, sect[1:]):
        for k in range(4):
            faces.append((s0[k], s0[(k + 1) % 4], s1[(k + 1) % 4], s1[k]))
    faces += [tuple(sect[0]), tuple(sect[-1])]
    objs.append(lib.mesh_object("mask_strap", verts, faces, topo="main", sharp_angle=50))
    return objs


# ------------------------------------------------------------------ entry point
def build():
    objs = [build_head(), build_hat(), build_band()]
    objs += build_band_details()
    objs.append(build_mask())
    objs += build_mask_details()
    return objs
