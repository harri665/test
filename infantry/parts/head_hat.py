"""Head + neck gaiter, shield face mask with ridge and slits, creased wide-brim hat, band + plaque.

Every position derives from spec v2 (HEAD_CENTER / HEAD_RADII / TORSO / CLOTH["hat_*"]); the
few part-only constants below say where in docs/ref/*_head.png / callout_hat.png they were read.

Topology summary
----------------
head        32 around the skull, 16 around the neck.  The skull is a 32-ring tube closed by an
            8x8 all-quad grid on top (inside the hat crown) and by an 8x8 grid underneath whose
            central 4x4 block is removed: that 16-vertex hole IS the top neck loop, so the loops
            run concentrically (16 -> 24 -> 32) into the jaw line.  Poles: 4 x valence 3 on the jaw
            line, 4 x valence 5 at the neck top, 4 x valence 3 on the (hidden) skull top.
            The neck below is a scarf / gaiter: stacked horizontal rolls, 16 around, down to z 1.55.
hat         one 32-around shell: creased top cap (8x8 grid), crown with front pinches, brim top,
            rounded rim loop, brim underside, sweatband ring that ends just inside the head (open
            there; no hidden closure cap).
hat_band    closed 32-around ribbon; rigid diamond plaque (curved to the band, raised frame,
            recessed field) with a small 4-point star; small diamond studs on both sides.
mask        rigid shield: column/row grid with a raised centre ridge and four recessed slits;
            bevelled rim turned back toward the face, open at the back (no hidden back shell).
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

# ------------------------------------------------------------------ hat (spec v2 + reads from the sheet)
BRIM_Z = C["hat_brim_z"]           # brim meets the crown
RIM_Z = C["hat_brim_rim_z"]        # rim height at the sides (front view)
BRIM_R = C["hat_brim_r"]
CROWN_R = C["hat_crown_r"]         # crown base, hidden under the band
CROWN_TOP = C["hat_crown_top_z"]
HAT_CY = HC.y
BRIM_TH = 0.009                    # rim thickness: callout_hat shows a clearly thick rounded rim
BRIM_FRONT_DROOP = 0.024           # side_head: the front rim hangs ~2.5 cm lower than the sides
BRIM_BACK_LIFT = 0.006             # side_head: the back rim sits slightly higher than the sides
BRIM_CURVE = 1.35                  # droop concentrated toward the rim (drooping cone)
CROWN_UPPER = 0.93                 # compare FRONT/SIDE: crown just above the band is 0.93 x hat_crown_r
CROWN_TAPER = 0.10                 # compare FRONT f=0.02: ... narrowing ~5 % more toward the top edge
CROWN_ROUND = 0.016                # front_head: soft, well-rounded top edge
BAND_H = 0.038                     # front_head: band is ~30 % of the crown height
BAND_OFF = 0.005                   # band outer face over the crown -> 0.093 half width (FRONT table)
BAND_Z = (BRIM_Z + 0.002, BRIM_Z + 0.002 + BAND_H)
CREASE_DEPTH = 0.0060              # front_head / back_head: notch in the middle of the top edge
CREASE_W = 0.026                   # half width (gaussian) of the centre crease
PINCH_DEPTH = 0.0110               # front_head: pinch dimples at the upper front corners
PINCH_A = math.radians(40)         # ... this far either side of the front centre line
FRONT_LEAN = 0.009                 # side_head: the crown front leans back toward the top
PLAQUE_HW, PLAQUE_HH = 0.035, 0.033   # front_head: diamond ~35 % of crown width, ~45 % of its height
PLAQUE_TH = 0.0045
STUD_ANGLES = (-19.0, -56.0, 27.0)    # side_head: three small diamond studs per side (deg, left side)

# ------------------------------------------------------------------ head / neck
HOOD = 0.003                       # balaclava cloth over the bare neck
Z_SKULL_TOP = HC.z + HR.z
Z_CAP_RING = HC.z + 0.625 * HR.z   # last planar skull ring (inside the crown)
Z_NECK_TOP = HC.z - 1.05 * HR.z    # neck loop under the jaw
Z_NECK_END = 1.55                  # DETAIL_PASS seam: neck runs down to here inside the collar
NECK_SECTION_MIN_Z = spec.TORSO[-1][0]          # front_head: the gaiter neck is clearly narrower than the head,
                                                # so it keeps the spec's neck-top section all the way down
GAITER = (0.003, 0.009)            # callout_collar: scarf rolls, valley / crest offset over the neck
GAITER_ROLLS = 5

# ------------------------------------------------------------------ mask (DETAIL_PASS checklist + front_head)
MASK_HW = 0.056                    # half width of the shield at the brow ...
MASK_HW_SIDE = 0.047               # ... narrowing to here at the cheek corners (diagonal edge in side_head)
MASK_Z_TOP = BRIM_Z - BRIM_TH - 0.004   # flat brow edge tucked under the brim
MASK_Z_SIDE = 1.690                # front_head: straight sides end here ...
MASK_Z_TIP = 1.630                 # ... and taper to the pointed chin
MASK_PROUD = 0.008
MASK_TH = 0.005
MASK_BEVEL = 0.0016
MASK_NR = 8
RIDGE_H, RIDGE_HW = 0.0030, 0.0065  # central vertical ridge
SLITS = (0.016, 0.034)             # slit centre lines, each side of the ridge (four slits)
SLIT_HW, SLIT_FLOOR_HW, SLIT_DEPTH = 0.0028, 0.0016, 0.0035
SLIT_ROWS = (1, 7)


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


# ------------------------------------------------------------------ head surface
# h = (z - centre) / half height ; factors of HEAD_RADII: half width, front depth, back depth.
# Under the brim (h ~ -0.2) the sheet's head is ~0.93 x HEAD_RADII.x wide (compare FRONT f=0.10).
_HEAD_KEYS = [
    (-1.10, 0.42, 0.60, 0.46),
    (-0.92, 0.58, 0.72, 0.58),   # chin
    (-0.67, 0.76, 0.80, 0.74),   # jaw
    (-0.33, 0.92, 0.86, 0.85),   # mouth / nape
    (0.00, 0.97, 0.90, 0.92),    # cheek bones
    (0.33, 0.99, 0.93, 0.97),    # widest part of the cranium (inside the hat)
    (0.625, 0.965, 0.91, 0.95),  # cap ring
    (0.85, 0.78, 0.74, 0.78),
    (1.00, 0.35, 0.33, 0.35),
]
_HEAD_POW = 2.2                  # back half: slightly boxier than an ellipse
_FACE_POW = 1.75                 # front half: wedge-like face plan, so the mask's sides sweep back (side_head)


def head_section(z):
    """(half width, front depth, back depth) of the hooded head at height z."""
    fx, ff, fb = _cr(_HEAD_KEYS, (z - HC.z) / HR.z)
    return fx * HR.x, ff * HR.y, fb * HR.y


def head_pt(a, z, grow=0.0):
    """Point on the head surface at plan angle a (0 = +X, 270 deg = front) and height z."""
    rx, df, db = head_section(z)
    c, s = math.cos(a), math.sin(a)
    e = 2.0 / (_HEAD_POW if s > 0 else _FACE_POW)
    d = db if s > 0 else df
    return V(((rx + grow) * _spow(c, e), HC.y + (d + grow) * _spow(s, e), z))


def head_front_y(x, z, grow=0.0):
    """y of the head's front surface (+grow) at lateral position x, height z."""
    rx, df, _ = head_section(z)
    rx, df = rx + grow, df + grow
    u = min(abs(x) / rx, 0.999)
    return HC.y - df * (1.0 - u ** _FACE_POW) ** (1.0 / _FACE_POW)


def _jaw_z(a):
    """Height of the lowest skull ring: chin at the front, jaw angle at the sides, skull base behind."""
    s = math.sin(a)
    zs = HC.z - 0.65 * HR.z
    return zs + (0.30 * HR.z * s ** 1.5 if s > 0 else -0.27 * HR.z * (-s) ** 1.5)


def _neck_ring(z, off=HOOD, tilt=0.0, n=N_NECK, wobble=0.0, seed=0.0):
    # clamp above the shoulder flare: the gaiter stays a tube down into the collar
    cy, rx, ry = spec.torso_section(min(max(z, NECK_SECTION_MIN_Z), spec.TORSO[-1][0]))
    out = []
    for k in range(n):
        a = TAU * k / n
        o = off + (lib.fold_wave(k / n, 0.5, amp=wobble, count=3.0, seed=seed, grow=0.0) if wobble else 0.0)
        out.append(V(((rx + o) * math.cos(a), cy + (ry + o) * math.sin(a), z + tilt * math.sin(a))))
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

    # --- top of the skull: 8x8 grid, domed (inside the crown)
    rx, df, db = head_section(Z_CAP_RING)

    def dome(p):
        d = db if p.y > HC.y else df
        rho2 = min((p.x / rx) ** 2 + ((p.y - HC.y) / d) ** 2, 1.0)
        return V((p.x, p.y, Z_CAP_RING + (Z_SKULL_TOP - Z_CAP_RING) * (1.0 - rho2) ** 0.62))
    faces += _coons(tube[rows], verts, rot=4, shape=dome)

    # --- underside: 8x8 grid minus the central 4x4 = neck hole (16 verts)
    neck0 = _add_ring(verts, _neck_ring(Z_NECK_TOP, off=HOOD + 0.004, tilt=0.010))
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

    # --- neck gaiter: GAITER_ROLLS stacked rolls (crest / valley rings), sagging a little at the front
    prev = neck0
    z_top = Z_NECK_TOP - 0.010
    nring = 2 * GAITER_ROLLS
    for k in range(nring + 1):
        t = k / nring
        z = z_top + (Z_NECK_END - z_top) * t
        crest = k % 2 == 1
        off = HOOD + (GAITER[1] if crest else GAITER[0])
        if k == nring:
            off = HOOD + GAITER[0]
        sag = 0.006 * (1.0 - t)                       # rolls dip at the front, rise behind
        cur = _add_ring(verts, _neck_ring(z, off=off, tilt=sag, wobble=0.0015 if crest else 0.0, seed=k * 0.37))
        faces += _bridge(prev, cur)
        prev = cur
    return lib.mesh_object("head", verts, faces, topo="main")


# ------------------------------------------------------------------ hat
def _brim_r(t):
    return (CROWN_R[0] + (BRIM_R[0] - CROWN_R[0]) * t, CROWN_R[1] + (BRIM_R[1] - CROWN_R[1]) * t)


def rim_z(a):
    """Rim height at plan angle a: hat_brim_rim_z at the sides, droops at the front, a little up behind."""
    s = math.sin(a)
    return RIM_Z - BRIM_FRONT_DROOP * max(-s, 0.0) ** 2 + BRIM_BACK_LIFT * max(s, 0.0) ** 2


def brim_z(t, a):
    """Top surface of the brim.  t: 0 at the crown, 1 at the rim."""
    return BRIM_Z + (rim_z(a) - BRIM_Z) * t ** BRIM_CURVE


def crown_r(z):
    """Crown ellipse radii at height z: hat_crown_r under the band, CROWN_UPPER x that above it."""
    f = 1.0 - (1.0 - CROWN_UPPER) * _smooth(BRIM_Z + 0.006, BAND_Z[1] - 0.004, z)
    f *= 1.0 - CROWN_TAPER * min(max((z - BAND_Z[1]) / (CROWN_TOP - BAND_Z[1]), 0.0), 1.0)
    return CROWN_R[0] * f, CROWN_R[1] * f


def _hat_ring(rx, ry, zf, n=N_HAT):
    return [V((rx * math.cos(TAU * i / n), HAT_CY + ry * math.sin(TAU * i / n),
               zf(TAU * i / n) if callable(zf) else zf)) for i in range(n)]


def _crown_shape(p):
    """Fedora deformation of crown / top points: centre crease along the top (front-back) and two
    pinch dimples at the upper front corners."""
    up = _smooth(BAND_Z[1] + 0.012, CROWN_TOP, p.z)
    if up <= 0.0:
        return p
    q = p.copy()
    # centre crease: deepest at the front half, a notch in the FRONT / BACK silhouettes
    g = math.exp(-(p.x / CREASE_W) ** 2)
    fb = 1.0 if p.y < HAT_CY else 0.8
    q.z -= CREASE_DEPTH * fb * g * _smooth(CROWN_TOP - 0.022, CROWN_TOP - 0.004, p.z)
    a = math.atan2(p.y - HAT_CY, p.x)
    # front of the crown leans back toward the top
    q.y += FRONT_LEAN * up * max(-math.sin(a), 0.0) ** 2
    # pinches
    for ap in (-math.pi / 2 + PINCH_A, -math.pi / 2 - PINCH_A):
        d = (a - ap + math.pi) % TAU - math.pi
        w = math.exp(-(d / 0.36) ** 2) * up
        if w > 1e-3:
            r = V((p.x, p.y - HAT_CY, 0.0))
            q -= r.normalized() * (PINCH_DEPTH * w) if r.length > 1e-6 else V()
    return q


def build_hat():
    n = N_HAT
    verts, faces = [], []
    rings = []
    # sweatband: ends just inside the head at brow level (open there; the head fills it)
    rings.append([head_pt(TAU * i / n, BRIM_Z - 0.002, grow=-0.002) for i in range(n)])
    # brim underside (inner -> rim)
    T_UNDER = (0.0, 0.24, 0.48, 0.72, 0.92)
    for t in T_UNDER:
        rx, ry = _brim_r(t)
        if t == 0.0:
            rx, ry = rx - 0.002, ry - 0.002
        th = BRIM_TH - 0.0015 * (1 - t)
        rings.append(_hat_ring(rx, ry, lambda a, t=t, th=th: brim_z(t, a) - th))
    # rounded rim: bottom edge, outer edge, top edge
    rxo, ryo = _brim_r(1.0)
    for dr, dz in ((0.0045, 1.0), (0.0013, 0.86), (0.0, 0.5), (0.0013, 0.14), (0.0045, 0.0)):
        rings.append(_hat_ring(rxo - dr, ryo - dr, lambda a, dz=dz: brim_z(1.0, a) - BRIM_TH * dz))
    # brim top (rim -> crown)
    for t in (0.92, 0.72, 0.5, 0.28, 0.1, 0.0):
        rx, ry = _brim_r(t)
        rings.append(_hat_ring(rx, ry, lambda a, t=t: brim_z(t, a)))
    # crown wall: hidden under the band, then ~2 cm rows up to the top edge
    zs = [BAND_Z[0] + 0.012, BAND_Z[1] - 0.010]
    z_edge = CROWN_TOP - CROWN_ROUND
    k = 4
    zs += [BAND_Z[1] + (z_edge - BAND_Z[1]) * i / k for i in range(k + 1)]
    for z in zs:
        rings.append(_hat_ring(*crown_r(z), z))
    # rounded top edge
    rx, ry = crown_r(CROWN_TOP)
    c = CROWN_ROUND
    rings.append(_hat_ring(rx - 0.29 * c, ry - 0.29 * c, CROWN_TOP - 0.29 * c))
    rings.append(_hat_ring(rx - c, ry - c, CROWN_TOP))
    idx = [_add_ring(verts, r) for r in rings]
    for a, b in zip(idx, idx[1:]):
        faces += _bridge(a, b)
    faces += _coons(idx[-1], verts, rot=4)
    verts = [_crown_shape(v) if v.z > BAND_Z[1] else v for v in verts]
    return lib.mesh_object("hat", verts, faces, topo="main")


def build_band():
    """Closed ribbon around the base of the crown."""
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


def _band_map(u, z, d, a0=-math.pi / 2):
    """Local plate coords -> world: u along the band (arc length from plan angle a0), height z,
    d out along the band normal measured from the band's outer face (at the band mid height)."""
    rx, ry = crown_r((BAND_Z[0] + BAND_Z[1]) / 2)
    rx, ry = rx + BAND_OFF, ry + BAND_OFF
    r_mean = 0.5 * (rx + ry)
    a = a0 + u / r_mean
    nrm = V((math.cos(a) / rx, math.sin(a) / ry, 0.0)).normalized()
    return V((rx * math.cos(a), HAT_CY + ry * math.sin(a), z)) + nrm * d


def _diamond_plate(name, hw, hh, th, a0, zc, grid=(-1, -0.82, -0.55, 0, 0.55, 0.82, 1),
                   frame=True, back=0.006):
    """Diamond plate bent onto the band: rotated quad grid (s, t), raised frame border, recessed
    field, chamfered outline, side walls running back `back` into the band / crown.  Open behind."""
    S = grid
    m = len(S)
    verts, faces = [], []
    top = [[None] * m for _ in range(m)]

    def depth(s, t):
        k = max(abs(s), abs(t))
        if k >= 0.999:
            return th * 0.55
        if not frame:
            return th * (1.0 - 0.25 * k)
        if k >= 0.8:
            return th
        if k >= 0.5:
            return th - 0.0013
        return th - 0.0013 + 0.0009 * (1 - k / 0.5)          # slight boss in the middle
    for j, t in enumerate(S):
        for i, s in enumerate(S):
            u = (s - t) * 0.5 * hw
            w = (s + t) * 0.5 * hh
            verts.append(_band_map(u, zc + w, depth(s, t), a0))
            top[j][i] = len(verts) - 1
    for j in range(m - 1):
        for i in range(m - 1):
            faces.append((top[j][i], top[j][i + 1], top[j + 1][i + 1], top[j + 1][i]))
    outline = ([top[0][i] for i in range(m)] + [top[j][m - 1] for j in range(1, m)]
               + [top[m - 1][i] for i in range(m - 2, -1, -1)] + [top[j][0] for j in range(m - 2, 0, -1)])
    wall = []
    for vi in outline:
        p = verts[vi]
        # same (u, w) pushed back by `back` along the band normal
        rx, ry = crown_r((BAND_Z[0] + BAND_Z[1]) / 2)
        q = V((p.x, p.y - HAT_CY, 0.0))
        nrm = V((q.x / (rx + BAND_OFF) ** 2, q.y / (ry + BAND_OFF) ** 2, 0.0)).normalized()
        verts.append(p - nrm * (back + th * 0.55))
        wall.append(len(verts) - 1)
    k = len(outline)
    for i in range(k):
        faces.append((outline[i], wall[i], wall[(i + 1) % k], outline[(i + 1) % k]))
    return lib.mesh_object(name, verts, faces, topo="rigid", sharp_angle=30)


def build_band_details():
    objs = []
    # front_head shows the diamond reaching well above the band, side_head keeps it near the band:
    # centred a little above the band's middle
    zc = 0.5 * (BAND_Z[0] + BAND_Z[1]) + 0.008
    objs.append(_diamond_plate("hat_plaque", PLAQUE_HW, PLAQUE_HH, PLAQUE_TH, -math.pi / 2, zc))
    p = _band_map(0.0, zc, PLAQUE_TH - 0.0008)
    objs.append(lib.emblem_cross("hat_emblem", size=0.0105, thickness=0.0025, loc=p, normal=(0, -1, 0),
                                 up=(0, 0, 1), long_tail=1.35))
    studs = []
    zm = (BAND_Z[0] + BAND_Z[1]) / 2
    for side in (1, -1):
        for k, deg in enumerate(STUD_ANGLES):
            a = math.radians(deg) if side > 0 else math.pi - math.radians(deg)
            studs.append(_diamond_plate(f"stud{side}{k}", 0.0075, 0.0085, 0.0026, a, zm,
                                        grid=(-1, -0.4, 0.4, 1), frame=False, back=0.002))
    objs.append(lib.join(studs, "hat_band_studs", topo="rigid"))
    return objs


# ------------------------------------------------------------------ face mask
_Z_SHAPE_MIN = HC.z - 0.60 * HR.z      # below this the mask stops following the receding chin


def _mask_base(x, z):
    """Mask outer surface (before ridge): head front + MASK_PROUD, running on straight below the jaw."""
    zz = max(z, _Z_SHAPE_MIN)
    y = head_front_y(x, zz, grow=MASK_PROUD)
    if z < zz:
        y += (zz - z) * 0.50          # lower point leans back toward the chin ...
    y = min(y, head_front_y(x, z, grow=MASK_PROUD * 0.8))   # ... but stays clear of the faceted chin
    return y


def mask_surf(X, v):
    """Outer surface of the shield.  X = lateral position (straight sides), v = 0 brow .. 1 bottom edge.
    The bottom edge is a V: the centre column reaches the chin tip, the outer columns stop at
    MASK_Z_SIDE, so rows run as chevrons toward the point."""
    X = max(-MASK_HW, min(MASK_HW, X))
    u = abs(X) / MASK_HW
    zbot = MASK_Z_TIP + (MASK_Z_SIDE - MASK_Z_TIP) * u
    z = MASK_Z_TOP + (zbot - MASK_Z_TOP) * v
    k = min(max((MASK_Z_TOP - z) / (MASK_Z_TOP - MASK_Z_SIDE), 0.0), 1.0)
    x = X * (1.0 - (1.0 - MASK_HW_SIDE / MASK_HW) * k)
    ridge = RIDGE_H * max(0.0, 1.0 - abs(x) / RIDGE_HW) ** 1.2
    return V((x, _mask_base(x, z) - ridge, z))


def _mask_normal_in(X, v):
    e = 1e-3
    du = mask_surf(X + e, v) - mask_surf(X - e, v)
    dv = mask_surf(X, min(v + e, 1.0)) - mask_surf(X, max(v - e, 0.0))
    nrm = du.cross(dv).normalized()
    return nrm if nrm.y > 0 else -nrm


def build_mask():
    NR = MASK_NR
    half = [0.0, RIDGE_HW]
    for sc in SLITS:
        half += [sc - SLIT_HW, sc + SLIT_HW]
    half += [0.5 * (SLITS[1] + SLIT_HW + MASK_HW), MASK_HW]   # X is the brow-level column position
    cols = [-x for x in reversed(half[1:])] + half
    NC = len(cols)
    slit_cols = [c for c in range(NC - 1) if any(abs((cols[c] + cols[c + 1]) / 2 - sg * sc) < 1e-6
                                                for sc in SLITS for sg in (1, -1))]
    verts, faces = [], []
    outer = [[None] * NC for _ in range(NR + 1)]
    mid = {}
    back = {}
    for r in range(NR + 1):
        for c, X in enumerate(cols):
            v = r / NR
            border = r in (0, NR) or c in (0, NC - 1)
            # the visible face stops MASK_BEVEL short of the outline -> chamfered edge
            Xo = math.copysign(abs(X) - MASK_BEVEL, X) if c in (0, NC - 1) else X
            vo = 0.012 if r == 0 else (1 - 0.012 if r == NR else v)
            verts.append(mask_surf(Xo, vo))
            outer[r][c] = len(verts) - 1
            if border:
                p = mask_surf(X, v)
                nin = _mask_normal_in(X, v)
                verts.append(p + nin * MASK_BEVEL)
                mid[(r, c)] = len(verts) - 1
                verts.append(p + nin * MASK_TH)
                back[(r, c)] = len(verts) - 1
    r1, r2 = SLIT_ROWS
    for r in range(NR):
        for c in range(NC - 1):
            if c in slit_cols and r1 <= r < r2:
                continue
            faces.append((outer[r][c], outer[r][c + 1], outer[r + 1][c + 1], outer[r + 1][c]))
    # slits: recessed grooves (inset along +Y)
    push = V((0, 1, 0))
    for c in slit_cols:
        gc = (cols[c] + cols[c + 1]) / 2
        L, R = [], []
        for r in range(r1, r2 + 1):
            rr = r + (0.45 if r == r1 else (-0.45 if r == r2 else 0.0))
            for lst, X in ((L, gc - SLIT_FLOOR_HW), (R, gc + SLIT_FLOOR_HW)):
                verts.append(mask_surf(X, rr / NR) + push * SLIT_DEPTH)
                lst.append(len(verts) - 1)
        for k in range(r2 - r1):
            r = r1 + k
            faces.append((outer[r][c], L[k], L[k + 1], outer[r + 1][c]))
            faces.append((L[k], R[k], R[k + 1], L[k + 1]))
            faces.append((R[k], outer[r][c + 1], outer[r + 1][c + 1], R[k + 1]))
        faces.append((outer[r1][c], outer[r1][c + 1], R[0], L[0]))
        faces.append((L[-1], R[-1], outer[r2][c + 1], outer[r2][c]))
    # bevel + rim turned back toward the face (open behind: no hidden back shell)
    loop = ([(0, c) for c in range(NC)] + [(r, NC - 1) for r in range(1, NR + 1)]
            + [(NR, c) for c in range(NC - 2, -1, -1)] + [(r, 0) for r in range(NR - 1, 0, -1)])
    for a, b in zip(loop, loop[1:] + loop[:1]):
        faces.append((outer[a[0]][a[1]], outer[b[0]][b[1]], mid[b], mid[a]))
        faces.append((mid[a], mid[b], back[b], back[a]))
    return lib.mesh_object("mask", verts, faces, topo="rigid", sharp_angle=32)


# ------------------------------------------------------------------ entry point
def build():
    objs = [build_head(), build_hat(), build_band()]
    objs += build_band_details()
    objs.append(build_mask())
    return objs
