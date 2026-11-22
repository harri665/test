"""Head + turtleneck, shield face mask with three raised slats, creased wide-brim hat, band + plaque.

Every position derives from spec v2 (HEAD_CENTER / HEAD_RADII / TORSO / CLOTH["hat_*"]); the
few part-only constants below say where in docs/ref/*_head.png / callout_hat.png they were read.

Topology summary
----------------
head        one 32-around tube from the flared turtleneck hem (z 1.53) to the crown of the skull,
            closed by an 8x8 all-quad grid on top (hidden inside the hat crown).  Every row is a
            closed loop round neck / jaw / face / skull, so the loops encircle the face.  Between
            the neck and the face the rows follow per-column Hermite curves (neck rises vertically,
            bends forward under the chin and runs into the face tangentially) -> a smooth egg-shaped
            hood that tapers into a slim neck, no flat underside, no jaw corners.  Poles: only the
            4 valence-3 corners of the (hidden) top grid.
hat         one 32-around shell: creased top cap (8x8 grid), crown with front pinches, brim top,
            rounded rim loop, brim underside, sweatband ring that ends just inside the head (open
            there; no hidden closure cap).
hat_band    closed 32-around ribbon; rigid diamond plaque (curved to the band, raised frame,
            recessed field) with a small 4-point star; small diamond studs on both sides.
mask        rigid shield plate, a thin shell offset from the face: 12 x 8 quad grid whose columns
            run parallel to the slats / side edges and whose rows run parallel to the straight top
            edge and the V bottom edge; a rolled rim (crest + rounded outer edge + side wall back
            toward the face) all round; open behind.
mask_slats  three rigid raised slats (flat top, chamfered edges, sloped ends) sitting on the plate.
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
N_HAT = 32

# ------------------------------------------------------------------ hat (spec v2 + reads from the sheet)
BRIM_Z = C["hat_brim_z"]           # brim meets the crown
RIM_Z = C["hat_brim_rim_z"]        # rim height at the sides (front view)
BRIM_R = C["hat_brim_r"]
CROWN_R = C["hat_crown_r"]         # crown base, hidden under the band
CROWN_TOP = C["hat_crown_top_z"]
HAT_CY = HC.y
BRIM_TH = 0.009                    # rim thickness: callout_hat shows a clearly thick rounded rim
BRIM_FRONT_DROOP = 0.031           # side_head + review: front rim hangs well below the sides (fedora snap)
BRIM_BACK_DROOP = 0.004            # review: the back curves down too (bowl); compare SIDE keeps the back rim ~1.74
BRIM_CURVE = 1.15                  # droop spread over the brim -> reads as a curved bowl, not a flat disc
BRIM_ROLL = 0.004                  # brim rises into the band: small upward fillet at the crown
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
# The hood is a smooth egg: compare FRONT f=0.10 (z 1.705) gives a half width of ~0.068, compare
# SIDE the same row ~0.083 in front of / ~0.072 behind HEAD_CENTER.y -> side depth only ~1.12 x the
# front width (HEAD_RADII.y is the old, too deep value; the hood uses its own sections below).
HOOD = 0.003                       # balaclava cloth over the bare neck
Z_SKULL_TOP = HC.z + HR.z          # 1.83, inside the crown
Z_CAP_RING = HC.z + 0.625 * HR.z   # last planar skull ring (inside the crown)
Z_FACE0 = 1.655                    # lowest horizontal face loop; below it the rows turn into the neck
Z_NECK_TOP = 1.600                 # highest horizontal neck loop
Z_NECK_END = 1.53                  # neck runs down to here inside the collar (coat neck hem meets it)
TURTLE = 0.006                     # smooth turtleneck over the neck (round-3 review: no ribbing)
TURTLE_FOLD = 0.0015               # amplitude of its two soft folds
NECK_FLARE = 0.012                 # bottom ring flares out to meet the coat's neck opening
NECK_TOP_SHIFT = -0.011            # upper neck sits forward under the skull ...
NECK_TOP_SLIM = (0.006, 0.008)     # ... and slimmer (x, y): slim neck, nape in line with the skull
NECK_LEAN = 0.45                   # throat leans forward into the chin (no S-shaped hook)
N_TRANS = 5                        # rows from the neck top loop to the first face loop

# ------------------------------------------------------------------ mask (front_head / side_head)
# Shield plate: straight top edge tucked under the brim, sides wrapping round the cheeks and running
# down nearly vertically (front view), then a V converging to a crisp point at the chin.
MASK_Z_TOP = BRIM_Z - BRIM_TH - 0.006   # top row of the plate grid (rim adds ~3.5 mm above)
MASK_Z_SIDE = 1.684                # front_head: sides run down to here ...
MASK_Z_TIP = 1.629                 # ... then the V edges converge to the point (rim adds ~5 mm below)
MASK_TH_TOP = math.radians(72)     # side_head: at the brow the plate wraps far round the cheeks
MASK_TH_SIDE = math.radians(55)    # ... less at the bottom of the side edge -> diagonal rear edge
MASK_P = 0.0055                    # plate outer surface over the head surface
MASK_T = 0.0045                    # plate thickness: the rim's side wall ends 1 mm off the head
MASK_NR = 8                        # rows (top edge -> V edge)
MASK_NH = 6                        # columns per half (2 between centre and slat, 4 slat -> side edge)
RIM = (0.0018, 0.0010, 0.0035, -0.0006)   # rim: crest (out, up), outer roll (out, down)
SLAT_X = (0.0300, 0.0265)          # front_head: side slats at the top / bottom (slightly converging)
SLAT_HW, SLAT_H, SLAT_CH = 0.0040, 0.0055, 0.0016   # 8 mm wide, 5.5 mm proud, 1.6 mm chamfers
SLAT_SINK = 0.0008
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
# Horizontal sections of the hood: (z, half width, front depth, back depth), metres from the
# HEAD_CENTER axis.  Rows below Z_FACE0 are guide keys only: the mask's lower point follows them
# (gently receding chin), the mesh itself turns into the neck there.
_HEAD_KEYS = [
    (1.590, 0.0530, 0.0560, 0.0560),
    (1.620, 0.0560, 0.0630, 0.0590),
    (1.655, 0.0605, 0.0730, 0.0640),   # lowest face loop: jaw tapers toward the slim neck
    (1.680, 0.0655, 0.0790, 0.0705),
    (1.705, 0.0690, 0.0825, 0.0765),   # compare row f=0.10
    (1.730, 0.0705, 0.0835, 0.0825),   # side_head: the skull bulges out behind just under the brim
    (1.760, 0.0715, 0.0830, 0.0845),   # brow, at the hat's sweatband
    (1.790, 0.0700, 0.0790, 0.0815),   # cap ring (inside the crown)
    (1.815, 0.0640, 0.0720, 0.0740),
]
_HEAD_POW = 2.05                 # back half: plain round skull
_FACE_POW = 2.0                  # front half: elliptical -> smooth egg, no cheek corners


def head_section(z):
    """(half width, front depth, back depth) of the hooded head at height z."""
    return _cr(_HEAD_KEYS, z)


def head_pt(a, z, grow=0.0):
    """Point on the head surface at plan angle a (0 = +X, 270 deg = front) and height z."""
    rx, df, db = head_section(z)
    c, s = math.cos(a), math.sin(a)
    e = 2.0 / (_HEAD_POW if s > 0 else _FACE_POW)
    d = db if s > 0 else df
    return V(((rx + grow) * _spow(c, e), HC.y + (d + grow) * _spow(s, e), z))


def _neck_ring(z, off, tilt=0.0, n=N_HEAD, wobble=0.0, seed=0.0):
    """Turtleneck loop: the spec's neck-top section (kept all the way down into the collar), moved a
    little forward and slimmed front-back toward the top so the nape lines up with the skull."""
    cy, rx, ry = spec.torso_section(spec.TORSO[-1][0])
    k = _smooth(1.536, 1.578, z)
    cy += NECK_TOP_SHIFT * k
    rx -= NECK_TOP_SLIM[0] * k
    ry -= NECK_TOP_SLIM[1] * k
    out = []
    for i in range(n):
        a = TAU * i / n
        o = off + (lib.fold_wave(i / n, 0.5, amp=wobble, count=3.0, seed=seed, grow=0.0) if wobble else 0.0)
        out.append(V(((rx + o) * math.cos(a), cy + (ry + o) * math.sin(a), z + tilt * math.sin(a))))
    return out


def _hermite(p0, t0, p1, t1, t):
    t2, t3 = t * t, t * t * t
    return ((2 * t3 - 3 * t2 + 1) * p0 + (t3 - 2 * t2 + t) * t0
            + (-2 * t3 + 3 * t2) * p1 + (t3 - t2) * t1)


def build_head():
    n = N_HEAD
    verts, rings = [], []
    # --- turtleneck: flared hem ring at Z_NECK_END, two soft folds, up to the neck top loop
    zs = [Z_NECK_END, 1.542, 1.557, 1.572, 1.587, Z_NECK_TOP]
    for k, z in enumerate(zs):
        off = HOOD + TURTLE + (TURTLE_FOLD if k in (2, 4) else 0.0)
        off += NECK_FLARE if k == 0 else (NECK_FLARE * 0.35 if k == 1 else 0.0)
        sag = 0.004 * (1.0 - k / (len(zs) - 1))       # folds dip slightly at the front
        rings.append(_neck_ring(z, off, tilt=sag, wobble=0.001 if k in (2, 4) else 0.0, seed=k * 0.37))
    # --- neck -> face: per column a Hermite curve leaving the neck vertically and entering the
    #     face loop along the face surface (rounded chin in front, straight nape behind)
    top = rings[-1]
    for k in range(1, N_TRANS):
        t = k / N_TRANS
        ring = []
        for i in range(n):
            a = TAU * i / n
            p0, p1 = top[i], head_pt(a, Z_FACE0)
            d1 = (head_pt(a, Z_FACE0 + 0.004) - head_pt(a, Z_FACE0 - 0.004)).normalized()
            L = (p1 - p0).length
            lean = NECK_LEAN * max(-math.sin(a), 0.0) ** 2     # front of the neck leans forward
            ring.append(_hermite(p0, V((0, -lean, 1)).normalized() * L, p1, d1 * L, t))
        rings.append(ring)
    # --- face / skull loops, ~1.3 cm apart up to the brow, then the cap ring inside the crown
    zb = 1.760
    nf = 8
    for k in range(nf + 1):
        z = Z_FACE0 + (zb - Z_FACE0) * k / nf
        rings.append([head_pt(TAU * i / n, z) for i in range(n)])
    rings.append([head_pt(TAU * i / n, Z_CAP_RING) for i in range(n)])
    idx = [_add_ring(verts, r) for r in rings]
    faces = []
    for a, b in zip(idx, idx[1:]):
        faces += _bridge(a, b)

    # --- top of the skull: 8x8 grid, domed (inside the crown)
    rx, df, db = head_section(Z_CAP_RING)

    def dome(p):
        d = db if p.y > HC.y else df
        rho2 = min((p.x / rx) ** 2 + ((p.y - HC.y) / d) ** 2, 1.0)
        return V((p.x, p.y, Z_CAP_RING + (Z_SKULL_TOP - Z_CAP_RING) * (1.0 - rho2) ** 0.62))
    faces += _coons(idx[-1], verts, rot=4, shape=dome)
    return lib.mesh_object("head", verts, faces, topo="main")



# ------------------------------------------------------------------ hat
def _brim_r(t):
    return (CROWN_R[0] + (BRIM_R[0] - CROWN_R[0]) * t, CROWN_R[1] + (BRIM_R[1] - CROWN_R[1]) * t)


def rim_z(a):
    """Rim height at plan angle a: hat_brim_rim_z at the sides, droops at the front, a little up behind."""
    s = math.sin(a)
    return RIM_Z - BRIM_FRONT_DROOP * max(-s, 0.0) ** 2 - BRIM_BACK_DROOP * max(s, 0.0) ** 2


def brim_z(t, a):
    """Top surface of the brim.  t: 0 at the crown, 1 at the rim."""
    return BRIM_Z + (rim_z(a) - BRIM_Z) * t ** BRIM_CURVE + BRIM_ROLL * max(0.0, 1.0 - t / 0.25) ** 2


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
def mask_pt(th, z, out=0.0):
    """Point on the plate's outer surface (+out along the surface normal, approx.).  th: angle across
    the face (0 = front centre, + toward the character's left), z: height."""
    return head_pt(-math.pi / 2 + th, z, grow=MASK_P + out)


def mask_normal(th, z):
    e = 1e-3
    du = mask_pt(th + e, z) - mask_pt(th - e, z)
    dv = mask_pt(th, z + e) - mask_pt(th, z - e)
    nrm = du.cross(dv).normalized()
    c = mask_pt(th, z)
    return nrm if nrm.dot(V((c.x, c.y - HC.y, 0.0))) > 0 else -nrm


def _th_of_x(x, z):
    """Angle across the face whose plate point lies at lateral position x (front view)."""
    rx = head_section(z)[0] + MASK_P
    s = min(abs(x) / rx, 0.999) ** (_FACE_POW / 2.0)
    return math.copysign(math.asin(s), x)


def _slat_x(z):
    k = (MASK_Z_TOP - z) / (MASK_Z_TOP - MASK_Z_TIP)
    return SLAT_X[0] + (SLAT_X[1] - SLAT_X[0]) * k


def _edge_th(z):
    """Half angle of the side edge: wide at the brow, narrower where the V begins."""
    k = min(max((z - MASK_Z_SIDE) / (MASK_Z_TOP - MASK_Z_SIDE), 0.0), 1.0)
    return MASK_TH_SIDE + (MASK_TH_TOP - MASK_TH_SIDE) * k


def _col_th(j, z):
    """Column j (0 = centre .. MASK_NH = side edge, sign = side) at height z.  The inner columns are
    straight and parallel to the slats (centre, half way, slat line); the outer ones fan out evenly
    to the side edge, so they run parallel to it."""
    sg = 1 if j >= 0 else -1
    j = abs(j)
    if j <= 2:
        return _th_of_x(sg * _slat_x(z) * j / 2.0, z)
    t0, t1 = _th_of_x(_slat_x(z), z), _edge_th(z)
    f = (j - 2) / (MASK_NH - 2)
    x0, x1 = _slat_x(z), mask_pt(t1, z).x
    tx = _th_of_x(x0 + (x1 - x0) * f, z)           # even in front view ...
    return sg * (0.5 * tx + 0.5 * (t0 + (t1 - t0) * f))   # ... blended with even on the surface


def _v_edge_z(x):
    """Bottom (V) edge height at lateral position x: straight V that bends up into the sides."""
    xe = abs(mask_pt(MASK_TH_SIDE, MASK_Z_SIDE).x)
    s = min(abs(x) / xe, 1.0)
    return MASK_Z_TIP + (MASK_Z_SIDE - MASK_Z_TIP) * (0.75 * s + 0.25 * s ** 3)


def _col_bottom(j):
    if abs(j) == MASK_NH:
        return MASK_Z_SIDE
    z = MASK_Z_SIDE
    for _ in range(6):
        z = _v_edge_z(mask_pt(_col_th(j, z), z).x)
    return z


def build_mask():
    NR, NH = MASK_NR, MASK_NH
    cols = list(range(-NH, NH + 1))
    NC = len(cols)
    verts, par, faces = [], [], []
    G = [[None] * NC for _ in range(NR + 1)]
    for c, j in enumerate(cols):
        zb = _col_bottom(j)
        for r in range(NR + 1):
            z = MASK_Z_TOP + (zb - MASK_Z_TOP) * r / NR
            th = _col_th(j, z)
            verts.append(mask_pt(th, z))
            par.append((th, z))
            G[r][c] = len(verts) - 1
    for r in range(NR):
        for c in range(NC - 1):
            faces.append((G[r][c], G[r][c + 1], G[r + 1][c + 1], G[r + 1][c]))
    # rolled rim round the outline: crest, rounded outer edge, side wall back toward the face
    loop = ([G[0][c] for c in range(NC)] + [G[r][NC - 1] for r in range(1, NR + 1)]
            + [G[NR][c] for c in range(NC - 2, -1, -1)] + [G[r][0] for r in range(NR - 1, 0, -1)])
    m = len(loop)
    cen = sum((verts[i] for i in loop), V()) / m
    rims = [[], [], []]
    for k in range(m):
        i, ip, inx = loop[k], loop[k - 1], loop[(k + 1) % m]
        p = verts[i]
        nrm = mask_normal(*par[i])
        perp = []
        for e in (p - verts[ip], verts[inx] - p):
            q = e.cross(nrm).normalized()
            perp.append(q if q.dot(p - cen) > 0 else -q)
        mit = (perp[0] + perp[1]).normalized()
        sc = 1.0 / max(mit.dot(perp[0]), 0.35)
        for ring, (d, h) in zip(rims, ((RIM[0], RIM[1]), (RIM[2], RIM[3]), (RIM[2], -MASK_T))):
            verts.append(p + mit * (d * sc) + nrm * h)
            ring.append(len(verts) - 1)
    for a, b in zip([loop] + rims[:-1], rims):
        faces += _bridge(a, b)
    return lib.mesh_object("mask", verts, faces, topo="rigid", sharp_angle=35)


def _slat(name, x0, z0, z1, nseg):
    """Raised slat on the plate along the line x = x0 * _slat_x(z) / SLAT_X[0], from z0 down to z1:
    flat top, chamfered top edges, sloped ends, closed underside sunk into the plate."""
    sx = x0 / SLAT_X[0]
    path = []
    for k in range(nseg + 1):
        z = z0 + (z1 - z0) * k / nseg
        th = _th_of_x(sx * _slat_x(z), z)
        path.append((mask_pt(th, z), mask_normal(th, z)))
    hw, h, ch = SLAT_HW, SLAT_H, SLAT_CH
    prof = [(-hw, -SLAT_SINK), (-hw, h - ch), (-hw + ch, h), (hw - ch, h), (hw, h - ch), (hw, -SLAT_SINK)]
    rings = []
    for k, (p, nrm) in enumerate(path):
        tg = (path[min(k + 1, nseg)][0] - path[max(k - 1, 0)][0]).normalized()
        bx = tg.cross(nrm).normalized()
        nz = bx.cross(tg).normalized()
        if nz.dot(nrm) < 0:
            nz = -nz
        rings.append([p + bx * u + nz * w for u, w in prof])
        if k in (0, nseg):                       # sloped end: lower, narrower ring 3 mm further on
            ext = -tg if k == 0 else tg
            q = p + ext * 0.003
            end = [q + bx * (u * 0.8) + nz * (w * 0.35 if w > 0 else w) for u, w in prof]
            if k == 0:
                rings.insert(0, end)
            else:
                rings.append(end)
    verts, faces = [], []
    idx = [_add_ring(verts, r) for r in rings]
    for a, b in zip(idx, idx[1:]):
        faces += _bridge(a, b)
    for r in (idx[0], idx[-1]):                  # hexagonal end caps -> 2 quads each
        faces += [(r[0], r[1], r[4], r[5]), (r[1], r[2], r[3], r[4])]
    return lib.mesh_object(name, verts, faces, topo="rigid", sharp_angle=30)


def build_slats():
    z_top = MASK_Z_TOP - 0.006
    side_bot = _v_edge_z(SLAT_X[1]) + 0.011
    slats = [_slat("slat_c", 0.0, z_top, MASK_Z_TIP + 0.017, 6)]
    for sg in (1, -1):
        slats.append(_slat(f"slat_{sg}", sg * SLAT_X[0], z_top - 0.003, side_bot, 5))
    return lib.join(slats, "mask_slats", topo="rigid")


# ------------------------------------------------------------------ entry point
def build():
    objs = [build_head(), build_hat(), build_band()]
    objs += build_band_details()
    objs += [build_mask(), build_slats()]
    return objs
