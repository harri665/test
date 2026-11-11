"""Legs: baggy trousers bloused into chunky combat boots (soles, straps, buckles).

Objects (build() order)
    trousers                 main     ONE connected all-quad mesh: pelvis (32) -> crotch gusset -> 2 leg tubes (20)
    trouser_knee_L / _R      support  3 tight knee loops as a thin reinforcement band lying on the trouser leg
    boot_upper_L / _R        main     shaft + cuff + foot, all quads (foot is extruded from a front patch of the
                                      shaft: 4 five-poles, no fan, no intersecting tubes)
    boot_sole_L / _R         rigid    welt + outsole slab with raised heel block / arch notch
    boot_straps_L / _R       main     shaft strap (z~0.24) + instep strap, with tails, real thickness
    boot_buckles_L / _R      rigid    frame buckles + prongs + keeper loops on the OUTER side

Everything is derived from spec (leg_point / leg_radius / HEEL / TOE / CLOTH).  Left side is built, the
right side is a mirror copy (the trousers are one mesh, built for both sides with slightly different folds).
"""
import math

import bpy  # noqa: F401
from mathutils import Vector as V
from mathutils.bvhtree import BVHTree

from .. import lib, spec as S

TAU = math.tau
C = S.CLOTH

# ------------------------------------------------------------------ small helpers


def _sstep(x):
    x = min(1.0, max(0.0, x))
    return x * x * (3 - 2 * x)


def _lerp(a, b, t):
    return a + (b - a) * t


def _interp(table, x):
    """Piecewise linear lookup in [(x, y), ...] sorted by ascending x (clamped)."""
    if x <= table[0][0]:
        return table[0][1]
    for (x0, y0), (x1, y1) in zip(table, table[1:]):
        if x <= x1:
            return _lerp(y0, y1, (x - x0) / (x1 - x0))
    return table[-1][1]


def _catmull(rows, u):
    """Catmull-Rom through rows (tuples of floats) at parameter u = row index (float)."""
    n = len(rows)
    u = min(max(u, 0.0), n - 1.0)
    i = min(int(u), n - 2)
    t = u - i
    p0, p1, p2, p3 = rows[max(i - 1, 0)], rows[i], rows[i + 1], rows[min(i + 2, n - 1)]
    t2, t3 = t * t, t * t * t
    return tuple(0.5 * (2 * b + (c - a) * t + (2 * a - 5 * b + 4 * c - d) * t2 + (-a + 3 * b - 3 * c + d) * t3)
                 for a, b, c, d in zip(p0, p1, p2, p3))


class MB:
    """Tiny mesh builder: explicit vertex indices, quad strips, Coons patches."""

    def __init__(self):
        self.v, self.f = [], []

    def add(self, pts):
        i0 = len(self.v)
        self.v += [V(p) for p in pts]
        return list(range(i0, i0 + len(pts)))

    def quad(self, a, b, c, d):
        self.f.append((a, b, c, d))

    def strip(self, r0, r1, closed=False):
        n = len(r0)
        for k in range(n if closed else n - 1):
            k2 = (k + 1) % n
            self.f.append((r0[k], r0[k2], r1[k2], r1[k]))

    def coons(self, top, right, bottom, left, bulge=None):
        """All-quad fill of a 4-sided boundary.  top/bottom run left->right, left/right run top->bottom."""
        nc, nr = len(top) - 1, len(left) - 1
        P = self.v
        G = [[None] * (nc + 1) for _ in range(nr + 1)]
        G[0], G[nr] = list(top), list(bottom)
        for r in range(nr + 1):
            G[r][0], G[r][nc] = left[r], right[r]
        for r in range(1, nr):
            for c in range(1, nc):
                u, v = c / nc, r / nr
                p = ((1 - v) * P[top[c]] + v * P[bottom[c]] + (1 - u) * P[left[r]] + u * P[right[r]]
                     - ((1 - u) * (1 - v) * P[top[0]] + u * (1 - v) * P[top[nc]]
                        + u * v * P[bottom[nc]] + (1 - u) * v * P[bottom[0]]))
                if bulge:
                    p = p + bulge(u, v)
                G[r][c] = self.add([p])[0]
        for r in range(nr):
            for c in range(nc):
                self.f.append((G[r][c], G[r][c + 1], G[r + 1][c + 1], G[r + 1][c]))
        return G

    def obj(self, name, topo, **kw):
        return lib.mesh_object(name, self.v, self.f, topo=topo, **kw)


# ------------------------------------------------------------------ leg guide
def leg_t(z):
    if z >= S.KNEE.z:
        return (S.HIP.z - z) / (S.HIP.z - S.KNEE.z)
    return 1.0 + (S.KNEE.z - z) / (S.KNEE.z - S.ANKLE.z)


def leg_c(z):
    p = S.leg_point(min(max(leg_t(z), 0.0), 2.0))
    return p.x, p.y


# ================================================================== TROUSERS
NP, NL = 32, 20                 # pelvis ring / leg ring vertex counts
GAP = 0.016                     # inner thigh never gets closer to the mirror plane than this
Z_SPLIT = 0.875                 # last full pelvis ring; the crotch gusset hangs below it
BLOUSE_Z0, BLOUSE_Z1, BLOUSE_R = C["trouser_blouse"]
BOOT_TOP = C["boot_top_z"]
PELVIS_Z = [1.06, 1.03, 1.00, 0.97, 0.94, 0.908, Z_SPLIT]


def _thigh_r(z):
    """Trouser radius above the blouse: bare leg + cloth (snug on the hip, ~0.02 baggy further down)."""
    t = max(leg_t(z), 0.0)
    return S.leg_radius(t) + 0.010 + 0.010 * _sstep(t / 0.45)


# (z, radius) rows of each trouser leg below the split ring.  Even ~3 cm loops, 3 tight loops on the knee,
# then the blouse: puffs out to BLOUSE_R, rolls under and tucks inside the boot cuff.
_Z_TH = [0.843, 0.811, 0.779, 0.748, 0.717, 0.686, 0.655, 0.624, 0.593, 0.562]
_R562 = _thigh_r(0.562)
LEG_ROWS = [(z, _thigh_r(z)) for z in _Z_TH] + [
    (0.535, _lerp(_R562, 0.092, 0.45)),
    (S.KNEE.z + 0.012, 0.0905), (S.KNEE.z, 0.0920), (S.KNEE.z - 0.012, 0.0935),      # knee support loops
    (0.462, 0.0965), (0.434, 0.0990), (0.406, BLOUSE_R - 0.005), (0.380, BLOUSE_R - 0.0055),
    (0.359, 0.0960), (0.346, 0.0880), (0.341, 0.0780),                               # roll
    (0.3425, 0.0680), (0.3385, 0.0610), (0.326, 0.0590), (0.308, 0.0585),             # tuck into the boot
]
_DIP = {0: 0.030, 1: 0.018, 2: 0.008}          # leg rings just under the crotch tilt down on the inside


def _fold(th, z, r, side):
    """Radial cloth folds (metres).  Different phase per side so the two legs are not mirror images."""
    wb = _sstep((0.58 - z) / 0.10)                   # blouse weight
    wt = _sstep((r - 0.070) / 0.020) if z < 0.40 else 1.0   # none on the tuck
    blouse = (0.0040 * math.sin(2 * th + 0.9 + 0.7 * side + 14.0 * z)
              + 0.0026 * math.sin(5 * th + 1.1 * side - 34.0 * z)
              + 0.0022 * math.sin(TAU * (z - 0.36) / 0.085 + 1.5 * math.sin(th + side)))
    thigh = 0.0022 * math.sin(3 * th + 21.0 * z + side)
    return wt * (wb * blouse + (1 - wb) * thigh)


def trouser_pt(z, r, th, side=1, dr=0.0, dip=0.0):
    """Point on the LEFT trouser leg at height z / base radius r / angle th (0 = outer +X, 90 = back)."""
    cx, cy = leg_c(z)
    wb = _sstep((0.56 - z) / 0.12) * (_sstep((r - 0.070) / 0.025) if z < 0.40 else 1.0)
    cx += 0.004 * wb                                  # the blouse hangs slightly to the outside ...
    cy += 0.003 * wb                                  # ... and to the back
    x_out, x_in = cx + r, max(cx - r, GAP)
    ex, rx = 0.5 * (x_out + x_in), 0.5 * (x_out - x_in)
    k = 1.0 + (_fold(th, z, r, side) + dr) / r
    x = max(ex + rx * math.cos(th) * k, GAP - 0.004)
    y = cy + r * math.sin(th) * k
    # roll droops unevenly over the boot top
    droop = 0.006 * (0.5 + 0.5 * math.sin(th + 0.6 + 1.5 * side)) if (z < 0.36 and r > 0.074) else 0.0
    zz = z - dip * ((1 - math.cos(th)) * 0.5) ** 1.5 - droop
    return V((x, y, zz))


def _split_xy(i):
    """Split ring (pelvis ring at Z_SPLIT): superellipse hugging both thighs.  -> (x, y, z_drop)"""
    a = TAU * i / NP
    c, s = math.cos(a), math.sin(a)
    cx, _ = leg_c(Z_SPLIT)
    rx = cx + _thigh_r(Z_SPLIT)
    ry = 0.116
    e = 2.0 / 2.4
    return (rx * math.copysign(abs(c) ** e, c), 0.001 + ry * math.copysign(abs(s) ** e, s),
            0.015 * abs(s) ** 6)


def _pelvis_pt(i, z):
    cy, rx, ry = S.torso_section(z)
    rx, ry = rx + 0.012, ry + 0.012
    a = TAU * i / NP
    pu = V((rx * math.cos(a), cy + ry * math.sin(a), z))
    sx, sy, dz = _split_xy(i)
    s = _sstep((0.985 - z) / (0.985 - Z_SPLIT))
    return pu.lerp(V((sx, sy, z - dz)), s)


def build_trousers():
    mb = MB()
    # ---- pelvis rings
    prs = [mb.add([_pelvis_pt(i, z) for i in range(NP)]) for z in PELVIS_Z]
    for a, b in zip(prs, prs[1:]):
        mb.strip(a, b, closed=True)
    split = prs[-1]
    # ---- crotch gusset: 2 quads wide, 6 long, from back centre (8) under the crotch to front centre (24)
    yb = 0.116
    ys = [yb * f for f in (0.66, 0.34, 0.0, -0.34, -0.66)]

    def saddle_z(y):
        return Z_SPLIT - 0.015 - 0.027 * math.sqrt(max(0.0, 1 - (y / yb) ** 2))
    cen = mb.add([V((0.0, y, saddle_z(y))) for y in ys])
    lft = mb.add([V((GAP, y, saddle_z(y) - 0.005)) for y in ys])
    rgt = mb.add([V((-GAP, y, saddle_z(y) - 0.005)) for y in ys])
    line_c = [split[8]] + cen + [split[24]]
    line_l = [split[7]] + lft + [split[25]]
    line_r = [split[9]] + rgt + [split[23]]
    mb.strip(line_l, line_c)
    mb.strip(line_c, line_r)
    # ---- legs.  leg ring index j: 0 = outer, 5 = back, 10 = inner, 15 = front
    for side, bridge in ((1, lft), (-1, rgt)):
        def pel(i):
            return split[i % NP] if side > 0 else split[(16 - i) % NP]
        ring = [pel(j) for j in range(0, 8)] + bridge + [pel(j) for j in range(25, 32)]
        for k, (z, r) in enumerate(LEG_ROWS):
            pts = []
            for j in range(NL):
                p = trouser_pt(z, r, TAU * j / NL, side, dip=_DIP.get(k, 0.0))
                pts.append(V((p.x * side, p.y, p.z)))
            new = mb.add(pts)
            mb.strip(ring, new, closed=True)
            ring = new
    return mb.obj("trousers", "main", sharp_angle=60)


def build_knee_band(side=1):
    """3 tight support loops on the knee as a thin band lying 2 mm outside the trouser leg (edges tucked in)."""
    zk = S.KNEE.z
    table = sorted((z, r) for z, r in LEG_ROWS if z > 0.40)
    rows = [(zk + 0.0275, -0.0015), (zk + 0.024, 0.002), (zk + 0.012, 0.002), (zk, 0.002),
            (zk - 0.012, 0.002), (zk - 0.024, 0.002), (zk - 0.0275, -0.0015)]
    rings = []
    for z, dr in rows:
        r = _interp(table, z)
        pts = [trouser_pt(z, r, TAU * j / NL, side, dr=dr) for j in range(NL)]
        rings.append([V((p.x * side, p.y, p.z)) for p in pts])
    return lib.lofted("trouser_knee_" + ("L" if side > 0 else "R"), rings, topo="support", sharp_angle=60)


# ================================================================== BOOT (left)
SOLE_T = 0.036                   # top of the sole slab
ZB = SOLE_T - 0.004              # bottom of the upper (sunk into the sole)
NS, PW, PR = 20, 6, 4            # shaft columns / foot patch width / foot patch rows
JP0 = 3 * NS // 4 - PW // 2      # patch columns JP0 .. JP0+PW, centred on the front (-Y)
JP1 = JP0 + PW
NF = 2 * (PW + PR)               # foot ring vertex count
CUFF_R = None                    # set below


def foot_cx(y):
    """Sole centre line: straight from HEEL to TOE (boots point forward, tiny toe-out as in spec)."""
    return S.HEEL.x + (S.HEEL.y - y) / (S.HEEL.y - S.TOE.y) * (S.TOE.x - S.HEEL.x)


def boot_c(z):
    return leg_c(max(z, S.ANKLE.z))


def _shaft_r(z):
    return S.leg_radius(leg_t(z)) + 0.011


_RS = [(0.032, 0.050), (0.053, 0.052), (0.075, 0.0535), (0.097, 0.0545), (0.120, 0.0548), (0.150, _shaft_r(0.150)),
       (0.200, _shaft_r(0.200)), (0.335, _shaft_r(0.335))]
_RB = [(0.032, 0.0730), (0.053, 0.0735), (0.075, 0.0715), (0.097, 0.0675), (0.120, 0.0625), (0.150, 0.0585),
       (0.200, _shaft_r(0.200)), (0.335, _shaft_r(0.335))]
_RF = [(0.032, 0.062), (0.150, 0.062), (0.175, 0.0595), (0.200, _shaft_r(0.200)), (0.335, _shaft_r(0.335))]
_PF = [(0.032, 4.5), (0.097, 3.4), (0.120, 2.6), (0.150, 2.0)]
CUFF_R = _shaft_r(0.325)

# full shaft rings: (z at the back, how much the front arches up over the instep crease)
SHAFT_ROWS = [(0.312, 0.0), (0.293, 0.0), (0.274, 0.0), (0.255, 0.0), (0.236, 0.0), (0.217, 0.0), (0.198, 0.0),
              (0.179, 0.004), (0.160, 0.010), (0.142, 0.018), (0.126, 0.027), (0.112, 0.038)]
HEEL_Z = [0.092, 0.072, 0.052, ZB]          # PR rows below the last full ring (heel counter)
# cuff profile on top of the shaft: (radial offset from CUFF_R, z)
CUFF = [(0.0045, 0.3105), (0.0056, 0.323), (0.0050, 0.3325), (0.0018, 0.3372), (-0.0028, 0.3365),
        (-0.0048, 0.330), (-0.0048, 0.313)]


def shaft_pt(zb, arch, j):
    psi = TAU * j / NS
    c, s = math.cos(psi), math.sin(psi)
    d = abs(math.degrees(psi) - 270.0)
    f = math.cos(math.radians(90.0 * d / 80.0)) ** 2 if d < 80.0 else 0.0
    z = zb + arch * f
    cx, cy = boot_c(z)
    rs = _interp(_RS, z)
    if s >= 0:
        return V((cx + rs * c, cy + _interp(_RB, z) * s, z))
    e = 2.0 / _interp(_PF, z)
    return V((cx + rs * math.copysign(abs(c) ** e, c), cy - _interp(_RF, z) * abs(s) ** e, z))


# foot cross-section keys, from the ankle patch (0) to the toe (8):
#   y of the sole line, (y, z) of the top centre line, max half width, half width at the sole, corner angle
_cy0 = S.ANKLE.y
FOOT = [
    (_cy0 - 0.0475, _cy0 - 0.062, 0.150, 0.0590, 0.0450, 44.0),
    (-0.059, -0.074, 0.1375, 0.0565, 0.0480, 46.0),
    (-0.076, -0.091, 0.1270, 0.0550, 0.0500, 48.0),
    (-0.094, -0.108, 0.1180, 0.0545, 0.0505, 50.0),
    (-0.112, -0.125, 0.1110, 0.0545, 0.0505, 52.0),
    (-0.130, -0.142, 0.1065, 0.0540, 0.0500, 53.0),
    (-0.148, -0.158, 0.1030, 0.0525, 0.0485, 54.0),
    (-0.165, -0.172, 0.0980, 0.0495, 0.0455, 55.0),
    (-0.180, -0.184, 0.0890, 0.0440, 0.0405, 56.0),
]
TOE_STEP = 0.0028                 # toe cap leather sits this far proud -> seam line
FOOT_US = [(1, 0), (2, 0), (3, 0), (4, 0), (4.84, 0), (4.96, 1), (6, 1), (7, 1), (8, 1)]
TOE_BULGE = 0.0165


def dome_ring(u, step=0):
    yb, yt, zt, W, Wb, phc = _catmull(FOOT, u)
    phc = math.radians(phc)
    if step:
        W += TOE_STEP
        Wb += TOE_STEP * 0.6
        zt += TOE_STEP
    e = 2.0 / 2.6
    cx = foot_cx(yb)

    def pt(phi):
        c, sn = math.cos(phi), math.sin(phi)
        h = abs(sn) ** e
        xl = math.copysign(abs(c) ** e * W - (W - Wb) * (1 - h) ** 3, c)
        return V((cx + xl, yb + (yt - yb) * h, ZB + (zt - ZB) * h))
    pts = [pt(math.pi - phc - (math.pi - 2 * phc) * q / PW) for q in range(PW + 1)]       # top, inner -> outer
    pts += [pt(phc * (1 - k / PR)) for k in range(1, PR + 1)]                             # outer side, down
    pts += [V((cx + Wb * (1 - 2 * k / PW), yb, ZB)) for k in range(1, PW + 1)]            # sole line, outer -> inner
    pts += [pt(math.pi - phc * k / PR) for k in range(1, PR)]                             # inner side, up
    return pts


def build_upper():
    mb = MB()
    idx = {}
    n_full = len(SHAFT_ROWS)
    itop = n_full - 1
    # cuff (top -> down the outside), then shaft
    cuff = []
    for dr, z in reversed(CUFF):
        cx, cy = boot_c(z)
        cuff.append(mb.add(lib.ring_z(z, CUFF_R + dr, n=NS, cx=cx, cy=cy)))
    for a, b in zip(cuff, cuff[1:]):
        mb.strip(a, b, closed=True)
    prev = cuff[-1]
    for i, (zb, arch) in enumerate(SHAFT_ROWS):
        ring = mb.add([shaft_pt(zb, arch, j) for j in range(NS)])
        for j in range(NS):
            idx[(i, j)] = ring[j]
        mb.strip(prev, ring, closed=True)
        prev = ring
    # heel counter rows: every column except the inside of the foot patch
    heel_cols = [j % NS for j in range(JP1, JP0 + NS + 1)]          # JP1 .. around the back .. JP0
    nr = itop + PR
    for k, z in enumerate(HEEL_Z):
        i = itop + 1 + k
        ids = mb.add([shaft_pt(z, 0.0, j) for j in heel_cols])
        for j, v in zip(heel_cols, ids):
            idx[(i, j)] = v
    for i in range(itop, nr):
        for a, b in zip(heel_cols, heel_cols[1:]):
            mb.quad(idx[(i, a)], idx[(i, b)], idx[(i + 1, b)], idx[(i + 1, a)])
    # straight front edge of the bottom row (sole line of the patch)
    pa, pb = mb.v[idx[(nr, JP0)]], mb.v[idx[(nr, JP1)]]
    for j in range(JP0 + 1, JP1):
        idx[(nr, j)] = mb.add([pa.lerp(pb, (j - JP0) / PW)])[0]
    # ---- foot: ring 0 = boundary of the patch, then dome rings to the toe
    ring0 = ([idx[(itop, j)] for j in range(JP0, JP1 + 1)]
             + [idx[(i, JP1)] for i in range(itop + 1, nr + 1)]
             + [idx[(nr, j)] for j in range(JP1 - 1, JP0 - 1, -1)]
             + [idx[(i, JP0)] for i in range(nr - 1, itop, -1)])
    assert len(ring0) == NF
    d0 = dome_ring(0.0)
    corr = [mb.v[a] - b for a, b in zip(ring0, d0)]
    prev = ring0
    for u, step in FOOT_US:
        fall = max(0.0, 1.0 - u / 3.0) ** 2
        ring = mb.add([p + c * fall for p, c in zip(dome_ring(u, step), corr)])
        mb.strip(prev, ring, closed=True)
        prev = ring
    # toe cap
    top = prev[0:PW + 1]
    right = prev[PW:PW + PR + 1]
    bottom = list(reversed(prev[PW + PR:2 * PW + PR + 1]))
    left = [prev[0]] + list(reversed(prev[2 * PW + PR:NF]))

    def bulge(u, v):
        return V((0.0, -TOE_BULGE * (1 - (2 * u - 1) ** 2) ** 0.55 * (1 - (2 * v - 0.85) ** 2 / 1.33) ** 0.6, 0.0))
    mb.coons(top, right, bottom, left, bulge)
    # heel bottom (hidden inside the sole, keeps the upper closed)
    def col(j):
        return idx[(nr, j % NS)]
    mb.coons([col(j) for j in range(JP0, JP1 + 1)],
             [col(JP1 + k) for k in range(0, PR + 1)],
             [col(JP0 - PR - k) for k in range(0, PW + 1)],
             [col(JP0 - k) for k in range(0, PR + 1)])
    assert (JP0 - PR - PW) % NS == (JP1 + PR) % NS
    return mb


# ------------------------------------------------------------------ sole
def _sole_w(y):
    return _interp([(-0.150, 0.0625), (-0.110, 0.0635), (-0.070, 0.0605), (-0.030, 0.0575), (0.0, 0.0570),
                    (0.035, 0.0575)], y)


def build_sole():
    mb = MB()
    T, ch, ch2, z1, z2 = SOLE_T, 0.003, 0.002, 0.0195, 0.0225
    notch = 0.012
    e = 2.0 / 2.4
    yh, rh = 0.035, S.HEEL.y - 0.035
    yt, rt = -0.150, -0.150 - S.TOE.y
    st = []                                           # (y, half width, bottom z)
    for a in (28, 50, 72):
        a = math.radians(a)
        st.append((yh + rh * math.cos(a) ** e, _sole_w(yh) * math.sin(a) ** e, 0.0))
    st += [(yh, _sole_w(yh), 0.0), (0.022, _sole_w(0.022), 0.0), (0.0115, _sole_w(0.0115), 0.0),
           (0.0095, _sole_w(0.0095), notch), (-0.008, _sole_w(-0.008), notch), (-0.026, _sole_w(-0.026), notch),
           (-0.040, _sole_w(-0.040), 0.0)]
    st += [(y, _sole_w(y), 0.0) for y in (-0.058, -0.078, -0.098, -0.118, -0.136, yt)]
    for a in (72, 50, 28):
        a = math.radians(a)
        st.append((yt - rt * math.cos(a) ** e, _sole_w(yt) * math.sin(a) ** e, 0.0))
    rings = []
    for y, w, zb in st:
        cx = foot_cx(y)
        wm = w - 0.0025
        pts = [(-(w - ch) + 2 * (w - ch) * k / 4, zb) for k in range(5)]
        pts += [(w, zb + ch), (w, z1), (wm, z2), (wm, T - ch2)]
        pts += [((wm - ch2) - 2 * (wm - ch2) * k / 4, T) for k in range(5)]
        pts += [(-wm, T - ch2), (-wm, z2), (-w, z1), (-w, zb + ch)]
        rings.append(mb.add([V((cx + x, y, z)) for x, z in pts]))
    for a, b in zip(rings, rings[1:]):
        mb.strip(a, b, closed=True)

    def cap(r, sign, depth):
        mb.coons([r[13], r[12], r[11], r[10], r[9]], [r[9], r[8], r[7], r[6], r[5], r[4]],
                 [r[0], r[1], r[2], r[3], r[4]], [r[13], r[14], r[15], r[16], r[17], r[0]],
                 lambda u, v: V((0.0, sign * depth * (1 - (2 * u - 1) ** 2) ** 0.6, 0.0)))
    cap(rings[0], 1.0, S.HEEL.y - st[0][0])
    cap(rings[-1], -1.0, st[-1][0] - S.TOE.y)
    return mb


# ------------------------------------------------------------------ straps + buckles
class Sampler:
    """Ray-cast frames on the boot upper, in a plane through `c` spanned by x / d (b = across the strap)."""

    def __init__(self, bvh, c, x, d):
        self.bvh, self.c, self.x, self.d = bvh, V(c), V(x).normalized(), V(d).normalized()
        self.b = self.x.cross(self.d).normalized()

    def hit(self, a, b=0.0):
        u = self.x * math.cos(a) + self.d * math.sin(a)
        o = self.c + self.b * b
        loc = self.bvh.ray_cast(o + u * 0.5, -u)[0]
        return loc if loc is not None else o + u * 0.05

    def frame(self, a, w):
        """-> ([p(-w), p(0), p(+w)], normal, tangent)"""
        u = self.x * math.cos(a) + self.d * math.sin(a)
        t = (self.hit(a + 0.06) - self.hit(a - 0.06)).normalized()
        n = t.cross(self.b)
        if n.dot(u) < 0:
            n = -n
        n = (n.normalized() + u).normalized()        # soften at tight corners
        return [self.hit(a, -w), self.hit(a, 0.0), self.hit(a, w)], n, t


def band(mb, frames, t_in, t_out, closed, edge=0.0015):
    """Strap with thickness from surface frames.  6-vertex section, all quads."""
    secs = []
    for (pm, p0, pp), n, _t in frames:
        wdir = (pp - pm).normalized()
        secs.append(mb.add([pm + n * t_in, p0 + n * t_in, pp + n * t_in,
                            pp + n * t_out - wdir * edge, p0 + n * t_out, pm + n * t_out + wdir * edge]))
    for a, b in zip(secs, secs[1:] + ([secs[0]] if closed else [])):
        mb.strip(a, b, closed=True)
    if not closed:
        for s in (secs[0], secs[-1]):
            mb.quad(s[0], s[1], s[4], s[5])
            mb.quad(s[1], s[2], s[3], s[4])


def buckle(mb, o, t, b, n, lt, lb, bar=0.0048, h=0.0048):
    """Rectangular frame buckle + prong.  o = centre on the strap's outer face; t along / b across / n out."""
    def P(tt, bb, nn):
        return o + t * tt + b * bb + n * nn

    def path(lt_, lb_, c):
        return [(lt_ - c, -lb_), (lt_, -lb_ + c), (lt_, 0.0), (lt_, lb_ - c), (lt_ - c, lb_), (0.0, lb_),
                (-lt_ + c, lb_), (-lt_, lb_ - c), (-lt_, 0.0), (-lt_, -lb_ + c), (-lt_ + c, -lb_), (0.0, -lb_)]
    outer, inner = path(lt, lb, 0.004), path(lt - bar, lb - bar, 0.002)
    n0, n1, ch = -0.0006, h - 0.0006, 0.0012
    secs = []
    for (to, bo), (ti, bi) in zip(outer, inner):
        k = ch / bar
        secs.append(mb.add([P(ti, bi, n0), P(to, bo, n0), P(to, bo, n1 - ch),
                            P(_lerp(to, ti, k), _lerp(bo, bi, k), n1), P(ti, bi, n1)]))
    for a, c in zip(secs, secs[1:] + [secs[0]]):
        mb.strip(a, c, closed=True)
    # prong: from the near bar across the window, resting on the far bar
    w2, z0, z1 = 0.0017, n1 - 0.0012, n1 + 0.0016
    a = mb.add([P(-lt + bar * 0.2, -w2, z0), P(-lt + bar * 0.2, w2, z0), P(-lt + bar * 0.2, w2, z1),
                P(-lt + bar * 0.2, -w2, z1)])
    m = mb.add([P(0.0, -w2, z0 - 0.0008), P(0.0, w2, z0 - 0.0008), P(0.0, w2, z1 - 0.0008), P(0.0, -w2, z1 - 0.0008)])
    c = mb.add([P(lt + 0.0015, -w2 * 0.7, z0 + 0.0004), P(lt + 0.0015, w2 * 0.7, z0 + 0.0004),
                P(lt + 0.0015, w2 * 0.7, z1 - 0.0004), P(lt + 0.0015, -w2 * 0.7, z1 - 0.0004)])
    mb.strip(a, m, closed=True)
    mb.strip(m, c, closed=True)
    mb.quad(*a)
    mb.quad(*c)


def keeper(mb, o, t, b, n, wb, top, depth=0.006, lt=0.0042, th=0.0022):
    """Strap keeper loop: a U band over the strap, legs sunk into the boot."""
    c = 0.002
    outer = [(-wb, -depth), (-wb, top - c), (-wb + c, top), (wb - c, top), (wb, top - c), (wb, -depth)]
    inner = [(-wb + th, -depth), (-wb + th, top - c - th * 0.4), (-wb + c + th * 0.4, top - th),
             (wb - c - th * 0.4, top - th), (wb - th, top - c - th * 0.4), (wb - th, -depth)]
    secs = []
    for (bo, no), (bi, ni) in zip(outer, inner):
        secs.append(mb.add([o + t * -lt + b * bi + n * ni, o + t * -lt + b * bo + n * no,
                            o + t * lt + b * bo + n * no, o + t * lt + b * bi + n * ni]))
    for a, c2 in zip(secs, secs[1:]):
        mb.strip(a, c2, closed=True)
    mb.quad(*secs[0])
    mb.quad(*secs[-1])


def strap_set(ms, mr, smp, angles, closed, w, a_buckle, tail_dir, ext=None, t_in=0.0012, t_out=0.0052):
    """One strap (+ tail, buckle, keeper) on the sampled surface."""
    frames = [smp.frame(a, w) for a in angles]
    if ext is not None:                                   # run the open ends down into the sole
        for end in (0, -1):
            pts, n, t = frames[end]
            f = ([p + ext for p in pts], n, t)
            frames = [f] + frames if end == 0 else frames + [f]
    band(ms, frames, t_in, t_out, closed)
    # local scale: metres per radian around the buckle
    per_rad = max((smp.hit(a_buckle + 0.05) - smp.hit(a_buckle - 0.05)).length / 0.1, 0.02)
    # tail: the free end lying on top of the strap, tapering to a blunt tip
    tail_len, n_tail = 0.058, 7
    tl = []
    for k in range(n_tail + 1):
        a = a_buckle + tail_dir * (tail_len * k / n_tail - 0.004) / per_rad
        ww = w * 0.90 * (1.0 if k < n_tail else 0.55) * (1.0 if k < n_tail - 1 else 0.9)
        pts, n, t = smp.frame(a, ww)
        lift = t_out + 0.0002 + (0.0010 if k == 0 else 0.0)
        tl.append(([p + n * lift for p in pts], n, t))
    band(ms, tl, 0.0, 0.0028, False, edge=0.0012)
    # buckle frame
    pts, n, t = smp.frame(a_buckle, w)
    if t.dot(smp.hit(a_buckle + tail_dir * 0.05) - smp.hit(a_buckle)) < 0:
        t = -t
    b = n.cross(t).normalized()
    buckle(mr, pts[1] + n * t_out, t, b, n, lt=0.0165, lb=w + 0.0075)
    # keeper
    ak = a_buckle + tail_dir * 0.036 / per_rad
    pts, n, t = smp.frame(ak, w)
    b = n.cross(t).normalized()
    keeper(mr, pts[1] + n * t_out, t, b, n, wb=w + 0.0028, top=0.0052)


def build_straps(upper):
    bvh = BVHTree.FromPolygons([tuple(v) for v in upper.v], [tuple(f) for f in upper.f])
    ms, mr = MB(), MB()
    # shaft strap: horizontal loop
    zs = 0.240
    cx, cy = boot_c(zs)
    smp = Sampler(bvh, (cx, cy, zs), (1, 0, 0), (0, 1, 0))
    strap_set(ms, mr, smp, [TAU * k / 24 for k in range(24)], True, 0.0155,
              a_buckle=math.radians(-28.0), tail_dir=1.0)
    # instep strap: arch over the ankle / instep in a forward-leaning plane, ends sunk into the sole welt
    bot = V((foot_cx(-0.008), -0.008, SOLE_T))
    top = V((foot_cx(-0.066), -0.068, 0.146))
    d = (top - bot)
    d.x = 0.0
    d.normalize()
    c = bot + d * 0.014
    smp = Sampler(bvh, c, (1, 0, 0), d)
    n_seg = 18
    strap_set(ms, mr, smp, [math.pi * k / n_seg for k in range(n_seg + 1)], False, 0.0135,
              a_buckle=math.radians(47.0), tail_dir=1.0, ext=-d * 0.017)
    return ms, mr


# ================================================================== build
def build():
    objs = [build_trousers(), build_knee_band(1), build_knee_band(-1)]
    upper = build_upper()
    straps, buckles = build_straps(upper)
    left = [upper.obj("boot_upper_L", "main", sharp_angle=50),
            build_sole().obj("boot_sole_L", "rigid", sharp_angle=30),
            straps.obj("boot_straps_L", "main", sharp_angle=35),
            buckles.obj("boot_buckles_L", "rigid", sharp_angle=30)]
    objs += left + [lib.mirror_copy(o) for o in left]
    return objs
