"""Legs (round 2): baggy bloused trousers, chunky splayed ankle boots with lugged soles, straps, buckles.

Objects (build() order)
    trousers                 main     pelvis (40) -> crotch gusset -> 2 thigh tubes (24) -> all-quad 24 -> 36
                                      transition band (hidden under the skirt), down to the knee loops
    trouser_knee_L / _R      support  the 3 tight knee loops (36 around, one broad soft fold over the front)
    trouser_blouse_L / _R    main     bloused lower leg (36 around): 2 stylised horizontal fold bands that wrap
                                      the leg (tilted, sagging to the front, edge loops along each crest and
                                      crease), a strong rounded bottom roll with a tight crease tucked under it,
                                      slim plain gaiter tucked into the boot
    boot_upper_L / _R        main     cuff + shaft + foot (foot extruded from a front patch of the shaft), toe cap
    boot_sole_L / _R         rigid    lugged outsole + heavy welt, heel block, arch notch
    boot_straps_L / _R       main     tongue + lace bars, ankle/instep strap (diagonal, square buckle),
                                      wide top cuff strap with tail (big square buckle front-outer)
    boot_hardware_L / _R     rigid    2 square frame buckles + prongs, keeper, punched eyelets, rivet

Boots are modelled in a local frame (x = outer side, -y = toe, origin under the ankle on the HEEL->TOE
line) and turned onto the spec's splayed HEEL -> TOE axis.  Right side = mirror copies, except the
blouse, which is built per side with different fold seeds (the reference's folds are not mirrored).
"""
import math

import bpy  # noqa: F401
from mathutils import Vector as V
from mathutils.bvhtree import BVHTree

from .. import lib, spec as S

TAU = math.tau
C = S.CLOTH

# ------------------------------------------------------------------ constants read off the reference sheet
# (docs/ref/front_legs.png, side_legs.png, back_legs.png, measured with tools/compare.py at 1.895 m figure height)
BOOT_DX = 0.008          # boot sits this far outside the spec HEEL->TOE line (ref FRONT: shaft x 0.113-0.22)
SY = 0.020               # heel counter centre this far behind the ankle (ref SIDE: heel back ~0.09 behind it)
SHAFT_EASE = (0.0145, 0.030, 0.0085)   # slim shaft = bare leg + this (side, back, front); ref shaft 0.107 wide,
                                     # ~0.12 deep in SIDE (the reference's shaft is deeper than wide)
SHAFT_DX = 0.008         # shaft axis this far outside the bare leg line (ref FRONT: shaft centre x ~0.167)
SOLE_T = 0.030           # sole + welt thickness (review: reference sole ~3 cm)
ZB = SOLE_T - 0.003      # bottom of the upper, sunk into the sole
BLOUSE_DX = 0.001        # blouse centre vs boot axis (ref FRONT: blouse x 0.064-0.256 at z 0.32 -> centre 0.16)
BLOUSE_DY = 0.030        # ... and hangs back (ref SIDE: blouse centre ~0.05 behind the hat centre line)
CUFF_STRAP_Z = (0.157, 0.207)   # wide strap round the boot top (ref FRONT: strap z 0.15-0.205)


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


# ------------------------------------------------------------------ boot frame (left)
_A = V((S.TOE.x - S.HEEL.x, S.TOE.y - S.HEEL.y, 0.0))
FOOT_LEN = _A.length
_A.normalize()                                    # toe direction (splayed outward)
_P = V((-_A.y, _A.x, 0.0))                        # outward perpendicular (+X-ish for the left boot)
if _P.x < 0:
    _P = -_P
S_ANK = (S.ANKLE - S.HEEL).dot(_A)                # ankle position along the foot
_O = V((S.HEEL.x, S.HEEL.y, 0.0)) + _A * S_ANK + _P * BOOT_DX
Y_HEEL, Y_TOE = S_ANK, S_ANK - FOOT_LEN           # local y of the sole's back / front tip
PHI = math.atan2(_P.y, _P.x)                      # rotation of the local frame (rad)


def to_world(p):
    """Local boot coords (x outer, y back, z up) -> world (left boot)."""
    x, y, z = p
    w = _O + _P * x - _A * y
    return V((w.x, w.y, z))


def to_local(w):
    d = V((w[0], w[1], 0.0)) - V((_O.x, _O.y, 0.0))
    return d.dot(_P), -d.dot(_A)


def shaft_axis(z):
    """Local xy of the boot shaft axis: follows the bare leg line above the ankle (so the leather always
    clears the spec body) and swings onto the heel counter centre (0, SY) at the foot."""
    p = S.leg_point(min(max(leg_t(max(z, S.ANKLE.z)), 0.0), 2.0))
    lx, ly = to_local((p.x + SHAFT_DX, p.y))
    w = _sstep((z - 0.060) / 0.050)
    return _lerp(0.0, lx, w), _lerp(SY, ly, w)


def _bare_r(z):
    return S.leg_radius(min(max(leg_t(max(z, S.ANKLE.z)), 0.0), 2.0))


def shaft_c(z):
    """World xy of the boot shaft axis."""
    sx, sy = shaft_axis(z)
    w = to_world((sx, sy, z))
    return w.x, w.y


# ------------------------------------------------------------------ leg guide
def leg_t(z):
    if z >= S.KNEE.z:
        return (S.HIP.z - z) / (S.HIP.z - S.KNEE.z)
    return 1.0 + (S.KNEE.z - z) / (S.KNEE.z - S.ANKLE.z)


def leg_c(z):
    p = S.leg_point(min(max(leg_t(z), 0.0), 2.0))
    return p.x, p.y


def lower_c(z):
    """Trouser centre line: the leg line above the knee, drifting onto the boot shaft (+ hang) below it."""
    lx, ly = leg_c(z)
    bx, by = shaft_c(C["boot_top_z"])
    s = _sstep((0.53 - z) / 0.25)
    return _lerp(lx, bx + BLOUSE_DX, s), _lerp(ly, by + BLOUSE_DY, s)


# ================================================================== TROUSERS
NP, NL = 40, 24                 # pelvis ring / leg ring vertex counts (leg = NP/2 - 1 pelvis verts + 5 gusset)
NK = 36                         # knee + blouse ring count (~1.4 cm at the knee, ~1.75 cm round the blouse)
SPLIT = set(range(1, NL, 4))    # thigh-ring segments split 1 -> 3 in the 24 -> 36 transition band
assert NL + 2 * len(SPLIT) == NK
GAP = 0.016                     # inner thighs never get closer to the mirror plane than this
CROTCH_Z = S.TORSO[0][0]
Z_SPLIT = CROTCH_Z + 0.015      # last full pelvis ring; the crotch gusset hangs below it
WAIST_Z = 1.200                 # open top edge, under coat + belt (belt zone allows <= +0.036)
PELVIS_Z = [WAIST_Z, 1.125, 1.050, 0.985, Z_SPLIT]
SPLIT_DIP = 0.028               # split ring sags this much at the centre front / back
SPLIT_RY = S.torso_section(Z_SPLIT)[2] + 0.012
BLOUSE_Z0, BLOUSE_Z1, BLOUSE_R = C["trouser_blouse"]
BOOT_TOP = C["boot_top_z"]
KZ = S.KNEE.z
KNEE_ROWS = [KZ + 0.030, KZ + 0.012, KZ, KZ - 0.012, KZ - 0.030]     # 3 tight loops inside a 6 cm band


def _thigh_r(z):
    """Trouser radius on the thigh: bare leg + cloth (snug at the hip, ~0.02 baggy further down)."""
    t = max(leg_t(z), 0.0)
    return S.leg_radius(t) + 0.012 + 0.008 * _sstep(t / 0.45)


THIGH_Z = [0.905, 0.860, 0.808, 0.750, 0.690, 0.630, KNEE_ROWS[0]]
_DIP = {0: 0.030, 1: 0.018, 2: 0.008}          # leg rings just under the crotch tilt down on the inside
_SEAT = {0: 0.50, 1: 0.24, 2: 0.08}            # ... and keep some of the seat depth (front / back)
_UNTWIST = {0: 0.55, 1: 0.85}                  # ... and part of the split ring's plan shape

# bloused lower leg, top -> bottom: (z, radius about lower_c, sag weight).  ONE smooth overhang (review:
# the reference blouse is a single puff), its rounded hem sags a little toward the front.
BLOUSE_ROWS = [
    (0.474, 0.0905, 0.00), (0.442, 0.0950, 0.00), (0.413, 0.0975, 0.00), (0.385, 0.0990, 0.05),
    (0.359, 0.1000, 0.15), (0.336, 0.1000, 0.35), (0.316, 0.0985, 0.60), (0.303, 0.0945, 0.85),
    (0.2955, 0.0870, 1.00), (0.2935, 0.0770, 1.00),                                          # rounded hem, underside
]
# (round 4: BLOUSE_ROWS is now only the smooth profile; the actual rows, folds and the gaiter tucked into
#  the boot are FOLD_ROWS / TUCK_ROWS further down.)
SAG = 0.008                     # how far the hem droops (most at the front)


def _creases(th, z, side):
    """Vertical creases (radial metres), strongest where the blouse gathers above the rolls."""
    u = th / TAU
    v = _sstep((0.50 - z) / 0.12)
    amp = 0.0040 * v * (1.0 - 0.40 * _sstep((0.36 - z) / 0.07)) + 0.0010
    return lib.fold_wave(u, (z - 0.25) / 0.30, amp=amp, count=3.0, seed=0.37 if side > 0 else 0.81, grow=0.0)


def _sag(th, side, z=0.3):
    """How much the rolls droop at angle th (1 = most): front-outer on the left leg, front-inner on the right.
    The droop direction wanders a little from roll to roll, so the folds are not parallel rings."""
    wander = 0.25 * math.sin(z * 60.0 + (1.3 if side > 0 else 4.1))
    return 0.55 + 0.45 * math.cos(th - math.radians(285.0 if side > 0 else 255.0) + wander)


def trouser_pt(z, r, th, side=1, dip=0.0, seat=0.0, sag=0.0, folds=True):
    """Point on the LEFT trouser leg (side only picks the fold pattern). th: 0 outer, 90 back, 270 front."""
    cx, cy = lower_c(z)
    x_out, x_in = cx + r, max(cx - r, GAP)
    ex, rx = 0.5 * (x_out + x_in), 0.5 * (x_out - x_in)
    dr = 0.0
    if folds:
        if z < 0.52:
            dr = _creases(th, z, side)
        else:
            dr = 0.0018 * math.sin(3 * th + 21.0 * z + side)
        dr = min(dr, BLOUSE_R + 0.002 - r)
    k = 1.0 + dr / r
    x = max(ex + rx * math.cos(th) * k, GAP - 0.004)
    y = cy + (r + seat * (SPLIT_RY - _thigh_r(Z_SPLIT))) * math.sin(th) * k
    zz = z - dip * ((1 - math.cos(th)) * 0.5) ** 1.5 - sag * SAG * _sag(th, side, z)
    return V((x, y, zz))


def boot_rel_pt(z, d, th):
    """Point at offset d outside the boot shaft surface (world angle th) - used for the tucked hem."""
    a = th - PHI
    sx, sy = shaft_axis(z)
    rx = rad_s(z) + d
    ry = (rad_b(z) if math.sin(a) > 0 else rad_f(z)) + d
    return to_world((sx + rx * math.cos(a), sy + ry * math.sin(a), z))


def _split_xy(i):
    """Split ring (pelvis ring at Z_SPLIT): superellipse hugging both thighs.  -> (x, y, z_drop)"""
    a = TAU * i / NP
    c, s = math.cos(a), math.sin(a)
    cx, _ = leg_c(Z_SPLIT)
    rx = cx + _thigh_r(Z_SPLIT)
    e = 2.0 / 2.4
    return (rx * math.copysign(abs(c) ** e, c), SPLIT_RY * math.copysign(abs(s) ** e, s),
            SPLIT_DIP * abs(s) ** 4)


def _pelvis_pt(i, z):
    cy, rx, ry = S.torso_section(z)
    rx, ry = rx + 0.012, ry + 0.012
    a = TAU * i / NP
    pu = V((rx * math.cos(a), cy + ry * math.sin(a), z))
    sx, sy, dz = _split_xy(i)
    s = _sstep((1.04 - z) / (1.04 - Z_SPLIT))
    return pu.lerp(V((sx, sy, z - dz)), s)


def _leg_ring(z, side, k=None, r=None, sag=0.0, n=NL, folds=True, ths=None):
    r = _thigh_r(z) if r is None else r
    pts = []
    for th in (ths or [TAU * j / n for j in range(n)]):
        p = trouser_pt(z, r, th, side, dip=_DIP.get(k, 0.0), seat=_SEAT.get(k, 0.0), sag=sag, folds=folds)
        pts.append(V((p.x * side, p.y, p.z)))
    return pts


def _transition(mb, top, bot):
    """All-quad band from the NL-vertex thigh ring `top` to the NK-vertex ring `bot` (both closed, same
    winding, index 0 = outer side).  Segments in SPLIT become 3 segments each (3-to-1 pattern:
    two mid-height vertices, four quads)."""
    P = mb.v
    b = 0
    for j in range(NL):
        t0, t1 = top[j], top[(j + 1) % NL]
        if j in SPLIT:
            b0, b1, b2, b3 = (bot[(b + k) % NK] for k in range(4))
            m1 = mb.add([(P[b1] + P[t0].lerp(P[t1], 0.36)) * 0.5])[0]
            m2 = mb.add([(P[b2] + P[t0].lerp(P[t1], 0.64)) * 0.5])[0]
            mb.f += [(t0, m1, b1, b0), (m1, m2, b2, b1), (m2, t1, b3, b2), (t0, t1, m2, m1)]
            b += 3
        else:
            mb.f.append((t0, t1, bot[(b + 1) % NK], bot[b % NK]))
            b += 1
    assert b == NK


def build_trousers():
    """Pelvis + crotch + both thighs down to the top knee loop.  Returns (object, bottom rings L/R)."""
    mb = MB()
    prs = [mb.add([_pelvis_pt(i, z) for i in range(NP)]) for z in PELVIS_Z]
    for a, b in zip(prs, prs[1:]):
        mb.strip(a, b, closed=True)
    split = prs[-1]
    hb, hf = NP // 4, 3 * NP // 4                      # back / front centre index
    # crotch gusset: 2 quads wide, 6 long, from back centre under the crotch to front centre
    yb = SPLIT_RY
    ys = [yb * f for f in (0.68, 0.35, 0.0, -0.35, -0.68)]

    def saddle_z(y):
        return Z_SPLIT - SPLIT_DIP - 0.016 * math.sqrt(max(0.0, 1 - (y / yb) ** 2))
    x7 = mb.v[split[hb - 1]].x

    def gx(y):
        return GAP + (x7 - GAP) * 0.75 * abs(y / yb) ** 2.5
    cen = mb.add([V((0.0, y, saddle_z(y))) for y in ys])
    lft = mb.add([V((gx(y), y, saddle_z(y) - 0.005 * (1 - abs(y / yb)))) for y in ys])
    rgt = mb.add([V((-gx(y), y, saddle_z(y) - 0.005 * (1 - abs(y / yb)))) for y in ys])
    mb.strip([split[hb - 1]] + lft + [split[hf + 1]], [split[hb]] + cen + [split[hf]])
    mb.strip([split[hb]] + cen + [split[hf]], [split[hb + 1]] + rgt + [split[hf - 1]])
    bottoms = {}
    for side, bridge in ((1, lft), (-1, rgt)):
        def pel(i):
            return split[i % NP] if side > 0 else split[(NP // 2 - i) % NP]
        ring = [pel(j) for j in range(0, hb)] + bridge + [pel(j) for j in range(hf + 1, NP)]
        assert len(ring) == NL
        base = [mb.v[i].copy() for i in ring]
        for k, z in enumerate(THIGH_Z[:-1]):
            pts = _leg_ring(z, side, k)
            w = _UNTWIST.get(k, 1.0)
            for j, p in enumerate(pts):
                p.x, p.y = _lerp(base[j].x, p.x, w), _lerp(base[j].y, p.y, w)
            new = mb.add(pts)
            mb.strip(ring, new, closed=True)
            ring = new
        # last thigh band: 24 -> 40 vertices (8 all-quad 1->3 splits), so the knee + blouse get the
        # ring density their wrinkles need.  Hidden under the coat skirt / under skirt.
        pts = _leg_ring(THIGH_Z[-1], side, len(THIGH_Z) - 1, n=NK)
        bottom = mb.add(pts)
        _transition(mb, ring, bottom)
        bottoms[side] = pts
    return mb.obj("trousers", "main", sharp_angle=75), bottoms


def _knee_r(z):
    return _lerp(_thigh_r(KNEE_ROWS[0]), BLOUSE_ROWS[0][1] - 0.006, (KNEE_ROWS[0] - z) / (KNEE_ROWS[0] - KNEE_ROWS[-1]))


# ------------------------------------------------------------------ stylised folds (round 4)
# Reference (front_legs / side_legs / back_legs): a full, slightly boxy blouse crossed by FEW, BIG, clean
# horizontal fold bands that wrap the leg, tilted a little and sagging toward the front, and a strong
# rounded roll at the bottom with a tight crease tucked under it into the slim gaiter.  The folds are
# built into the ring radii / heights themselves: every crest and crease is an edge loop that follows the
# fold line (tilt + front sag), with tight support loops at each fold and even ~2 cm spacing between.
# fold lines: (sag toward the front, tilt amplitude, tilt phase deg) per side.  Heights are in the rows.
FOLD_LINES = {
    "K": dict(sag=0.000, tilt=0.000, ph=(0, 0)),         # knee loops: level
    "F1": dict(sag=0.009, tilt=0.0035, ph=(20, 150)),    # upper band, mostly under the coat skirt
    "F2": dict(sag=0.016, tilt=0.0060, ph=(-30, 200)),   # main band, just below the skirt points
    "R": dict(sag=0.011, tilt=0.0035, ph=(10, 170)),     # bottom roll + the crease tucked under it
}
# blouse rows below the knee loops: (nominal z, radial offset from the smooth profile, fold line or None).
# Within a fold the cloth swells gently toward the crest, rounds over and steps in sharply at the crease
# (a silhouette step), then returns to the flat profile.  "abs" rows give the roll's underside radius.
FOLD_ROWS = [
    (0.490, 0.0000, None), (0.468, 0.0000, None),
    (0.447, 0.0004, "F1"), (0.437, 0.0016, "F1"), (0.4305, 0.0012, "F1"), (0.4250, -0.0042, "F1"),
    (0.4190, -0.0022, "F1"),
    (0.401, 0.0000, None),
    (0.387, 0.0008, "F2"), (0.377, 0.0034, "F2"), (0.3700, 0.0030, "F2"), (0.3640, -0.0055, "F2"),
    (0.3580, -0.0030, "F2"),
    (0.343, 0.0000, None),
    (0.329, 0.0005, "R"), (0.319, 0.0040, "R"), (0.3100, 0.0062, "R"), (0.3025, 0.0045, "R"),
    (0.2965, ("abs", -0.0090), "R"), (0.2940, ("abs", -0.0215), "R"),
]
# slim gaiter tucked over the boot rim: (z, offset from the boot shaft surface, share of the roll's sag)
TUCK_ROWS = [(0.2895, 0.0070, 1.0), (0.2780, 0.0058, 0.5), (0.2640, 0.0045, 0.15), (0.2505, 0.0026, 0.0),
             (0.2490, -0.0006, 0.0), (0.2470, -0.0044, 0.0), (0.2300, -0.0060, 0.0)]
# one broad, soft fold over the front of the knee: (knee row index, radial offset at the very front)
KNEE_FOLD = {2: -0.0030, 3: 0.0018}

_PROF = [_catmull(BLOUSE_ROWS, k * 0.05) for k in range(0, 161)]       # smooth blouse profile, top -> hem
_PROF_Z = sorted((z, r) for z, r, _ in _PROF)


def _r_base(z):
    """Smooth blouse radius (no folds) at nominal height z: knee loops -> blouse profile."""
    z0, r0 = KNEE_ROWS[-1], _knee_r(KNEE_ROWS[-1])
    if z >= BLOUSE_ROWS[0][0]:
        return _lerp(BLOUSE_ROWS[0][1], r0, _sstep((z - BLOUSE_ROWS[0][0]) / (z0 - BLOUSE_ROWS[0][0])))
    return _interp(_PROF_Z, z)


def _front(th):
    """1 at the very front of the leg (th = 270 deg), 0 at the back."""
    return (0.5 - 0.5 * math.sin(th)) ** 1.5


def _line_dz(g, th, side):
    """Height offset of fold line g at angle th: sag toward the front + a slight tilt (per side)."""
    if g is None:
        return 0.0
    f = FOLD_LINES[g]
    ph = math.radians(f["ph"][0 if side > 0 else 1])
    return -f["sag"] * _front(th) + f["tilt"] * math.cos(th - ph)


def _depth(th, side, g):
    """How strongly a fold shows at angle th: strongest at the front / outer side where the cloth sags,
    softer on the inner side, with a slow per-side wobble so the legs don't mirror."""
    seed = {"F1": 0.7, "F2": 2.1, "R": 4.0, "K": 0.0}[g] + (0.0 if side > 0 else 1.9)
    return 0.52 + 0.48 * _front(th) + 0.14 * math.cos(th) + 0.10 * math.sin(2 * th + seed)


def _row_dz(i, th, side):
    """Height offset of FOLD_ROWS[i] at th: its own fold line, or a blend of the neighbouring lines."""
    z, _, g = FOLD_ROWS[i]
    if g is not None:
        return _line_dz(g, th, side)
    up = [(KNEE_ROWS[-1], "K")] + [(r[0], r[2]) for r in FOLD_ROWS[:i] if r[2]]
    dn = [(r[0], r[2]) for r in FOLD_ROWS[i + 1:] if r[2]]
    (za, ga), (zb, gb) = up[-1], dn[0]
    return _lerp(_line_dz(ga, th, side), _line_dz(gb, th, side), (za - z) / (za - zb))


def build_knee_and_blouse(side, top_ring):
    """Knee band (support) + bloused lower leg (main) for one side.  Both share their boundary ring.
    Rings are NK around; the folds are shaped by each ring's radius and tilt (edge loops along the folds)."""
    sfx = "L" if side > 0 else "R"
    ths = [TAU * j / NK for j in range(NK)]

    def mk(z, r, th, dz=0.0):
        p = trouser_pt(z + dz, r, th, side, folds=False)
        return V((p.x * side, p.y, p.z))
    knee = [top_ring]
    for k, z in enumerate(KNEE_ROWS[1:], 1):
        a = KNEE_FOLD.get(k, 0.0)
        knee.append([mk(z, _knee_r(z) + a * _front(th) ** 1.3, th) for th in ths])
    knee_ob = lib.lofted("trouser_knee_" + sfx, knee, topo="support", sharp_angle=75)
    rings = [knee[-1]]
    r_lip = _r_base(0.316)
    for i, (z, dr, g) in enumerate(FOLD_ROWS):
        ring = []
        for th in ths:
            if isinstance(dr, tuple):
                r = r_lip + dr[1] * (0.85 + 0.15 * _depth(th, side, g))
            else:
                r = _r_base(z) + dr * (_depth(th, side, g) if g else 1.0)
            ring.append(mk(z, r, th, _row_dz(i, th, side)))
        rings.append(ring)
    for z, d, w in TUCK_ROWS:
        ring = []
        for th in ths:
            p = boot_rel_pt(z + w * _line_dz("R", th, side), d, th)
            ring.append(V((p.x * side, p.y, p.z)))
        rings.append(ring)
    blouse = lib.lofted("trouser_blouse_" + sfx, rings, topo="main", sharp_angle=75)
    return [knee_ob, blouse]


# ================================================================== BOOT UPPER (left, local frame)
NS, PW, PR = 24, 8, 4            # shaft columns / foot patch width / foot patch rows
JP0 = 3 * NS // 4 - PW // 2      # patch columns JP0 .. JP1, centred on the front (-y)
JP1 = JP0 + PW
NF = 2 * (PW + PR)               # foot ring vertex count
T = BOOT_TOP

# shaft radii vs z: side (x), back (+y), front (-y); front superellipse power (boxy where the foot joins)
_RS = [(ZB, 0.0540), (0.050, 0.0555), (0.075, 0.0560), (0.100, 0.0540), (0.135, 0.0400)]
_RB = [(ZB, 0.0860), (0.050, 0.0860), (0.075, 0.0800), (0.100, 0.0700), (0.125, 0.0620), (0.160, 0.0500)]
_RF = [(ZB, 0.0640), (0.110, 0.0620), (0.150, 0.0500)]
_PF = [(ZB, 4.5), (0.100, 3.4), (0.125, 2.6), (0.150, 2.0)]


def rad_s(z):
    """Shaft half width (local x): heel-counter table at the foot, slim bare leg + ease above it."""
    return max(_interp(_RS, z), _bare_r(z) + SHAFT_EASE[0])


def rad_b(z):
    return max(_interp(_RB, z), _bare_r(z) + SHAFT_EASE[1])


def rad_f(z):
    return max(_interp(_RF, z), _bare_r(z) + SHAFT_EASE[2])

# full shaft rings: (z at the back, how far the front arches up over the instep crease)
SHAFT_ROWS = [(0.206, 0.0), (0.189, 0.0), (0.172, 0.004), (0.156, 0.012), (0.141, 0.022), (0.126, 0.034)]
HEEL_Z = [0.100, 0.076, 0.051, ZB]          # PR rows below the last full ring (heel counter)
# rolled cuff on top of the shaft: (radial offset, z), outside-bottom -> over the rim -> inner lip
CUFF = [(0.0010, T - 0.016), (0.0016, T - 0.004), (0.0008, T + 0.0015), (-0.0012, T + 0.0025),
        (-0.0030, T - 0.003), (-0.0030, T - 0.015)]

# foot cross-sections, from the ankle patch (0) to the toe (9):
#   y of the sole line, (y, z) of the top centre line, max half width, half width at the sole, corner angle
FOOT = [
    (-0.048, -0.050, 0.1600, 0.0540, 0.0530, 44.0),
    (-0.066, -0.070, 0.1360, 0.0620, 0.0600, 46.0),
    (-0.084, -0.089, 0.1280, 0.0690, 0.0670, 48.0),
    (-0.102, -0.107, 0.1185, 0.0750, 0.0730, 50.0),
    (-0.120, -0.124, 0.1150, 0.0790, 0.0770, 52.0),
    (-0.137, -0.141, 0.1140, 0.0820, 0.0790, 53.0),
    (-0.153, -0.157, 0.1120, 0.0830, 0.0800, 54.0),
    (-0.167, -0.170, 0.1090, 0.0815, 0.0785, 55.0),
    (-0.178, -0.180, 0.1035, 0.0770, 0.0740, 56.0),
    (-0.186, -0.187, 0.0935, 0.0670, 0.0640, 57.0),
]
TOE_STEP = 0.0028                 # toe cap leather sits this far proud -> seam line
FOOT_US = [(1, 0), (2, 0), (3, 0), (4, 0), (4.86, 0), (4.97, 1), (6, 1), (7, 1), (8, 1), (9, 1)]
TOE_BULGE = 0.0085


def shaft_pt(zb, arch, j, dr=0.0):
    psi = TAU * j / NS
    c, s = math.cos(psi), math.sin(psi)
    d = abs(math.degrees(psi) - 270.0)
    f = math.cos(math.radians(90.0 * d / 80.0)) ** 2 if d < 80.0 else 0.0
    z = zb + arch * f
    sx, sy = shaft_axis(z)
    rs = rad_s(z) + dr
    if s >= 0:
        return V((sx + rs * c, sy + (rad_b(z) + dr) * s, z))
    e = 2.0 / _interp(_PF, z)
    return V((sx + rs * math.copysign(abs(c) ** e, c), sy - (rad_f(z) + dr) * abs(s) ** e, z))


def dome_ring(u, step=0):
    yb, yt, zt, W, Wb, phc = _catmull(FOOT, u)
    phc = math.radians(phc)
    if step:
        W += TOE_STEP
        Wb += TOE_STEP * 0.6
        zt += TOE_STEP
    e = 2.0 / 2.6

    def pt(phi):
        c, sn = math.cos(phi), math.sin(phi)
        h = abs(sn) ** e
        xl = math.copysign(abs(c) ** e * W - (W - Wb) * (1 - h) ** 3, c)
        return V((xl, yb + (yt - yb) * h, ZB + (zt - ZB) * h))
    pts = [pt(math.pi - phc - (math.pi - 2 * phc) * q / PW) for q in range(PW + 1)]       # top, inner -> outer
    pts += [pt(phc * (1 - k / PR)) for k in range(1, PR + 1)]                             # outer side, down
    pts += [V((Wb * (1 - 2 * k / PW), yb, ZB)) for k in range(1, PW + 1)]                 # sole line, outer -> inner
    pts += [pt(math.pi - phc * k / PR) for k in range(1, PR)]                             # inner side, up
    return pts


def build_upper():
    mb = MB()
    idx = {}
    itop = len(SHAFT_ROWS) - 1
    cuff = []
    for dr, z in reversed(CUFF):
        cuff.append(mb.add([shaft_pt(z, 0.0, j, dr) for j in range(NS)]))
    for a, b in zip(cuff, cuff[1:]):
        mb.strip(a, b, closed=True)
    prev = cuff[-1]
    for i, (zb, arch) in enumerate(SHAFT_ROWS):
        ring = mb.add([shaft_pt(zb, arch, j) for j in range(NS)])
        for j in range(NS):
            idx[(i, j)] = ring[j]
        mb.strip(prev, ring, closed=True)
        prev = ring
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
    pa, pb = mb.v[idx[(nr, JP0)]], mb.v[idx[(nr, JP1)]]
    for j in range(JP0 + 1, JP1):
        idx[(nr, j)] = mb.add([pa.lerp(pb, (j - JP0) / PW)])[0]
    # foot: ring 0 = boundary of the front patch, then dome rings to the toe
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
    top = prev[0:PW + 1]
    right = prev[PW:PW + PR + 1]
    bottom = list(reversed(prev[PW + PR:2 * PW + PR + 1]))
    left = [prev[0]] + list(reversed(prev[2 * PW + PR:NF]))

    def bulge(u, v):
        return V((0.0, -TOE_BULGE * (1 - (2 * u - 1) ** 2) ** 0.55 * (1 - (2 * v - 1) ** 2) ** 0.55, 0.0))
    mb.coons(top, right, bottom, left, bulge)

    def col(j):
        return idx[(nr, j % NS)]
    mb.coons([col(j) for j in range(JP0, JP1 + 1)],
             [col(JP1 + k) for k in range(0, PR + 1)],
             [col(JP0 - PR - k) for k in range(0, PW + 1)],
             [col(JP0 - k) for k in range(0, PR + 1)])
    assert (JP0 - PR - PW) % NS == (JP1 + PR) % NS
    return mb


# ================================================================== SOLE (local frame)
OUT_Z = 0.016             # top of the outsole (a welt band sits above it)
NOTCH = 0.011             # arch notch height between the heel block and the forefoot


def sole_w(y):
    """Half width of the sole along the foot: follows the upper's footprint + ~6 mm, waisted at the arch."""
    return _interp([(-0.150, 0.0860), (-0.120, 0.0850), (-0.090, 0.0800), (-0.060, 0.0700), (-0.035, 0.0610),
                    (-0.010, 0.0590), (0.020, 0.0605), (0.060, 0.0610), (0.080, 0.0600)], y)


def _sole_sections():
    """(y, half width, bottom z) cross-sections from heel to toe: rounded heel, heel block, arch notch,
    forefoot, rounded toe."""
    rh, rt = 0.040, 0.054
    yh, yt = Y_HEEL - rh, Y_TOE + rt
    st = [(yh + rh * math.sin(math.radians(a)), sole_w(yh) * math.cos(math.radians(a)) ** 0.8, 0.0)
          for a in (76, 60, 42, 22)]
    st += [(yh, sole_w(yh), 0.0), (0.040, sole_w(0.040), 0.0), (0.033, sole_w(0.033), 0.0),
           (0.029, sole_w(0.029), NOTCH), (-0.026, sole_w(-0.026), NOTCH), (-0.036, sole_w(-0.036), 0.0)]
    st += [(y, sole_w(y), 0.0) for y in (-0.066, -0.096, -0.124, yt)]
    st += [(yt - rt * math.sin(math.radians(a)), sole_w(yt) * math.cos(math.radians(a)) ** 0.8, 0.0)
           for a in (22, 42, 60, 76)]
    return st


def build_sole():
    mb = MB()
    ch = 0.0025
    rings = []
    st = _sole_sections()
    for y, w, zb in st:
        r = [(-(w - ch), zb), (0.0, zb), (w - ch, zb)]
        r += [(w, zb + ch), (w, OUT_Z), (w - 0.0022, OUT_Z + 0.0022), (w + 0.0012, OUT_Z + 0.0045),
              (w + 0.0012, SOLE_T - 0.002)]
        r += [(w - 0.001, SOLE_T), (0.0, SOLE_T), (-(w - 0.001), SOLE_T)]
        r += [(-(w + 0.0012), SOLE_T - 0.002), (-(w + 0.0012), OUT_Z + 0.0045), (-(w - 0.0022), OUT_Z + 0.0022),
              (-w, OUT_Z), (-w, zb + ch)]
        rings.append(mb.add([V((x, y, z)) for x, z in r]))
    for a, b in zip(rings, rings[1:]):
        mb.strip(a, b, closed=True)

    def cap(r, sign, depth):
        mb.coons([r[10], r[9], r[8]], [r[8], r[7], r[6], r[5], r[4], r[3], r[2]],
                 [r[0], r[1], r[2]], [r[10], r[11], r[12], r[13], r[14], r[15], r[0]],
                 lambda u, v: V((0.0, sign * depth * (1 - (2 * u - 1) ** 2) ** 0.6, 0.0)))
    cap(rings[0], 1.0, Y_HEEL - st[0][0])
    cap(rings[-1], -1.0, st[-1][0] - Y_TOE)
    return mb


# ================================================================== STRAPS / HARDWARE
def _bvh(mbs):
    v, f = [], []
    for m in mbs:
        o = len(v)
        v += [tuple(p) for p in m.v]
        f += [tuple(i + o for i in fc) for fc in m.f]
    return BVHTree.FromPolygons(v, f)


def ring_on(bvh, c, z, angles):
    """Horizontal ring of surface points around local axis point c at height z (ray cast from outside)."""
    out = []
    for a in angles:
        u = V((math.cos(a), math.sin(a), 0.0))
        o = V((c[0], c[1], z))
        hit = bvh.ray_cast(o + u * 0.4, -u)
        loc, n = (hit[0], hit[1]) if hit[0] is not None else (o + u * 0.06, u)
        n = V((n.x, n.y, 0.0)).normalized() if abs(n.z) < 0.9 else u
        if n.dot(u) < 0:                              # BVH normals follow the (unrecalculated) winding
            n = -n
        out.append((loc, n))
    return out


def _core(p):
    """Foot / shaft 'core' point used to aim rays at the boot from outside."""
    w = _sstep((p.z - 0.12) / 0.05)
    sx, sy = shaft_axis(p.z)
    return V((0.0, p.y, 0.055)).lerp(V((sx, sy, p.z)), w)


def _surf_normal(bvh, p, d, r=0.010):
    """Averaged surface normal around p (5 parallel rays along -d) -> smooth, outward."""
    e1 = d.orthogonal().normalized()
    e2 = d.cross(e1)
    acc = V()
    for o in (V(), e1 * r, -e1 * r, e2 * r, -e2 * r):
        hit = bvh.ray_cast(p + o + d * 0.12, -d)
        if hit[0] is not None:
            n = hit[1] if hit[1].dot(d) > 0 else -hit[1]
            acc += n
    return acc.normalized() if acc.length > 1e-6 else d


def path_frames(bvh, pts, subdiv=3):
    """A local poly line laid onto the boot: points pushed out from the foot's core onto the outermost
    surface, with area-averaged normals smoothed along the path."""
    raw = []
    for p in lib.smooth_path(pts, subdiv):
        p = V(p)
        d = (p - _core(p)).normalized()
        hit = bvh.ray_cast(p + d * 0.12, -d)
        q = hit[0] if hit[0] is not None else p
        raw.append((q, _surf_normal(bvh, q, d)))
    return [(q, sum((raw[k][1] for k in range(max(i - 2, 0), min(i + 3, len(raw)))), V()).normalized())
            for i, (q, _n) in enumerate(raw)]


def band(mb, bvh, frames, width, thick, closed=False, clear=0.0007):
    """Strap / panel with thickness that conforms to the surface across its width as well as along it:
    each section has 3 bottom verts (edge, middle, edge), each lifted just clear of the surface (checked
    with extra rays between them) and smoothed along the strap, so edges never cut into the shell.
    Returns per-section (centre on the top face, along, across, normal) for placing hardware."""
    n_f = len(frames)
    hw = width / 2
    sec = []
    for i, (p, n) in enumerate(frames):
        a = frames[(i - 1) % n_f if closed else max(i - 1, 0)][0]
        b = frames[(i + 1) % n_f if closed else min(i + 1, n_f - 1)][0]
        t = (b - a).normalized()
        sx = n.cross(t).normalized()
        nn = t.cross(sx).normalized()
        if nn.dot(n) < 0:
            nn, sx = -nn, -sx
        sec.append([p, t, sx, nn])

    def height(q, nn):
        hit = bvh.ray_cast(q + nn * 0.04, -nn)
        return (0.04 - hit[3]) if hit[0] is not None else -1.0
    lifts = []
    for p, t, sx, nn in sec:
        h = [height(p + sx * (hw * f), nn) + clear for f in (-1.0, -0.5, 0.0, 0.5, 1.0)]
        L = [h[0], h[2], h[4]]
        for k, (ia, ib) in ((1, (0, 1)), (3, (1, 2))):          # keep the facets above the half-way rays
            deficit = h[k] - 0.5 * (L[ia] + L[ib])
            if deficit > 0:
                L[ia] += deficit
                L[ib] += deficit
        lifts.append(L)
    for _ in range(2):                                         # smooth along the strap, never lowering
        new = []
        for i in range(n_f):
            ia = (i - 1) % n_f if closed else max(i - 1, 0)
            ib = (i + 1) % n_f if closed else min(i + 1, n_f - 1)
            new.append([max(lifts[i][c], 0.5 * (lifts[ia][c] + lifts[ib][c])) for c in range(3)])
        lifts = new
    rings, out = [], []
    for (p, t, sx, nn), L in zip(sec, lifts):
        bl, bm, br = p - sx * hw + nn * L[0], p + nn * L[1], p + sx * hw + nn * L[2]
        rings.append(mb.add([bl, bm, br, br + nn * thick, bm + nn * thick, bl + nn * thick]))
        out.append((bm + nn * thick, t, sx, nn))
    for a, c in zip(rings, rings[1:] + ([rings[0]] if closed else [])):
        mb.strip(a, c, closed=True)
    if not closed:
        for r in (rings[0], rings[-1]):
            mb.quad(r[0], r[1], r[4], r[5])
            mb.quad(r[1], r[2], r[3], r[4])
    return out


def buckle(mb, o, t, b, n, lt, lb, bar=0.0050, h=0.0045):
    """Single square frame buckle with a thin prong.  o = centre on the strap's top; t along / b across / n out."""
    def P(tt, bb, nn):
        return o + t * tt + b * bb + n * nn

    def path(lt_, lb_, c):
        return [(lt_ - c, -lb_), (lt_, -lb_ + c), (lt_, 0.0), (lt_, lb_ - c), (lt_ - c, lb_), (0.0, lb_),
                (-lt_ + c, lb_), (-lt_, lb_ - c), (-lt_, 0.0), (-lt_, -lb_ + c), (-lt_ + c, -lb_), (0.0, -lb_)]
    outer, inner = path(lt, lb, 0.0035), path(lt - bar, lb - bar, 0.0015)
    n0, n1, ch = -0.0005, h - 0.0005, 0.0011
    secs = []
    for (to, bo), (ti, bi) in zip(outer, inner):
        k = ch / bar
        secs.append(mb.add([P(ti, bi, n0), P(to, bo, n0), P(to, bo, n1 - ch),
                            P(_lerp(to, ti, k), _lerp(bo, bi, k), n1), P(ti, bi, n1)]))
    for a, c in zip(secs, secs[1:] + [secs[0]]):
        mb.strip(a, c, closed=True)
    # prong: thin pin hinged on the near bar, lying low across the window onto the far bar
    w2, z0, z1 = 0.0011, 0.0004, 0.0026
    x0, x1 = -lt + bar * 0.5, lt - bar * 0.4
    a = mb.add([P(x0, -w2, z0 + 0.0008), P(x0, w2, z0 + 0.0008), P(x0, w2, z1 + 0.0008), P(x0, -w2, z1 + 0.0008)])
    c = mb.add([P(x1, -w2, z0 + 0.0012), P(x1, w2, z0 + 0.0012), P(x1, w2, z1 + 0.0010), P(x1, -w2, z1 + 0.0010)])
    mb.strip(a, c, closed=True)
    mb.quad(*reversed(a))
    mb.quad(*c)


def build_straps(upper):
    """-> (straps MB [main], hardware MB [rigid])  -- all in the boot's local frame."""
    sm, hw = MB(), MB()
    # ---- tongue / lace panel down the instep, from under the cuff strap to the toe-cap seam
    bvh = _bvh([upper])
    tongue_pts = [(0.0, -0.040, 0.166), (0.0, -0.052, 0.148), (0.0, -0.068, 0.132), (0.0, -0.088, 0.118),
                  (0.0, -0.108, 0.109), (0.0, -0.128, 0.104)]
    tf = path_frames(bvh, tongue_pts, subdiv=2)
    tongue = band(sm, bvh, tf, 0.044, 0.0030)
    bvh = _bvh([upper, sm])
    for i in (2, 4):
        p, t, b, n = tongue[i]
        lf = path_frames(bvh, [p - b * 0.026, p, p + b * 0.026], subdiv=2)
        band(sm, bvh, lf, 0.006, 0.0022)
    # ---- ankle / instep strap: from the outer ankle (under the cuff strap) diagonally over the instep to the
    #      inner welt, with a square buckle on the outer side
    bvh = _bvh([upper, sm])
    ip = [(0.056, 0.000, 0.146), (0.056, -0.026, 0.136), (0.048, -0.054, 0.127), (0.025, -0.076, 0.118),
          (-0.004, -0.090, 0.110), (-0.034, -0.101, 0.096), (-0.056, -0.108, 0.075), (-0.068, -0.112, 0.052),
          (-0.072, -0.114, 0.030)]
    inst = band(sm, bvh, path_frames(bvh, ip, subdiv=3), 0.036, 0.0040)
    p, t, b, n = inst[4]
    buckle(hw, p, t, b, n, lt=0.018, lb=0.023)
    # ---- wide top cuff strap round the boot (+ tail lying on it, punched eyelets, rivet)
    bvh = _bvh([upper, sm])
    zs = 0.5 * (CUFF_STRAP_Z[0] + CUFF_STRAP_Z[1])
    ws = CUFF_STRAP_Z[1] - CUFF_STRAP_Z[0]
    ax = shaft_axis(zs)
    cuff = band(sm, bvh, ring_on(bvh, ax, zs, [TAU * k / NS + 1e-4 for k in range(NS)]), ws, 0.0055,
                closed=True, clear=0.0015)
    a_b = math.radians(-68.0)                    # buckle on the front-outer side (ref FRONT)
    bvh = _bvh([upper, sm])
    tail_a = [a_b - math.radians(d) for d in (12, 20, 28, 36, 44, 52, 60, 67)]
    tail = band(sm, bvh, ring_on(bvh, ax, zs, tail_a), ws * 0.84, 0.0028)
    p, t, b, n = band(MB(), bvh, ring_on(bvh, ax, zs, [a_b - 0.06, a_b, a_b + 0.06]), ws, 0.0)[1]
    buckle(hw, p + n * 0.0030, -t, b, n, lt=0.021, lb=ws * 0.5 + 0.005)
    for k in (3, 4, 5):
        p, t, b, n = tail[k]
        hw_r = lib.rivet("eyelet", radius=0.0036, height=0.0011, n=8, loc=p - n * 0.0003, normal=n)
        _absorb(hw, hw_r)
    p, t, b, n = cuff[(round(math.radians(20.0) / (TAU / NS))) % NS]
    _absorb(hw, lib.rivet("rivet", radius=0.0048, height=0.0020, n=8, loc=p - n * 0.0003, normal=n))
    return sm, hw


def _absorb(mb, ob):
    """Move a helper object's mesh into an MB and delete the object."""
    off = len(mb.v)
    mb.add([v.co.copy() for v in ob.data.vertices])
    mb.f += [tuple(i + off for i in p.vertices) for p in ob.data.polygons]
    me = ob.data
    bpy.data.objects.remove(ob)
    bpy.data.meshes.remove(me)


# ================================================================== build
def _to_world_ob(ob, sharp):
    for v in ob.data.vertices:
        v.co = to_world(v.co)
    ob.data.update()
    return lib.finalize(ob, sharp_angle=sharp)


def build():
    tr, bottoms = build_trousers()
    objs = [tr]
    for side in (1, -1):
        objs += build_knee_and_blouse(side, bottoms[side])
    upper = build_upper()
    straps, hw = build_straps(upper)
    left = [upper.obj("boot_upper_L", "main", sharp_angle=50),
            build_sole().obj("boot_sole_L", "rigid", sharp_angle=30),
            straps.obj("boot_straps_L", "main", sharp_angle=40),
            hw.obj("boot_hardware_L", "rigid", sharp_angle=30)]
    for o, sharp in zip(left, (50, 30, 40, 30)):
        _to_world_ob(o, sharp)
    objs += left + [lib.mirror_copy(o) for o in left]
    return objs
