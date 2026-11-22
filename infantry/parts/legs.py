"""Legs (round 5, matched to the user's FBX game model): straight, slightly loose trousers with a few
crisp knee crumples and a scrunched hem over a slim lower leg, low heeled ankle boots.

Objects (build() order)
    trousers                 main     pelvis (40) -> crotch gusset -> 2 thigh tubes (24), down to the knee loops
    trouser_knee_L / _R      support  the 3 tight knee loops (own object, shares its boundary rings)
    trouser_blouse_L / _R    main     lower trouser leg (name kept for the pose stage): straight shin with knee
                                      crumples -> 3-4 compressed hem folds -> hem tucked onto a slim lower leg
                                      -> pinched under the ankle cuff -> short crisp crumples -> into the boot
    boot_upper_L / _R        main     low boot: rolled collar + short shaft + heel, foot extruded from a front
                                      patch, long low squared toe with a proud toe-cap panel (seam step)
    boot_counter_L / _R      main     raised heel-counter panel wrapping the back of the heel, with a raised rim
    boot_sole_L / _R         rigid    thin forefoot sole plate with a protruding lip, rising through the arch
                                      onto the heel block (clear arch gap underneath), slight toe spring
    boot_heel_L / _R         rigid    tall block heel with a flared top lip
    boot_straps_L / _R       main     2 wide flat instep straps stacked diagonally (rear one on top) with end
                                      tabs, wrapped ankle cuff band, vertical gaiter strip on the outer side
    boot_hardware_L / _R     rigid    square cuff buckle (outer side) with prong

Boots are modelled in a local frame (x = outer side, -y = toe, origin under the ankle on the HEEL->TOE
line) and turned onto the spec's splayed HEEL -> TOE axis.  Right side = mirror copies, except the
lower trouser leg, which is built per side with different fold phases (folds are not mirrored).

Spec-derived dimensions adjusted here (FBX: longer, lower forefoot, heel counter close to the ankle):
the sole tip sits TOE_EXT beyond spec TOE along the HEEL->TOE line and the sole back HEEL_IN in front
of spec HEEL.
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


# ------------------------------------------------------------------ boot frame (left)
TOE_EXT = 0.030          # sole tip this far beyond spec TOE (FBX: long low forefoot, ~0.23 in front of the ankle)
HEEL_IN = 0.025          # sole back this far in front of spec HEEL (FBX: heel counter ~7 cm behind the ankle)
BOOT_DX = 0.008          # boot sits this far outside the spec HEEL->TOE line
SY = 0.022               # heel counter centre this far behind the ankle
SHAFT_DX = 0.002         # lower-leg axis this far outside the bare leg line

_A = V((S.TOE.x - S.HEEL.x, S.TOE.y - S.HEEL.y, 0.0))
FOOT_LEN = _A.length
_A.normalize()                                    # toe direction (splayed outward)
_P = V((-_A.y, _A.x, 0.0))                        # outward perpendicular (+X-ish for the left boot)
if _P.x < 0:
    _P = -_P
S_ANK = (S.ANKLE - S.HEEL).dot(_A)                # ankle position along the foot
_O = V((S.HEEL.x, S.HEEL.y, 0.0)) + _A * S_ANK + _P * BOOT_DX
Y_HEEL = S_ANK - HEEL_IN                          # local y of the sole's back ...
Y_TOE = S_ANK - FOOT_LEN - TOE_EXT                # ... and front tip
PHI = math.atan2(_P.y, _P.x)                      # rotation of the local frame (rad)


def to_world(p):
    """Local boot coords (x outer, y back, z up) -> world (left boot)."""
    x, y, z = p
    w = _O + _P * x - _A * y
    return V((w.x, w.y, z))


def to_local(w):
    d = V((w[0], w[1], 0.0)) - V((_O.x, _O.y, 0.0))
    return d.dot(_P), -d.dot(_A)


def leg_t(z):
    if z >= S.KNEE.z:
        return (S.HIP.z - z) / (S.HIP.z - S.KNEE.z)
    return 1.0 + (S.KNEE.z - z) / (S.KNEE.z - S.ANKLE.z)


def leg_c(z):
    p = S.leg_point(min(max(leg_t(z), 0.0), 2.0))
    return p.x, p.y


def _bare_r(z):
    return S.leg_radius(min(max(leg_t(max(z, S.ANKLE.z)), 0.0), 2.0))


def shaft_axis(z):
    """Local xy of the lower-leg / boot axis: follows the bare leg line above the ankle and swings onto the
    heel counter centre (0, SY) at the foot."""
    p = S.leg_point(min(max(leg_t(max(z, S.ANKLE.z)), 0.0), 2.0))
    lx, ly = to_local((p.x + SHAFT_DX, p.y))
    w = _sstep((z - 0.060) / 0.050)
    return _lerp(0.0, lx, w), _lerp(SY, ly, w)


def shaft_c(z):
    """World xy of the lower-leg axis."""
    sx, sy = shaft_axis(z)
    w = to_world((sx, sy, z))
    return w.x, w.y


def _front_w(psi):
    """1 at the front of a ring (local angle 270 deg), 0 beyond +-80 deg."""
    d = abs(math.degrees(psi % TAU) - 270.0)
    return math.cos(math.radians(90.0 * d / 80.0)) ** 2 if d < 80.0 else 0.0


# ------------------------------------------------------------------ slim lower leg (trouser under the hem)
TUBE_EASE = (0.0065, 0.0100, 0.0060)      # side, back, front ease over the bare leg (FBX: slim ~8 cm shaft)


def tube_pt(z, a, d=0.0):
    """Local point on the slim lower-leg surface at local angle a (0 outer, 90 back), offset d outward."""
    sx, sy = shaft_axis(z)
    b = _bare_r(z)
    s = math.sin(a)
    ry = (b + TUBE_EASE[1] if s > 0 else b + TUBE_EASE[2]) + d
    return V((sx + (b + TUBE_EASE[0] + d) * math.cos(a), sy + ry * s, z))


# ================================================================== TROUSERS
NP, NL = 40, 24                 # pelvis ring / leg ring vertex counts (leg = NP/2 - 1 pelvis verts + 5 gusset)
GAP = 0.016                     # inner thighs never get closer to the mirror plane than this
CROTCH_Z = S.TORSO[0][0]
Z_SPLIT = CROTCH_Z + 0.015      # last full pelvis ring; the crotch gusset hangs below it
WAIST_Z = 1.200                 # open top edge, under coat + belt (belt zone allows <= +0.036)
PELVIS_Z = [WAIST_Z, 1.125, 1.050, 0.985, Z_SPLIT]
SPLIT_DIP = 0.028               # split ring sags this much at the centre front / back
SPLIT_RY = S.torso_section(Z_SPLIT)[2] + 0.012
KZ = S.KNEE.z
KNEE_ROWS = [KZ + 0.030, KZ + 0.012, KZ, KZ - 0.012, KZ - 0.030]     # 3 tight loops inside a 6 cm band
CALF = 0.07                     # lower leg is a little deeper than wide (behind the shin)


def _thigh_r(z):
    """Trouser radius on the thigh: bare leg + a slim, slightly loose ease (FBX: straight trousers)."""
    t = max(leg_t(z), 0.0)
    return S.leg_radius(t) + 0.004 + 0.007 * _sstep(t / 0.6)


THIGH_Z = [0.905, 0.860, 0.808, 0.750, 0.690, 0.640, 0.600, KNEE_ROWS[0]]
_DIP = {0: 0.030, 1: 0.018, 2: 0.008}          # leg rings just under the crotch tilt down on the inside
_SEAT = {0: 0.50, 1: 0.24, 2: 0.08}            # ... and keep some of the seat depth (front / back)
_UNTWIST = {0: 0.55, 1: 0.85}                  # ... and part of the split ring's plan shape

# straight lower leg below the knee loops: (z, radius about lower_c)
LOWER_ROWS = [(0.492, 0.0730), (0.474, 0.0720), (0.452, 0.0705), (0.430, 0.0690), (0.408, 0.0678),
              (0.388, 0.0668), (0.372, 0.0660)]
# scrunched hem: (z, radius, crest weight).  Short compressed folds (crest / valley pairs ~1 cm apart).
BUNCH_ROWS = [(0.361, 0.0660, 0.45), (0.351, 0.0645, -0.25), (0.338, 0.0665, 1.00), (0.326, 0.0645, -0.30),
              (0.313, 0.0670, 1.00), (0.301, 0.0650, -0.25), (0.291, 0.0665, 0.85), (0.2835, 0.0620, 0.20)]
BUNCH_AMP = 0.0100
# tuck under the hem onto the slim lower leg, then down the leg, under the ankle cuff, crumpled above the
# boot collar and into the boot: (z, offset from the tube surface, crumple weight)
CUFF_Z = (0.153, 0.198)        # wrapped ankle cuff band (FBX: cuff z ~0.15-0.20, square buckle outer side)
COLLAR_Z, COLLAR_ARCH = 0.116, 0.016     # boot collar height at the back, rise over the instep
TUBE_ROWS = [(0.2785, 0.0105, 0.0), (0.2735, 0.0062, 0.0), (0.2670, 0.0030, 0.0), (0.2520, 0.0018, 0.0),
             (0.2330, 0.0016, 0.0), (0.2150, 0.0020, 0.0), (0.2050, 0.0030, 0.0), (0.1995, 0.0004, 0.0),
             (0.1760, -0.0004, 0.0), (0.1525, 0.0004, 0.0),
             (0.1490, 0.0040, 0.30), (0.1452, 0.0060, 0.90), (0.1418, 0.0060, 0.60), (0.1388, 0.0040, -0.30),
             (0.1352, 0.0060, 0.80), (0.1316, 0.0060, 1.00), (0.1282, 0.0045, 0.40), (0.1256, 0.0030, -0.30),
             (0.1224, 0.0050, 0.70), (0.1190, 0.0030, 0.20), (0.1120, 0.0005, 0.0),
             (0.1000, -0.0015, 0.0), (0.0920, -0.0025, 0.0)]
CRUMPLE_AMP = 0.0085
_CRUMPLE_GROUP = {0.149: 0, 0.1452: 0, 0.1418: 0, 0.1388: 1, 0.1352: 1, 0.1316: 1, 0.1282: 1, 0.1256: 2,
                  0.1224: 2, 0.119: 2}


def _knee_ridges(side):
    """Crisp knee crumples (fold_ridges, (s, l) in metres; s = angle * 0.075)."""
    R = 0.075
    sd = 0.0 if side > 0 else 1.7

    def s(deg):
        return R * math.radians(deg)
    out = lib.zigzag_ridges(s(50), s(130), 0.492, 0.604, 3, amp=0.0065, width=0.012, sharp=0.7, seed=0.4 + sd)
    out += [dict(pts=[(s(200), 0.606), (s(250), 0.582), (s(305), 0.594)], amp=0.0060, width=0.013, sharp=0.7),
            dict(pts=[(s(220), 0.524), (s(272), 0.500), (s(330), 0.518)], amp=0.0065, width=0.013, sharp=0.7),
            dict(pts=[(s(325), 0.570), (s(370), 0.546), (s(410), 0.560)], amp=0.0050, width=0.012, sharp=0.65),
            dict(pts=[(s(140), 0.472), (s(185), 0.456), (s(225), 0.466)], amp=0.0040, width=0.012, sharp=0.6),
            # long drape creases down the shin, fading into the hem bunch
            dict(pts=[(s(262 + 20 * sd), 0.480), (s(256 + 20 * sd), 0.420), (s(266 + 20 * sd), 0.365)],
                 amp=0.0030, width=0.012, sharp=0.55),
            dict(pts=[(s(318), 0.470), (s(326), 0.410), (s(316), 0.370)], amp=0.0028, width=0.012, sharp=0.55),
            dict(pts=[(s(28), 0.455), (s(20), 0.400)], amp=0.0024, width=0.012, sharp=0.5)]
    return out


_RIDGES = {1: _knee_ridges(1), -1: _knee_ridges(-1)}


def _folds(th, z, side):
    """Radial cloth displacement: faint long creases + the knee crumples."""
    dr = 0.0012 * math.sin(3 * th + 21.0 * z + side)
    if 0.355 < z < 0.64:
        dr += lib.fold_ridges(0.075 * th, z, _RIDGES[side], period=0.075 * TAU)
    return dr


def lower_c(z):
    """Trouser centre line: the leg line, drifting onto the slim lower-leg axis toward the ankle."""
    lx, ly = leg_c(z)
    bx, by = shaft_c(z)
    s = _sstep((0.50 - z) / 0.20)
    return _lerp(lx, bx, s), _lerp(ly, by, s)


def trouser_pt(z, r, th, side=1, dip=0.0, seat=0.0, folds=True, dz=0.0):
    """Point on the LEFT trouser leg (side only picks the fold pattern). th: 0 outer, 90 back, 270 front."""
    cx, cy = lower_c(z)
    x_out, x_in = cx + r, max(cx - r, GAP)
    ex, rx = 0.5 * (x_out + x_in), 0.5 * (x_out - x_in)
    dr = _folds(th, z, side) if folds else 0.0
    k = 1.0 + dr / r
    s = math.sin(th)
    calf = CALF * _sstep((0.52 - z) / 0.08) * max(0.0, s)
    x = max(ex + rx * math.cos(th) * k, GAP - 0.004)
    y = cy + (r * (1.0 + calf) + seat * (SPLIT_RY - _thigh_r(Z_SPLIT))) * s * k
    zz = z - dip * ((1 - math.cos(th)) * 0.5) ** 1.5 + dz
    return V((x, y, zz))


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


def _leg_ring(z, side, k=None, r=None):
    r = _thigh_r(z) if r is None else r
    pts = []
    for j in range(NL):
        p = trouser_pt(z, r, TAU * j / NL, side, dip=_DIP.get(k, 0.0), seat=_SEAT.get(k, 0.0))
        pts.append(V((p.x * side, p.y, p.z)))
    return pts


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
        pts = None
        for k, z in enumerate(THIGH_Z):
            pts = _leg_ring(z, side, k)
            w = _UNTWIST.get(k, 1.0)
            for j, p in enumerate(pts):
                p.x, p.y = _lerp(base[j].x, p.x, w), _lerp(base[j].y, p.y, w)
            new = mb.add(pts)
            mb.strip(ring, new, closed=True)
            ring = new
        bottoms[side] = pts
    return mb.obj("trousers", "main", sharp_angle=75), bottoms


def _knee_r(z):
    return _lerp(_thigh_r(KNEE_ROWS[0]), LOWER_ROWS[0][1] + 0.001,
                 (KNEE_ROWS[0] - z) / (KNEE_ROWS[0] - KNEE_ROWS[-1]))


def _phase(side, k):
    return (1.3 + 2.1 * k) if side > 0 else (4.0 + 1.7 * k)


def _bunch_ring(z, r, w, k, side):
    """One row of the scrunched hem: crest weight w (+ crest, - valley), tilted and uneven around the leg."""
    pts = []
    ph = _phase(side, k)
    for j in range(NL):
        th = TAU * j / NL
        mod = 0.50 + 0.60 * math.cos(th - ph) + 0.25 * math.cos(2 * th + 1.7 * ph) + 0.10 * math.cos(5 * th + ph)
        amp = BUNCH_AMP * w * (max(mod, 0.05) if w > 0 else 1.0)
        dz = (0.0065 * math.sin(th + 0.7 * ph) + 0.0020 * math.sin(3 * th + 2.3 * ph)) * (1.0 if w > 0 else 0.7)
        p = trouser_pt(z, r + amp, th, side, folds=False, dz=dz)
        pts.append(V((p.x * side, p.y, p.z)))
    return pts


def _tube_z(z, psi):
    """Crumple rows sit between the cuff band bottom and the boot collar, which rises over the instep: rows
    are squeezed into the shorter gap at the front instead of crossing the collar."""
    top, f = 0.150, _front_w(psi)
    zc = COLLAR_Z + COLLAR_ARCH * f
    if z >= top:
        return z
    if z >= COLLAR_Z:
        return top - (top - z) * (top - zc) / (top - COLLAR_Z)
    return z + COLLAR_ARCH * f


def tube_row(z, d, w, k, side):
    """Local points of one lower-leg row (NL columns at world angles), crumples applied."""
    out = []
    ph = _phase(side, k + 5)
    for j in range(NL):
        th = TAU * j / NL
        a = th - PHI
        f = _front_w(a)
        dd = d
        zz = _tube_z(z, a)
        if w:
            ph = _phase(side, 7 + _CRUMPLE_GROUP.get(round(z, 4), 0))    # rows of one fold share a phase
            mod = 0.50 + 0.50 * math.cos(a - ph) + 0.22 * math.cos(2 * a + 1.7 * ph) + 0.08 * math.cos(5 * a + ph)
            dd += CRUMPLE_AMP * w * (min(max(mod, 0.08), 1.1) if w > 0 else 1.0) * (1.0 - 0.55 * f)
            zz += (0.0030 * math.sin(a + 0.8 * ph) + 0.0012 * math.sin(3 * a + 2.3 * ph)) * (1.0 - f)
        out.append(tube_pt(zz, a, dd))
    return out


def lower_rows_local(side):
    """All tube rows (local coords), shared by the trouser mesh and the straps' surface queries."""
    return [tube_row(z, d, w, k, side) for k, (z, d, w) in enumerate(TUBE_ROWS)]


def build_knee_and_blouse(side, top_ring):
    """Knee band (support) + lower trouser leg (main) for one side.  Both share their boundary ring."""
    knee = [top_ring] + [_leg_ring(z, side, r=_knee_r(z)) for z in KNEE_ROWS[1:]]
    sfx = "L" if side > 0 else "R"
    knee_ob = lib.lofted("trouser_knee_" + sfx, knee, topo="support", sharp_angle=75)
    rings = [knee[-1]]
    for z, r in LOWER_ROWS:
        rings.append(_leg_ring(z, side, r=r))
    for k, (z, r, w) in enumerate(BUNCH_ROWS):
        rings.append(_bunch_ring(z, r, w, k, side))
    for row in lower_rows_local(side):
        ring = []
        for p in row:
            q = to_world(p)
            ring.append(V((q.x * side, q.y, q.z)))
        rings.append(ring)
    blouse = lib.lofted("trouser_blouse_" + sfx, rings, topo="main", sharp_angle=60)
    return [knee_ob, blouse]


# ================================================================== SOLE + HEEL (local frame)
HB_FRONT = 0.004          # front face of the heel block (local y)
HB_H = 0.040              # heel block height
Y_BALL = -0.088           # sole leaves the ground behind the ball of the foot (arch gap starts)
Y_SPRING = -0.172         # toe spring starts here ...
TOE_SPRING = 0.008        # ... and lifts the tip this much
SOLE_LIP = 0.0035         # protruding lip at the top of the sole plate


def sole_bot(y):
    if y >= HB_FRONT:
        return HB_H - 0.001
    if y >= Y_BALL:
        return (HB_H - 0.001) * _sstep((y - Y_BALL) / (HB_FRONT - Y_BALL))
    if y <= Y_SPRING:
        t = (Y_SPRING - y) / (Y_SPRING - Y_TOE)
        return TOE_SPRING * t * t
    return 0.0


def sole_t(y):
    return _lerp(0.0145, 0.0130, _sstep((y - Y_BALL) / (HB_FRONT - Y_BALL)))


def sole_top(y):
    return sole_bot(y) + sole_t(y)


def zb_up(y):
    """Bottom edge of the boot upper (sunk 2 mm into the sole plate)."""
    return sole_top(y) - 0.002


def sole_w(y):
    """Half width of the sole plate (without its lip) along the foot."""
    return _interp([(-0.220, 0.046), (-0.200, 0.0525), (-0.180, 0.0570), (-0.150, 0.0595), (-0.120, 0.0600),
                    (-0.095, 0.0585), (-0.070, 0.0545), (-0.045, 0.0505), (-0.020, 0.0490), (0.010, 0.0495),
                    (0.050, 0.0500), (0.090, 0.0485)], y)


def _ends(y_back, r_back, p_back, y_front, r_front, p_front, mids, wfun):
    """(y, half width) sections from the back tip to the front tip with superellipse-rounded ends."""
    st = []
    yc = y_back - r_back
    for s in (0.96, 0.85, 0.66, 0.38):
        st.append((yc + r_back * s, wfun(yc) * (1 - s ** p_back) ** (1 / p_back)))
    st += [(y, wfun(y)) for y in mids]
    yc = y_front + r_front
    for s in (0.38, 0.66, 0.85, 0.96):
        st.append((yc - r_front * s, wfun(yc) * (1 - s ** p_front) ** (1 / p_front)))
    return st


def _cap(mb, r, top, right, bottom, left, sign, depth):
    mb.coons([r[i] for i in top], [r[i] for i in right], [r[i] for i in bottom], [r[i] for i in left],
             lambda u, v: V((0.0, sign * depth * (1 - (2 * u - 1) ** 2) ** 0.6 * (1 - (2 * v - 1) ** 2) ** 0.3, 0.0)))


def build_sole():
    """Thin sole plate: flat forefoot with a lip, rising through the arch onto the heel block."""
    mb = MB()
    rh, rt = 0.036, 0.034
    mids = [Y_HEEL - rh, 0.030, HB_FRONT, -0.012, -0.028, -0.044, -0.060, -0.075, Y_BALL, -0.108, -0.130,
            -0.150, -0.168, -0.184, Y_TOE + rt]
    st = _ends(Y_HEEL, rh, 2.2, Y_TOE, rt, 2.8, mids, sole_w)
    ch, lip = 0.0022, SOLE_LIP
    rings = []
    for y, w in st:
        b, T = sole_bot(y), sole_t(y)
        k1, k2 = 0.52 * T, 0.60 * T
        r = [(-(w - ch), b), (0.0, b), (w - ch, b), (w, b + ch), (w, b + k1), (w + lip, b + k2),
             (w + lip, b + T - ch), (w + lip - ch, b + T), (0.0, b + T), (-(w + lip - ch), b + T),
             (-(w + lip), b + T - ch), (-(w + lip), b + k2), (-w, b + k1), (-w, b + ch)]
        rings.append(mb.add([V((x, y, z)) for x, z in r]))
    for a, b in zip(rings, rings[1:]):
        mb.strip(a, b, closed=True)
    caps = ((9, 8, 7), (7, 6, 5, 4, 3, 2), (0, 1, 2), (9, 10, 11, 12, 13, 0))
    _cap(mb, rings[0], *caps, 1.0, Y_HEEL - st[0][0])
    _cap(mb, rings[-1], *caps, -1.0, st[-1][0] - Y_TOE)
    return mb


def build_heel():
    """Tall block heel under the back of the sole plate, flared top lip, sloped front face."""
    mb = MB()
    rh = 0.032
    yb = Y_HEEL - 0.004

    def hw(y):
        return sole_w(y) - 0.0045
    mids = [yb - rh, 0.030, 0.016, HB_FRONT]
    st = _ends(yb, rh, 2.3, HB_FRONT - 0.001, 0.0, 2.0, mids, hw)[:-4]     # flat front: no front rounding
    H, ch, fl = HB_H, 0.002, 0.0025
    rings = []
    for i, (y, w) in enumerate(st):
        r = [(-(w - ch), 0.0), (0.0, 0.0), (w - ch, 0.0), (w, ch), (w, H - 0.007), (w + fl, H - 0.0045),
             (w + fl, H), (0.0, H), (-(w + fl), H), (-(w + fl), H - 0.0045), (-w, H - 0.007), (-w, ch)]
        slope = 0.006 if i == len(st) - 1 else 0.0              # front face leans back at the bottom
        rings.append(mb.add([V((x, y + slope * (1 - z / H), z)) for x, z in r]))
    for a, b in zip(rings, rings[1:]):
        mb.strip(a, b, closed=True)
    caps = ((8, 7, 6), (6, 5, 4, 3, 2), (0, 1, 2), (8, 9, 10, 11, 0))
    _cap(mb, rings[0], *caps, 1.0, yb - st[0][0])
    _cap(mb, rings[-1], *caps, -1.0, 0.0)
    return mb


# ================================================================== BOOT UPPER (left, local frame)
NS, PW, PR = 24, 8, 4            # shaft columns / foot patch width / foot patch rows
JP0 = 3 * NS // 4 - PW // 2      # patch columns JP0 .. JP1, centred on the front (-y)
JP1 = JP0 + PW
NF = 2 * (PW + PR)               # foot ring vertex count

# shaft radii vs z: side (x), back (+y), front (-y); front superellipse power (boxy where the foot joins)
_RS = [(0.040, 0.0470), (0.070, 0.0500), (0.095, 0.0520)]
_RB = [(0.040, 0.0620), (0.070, 0.0610), (0.090, 0.0600), (0.110, 0.0560)]
_RF = [(0.040, 0.0500), (0.100, 0.0520)]
_PF = [(0.040, 4.0), (0.090, 3.0), (0.125, 2.2)]
COLLAR_EASE = (0.0125, 0.0160, 0.0110)


def rad_s(z):
    return max(_interp(_RS, z), _bare_r(z) + COLLAR_EASE[0])


def rad_b(z):
    return max(_interp(_RB, z), _bare_r(z) + COLLAR_EASE[1])


def rad_f(z):
    return max(_interp(_RF, z), _bare_r(z) + COLLAR_EASE[2])


# full shaft rings below the collar: (z at the back, rise at the front)
SHAFT_ROWS = [(0.1030, 0.017), (0.0920, 0.024)]
# rolled collar: (radial offset, dz from the collar line), outside-bottom -> over the rim -> inner lip
CUFF = [(0.0012, -0.0085), (0.0018, -0.0030), (0.0009, 0.0015), (-0.0012, 0.0025), (-0.0030, -0.0030),
        (-0.0030, -0.0130)]

# foot cross-sections, ankle patch (0) -> toe (10):
#   y of the sole line, (y, z) of the top centre line, max half width, half width at the sole, corner angle
FOOT = [
    (-0.050, -0.054, 0.1170, 0.0500, 0.0490, 44.0),
    (-0.066, -0.070, 0.1090, 0.0530, 0.0520, 46.0),
    (-0.085, -0.088, 0.0990, 0.0550, 0.0540, 48.0),
    (-0.105, -0.107, 0.0910, 0.0565, 0.0555, 50.0),
    (-0.125, -0.127, 0.0830, 0.0575, 0.0565, 52.0),
    (-0.144, -0.146, 0.0770, 0.0575, 0.0568, 54.0),
    (-0.162, -0.164, 0.0730, 0.0570, 0.0562, 56.0),
    (-0.179, -0.181, 0.0690, 0.0555, 0.0548, 58.0),
    (-0.194, -0.196, 0.0650, 0.0525, 0.0518, 60.0),
    (-0.206, -0.208, 0.0600, 0.0485, 0.0478, 62.0),
    (-0.215, -0.216, 0.0540, 0.0430, 0.0425, 64.0),
]
TOE_STEP = 0.0030                 # toe cap leather sits this far proud -> seam step across the toe box
FOOT_US = [(1, 0), (2, 0), (3, 0), (4, 0), (5, 0), (5.55, 0), (5.66, 1), (6.5, 1), (7.5, 1), (8.5, 1),
           (9.3, 1), (10, 1)]
TOE_BULGE = 0.0105


def shaft_pt(zb, arch, j, dr=0.0):
    psi = TAU * j / NS
    c, s = math.cos(psi), math.sin(psi)
    z = zb + arch * _front_w(psi)
    sx, sy = shaft_axis(z)
    rs = rad_s(z) + dr
    if s >= 0:
        return V((sx + rs * c, sy + (rad_b(z) + dr) * s, z))
    e = 2.0 / _interp(_PF, z)
    return V((sx + rs * math.copysign(abs(c) ** e, c), sy - (rad_f(z) + dr) * abs(s) ** e, z))


def dome_ring(u, step=0):
    yb, yt, zt, W, Wb, phc = _catmull(FOOT, u)
    zb = zb_up(yb)
    phc = math.radians(phc)
    if step:
        W += TOE_STEP
        Wb += TOE_STEP * 0.6
        zt += TOE_STEP
    e = 2.0 / 3.4

    def pt(phi):
        c, sn = math.cos(phi), math.sin(phi)
        h = abs(sn) ** e
        xl = math.copysign(abs(c) ** e * W - (W - Wb) * (1 - h) ** 3, c)
        return V((xl, yb + (yt - yb) * h, zb + (zt - zb) * h))
    pts = [pt(math.pi - phc - (math.pi - 2 * phc) * q / PW) for q in range(PW + 1)]       # top, inner -> outer
    pts += [pt(phc * (1 - k / PR)) for k in range(1, PR + 1)]                             # outer side, down
    pts += [V((Wb * (1 - 2 * k / PW), yb, zb)) for k in range(1, PW + 1)]                 # sole line, outer -> inner
    pts += [pt(math.pi - phc * k / PR) for k in range(1, PR)]                             # inner side, up
    return pts


def _heel_pt(j, frac):
    """Point of the heel rows below the last full shaft ring: frac 0 (ring) -> 1 (sole line)."""
    z0 = SHAFT_ROWS[-1][0] + SHAFT_ROWS[-1][1] * _front_w(TAU * j / NS)
    z = _lerp(z0, 0.05, frac)
    for _ in range(3):
        p = shaft_pt(z, 0.0, j)
        z = _lerp(z0, zb_up(p.y), frac)
    return shaft_pt(z, 0.0, j)


def build_upper():
    mb = MB()
    idx = {}
    itop = len(SHAFT_ROWS) - 1
    cuff = []
    for dr, dz in reversed(CUFF):
        cuff.append(mb.add([shaft_pt(COLLAR_Z + dz, COLLAR_ARCH, j, dr) for j in range(NS)]))
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
    for k in range(1, PR + 1):
        i = itop + k
        ids = mb.add([_heel_pt(j, k / PR) for j in heel_cols])
        for j, v in zip(heel_cols, ids):
            idx[(i, j)] = v
    for i in range(itop, nr):
        for a, b in zip(heel_cols, heel_cols[1:]):
            mb.quad(idx[(i, a)], idx[(i, b)], idx[(i + 1, b)], idx[(i + 1, a)])
    pa, pb = mb.v[idx[(nr, JP0)]], mb.v[idx[(nr, JP1)]]
    for j in range(JP0 + 1, JP1):
        q = pa.lerp(pb, (j - JP0) / PW)
        q.z = zb_up(q.y)
        idx[(nr, j)] = mb.add([q])[0]
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
        fall = max(0.0, 1.0 - u / 2.5) ** 2
        ring = mb.add([p + c * fall for p, c in zip(dome_ring(u, step), corr)])
        mb.strip(prev, ring, closed=True)
        prev = ring
    top = prev[0:PW + 1]
    right = prev[PW:PW + PR + 1]
    bottom = list(reversed(prev[PW + PR:2 * PW + PR + 1]))
    left = [prev[0]] + list(reversed(prev[2 * PW + PR:NF]))

    def bulge(u, v):      # squared toe: flat-ish front, rounded only near the edges
        return V((0.0, -TOE_BULGE * (1 - abs(2 * u - 1) ** 3.0) ** 0.5 * (1 - abs(2 * v - 1) ** 2.5) ** 0.5, 0.0))
    mb.coons(top, right, bottom, left, bulge)

    def col(j):
        return idx[(nr, j % NS)]
    mb.coons([col(j) for j in range(JP0, JP1 + 1)],
             [col(JP1 + k) for k in range(0, PR + 1)],
             [col(JP0 - PR - k) for k in range(0, PW + 1)],
             [col(JP0 - k) for k in range(0, PR + 1)])
    assert (JP0 - PR - PW) % NS == (JP1 + PR) % NS
    return mb


# ================================================================== surface helpers
def _bvh(mbs):
    v, f = [], []
    for m in mbs:
        o = len(v)
        v += [tuple(p) for p in m.v]
        f += [tuple(i + o for i in fc) for fc in m.f]
    return BVHTree.FromPolygons(v, f)


def _core(p):
    """Foot / shaft 'core' point used to aim rays at the boot from outside."""
    w = _sstep((p.z - 0.085) / 0.050)
    sx, sy = shaft_axis(p.z)
    return V((0.0, p.y, 0.032)).lerp(V((sx, sy, p.z)), w)


def _surf_normal(bvh, p, d, r=0.008):
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


def band(mb, bvh, frames, width, thick, closed=False, clear=0.0007, bevel=0.0):
    """Strap / panel with thickness that conforms to the surface across its width as well as along it:
    each section has 3 bottom verts (edge, middle, edge), each lifted just clear of the surface (checked
    with extra rays between them) and smoothed along the strap, so edges never cut into the shell.
    bevel > 0 chamfers the top edges (8-vertex sections) for a crisp hard-surface read.
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
        return (0.04 - hit[3]) if hit[0] is not None else 0.0
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
        if bevel:
            rings.append(mb.add([bl, bm, br, br + nn * (thick - bevel), br - sx * bevel + nn * thick,
                                 bm + nn * thick, bl + sx * bevel + nn * thick, bl + nn * (thick - bevel)]))
        else:
            rings.append(mb.add([bl, bm, br, br + nn * thick, bm + nn * thick, bl + nn * thick]))
        out.append((bm + nn * thick, t, sx, nn))
    for a, c in zip(rings, rings[1:] + ([rings[0]] if closed else [])):
        mb.strip(a, c, closed=True)
    if not closed:
        for r in (rings[0], rings[-1]):
            if bevel:
                mb.quad(r[7], r[0], r[1], r[6])
                mb.quad(r[1], r[2], r[5], r[6])
                mb.quad(r[2], r[3], r[4], r[5])
            else:
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


# ================================================================== HEEL COUNTER
CT_TOP = 0.097            # top of the heel counter at the back (just under the collar roll)
CT_SPAN = math.radians(78.0)
NCT = 18


def build_counter(bvh):
    """Raised panel wrapping the back of the heel: its top edge sweeps down to the sole at both front ends
    (rounded side outline), with a 5 mm raised rim along that edge."""
    mb = MB()

    def cast(z, psi):
        sx, sy = shaft_axis(z)
        u = V((math.cos(psi), math.sin(psi), 0.0))
        o = V((sx, sy, z))
        hit = bvh.ray_cast(o + u * 0.3, -u)
        p = hit[0] if hit[0] is not None else o + u * 0.06
        return p, _surf_normal(bvh, p, u, r=0.006)
    grid = []
    for c in range(NCT + 1):
        t = -1.0 + 2.0 * c / NCT
        psi = math.pi / 2 + t * CT_SPAN
        p0, _ = cast(0.06, psi)
        zb = zb_up(p0.y) + 0.0015
        h = zb + max((CT_TOP - zb) * (1 - abs(t) ** 1.9) ** 0.65, 0.016)
        span = h - zb
        rim = 0.0070 * min(1.0, span / 0.03)
        dome = 0.0030 * (1 - abs(t) ** 2)                    # domed at the back of the heel
        rows = [(zb, 0.0045), (_lerp(zb, h - rim - 0.002, 0.40), 0.0050 + dome),
                (_lerp(zb, h - rim - 0.002, 0.75), 0.0048 + dome * 0.8),
                (h - rim - 0.0015, 0.0042), (h - rim, 0.0078), (h, 0.0078), (h, 0.0004)]
        col = []
        for z, off in rows:
            p, n = cast(z, psi)
            col.append(p + n * off)
        grid.append(mb.add(col))
    for a, b in zip(grid, grid[1:]):
        mb.strip(a, b)
    # end walls: drop the outer face of the first / last column onto the surface (row 5 lands on row 6)
    for c in (0, NCT):
        t = -1.0 + 2.0 * c / NCT
        psi = math.pi / 2 + t * CT_SPAN
        wall = []
        for vi in grid[c][:5]:
            q, n = cast(mb.v[vi].z, psi)
            wall.append(q + n * 0.0004)
        mb.strip(grid[c][:6], mb.add(wall) + [grid[c][6]])
    return mb


# ================================================================== STRAPS / CUFF / HARDWARE
# two wide flat straps over the instep, diagonal (forward on top, sweeping back-down on the outer side to
# the heel counter's front edge, straight down the inner side to the welt); the rear one lies on top
STRAP_A = [(-0.061, -0.128, 0.020), (-0.057, -0.123, 0.050), (-0.040, -0.114, 0.080), (-0.012, -0.104, 0.094),
           (0.018, -0.092, 0.092), (0.042, -0.072, 0.082), (0.055, -0.046, 0.066), (0.057, -0.020, 0.056),
           (0.054, -0.002, 0.050)]
STRAP_B = [(-0.057, -0.094, 0.022), (-0.053, -0.090, 0.055), (-0.040, -0.084, 0.088), (-0.014, -0.077, 0.106),
           (0.016, -0.067, 0.108), (0.040, -0.047, 0.098), (0.054, -0.019, 0.084), (0.056, 0.008, 0.074),
           (0.052, 0.030, 0.068)]


def build_straps(upper, counter, side_rows):
    """-> (straps MB [main], hardware MB [rigid]) -- all in the boot's local frame."""
    sm, hw = MB(), MB()
    bvh = _bvh([upper, counter])
    band(sm, bvh, path_frames(bvh, STRAP_A, subdiv=2), 0.040, 0.0058, bevel=0.0012)
    bvh = _bvh([upper, counter, sm])
    band(sm, bvh, path_frames(bvh, STRAP_B, subdiv=2), 0.046, 0.0062, bevel=0.0012)
    # wrapped ankle cuff: closed crisp band on the slim lower leg, square buckle on the outer side
    z0, z1 = CUFF_Z
    d_in, d_out, ch = 0.0015, 0.0064, 0.0013
    prof = [(z0, d_in), (z0, d_out - ch), (z0 + ch, d_out), (z1 - ch, d_out), (z1, d_out - ch), (z1, d_in)]
    rows = [sm.add([tube_pt(z, TAU * k / NS, d) for k in range(NS)]) for z, d in prof]
    for r0, r1 in zip(rows, rows[1:] + [rows[0]]):
        sm.strip(r0, r1, closed=True)
    zc = 0.5 * (z0 + z1)
    o = tube_pt(zc, 0.0, d_out)
    buckle(hw, o, V((0.0, 1.0, 0.0)), V((0.0, 0.0, 1.0)), V((1.0, 0.0, 0.0)), lt=0.0195, lb=0.0215,
           bar=0.0062, h=0.0052)
    # vertical gaiter strip on the outer side, from the cuff up under the trouser hem
    tm = MB()
    prev = None
    for row in side_rows[2:9]:
        cur = tm.add(row)
        if prev:
            tm.strip(prev, cur, closed=True)
        prev = cur
    tb = _bvh([tm])
    fr = []
    for z in (0.196, 0.212, 0.228, 0.244, 0.258, 0.270):
        p = tube_pt(z, 0.0, 0.0)
        hit = tb.ray_cast(p + V((0.05, 0.0, 0.0)), V((-1.0, 0.0, 0.0)))
        fr.append((hit[0] if hit[0] is not None else p, V((1.0, 0.0, 0.0))))
    band(sm, tb, fr, 0.022, 0.0026, bevel=0.0007)
    return sm, hw


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
    counter = build_counter(_bvh([upper]))
    straps, hw = build_straps(upper, counter, lower_rows_local(1))
    left = [(upper.obj("boot_upper_L", "main", sharp_angle=50), 50),
            (counter.obj("boot_counter_L", "main", sharp_angle=40), 40),
            (build_sole().obj("boot_sole_L", "rigid", sharp_angle=30), 30),
            (build_heel().obj("boot_heel_L", "rigid", sharp_angle=30), 30),
            (straps.obj("boot_straps_L", "main", sharp_angle=40), 40),
            (hw.obj("boot_hardware_L", "rigid", sharp_angle=30), 30)]
    for o, sharp in left:
        _to_world_ob(o, sharp)
    objs += [o for o, _ in left] + [lib.mirror_copy(o) for o, _ in left]
    return objs
