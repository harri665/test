"""Legs (round 2): baggy bloused trousers, chunky splayed ankle boots with lugged soles, straps, buckles.

Objects (build() order)
    trousers                 main     pelvis (40) -> crotch gusset -> 2 thigh tubes (24), down to the knee loops
    trouser_knee_L / _R      support  the 3 tight knee loops (own object, shares its boundary rings)
    trouser_blouse_L / _R    main     bloused lower leg: 3 sagging fold rolls, roll-over hem, tucked into the boot
    boot_upper_L / _R        main     cuff + shaft + foot (foot extruded from a front patch of the shaft), toe cap
    boot_sole_L / _R         rigid    lugged outsole + heavy welt, heel block, arch notch
    boot_straps_L / _R       main     tongue/lace panel, instep strap, wide cuff strap + tail
    boot_hardware_L / _R     rigid    2 square frame buckles + prongs, keeper, eyelets, rivet

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
SY = 0.020               # boot shaft axis this far behind the ankle (ref SIDE: shaft spans heel-0.13 .. heel)
SOLE_T = 0.045           # sole + welt thickness (checklist: thick lugged sole about 0.045)
ZB = SOLE_T - 0.003      # bottom of the upper, sunk into the sole
BLOUSE_DX = -0.010       # blouse centre vs boot axis (ref FRONT: blouse x 0.064-0.256 at z 0.32 -> centre 0.16)
BLOUSE_DY = 0.010        # ... and hangs back a little (ref SIDE: blouse sits ~0.03 behind our boot axis)
CUFF_STRAP_Z = (0.153, 0.201)   # wide strap round the boot top (ref FRONT: strap z 0.15-0.205)


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


def shaft_c(z):
    """World xy of the boot shaft axis."""
    w = to_world((0.0, SY, z))
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

# bloused lower leg, top -> bottom: (z, radius about lower_c, sag weight).  3 fold rolls (crest / crease
# pairs) sag toward the boot, the last one rolls over the boot top.  Radii respect trouser_blouse.
BLOUSE_ROWS = [
    (0.474, 0.0925, 0.00), (0.441, 0.0970, 0.03), (0.413, 0.0990, 0.10),
    (0.406, 0.0925, 0.20), (0.393, 0.0985, 0.30), (0.381, 0.0980, 0.40),                    # roll 1
    (0.369, 0.0915, 0.55), (0.357, 0.0990, 0.65), (0.345, 0.0985, 0.75),                    # roll 2
    (0.333, 0.0920, 0.85), (0.321, 0.1000, 0.95), (0.310, 0.0990, 1.00), (0.302, 0.0940, 1.00),   # roll 3
    (0.2975, 0.0850, 1.00), (0.2970, 0.0760, 0.90),                                          # underside
]
# then gathered tight round the shin (reads as the boot shaft in the reference) and tucked over the rim
# into the boot: (z, offset from the boot shaft surface)
# (z, offset from the boot shaft surface, sag weight)
TUCK_ROWS = [(0.2885, 0.0055, 0.9), (0.2730, 0.0040, 0.3), (0.2540, 0.0040, 0.0), (0.2522, 0.0005, 0.0),
             (0.2505, -0.0050, 0.0), (0.2440, -0.0085, 0.0), (0.2320, -0.0090, 0.0)]
SAG = 0.011                     # how far the fold rolls droop (most at the front)


def _creases(th, z, side):
    """Vertical creases (radial metres), strongest where the blouse gathers above the rolls."""
    u = th / TAU
    v = _sstep((0.50 - z) / 0.12)
    amp = 0.0055 * v * (1.0 - 0.45 * _sstep((0.40 - z) / 0.10)) + 0.0015
    return lib.fold_wave(u, (z - 0.25) / 0.30, amp=amp, count=4.0, seed=0.37 if side > 0 else 0.81, grow=0.0)


def _sag(th, side):
    """How much the rolls droop at angle th (1 = most): front-outer on the left leg, front-inner on the right."""
    return 0.55 + 0.45 * math.cos(th - math.radians(285.0 if side > 0 else 255.0))


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
    zz = z - dip * ((1 - math.cos(th)) * 0.5) ** 1.5 - sag * SAG * _sag(th, side)
    return V((x, y, zz))


def boot_rel_pt(z, d, th):
    """Point at offset d outside the boot shaft surface (world angle th) - used for the tucked hem."""
    a = th - PHI
    rx = _interp(_RS, z) + d
    ry = (_interp(_RB, z) if math.sin(a) > 0 else _interp(_RF, z)) + d
    return to_world((rx * math.cos(a), SY + ry * math.sin(a), z))


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


def _leg_ring(z, side, k=None, r=None, sag=0.0):
    r = _thigh_r(z) if r is None else r
    pts = []
    for j in range(NL):
        p = trouser_pt(z, r, TAU * j / NL, side, dip=_DIP.get(k, 0.0), seat=_SEAT.get(k, 0.0), sag=sag)
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
    return _lerp(_thigh_r(KNEE_ROWS[0]), BLOUSE_ROWS[0][1] - 0.006, (KNEE_ROWS[0] - z) / (KNEE_ROWS[0] - KNEE_ROWS[-1]))


def build_knee_and_blouse(side, top_ring):
    """Knee band (support) + bloused lower leg (main) for one side.  Both share their boundary ring."""
    knee = [top_ring] + [_leg_ring(z, side, r=_knee_r(z)) for z in KNEE_ROWS[1:]]
    sfx = "L" if side > 0 else "R"
    knee_ob = lib.lofted("trouser_knee_" + sfx, knee, topo="support", sharp_angle=75)
    rings = [knee[-1]]
    for z, r, sag in BLOUSE_ROWS:
        rings.append(_leg_ring(z, side, r=r, sag=sag))
    for z, d, sw in TUCK_ROWS:
        ring = []
        for j in range(NL):
            th = TAU * j / NL
            p = boot_rel_pt(z, d, th)
            p.z -= sw * SAG * _sag(th, side)
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
_RS = [(ZB, 0.0550), (0.066, 0.0565), (0.090, 0.0580), (0.115, 0.0585), (0.140, 0.0585), (0.175, 0.0580),
       (0.210, 0.0572), (0.260, 0.0568)]
_RB = [(ZB, 0.0800), (0.066, 0.0810), (0.090, 0.0785), (0.115, 0.0730), (0.140, 0.0680), (0.175, 0.0650),
       (0.260, 0.0640)]
_RF = [(ZB, 0.0680), (0.260, 0.0680)]
_PF = [(ZB, 4.5), (0.100, 3.4), (0.125, 2.6), (0.150, 2.0)]

# full shaft rings: (z at the back, how far the front arches up over the instep crease)
SHAFT_ROWS = [(0.206, 0.0), (0.189, 0.0), (0.172, 0.004), (0.156, 0.012), (0.141, 0.022), (0.126, 0.034)]
HEEL_Z = [0.103, 0.081, 0.060, ZB]          # PR rows below the last full ring (heel counter)
# rolled cuff on top of the shaft: (radial offset, z), outside-bottom -> over the rim -> inner lip
CUFF = [(0.0018, T - 0.022), (0.0030, T - 0.008), (0.0022, T + 0.0015), (-0.0008, T + 0.0035),
        (-0.0040, T - 0.003), (-0.0040, T - 0.018)]

# foot cross-sections, from the ankle patch (0) to the toe (9):
#   y of the sole line, (y, z) of the top centre line, max half width, half width at the sole, corner angle
FOOT = [
    (-0.048, -0.050, 0.160, 0.0540, 0.0530, 44.0),
    (-0.066, -0.070, 0.136, 0.0620, 0.0600, 46.0),
    (-0.084, -0.089, 0.118, 0.0690, 0.0670, 48.0),
    (-0.102, -0.107, 0.1085, 0.0750, 0.0730, 50.0),
    (-0.120, -0.124, 0.1050, 0.0790, 0.0770, 52.0),
    (-0.137, -0.141, 0.1040, 0.0820, 0.0790, 53.0),
    (-0.153, -0.157, 0.1020, 0.0830, 0.0800, 54.0),
    (-0.167, -0.170, 0.0990, 0.0815, 0.0785, 55.0),
    (-0.178, -0.180, 0.0935, 0.0770, 0.0740, 56.0),
    (-0.186, -0.187, 0.0835, 0.0670, 0.0640, 57.0),
]
TOE_STEP = 0.0028                 # toe cap leather sits this far proud -> seam line
FOOT_US = [(1, 0), (2, 0), (3, 0), (4, 0), (4.86, 0), (4.97, 1), (6, 1), (7, 1), (8, 1), (9, 1)]
TOE_BULGE = 0.0085


def shaft_pt(zb, arch, j):
    psi = TAU * j / NS
    c, s = math.cos(psi), math.sin(psi)
    d = abs(math.degrees(psi) - 270.0)
    f = math.cos(math.radians(90.0 * d / 80.0)) ** 2 if d < 80.0 else 0.0
    z = zb + arch * f
    rs = _interp(_RS, z)
    if s >= 0:
        return V((rs * c, SY + _interp(_RB, z) * s, z))
    e = 2.0 / _interp(_PF, z)
    return V((rs * math.copysign(abs(c) ** e, c), SY - _interp(_RF, z) * abs(s) ** e, z))


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
        pts = [shaft_pt(z, 0.0, j) for j in range(NS)]
        cuff.append(mb.add([V((p.x * (1 + dr / _interp(_RS, z)), SY + (p.y - SY) * (1 + dr / _interp(_RB, z)), z))
                            for p in pts]))
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
LUG_Z = 0.025             # top of the tread lugs
NOTCH = 0.017             # arch notch height between heel block and forefoot
LUG_IN = 0.0045           # groove depth between lugs


def sole_w(y):
    """Half width of the sole (top of the welt) along the foot."""
    return _interp([(-0.175, 0.0905), (-0.150, 0.0920), (-0.120, 0.0905), (-0.085, 0.0820), (-0.045, 0.0710),
                    (0.000, 0.0650), (0.040, 0.0640), (0.080, 0.0640)], y)


def _sole_sections():
    """(y, half width, bottom z, lug inset) cross-sections from heel to toe."""
    e = 2.0 / 2.4
    yh, rh = Y_HEEL - 0.036, 0.036
    yt, rt = Y_TOE + 0.036, 0.036
    st = []
    for a in (28, 52, 74):
        a = math.radians(a)
        st.append((yh + rh * math.cos(a) ** e, sole_w(yh) * math.sin(a) ** e, 0.0, 0.0))

    def lugs(y0, y1, n, zb):
        L = (y0 - y1)
        g = 0.006
        lug = (L - (n - 1) * g) / n
        y = y0
        for k in range(n):
            st.append((y, sole_w(y), zb, 0.0))
            st.append((y - lug, sole_w(y - lug), zb, 0.0))
            y -= lug
            if k < n - 1:
                st.append((y - g / 2, sole_w(y - g / 2), zb, LUG_IN))
                y -= g
    lugs(yh, 0.034, 2, 0.0)
    st.append((0.030, sole_w(0.030), NOTCH, 0.0035))
    st.append((-0.024, sole_w(-0.024), NOTCH, 0.0035))
    lugs(-0.032, yt, 4, 0.0)
    for a in (74, 52, 28):
        a = math.radians(a)
        st.append((yt - rt * math.cos(a) ** e, sole_w(yt) * math.sin(a) ** e, 0.0, 0.0))
    return st


def build_sole():
    mb = MB()
    ch, ch2 = 0.003, 0.002
    rings = []
    for y, w, zb, inset in _sole_sections():
        wl = w - 0.002 - inset
        r = [(-(wl - ch), zb), (0.0, zb), (wl - ch, zb)]
        r += [(wl, zb + ch), (wl, LUG_Z), (w - 0.004, LUG_Z + 0.0035), (w, LUG_Z + 0.007), (w, SOLE_T - ch2)]
        r += [(w - ch2, SOLE_T), (0.0, SOLE_T), (-(w - ch2), SOLE_T)]
        r += [(-w, SOLE_T - ch2), (-w, LUG_Z + 0.007), (-(w - 0.004), LUG_Z + 0.0035), (-wl, LUG_Z), (-wl, zb + ch)]
        rings.append(mb.add([V((x, y, z)) for x, z in r]))
    for a, b in zip(rings, rings[1:]):
        mb.strip(a, b, closed=True)

    def cap(r, sign, depth):
        mb.coons([r[10], r[9], r[8]], [r[8], r[7], r[6], r[5], r[4], r[3], r[2]],
                 [r[0], r[1], r[2]], [r[10], r[11], r[12], r[13], r[14], r[15], r[0]],
                 lambda u, v: V((0.0, sign * depth * (1 - (2 * u - 1) ** 2) ** 0.6, 0.0)))
    st = _sole_sections()
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


def ring_on(bvh, c, z, angles, lift):
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
        out.append((loc + n * lift, n))
    return out


def _core(p):
    """Foot / shaft 'core' point used to aim rays at the boot from outside."""
    w = _sstep((p.z - 0.12) / 0.05)
    return V((0.0, p.y, 0.062)).lerp(V((0.0, SY, p.z)), w)


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


def path_cast(bvh, pts, lift, subdiv=3, bridge=1):
    """Lay a local poly line onto the boot: each point is pushed out from the foot's core until it meets
    the outermost surface.  Normals are area-averaged and smoothed along the path (straps never twist),
    heights are max-filtered so a strap bridges over edges instead of stepping down."""
    raw = []
    for p in lib.smooth_path(pts, subdiv):
        p = V(p)
        d = (p - _core(p)).normalized()
        hit = bvh.ray_cast(p + d * 0.12, -d)
        q = hit[0] if hit[0] is not None else p
        raw.append([q, _surf_normal(bvh, q, d)])
    ns = [sum((raw[k][1] for k in range(max(i - 2, 0), min(i + 3, len(raw)))), V()).normalized()
          for i in range(len(raw))]
    hs = []
    for i, (q, _n) in enumerate(raw):
        h = 0.0
        for k in range(max(i - bridge, 0), min(i + bridge + 1, len(raw))):
            h = max(h, (raw[k][0] - q).dot(ns[i]))
        hs.append(h)
    return [(q + n * (h + lift), n) for (q, _), n, h in zip(raw, ns, hs)]


def path_on(bvh, pts, lift, subdiv=3):
    """Snap a local poly line onto the nearest surface; -> [(point, normal)] with smoothed normals."""
    raw = []
    for p in lib.smooth_path(pts, subdiv):
        loc, n, _i, _d = bvh.find_nearest(V(p))
        if n.dot(loc - V((0.0, loc.y, 0.065))) < 0:  # orient away from the foot's core
            n = -n
        raw.append((loc, n))
    ns = []
    for i in range(len(raw)):
        n = sum((raw[k][1] for k in range(max(i - 1, 0), min(i + 2, len(raw)))), V())
        ns.append(n.normalized())
    return [(p + n * lift, n) for (p, _), n in zip(raw, ns)]


def strap(name, frames, width, thick, closed=False):
    return lib.strip_along(name, [p for p, _ in frames], [n for _, n in frames], width=width, thickness=thick,
                           sink=0.0004, closed=closed, profile="flat")


def frame_at(frames, i, closed=False):
    n = len(frames)
    a = frames[(i - 1) % n if closed else max(i - 1, 0)][0]
    b = frames[(i + 1) % n if closed else min(i + 1, n - 1)][0]
    p, nrm = frames[i]
    t = (b - a).normalized()
    nrm = (nrm - t * nrm.dot(t)).normalized()
    return p, t, nrm.cross(t).normalized(), nrm


def buckle(mb, o, t, b, n, lt, lb, bar=0.0055, h=0.0050):
    """Square frame buckle + prong.  o = centre on the strap's outer face; t along / b across / n out."""
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
    w2, z0, z1 = 0.0019, n1 - 0.0012, n1 + 0.0018
    a = mb.add([P(-lt + bar * 0.2, -w2, z0), P(-lt + bar * 0.2, w2, z0), P(-lt + bar * 0.2, w2, z1),
                P(-lt + bar * 0.2, -w2, z1)])
    m = mb.add([P(0.0, -w2, z0 - 0.0008), P(0.0, w2, z0 - 0.0008), P(0.0, w2, z1 - 0.0008), P(0.0, -w2, z1 - 0.0008)])
    c = mb.add([P(lt + 0.0015, -w2 * 0.7, z0 + 0.0004), P(lt + 0.0015, w2 * 0.7, z0 + 0.0004),
                P(lt + 0.0015, w2 * 0.7, z1 - 0.0004), P(lt + 0.0015, -w2 * 0.7, z1 - 0.0004)])
    mb.strip(a, m, closed=True)
    mb.strip(m, c, closed=True)
    mb.quad(*a)
    mb.quad(*c)


def keeper(mb, o, t, b, n, wb, top, depth=0.005, lt=0.0045, th=0.0022):
    """Strap keeper loop: a U band over the strap + tail, legs sunk into the strap below."""
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


def _mb_from(ob):
    m = MB()
    m.add([v.co.copy() for v in ob.data.vertices])
    m.f += [tuple(p.vertices) for p in ob.data.polygons]
    return m


def build_straps(upper):
    """-> (strap objects [main], hardware MB [rigid])  -- all in the boot's local frame."""
    hw = MB()
    objs = []
    # ---- tongue / lace panel down the instep, from under the cuff strap to the toe-cap seam
    bvh = _bvh([upper])
    tongue_pts = [(0.0, -0.050, 0.158), (0.0, -0.060, 0.145), (0.0, -0.072, 0.131), (0.0, -0.090, 0.118),
                  (0.0, -0.108, 0.109), (0.0, -0.128, 0.104)]
    tongue = path_cast(bvh, tongue_pts, 0.0008, subdiv=2, bridge=0)
    objs.append(strap("tongue", tongue, 0.046, 0.0032))
    tm = _mb_from(objs[-1])
    # lace bars across the tongue
    bvh_t = _bvh([upper, tm])
    for i in (2, 4, 9):
        p, t, b, n = frame_at(tongue, i)
        bar = path_cast(bvh_t, [p - b * 0.027, p, p + b * 0.027], 0.0006, subdiv=2, bridge=0)
        objs.append(strap("lace", bar, 0.006, 0.0022))
    # ---- instep strap: from the outer ankle (under the cuff strap) diagonally over the instep to the inner welt
    bvh2 = _bvh([upper, tm])
    ip = [(0.064, -0.004, 0.152), (0.062, -0.030, 0.140), (0.050, -0.058, 0.128), (0.026, -0.080, 0.119),
          (-0.004, -0.094, 0.112), (-0.034, -0.104, 0.099), (-0.058, -0.110, 0.080), (-0.072, -0.114, 0.060),
          (-0.076, -0.116, 0.044)]
    inst = path_cast(bvh2, ip, 0.0040, subdiv=3, bridge=1)
    objs.append(strap("instep_strap", inst, 0.038, 0.0042))
    p, t, b, n = frame_at(inst, 3)
    buckle(hw, p + n * 0.0042, t, b, n, lt=0.019, lb=0.025)
    # ---- wide cuff strap round the boot top (+ tail lying on it, punched eyelets)
    smb = _mb_from(objs[-1])
    bvh3 = _bvh([upper, tm, smb])
    zs = 0.5 * (CUFF_STRAP_Z[0] + CUFF_STRAP_Z[1])
    ws = CUFF_STRAP_Z[1] - CUFF_STRAP_Z[0]
    angs = [TAU * k / NS + 1e-4 for k in range(NS)]
    ring = ring_on(bvh3, (0.0, SY), zs, angs, 0.0022)
    objs.append(strap("cuff_strap", ring, ws, 0.0048, closed=True))
    a_b = math.radians(-68.0)                    # buckle on the front-outer side
    tail_a = [a_b - math.radians(d) for d in (13, 20, 28, 36, 44, 52, 60, 68)]
    tail = ring_on(bvh3, (0.0, SY), zs, tail_a, 0.0022 + 0.0050)
    objs.append(strap("cuff_tail", tail, ws * 0.86, 0.0030))
    p, t, b, n = frame_at(ring_on(bvh3, (0.0, SY), zs, [a_b - 0.06, a_b, a_b + 0.06], 0.0022), 1)
    buckle(hw, p + n * 0.0072, -t, b, n, lt=0.021, lb=ws * 0.5 + 0.006)
    for k in (3, 4, 5):
        pk, nk = tail[k]
        hb = lib.rivet("eyelet", radius=0.0038, height=0.0012, n=8, loc=pk + nk * 0.0028, normal=nk)
        objs.append(hb)
    pk, nk = ring_on(bvh3, (0.0, SY), zs, [math.radians(20.0)], 0.0022 + 0.0046)[0]
    objs.append(lib.rivet("rivet", radius=0.0050, height=0.0022, n=8, loc=pk, normal=nk))
    p, t, b, n = frame_at(tail, 1)
    keeper(hw, p + n * -0.002, t, b, n, wb=ws * 0.5 + 0.0025, top=0.0095, depth=0.004)
    return objs, hw


# ================================================================== build
def _to_world_ob(ob):
    for v in ob.data.vertices:
        v.co = to_world(v.co)
    ob.data.update()
    lib.finalize(ob, sharp_angle=ob.get("sharp", 40.0))
    return ob


def build():
    tr, bottoms = build_trousers()
    objs = [tr]
    for side in (1, -1):
        objs += build_knee_and_blouse(side, bottoms[side])
    upper = build_upper()
    strap_objs, hw = build_straps(upper)
    soft = []
    for o in strap_objs:
        if o.get("topo") == "rigid":                 # rivets / eyelets -> hardware mesh
            hw_o = _mb_from(o)
            off = len(hw.v)
            hw.v += hw_o.v
            hw.f += [tuple(i + off for i in f) for f in hw_o.f]
            m = o.data
            bpy.data.objects.remove(o)
            bpy.data.meshes.remove(m)
        else:
            soft.append(o)
    strap_objs = soft
    straps = lib.join(strap_objs, "boot_straps_L", topo="main")
    left = [upper.obj("boot_upper_L", "main", sharp_angle=50),
            build_sole().obj("boot_sole_L", "rigid", sharp_angle=30),
            straps,
            hw.obj("boot_hardware_L", "rigid", sharp_angle=30)]
    for o in left:
        _to_world_ob(o)
    objs += left + [lib.mirror_copy(o) for o in left]
    return objs
