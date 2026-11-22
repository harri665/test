"""Rigid hard-surface accessories, round 5 (match the user's FBX game model, docs/FBX_MATCH.md I1 / I7,
docs/FBX_COMPARISON.md C7, C8, W1-W4).

    shoulder_board_L/R  C7  flat rectangular epaulette board lying on the shoulder, arched gently
                            front-to-back, its outer end overhanging the upper arm; two stacked plates
                            (base board + smaller top plate) and a plain strap along its crown; crisp
                            chamfered edges, no buttons / emblems
    pauldron_L/R        C8  plain upper-arm pad: a curved plate over the outside of the upper arm, its
                            top tucked under the board, its bottom edge a shallow V with a raised,
                            bevelled lip band that flares out; no emblem, no rivets
    belt                W1  thin continuous band z 1.195-1.245 (hides the coat / skirt seam all round),
                            outer face bare + 0.040; two round buttons at the back left / right of centre.
                            At the front it passes under the coat's wrap flap (I2, >= bare + 0.048)
    belt_tab_L/R        W1  leather side tab at each front side (just outside the flap edges) with a
                            rectangular loop buckle and a keeper
    belt_strap_L/R      W4  long thin strap hanging from each side tab down the hip, lying on the skirt
    (W2 / W3: the pouches, the canister and the hoses are gone - the FBX has none)

Layering:
    coat / skirt inside the belt zone <= bare + 0.036 -> belt inner wall +0.0375, face +0.040
    -> tab +0.0403 .. +0.0440 -> keeper / buckle frame on the tab (all < +0.048: nothing pokes
    through the coat's wrap flap, and the tabs sit outside its edges anyway)
    skirt (skirt.outer_radius when importable) -> hanging strap +0.003
    upper sleeve (<= arm radius + 0.022 above z 1.36) -> pad inner surface >= +0.011 outside that
    coat shoulder -> board underside >= ~0.010 above it

Belt angle convention in this file: theta = 0 at FRONT centre (-Y), +90 deg at the wearer's LEFT
(+X), 180 deg at the back.  skirt.outer_radius uses lib.ring_z angles (0 = +X, 270 = front), so
a_deg = theta_deg - 90.
"""
import bisect
import math

import bpy  # noqa: F401
from mathutils import Matrix, Vector as V

from .. import lib, spec

C = spec.CLOTH
RAD = math.radians
DEG = math.degrees

# ============================================================================ constants
# --- C7 shoulder board (left).  Underside z(x, y) = BD_Z0 - BD_SLOPE (x - BD_X0) - BD_ARCH (y - BD_YC)^2
#     (the coat shoulder dome measures z 1.587 / 1.579 / 1.562 at x 0.16 / 0.20 / 0.24, y 0)
BD_X0, BD_X1 = 0.165, 0.278       # inner end (beside the collar) .. outer end (overhangs the arm)
BD_Y0, BD_Y1 = -0.090, 0.088      # front .. back edge (FBX: a long board front-to-back, ~9 x 17 cm)
BD_YC = -0.008
BD_Z0 = 1.6045
BD_SLOPE = 0.20                   # falls gently outward along the shoulder line
BD_ARCH = 5.6                     # front-to-back arch (front / back legs ~4-5 cm lower than the crown;
                                  # more would cut into the coat's chest / back yoke at the inner end)
BD_T = 0.0070                     # base board
BD_T2 = 0.0050                    # stacked top plate
BD_INSET = (0.010, 0.009)         # top plate inset from the base board's edges: x ends, y sides
BD_STRAP = (0.022, 0.0026)        # strap along the crown: width, thickness

# --- C8 upper-arm pad (left)
PD_CLR = 0.0105                   # inner surface outside the sleeve envelope (arm radius + 0.022)
PD_OVAL = 0.000                   # a little deeper front / back than out (FBX pads are oval)
PD_FLARE = 0.006                  # bell flare towards the bottom edge
PD_T = 0.0045
PD_LIP = (0.014, 0.0035, 0.004)   # top lip band: width, height over the plate, bevel run
PD_CHEVRON = 0.016                # the lip band dips into a small point straight out (FBX)
PD_FLARE_TOP = 0.004              # the lip end of the plate turns out a little (angled lip)
PD_PHI = (RAD(-96), RAD(102))    # front edge .. back edge, 0 = straight out from the arm
PD_BOT_Z = 1.378                  # bottom edge height straight out ...
PD_BOT_V = 0.010                  # ... rising this much at the front / back ends (shallow V)
PD_TOP = (1.556, 1.500)           # top edge: height straight out (under the board) / at the ends
PD_TOP_ZERO = RAD(108)
PD_GAP = 0.003                    # air between pad top and the board's underside

# --- W1 belt
BELT_Z0, BELT_Z1 = 1.195, 1.245   # I1
BELT_ZM = 0.5 * (BELT_Z0 + BELT_Z1)
BELT_RO = C["belt_offset"]        # 0.040
BELT_RI = 0.0375                  # cloth inside the belt zone stays <= +0.036
BELT_CH = 0.0015
BELT_N = 48
# side tabs: angles along the belt (deg), inner end just outside the coat flap's edge
TAB_TH_IN = (44.0, 38.0)        # inner tip, wearer's left / right (coat flap edges: +39 / -32 deg)
TAB_TH_OUT = 100.0             # inner tip .. outer end
TAB_Z = (1.193, 1.247)
TAB_OFF = (BELT_RO + 0.0003, BELT_RO + 0.0040)
BUCKLE_DTH = 10.0                 # centre of the rectangular loop buckle, deg behind the tip
BUCKLE_W, BUCKLE_H = 0.026, 0.066 # outer size along the belt / height
BUCKLE_BAR, BUCKLE_D = 0.0060, 0.0055
KEEPER_DTH = 22.0
KEEPER_W, KEEPER_T = 0.009, 0.0028
BUTTON_X = 0.090                  # back buttons at x = +-this (FBX back view)
BUTTON_R, BUTTON_H = 0.0190, 0.0065

# --- W4 hanging strap (from the tab's outer end down the hip)
ST_TH = (95.0, 104.0)              # angle at the belt .. at the tip (drifts a little backward)
ST_W, ST_T = 0.016, 0.0030
ST_Z = (1.238, 0.985)             # top (on the tab) .. pointed tip
ST_CLR = 0.0030                   # over the skirt


# ============================================================================ small helpers
def _clamp(x, a=0.0, b=1.0):
    return max(a, min(b, x))


def _lerp(a, b, t):
    return a + (b - a) * t


def _smooth(e0, e1, x):
    t = _clamp((x - e0) / (e1 - e0))
    return t * t * (3 - 2 * t)


def _arc_map(f, n=200):
    """f: t in [0,1] -> Vector.  Returns (g, L): g(v) gives the t at arc-length fraction v, L = length."""
    pts = [f(i / n) for i in range(n + 1)]
    cum = [0.0]
    for a, b in zip(pts, pts[1:]):
        cum.append(cum[-1] + (b - a).length)
    L = cum[-1]

    def g(v):
        target = _clamp(v) * L
        i = max(0, min(n - 1, bisect.bisect_right(cum, target) - 1))
        seg = cum[i + 1] - cum[i]
        return (i + ((target - cum[i]) / seg if seg > 1e-12 else 0.0)) / n
    return g, L


def _torus(rings):
    """Quad loft closed in BOTH directions (ring of rings)."""
    n, m = len(rings[0]), len(rings)
    verts = [V(p) for r in rings for p in r]
    faces = []
    for i in range(m):
        i2 = (i + 1) % m
        for j in range(n):
            j2 = (j + 1) % n
            faces.append((i * n + j, i * n + j2, i2 * n + j2, i2 * n + j))
    return verts, faces


def _slab(P, us, vs, heights, ch=0.0015, flip=False):
    """Closed all-quad plate with real thickness over the INNER surface P(u, v), u, v in 0..1.
    us / vs: station lists including 0 and 1 (put a station at the chamfer inset next to each end);
    a u station may be a callable of v (rows that bend, e.g. a chevron point in a lip band).
    heights(i) -> top height of station row i (along u).  The outermost ring of the top grid is
    lowered by `ch` (crisp chamfered rim), the underside's outermost ring is lifted by `ch`.
    Returns (verts, faces, surf) with surf(u, v, h) -> (point h along the normal, normal)."""
    nu, nv = len(us) - 1, len(vs) - 1

    def frame(u, v):
        e = 1e-3
        du = P(min(u + e, 1.0), v) - P(max(u - e, 0.0), v)
        dv = P(u, min(v + e, 1.0)) - P(u, max(v - e, 0.0))
        n = du.cross(dv).normalized()
        return P(u, v), (-n if flip else n)

    def surf(u, v, h=0.0):
        p, n = frame(u, v)
        return p + n * h, n

    verts, faces = [], []
    T = [[0] * (nv + 1) for _ in range(nu + 1)]
    B = [[0] * (nv + 1) for _ in range(nu + 1)]
    for i in range(nu + 1):
        for j in range(nv + 1):
            p, n = frame(us[i](vs[j]) if callable(us[i]) else us[i], vs[j])
            rim = i in (0, nu) or j in (0, nv)
            T[i][j] = len(verts)
            verts.append(p + n * (heights(i) - (ch if rim else 0.0)))
            B[i][j] = len(verts)
            verts.append(p + n * (ch if rim else 0.0))
    for i in range(nu):
        for j in range(nv):
            faces.append((T[i][j], T[i + 1][j], T[i + 1][j + 1], T[i][j + 1]))
            faces.append((B[i][j], B[i][j + 1], B[i + 1][j + 1], B[i + 1][j]))
    rim = ([(i, 0) for i in range(nu)] + [(nu, j) for j in range(nv)]
           + [(i, nv) for i in range(nu, 0, -1)] + [(0, j) for j in range(nv, 0, -1)])
    for k in range(len(rim)):
        (a, b), (c, d) = rim[k], rim[(k + 1) % len(rim)]
        faces.append((T[a][b], B[a][b], B[c][d], T[c][d]))
    return verts, faces, surf


def _stations(edge, interior, L):
    """0, edge insets (metres, converted with the side length L), interior fractions, mirrored insets, 1."""
    return [0.0] + [e / L for e in edge] + list(interior) + [1.0 - e / L for e in reversed(edge)] + [1.0]


# ============================================================================ body / cloth guides
def _ell(sec, th, off=0.0, z=0.0):
    """Point on an ellipse section (cy, rx, ry) at belt angle th, pushed `off` along the outward
    normal.  Returns (point, normal, tangent towards increasing th)."""
    cy, rx, ry = sec
    s, c = math.sin(th), math.cos(th)
    n = V((s / rx, -c / ry, 0.0)).normalized()
    t = V((rx * c, ry * s, 0.0)).normalized()
    return V((rx * s, cy - ry * c, z)) + n * off, n, t


def _belt_pt(th, off, z):
    """Point `off` outside the BARE torso at height z (belt face = off 0.040)."""
    return _ell(spec.torso_section(z), th, off, z)[0]


def _belt_th(x, off=None):
    """Belt angle on the front half whose point (at the belt face) has the given x."""
    cy, rx, ry = spec.torso_section(BELT_ZM)
    off = BELT_RO if off is None else off
    th = math.asin(_clamp(x / (rx + off), -1.0, 1.0))
    for _ in range(6):
        th += (x - _belt_pt(th, off, BELT_ZM).x) / max((rx + off) * math.cos(th), 1e-4)
    return th


def _back(th):
    """Mirror a front-half belt angle to the back half (same x)."""
    return math.copysign(math.pi, th) - th


# ---- the skirt the hanging strap lies on
try:
    from . import skirt as _skirt_mod
    _OUTER = _skirt_mod.outer_radius
except Exception:          # skirt not importable (being rewritten): fall back on the spec guide
    _OUTER = None


def _guide_r(th, z):
    """Fallback skirt surface: bare torso + coat offset under the belt, linear flare to the hem
    ellipse, one overlap layer (+0.008) on top."""
    z_top, z_hem = BELT_Z1, C["skirt_hem_z"]
    cy, rx, ry = spec.torso_section(z_top)
    rx, ry = rx + C["coat_offset"], ry + C["coat_offset"]
    k = _clamp((z_top - z) / (z_top - z_hem), 0.0, 1.5)
    s, c = math.sin(th), math.cos(th)
    _, bx, by = spec.torso_section(z)
    bare = 1.0 / math.sqrt((s / (bx + C["coat_offset"])) ** 2 + (c / (by + C["coat_offset"])) ** 2)
    hx = _lerp(rx, C["skirt_hem_rx"], k)
    hy = _lerp(ry, C["skirt_hem_ry_back"] if c < 0 else C["skirt_hem_ry"], k)
    return max(bare, 1.0 / math.sqrt((s / hx) ** 2 + (c / hy) ** 2)) + 0.008


def _outer_ok():
    if _OUTER is None:
        return False
    try:
        for a in (0.0, 90.0, 180.0, 270.0):
            for z in (BELT_Z0, 1.05, 0.90):
                r = _OUTER(a, z)
                if not (0.08 < r < 0.6) or r < _guide_r(RAD(a + 90.0), z) - 0.05:
                    return False
        return True
    except Exception:
        return False


_USE_OUTER = _outer_ok()
SKIRT_MARGIN = 0.002       # outer_radius vs the real skirt mesh (faces bulge a little between loops)


def _skirt_r(th, z):
    """Horizontal distance from the Z axis to the outermost skirt cloth at belt angle th, height z."""
    if _USE_OUTER:
        return _OUTER((DEG(abs(th)) - 90.0) % 360.0, z) + SKIRT_MARGIN
    return _guide_r(th, z) + SKIRT_MARGIN


def _skirt_rmax(th, z, half):
    if half <= 0.0:
        return _skirt_r(th, z)
    return max(_skirt_r(th + half * k / 3.0, z) for k in range(-3, 4))


def _skirt_pt(th, off, z, half=0.0):
    """Point `off` outside the skirt along its horizontal normal at (th, z)."""
    e = 0.05
    p = lambda t: V((math.sin(t), -math.cos(t), 0.0)) * _skirt_rmax(t, z, half)
    c = p(th)
    tg = (p(th + e) - p(th - e)).normalized()
    n = tg.cross(V((0, 0, 1))).normalized()
    if n.dot(c) < 0:
        n = -n
    return c + n * off + V((0, 0, z))


# ============================================================================ C7 shoulder board
def _bd_under(x, y):
    return BD_Z0 - BD_SLOPE * (x - BD_X0) - BD_ARCH * (y - BD_YC) ** 2


def _bd_P(u, v):
    x, y = _lerp(BD_X0, BD_X1, u), _lerp(BD_Y0, BD_Y1, v)
    return V((x, y, _bd_under(x, y)))


def _bd_under_z(x, y):
    """Height of the board's underside above (x, y); None outside its footprint."""
    if not (BD_X0 - 0.002 <= x <= BD_X1 + 0.004 and BD_Y0 - 0.004 <= y <= BD_Y1 + 0.004):
        return None
    return _bd_under(_clamp(x, BD_X0, BD_X1), _clamp(y, BD_Y0, BD_Y1))


def _shoulder_board_left():
    parts = []
    Lx, Ly = BD_X1 - BD_X0, BD_Y1 - BD_Y0
    ch = 0.0015
    us = _stations([ch], [0.2, 0.4, 0.6, 0.8], Lx)
    vs = _stations([ch], [k / 8 for k in range(1, 8)], Ly)
    vs1, fs1, surf1 = _slab(_bd_P, us, vs, lambda i: BD_T, ch=ch)
    parts.append(lib.mesh_object("board_base", vs1, fs1, topo="rigid", sharp_angle=30))
    # stacked top plate, inset, resting on the base board (sunk 0.4 mm so the faces never coincide)
    iu0, iu1 = BD_INSET[0] / Lx, 1 - BD_INSET[0] / Lx
    iv0, iv1 = BD_INSET[1] / Ly, 1 - BD_INSET[1] / Ly

    def P2(u, v):
        return surf1(_lerp(iu0, iu1, u), _lerp(iv0, iv1, v), BD_T - 0.0004)[0]
    Lx2, Ly2 = Lx * (iu1 - iu0), Ly * (iv1 - iv0)
    us2 = _stations([ch], [0.25, 0.5, 0.75], Lx2)
    vs2 = _stations([ch], [k / 6 for k in range(1, 6)], Ly2)
    vs_, fs_, surf2 = _slab(P2, us2, vs2, lambda i: BD_T2 + 0.0004, ch=ch)
    parts.append(lib.mesh_object("board_top", vs_, fs_, topo="rigid", sharp_angle=30))
    # plain strap along the crown, from the inner end over the top plate, stopping short of the outer end
    w, t = BD_STRAP
    vc = (BD_YC - BD_Y0 - Ly * iv0) / Ly2               # top-plate v of the crown line
    hv = w / 2 / Ly2

    def P3(u, v):
        return surf2(_lerp(0.0, 0.80, u), vc + (2 * v - 1) * hv, BD_T2 + 0.0004 - 0.0004)[0]
    us3 = _stations([0.001], [0.33, 0.66], Lx2 * 0.8)
    vs3 = _stations([0.001], [0.5], w)
    vs_, fs_, _ = _slab(P3, us3, vs3, lambda i: t + 0.0004, ch=0.001)
    parts.append(lib.mesh_object("board_strap", vs_, fs_, topo="rigid", sharp_angle=30))
    return lib.join(parts, "shoulder_board_L", topo="rigid")


# ============================================================================ C8 upper-arm pad
_A0 = spec.SHOULDER.copy()
_LU = (spec.ELBOW - spec.SHOULDER).length
_AD = (spec.ELBOW - spec.SHOULDER).normalized()             # down the upper arm
_AB = V((0.0, 1.0, 0.0))
_AB = (_AB - _AD * _AB.dot(_AD)).normalized()               # towards the back
_AO = _AD.cross(_AB).normalized()                           # outward (+X, a little up)


def _pd_r(s, phi=0.0):
    sleeve = spec.arm_radius(max(s, 0.0) / _LU) + 0.022
    return sleeve + PD_CLR + PD_OVAL * math.sin(phi) ** 2


def _pd_point(s, phi, extra=0.0):
    """Pad surface: a tube round the upper arm for s >= 0, continued above the shoulder joint
    (s < 0) as a spherical cap round spec.SHOULDER, so its top can tuck under the board."""
    r = _pd_r(s, phi) + extra
    d = _AO * math.cos(phi) + _AB * math.sin(phi)
    if s >= 0.0:
        return _A0 + _AD * s + d * r
    psi = -s / r
    return _A0 + (d * math.cos(psi) - _AD * math.sin(psi)) * r


def _pd_s_range(phi):
    a = min(abs(phi) / max(abs(PD_PHI[0]), abs(PD_PHI[1])), 1.0)
    k = max(0.0, math.cos(min(abs(phi), PD_TOP_ZERO) / PD_TOP_ZERO * math.pi / 2))
    cap0 = PD_TOP[1] + (PD_TOP[0] - PD_TOP[1]) * k

    def above(s):                                        # > 0 while the top edge is too high
        q = _pd_point(s, phi, PD_T)
        cap = cap0
        zu = _bd_under_z(q.x, q.y)
        if zu is not None:
            cap = min(cap, zu - PD_GAP)
        return q.z - cap
    lo, hi = -0.06, 0.12
    for _ in range(40):
        m = 0.5 * (lo + hi)
        if above(m) > 0:
            lo = m
        else:
            hi = m
    s_top = hi
    z_bot = PD_BOT_Z + PD_BOT_V * a ** 1.6               # shallow V: lowest straight out
    s_bot = 0.15
    for _ in range(8):
        q = _pd_point(s_bot, phi)
        s_bot += (q.z - z_bot) / -_AD.z
    return s_top, s_bot


_pd_pmap, _PD_LV = _arc_map(lambda t: V((math.cos(_lerp(*PD_PHI, t)), math.sin(_lerp(*PD_PHI, t)), 0.0))
                            * _pd_r(0.08, _lerp(*PD_PHI, t)))


def _pd_phi(v):
    return _lerp(*PD_PHI, _pd_pmap(v))


def _pad_P(u, v):
    phi = _pd_phi(v)
    s0, s1 = _pd_s_range(phi)
    flare = PD_FLARE * _smooth(0.45, 1.0, u) ** 1.4 + PD_FLARE_TOP * (1.0 - _smooth(0.0, 0.16, u))
    return _pd_point(_lerp(s0, s1, u), phi, flare)


def _pauldron_left():
    """Plain pad: raised, bevelled lip band along the (peaked) top edge with a small chevron point
    straight out, like the FBX; plain flared bottom edge."""
    lw, lh, lb = PD_LIP
    lu = 0.18                                           # ~ length along the arm
    ch = 0.0015
    vc = (0.0 - PD_PHI[0]) / (PD_PHI[1] - PD_PHI[0])    # ~ v straight out (arc map is near linear)

    def lip_u(extra):
        def f(v):
            phi = _pd_phi(v)
            point = PD_CHEVRON * max(0.0, 1.0 - abs(phi) / RAD(20)) ** 1.2
            return (lw + point + extra) / lu
        return f
    # rows: top chamfer, lip top, bevel down to the plate, interior, bottom chamfer
    us = [0.0, ch / lu, lip_u(0.0), lip_u(lb), 0.30, 0.44, 0.58, 0.72, 0.86, 1 - ch / lu, 1.0]
    vs = _stations([ch], [k / 10 for k in range(1, 10)], _PD_LV * 1.1)

    def heights(i):
        return PD_T + (lh if i <= 2 else 0.0)
    verts, faces, _ = _slab(_pad_P, us, vs, heights, ch=ch)
    return lib.mesh_object("pauldron_L", verts, faces, topo="rigid", sharp_angle=28)


# ============================================================================ W1 belt
def _band_rings(th0, th1, n, prof):
    """Rings across the belt (profile = [(off, z)]) at n+1 angles from th0 to th1."""
    return [[_belt_pt(_lerp(th0, th1, k / n), off, z) for off, z in prof] for k in range(n + 1)]


def _belt_band():
    ch, ri, ro, z0, z1 = BELT_CH, BELT_RI, BELT_RO, BELT_Z0, BELT_Z1
    prof = [(ri, z0), (ro - ch, z0), (ro, z0 + ch), (ro, z1 - ch), (ro - ch, z1), (ri, z1)]
    rings = [[_belt_pt(math.tau * i / BELT_N, off, z) for off, z in prof] for i in range(BELT_N)]
    return lib.mesh_object("belt_band", *_torus(rings), topo="rigid", sharp_angle=35)


def _button(name, th):
    """Round flat button with a bevelled rim, lying on the belt face (16-gon, all-quad caps)."""
    p, n, t = _ell(spec.torso_section(BELT_ZM), th, BELT_RO - 0.0008, BELT_ZM)
    up = V((0, 0, 1))
    r, h = BUTTON_R, BUTTON_H
    prof = [(r, 0.0), (r, h - 0.0028), (r - 0.0020, h - 0.0006), (r - 0.0045, h), (r * 0.45, h + 0.0006)]
    rings = []
    for rr, hh in prof:
        ring = []
        for i in range(16):
            a = math.tau * i / 16
            ring.append(p + n * hh + t * (rr * math.cos(a)) + up * (rr * math.sin(a)))
        rings.append(ring)
    return lib.lofted(name, rings, cap_start=True, cap_end=True, topo="rigid", sharp_angle=30)


def _belt():
    parts = [_belt_band()]
    for s in (1, -1):
        parts.append(_button("belt_button", _back(_belt_th(s * BUTTON_X))))
    return lib.join(parts, "belt", topo="rigid")


def _tab_left(th_in_deg, name="belt_tab_L"):
    """Side tab on the belt face: a leather strap from the side (outer end) towards the front, its
    rounded tip threaded through a rectangular loop buckle, one keeper behind the buckle."""
    th0, th1 = RAD(TAB_TH_OUT), RAD(th_in_deg)          # outer end -> inner tip
    za, zb = TAB_Z
    oa, ob = TAB_OFF
    ch = 0.0012
    parts = []
    n = 16
    rings = []
    for k in range(n + 1):
        t = k / n
        th = _lerp(th0, th1, t)
        # rounded tip over the last 12 %
        q = _clamp((t - 0.88) / 0.12)
        hw = 0.5 * (zb - za) * (1.0 - 0.55 * q ** 2)
        zm = 0.5 * (za + zb)
        z0, z1 = zm - hw, zm + hw
        rings.append([_belt_pt(th, oa, z0), _belt_pt(th, ob - ch, z0), _belt_pt(th, ob, z0 + ch),
                      _belt_pt(th, ob, z1 - ch), _belt_pt(th, ob - ch, z1), _belt_pt(th, oa, z1)])
    parts.append(lib.lofted("tab_strap", rings, cap_start=True, cap_end=True, topo="rigid", sharp_angle=35))
    # keeper: narrow loop wrapped round the tab (front face + top / bottom flanges back to the belt)
    r_mid = spec.torso_section(BELT_ZM)[1] + ob
    kc = RAD(th_in_deg + KEEPER_DTH)
    hw = KEEPER_W / 2 / r_mid
    g, T, d = 0.0004, KEEPER_T, 0.7071
    rf = ob + g + T / 2
    zt, zu = zb + g + T / 2, za - g - T / 2
    back = BELT_RO - 0.0010
    path = [(back, zt, 0, 1), (rf - 0.0016, zt, 0, 1), (rf - 0.0005, zt - 0.0005, d, d), (rf, zt - 0.0016, 1, 0),
            (rf, zu + 0.0016, 1, 0), (rf - 0.0005, zu + 0.0005, d, -d), (rf - 0.0016, zu, 0, -1), (back, zu, 0, -1)]
    kr = []
    for off, z, nx, nz in path:
        kr.append([_belt_pt(kc - hw, off - nx * T / 2, z - nz * T / 2), _belt_pt(kc - hw, off + nx * T / 2, z + nz * T / 2),
                   _belt_pt(kc + hw, off + nx * T / 2, z + nz * T / 2), _belt_pt(kc + hw, off - nx * T / 2, z - nz * T / 2)])
    parts.append(lib.lofted("tab_keeper", kr, cap_start=True, cap_end=True, topo="rigid", sharp_angle=35))
    # rectangular loop buckle (frame only), lying on the tab
    parts.append(_buckle_frame(RAD(th_in_deg + BUCKLE_DTH), ob + 0.0003))
    return lib.join(parts, name, topo="rigid")


def _buckle_frame(thc, base):
    hw, hh, bw, d, c = BUCKLE_W / 2, BUCKLE_H / 2, BUCKLE_BAR / 2, BUCKLE_D, 0.0012
    rm = spec.torso_section(BELT_ZM)[1] + base

    def W(x, depth, z):
        return _belt_pt(thc + x / rm, base + depth, z)

    corners = [V((-hw, -hh)), V((hw, -hh)), V((hw, hh)), V((-hw, hh))]
    norms = []
    for k in range(4):
        e = corners[(k + 1) % 4] - corners[k]
        norms.append(V((e.y, -e.x)).normalized())
    prof = [(-bw, 0.0), (bw, 0.0), (bw, d - c), (bw - c, d), (-bw + c, d), (-bw, d - c)]
    rings = []
    for k in range(4):
        a, b = corners[k], corners[(k + 1) % 4]
        for f, m in ((0.0, norms[k - 1] + norms[k]), (0.5, norms[k])):
            pos = a.lerp(b, f)
            rings.append([W(pos.x + m.x * o, dep, BELT_ZM + pos.y + m.y * o) for o, dep in prof])
    return lib.mesh_object("tab_buckle", *_torus(rings), topo="rigid", sharp_angle=35)


# ============================================================================ W4 hanging strap
def _strap_left():
    """Long thin strap from the tab's outer end down the hip, lying on the skirt, pointed tip."""
    za, zb = ST_Z
    top_off = TAB_OFF[1] + 0.0003                       # starts on the tab face
    n = 22
    pts = []
    for k in range(n + 1):
        t = k / n
        z = _lerp(za, zb, t)
        th = RAD(_lerp(ST_TH[0], ST_TH[1], _smooth(0.0, 1.0, t)))
        pb = _belt_pt(th, top_off, z) if z > BELT_Z0 - 0.012 else None
        ps = _skirt_pt(th, ST_CLR, z)
        if pb is not None and z > TAB_Z[0] - 0.002:
            p = pb                                      # on the tab
        elif pb is not None:                            # over the tab's / belt's bottom edge
            ps_r = math.hypot(ps.x, ps.y)
            pb_r = math.hypot(pb.x, pb.y)
            p = pb if pb_r >= ps_r else ps
        else:
            p = ps
        pts.append((p, th))
    # make the path never dip inward (the strap falls straight off the belt edge onto the skirt)
    rings = []
    for k, (p, th) in enumerate(pts):
        a = pts[max(k - 1, 0)][0]
        b = pts[min(k + 1, n)][0]
        tg = (b - a).normalized()
        side = V((math.cos(th), math.sin(th), 0.0))      # along the belt, towards increasing theta
        nrm = side.cross(tg).normalized()
        if nrm.dot(V((p.x, p.y, 0.0))) < 0:
            nrm = -nrm
        side = tg.cross(nrm).normalized()
        q = _clamp((k / n - 0.90) / 0.10)
        hw = ST_W / 2 * (1.0 - 0.9 * q)
        rings.append([p - side * hw, p - side * hw + nrm * ST_T, p + side * hw + nrm * ST_T, p + side * hw])
    verts, faces = lib.loft(rings, closed=True)
    last = n * 4
    faces += [(3, 2, 1, 0), (last, last + 1, last + 2, last + 3)]
    return lib.mesh_object("belt_strap_L", verts, faces, topo="rigid", sharp_angle=35)


# ============================================================================ build
def build():
    objs = []
    for make in (_shoulder_board_left, _pauldron_left):
        ob = make()
        objs += [ob, lib.mirror_copy(ob)]
    objs.append(_belt())
    objs.append(_tab_left(TAB_TH_IN[0]))
    tmp = _tab_left(TAB_TH_IN[1], "belt_tab_tmp")
    objs.append(lib.mirror_copy(tmp, "belt_tab_R"))
    me = tmp.data
    bpy.data.objects.remove(tmp)
    bpy.data.meshes.remove(me)
    st = _strap_left()
    objs += [st, lib.mirror_copy(st)]
    for o in objs:
        o["topo"] = "rigid"
    return objs
