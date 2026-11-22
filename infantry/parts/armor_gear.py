"""Rigid hard-surface accessories (round 2, spec v2): layered shoulder armour, belt with double-bar
buckle / tongue / keepers / D-rings, flap pouches, hip canister with ribbed hoses.

Layering (see docs/PLAN.md and docs/DETAIL_PASS.md):
    coat torso (bare + 0.028, <= +0.036 inside the belt zone) -> belt inner wall +0.0365,
    field +0.040 (belt_offset), stitched rims +0.0425, tongue +0.0445, keepers from +0.046
    skirt (skirt.outer_radius when importable, else a guide from the v2 hem numbers) ->
    strap hangers +0.004, pouches / canister wall >= +0.012, hoses >= +0.012
    upper sleeve (<= arm radius + 0.022) -> upper-arm guard inner surface +0.010 outside that
    coat shoulder / armhole -> shoulder under-plate >= +0.008

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

# ============================================================================ constants read off the reference
# (docs/ref/front_torso, callout_shoulder, side_torso, back_torso, front_waist, back_waist; the
#  measurements were taken with a metric grid laid over the sheet at the compare.py scale)

# shoulder plate (P1) lying on the shoulder slope, with a slightly larger under-plate (P2) below it
PL_X0 = 0.140            # inner end, against the collar (ref front: 0.145 at the top edge)
PL_X1 = 0.252            # outer end of the top plate's crown (ref front: 0.258 at z 1.56, incl. the
                         # under-plate step)
PL_STEP = 0.010          # the under-plate peeks out this far beyond the top plate (ref: double edge)
PL_LEAN = 0.55           # ends slant inward going down (ref front: outer end 0.258@1.56 -> 0.205@1.495)
PL_W = (0.128, 0.126)    # under-plate half width front-back at the inner / outer end (ref side arch;
                         # wide at the collar end so the front leg passes in front of the chest)
PL_D = (0.032, 0.095)    # front / back legs drop this far below the crown at inner / outer end
PL_YC = 0.008            # arch centre a little behind the shoulder line (ref side)
PL_POW = 2.6             # arch section superellipse power
PL_CLR = 0.008           # under-plate underside above the coat shoulder / armhole top
PL_SLOPE = 0.06          # linear fall of the crown from the collar end (follows the coat yoke)
PL_Z_IN = 1.5918         # under-plate crown height at its inner end (coat yoke ~1.591 there + 0.0045;
                         # the ref plate tops out at 1.605, so the stack has to stay thin)
PL_T2, PL_T1 = 0.0045, 0.0070            # under-plate / top plate thickness
PL_TRIM = (0.012, 0.0025)                # top plate raised border frame: width, height

# curved upper-arm guard (P3)
GD_CLR = 0.010           # inner surface stays this far outside the sleeve
# sleeve envelope over the bare arm vs. the angle around the arm (0 = straight out, +-90 = back /
# front).  The final arms.py sleeve is pressed flat on its outer side (+0.000..0.004 within
# +-30 deg) and fuller front / back (<= +0.020); measured on its mesh, this stays >= all samples.
GD_ENV = (0.005, 0.0175, RAD(85), 1.6)          # base, extra, angle where it saturates, exponent
GD_T = 0.0050
GD_TRIM = (0.012, 0.0022)
GD_PHI = (RAD(-96), RAD(104))   # front edge .. back edge, 0 = straight out from the arm (ref side:
                                # the guard sits a little behind the arm centre)
GD_TOP_Z = (1.506, 1.503)
# top edge cap: on the outer face / on the front and back legs.  The ref guard runs right up under
# the plate's arch, but our sleeve alone already fills the ref outline at z 1.516 (x 0.271), so any
# guard over it there would widen the FRONT / BACK silhouette by 6-12 %: it stops just below.
GD_GAP = 0.004           # air between guard top and the under-plate's underside

# belt
BELT_Z0, BELT_Z1 = C["belt_z"]
BELT_ZM = 0.5 * (BELT_Z0 + BELT_Z1)
BELT_RO = C["belt_offset"]
BELT_RI = 0.0375         # DETAIL_PASS: cloth inside the belt zone stays <= +0.036 (the skirt measures
                         # +0.0364; 40-segment chords sag 0.45 mm inward, so keep a millimetre)
BELT_RIM = 0.0025        # stitched rims stand proud of the +0.040 field
BELT_N = 40
TONGUE_T, TONGUE_W = 0.0045, 0.040
KEEPER_T = 0.004
BUCKLE_X = 0.018         # ref front: buckle frame spans x -0.015 .. +0.063 (offset to the wearer's left)
BUCKLE_HW, BUCKLE_HH = 0.045, 0.039      # checklist: about 0.09 x 0.075 (ref 0.08 x 0.08)

# positions along the belt given as front-view x (ref front_waist / back_waist), converted to angles
X_KEEPERS_FRONT = ()                     # front: the canister's inner strap holds the tongue, the
                                         # side sleeves and hangers are the other loops
X_KEEPERS_BACK = (0.060,)                # both sides
X_TONGUE_TIP = -0.090
X_EYELETS = (-0.043, -0.060)             # punched holes in the tongue
X_DRING_FRONT = 0.082
X_DRING_BACK = 0.082
POUCH_FRONT = (0.152, 0.074, 0.136, 0.050)   # centre x, width, height, depth (ref x 0.11 .. 0.19, tall)
POUCH_BACK = (0.148, 0.080, 0.120, 0.045)    # ref back: |x| 0.10 .. 0.195, z 1.05 .. 1.17
POUCH_BACK_DROP = 0.016                      # back pouches hang this much lower on their straps
POUCH_CLR = 0.012
# wide leather sleeve over the belt at the front sides: the final skirt's side-panel binding bulges
# to +0.0439 between theta 41 and 61 deg at z 1.186 - 1.221 (above the +0.040 belt face); the
# sleeve hides it.  Straps, hangers and D-rings crossing it ride on top (see _belt_top).
SLEEVE_TH = RAD(50.8)
SLEEVE_W = 0.057
SLEEVE_IN = 0.0450

# canister on the wearer's right front hip (ref front: axis x ~-0.15, z 0.89 .. 1.17)
CAN_X = -0.150
CAN_R = 0.040
CAN_L = 0.250
CAN_TOP_Z = BELT_Z0 - 0.014
CAN_CLR = 0.012
CAN_PLUMB = 0.5          # 0 = lies along the flaring skirt, 1 = hangs plumb (ref: nearly vertical)
HOSE_R = 0.011


# ============================================================================ small helpers
def _sp(x, e):
    """signed power"""
    return math.copysign(abs(x) ** e, x)


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


def _compact(verts, faces):
    """Drop vertices no face uses."""
    used = sorted({i for f in faces for i in f})
    remap = {o: n for n, o in enumerate(used)}
    return [verts[i] for i in used], [tuple(remap[i] for i in f) for f in faces]


def _plate(P, us_int, vs_int, Lu, Lv, thick, trim_w=0.0, trim_h=0.0, ch=0.002, step=0.002,
           flip=False, keep_under=None):
    """All-quad plate with real thickness built over the INNER surface P(u, v) (u, v in 0..1).

    Edge loops are placed at fixed physical insets from the rim (Lu / Lv = approximate side lengths):
    a chamfer loop at `ch`, and - when trim_w is set - a raised border `trim_h` high and `trim_w`
    wide that steps down to the field over `step`.  keep_under = number of underside face rings kept
    along the rim (None = closed plate); the rest of the underside rests on cloth / another plate
    and is deleted.  Returns (verts, faces, surf), surf(u, v, h=None) -> (outer point, normal)."""
    edge = [0.0, ch] + ([trim_w, trim_w + step] if trim_w else [])
    us = [e / Lu for e in edge] + list(us_int) + [1.0 - e / Lu for e in reversed(edge)]
    vs = [e / Lv for e in edge] + list(vs_int) + [1.0 - e / Lv for e in reversed(edge)]
    nu, nv = len(us) - 1, len(vs) - 1

    def frame(u, v):
        e = 2e-3
        du = P(min(u + e, 1.0), v) - P(max(u - e, 0.0), v)
        dv = P(u, min(v + e, 1.0)) - P(u, max(v - e, 0.0))
        n = du.cross(dv).normalized()
        return P(u, v), (-n if flip else n)

    def surf(u, v, h=None):
        p, n = frame(u, v)
        return p + n * (thick if h is None else h), n

    top_h = ([thick + trim_h - ch, thick + trim_h, thick + trim_h] if trim_w else [thick - ch])
    verts, faces = [], []
    T = [[0] * (nv + 1) for _ in range(nu + 1)]
    B = [[0] * (nv + 1) for _ in range(nu + 1)]
    for i in range(nu + 1):
        for j in range(nv + 1):
            p, n = frame(us[i], vs[j])
            d = min(i, nu - i, j, nv - j)
            T[i][j] = len(verts)
            verts.append(p + n * (top_h[d] if d < len(top_h) else thick))
            B[i][j] = len(verts)
            verts.append(p + n * (ch if d == 0 else 0.0))
    for i in range(nu):
        for j in range(nv):
            faces.append((T[i][j], T[i + 1][j], T[i + 1][j + 1], T[i][j + 1]))
            d = min(i, nu - 1 - i, j, nv - 1 - j)
            if keep_under is None or d < keep_under:
                faces.append((B[i][j], B[i][j + 1], B[i + 1][j + 1], B[i + 1][j]))
    rim = ([(i, 0) for i in range(nu)] + [(nu, j) for j in range(nv)]
           + [(i, nv) for i in range(nu, 0, -1)] + [(0, j) for j in range(nv, 0, -1)])
    for k in range(len(rim)):
        (a, b), (c, d) = rim[k], rim[(k + 1) % len(rim)]
        faces.append((T[a][b], B[a][b], B[c][d], T[c][d]))
    verts, faces = _compact(verts, faces)
    return verts, faces, surf


def _rivet(name, loc, normal, r=0.006, h=0.004, n=8):
    ob = lib.rivet(name, radius=r, height=h, n=n, normal=normal, loc=V(loc) - V(normal).normalized() * 0.0012)
    return ob


def _grommet(name, loc, normal, tangent, r=0.0045, tube=0.0013, n=6):
    """Punched hole with a metal eyelet: a flat ring lying on the surface (6 x 4 quads)."""
    nrm = V(normal).normalized()
    x = (V(tangent) - nrm * V(tangent).dot(nrm)).normalized()
    y = nrm.cross(x)
    rings = []
    for i in range(n):
        a = math.tau * (i + 0.5) / n
        d = x * math.cos(a) + y * math.sin(a)
        c = V(loc) + d * r
        rings.append([c - d * tube - nrm * 0.0008, c + d * tube - nrm * 0.0008,
                      c + d * tube + nrm * 0.0012, c - d * tube + nrm * 0.0012])
    return lib.mesh_object(name, *_torus(rings), topo="rigid", sharp_angle=50)


class _Frame:
    """Local frame: x = along the belt (towards increasing theta), y = outward, z = up along the surface."""

    def __init__(self, o, x, y, z):
        self.o, self.x, self.y, self.z = V(o), V(x), V(y), V(z)

    def __call__(self, x, y, z):
        return self.o + self.x * x + self.y * y + self.z * z

    def place(self, ob, loc=(0, 0, 0)):
        m = Matrix((self.x, self.y, self.z)).transposed().to_4x4()
        m.translation = self(*loc)
        ob.data.transform(m)
        ob.data.update()
        return lib.finalize(ob)


def _bend(ob, mapping):
    """Re-place every vertex of a flat detail (built in its local XY plane, +Z out) with mapping(x, y, z)."""
    for v in ob.data.vertices:
        v.co = mapping(v.co.x, v.co.y, v.co.z)
    ob.data.update()
    return lib.finalize(ob, sharp_angle=25)


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
    """Point `off` outside the BARE torso at height z (belt field = off 0.040)."""
    return _ell(spec.torso_section(z), th, off, z)[0]


def _belt_th(x, off=None):
    """Belt angle on the front half whose point (at the belt field) has the given x."""
    cy, rx, ry = spec.torso_section(BELT_ZM)
    off = BELT_RO if off is None else off
    th = math.asin(_clamp(x / (rx + off), -1.0, 1.0))
    for _ in range(4):
        th += (x - _belt_pt(th, off, BELT_ZM).x) / max((rx + off) * math.cos(th), 1e-4)
    return th


def _back(th):
    """Mirror a front-half belt angle to the back half (same x)."""
    return math.copysign(math.pi, th) - th


# ---- the skirt the hanging gear rests on
try:
    from . import skirt as _skirt_mod
    _OUTER = _skirt_mod.outer_radius
except Exception:          # skirt not importable (being rewritten): fall back on the spec guide
    _OUTER = None


def _guide_r(th, z):
    """Fallback skirt surface from the v2 numbers: bare torso + coat offset under the belt, linear
    flare to the hem ellipse, one overlap layer (+0.008) on top."""
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
                th = RAD(a + 90.0)
                if not (0.08 < r < 0.6) or r < _guide_r(th, z) - 0.05:
                    return False
        return True
    except Exception:
        return False


_USE_OUTER = _outer_ok()


SKIRT_TOP_MARGIN = 0.009   # the final skirt's side binding bulges up to 8.6 mm past outer_radius
                           # just under the belt (z ~1.18; measured on its mesh), nowhere else
                           # near the gear


def _skirt_r(th, z):
    """Horizontal distance from the Z axis to the outermost skirt cloth at belt angle th, height z."""
    extra = SKIRT_TOP_MARGIN * _smooth(1.150, 1.172, z)
    if _USE_OUTER:
        return _OUTER((DEG(abs(th)) - 90.0) % 360.0, z) + extra
    return _guide_r(th, z) + extra


def _skirt_rmax(th, z, half):
    """Outermost skirt radius over the angular window th +- half (gear spanning a panel edge rests
    on the outer panel; the overlap panels make outer_radius step at their edges)."""
    if half <= 0.0:
        return _skirt_r(th, z)
    return max(_skirt_r(th + half * k / 3.0, z) for k in range(-3, 4))


def _skirt_pt(th, off, z, half=0.0):
    """Point `off` outside the skirt along its horizontal normal at (th, z).  The normal comes from
    a wide baseline so steps at panel edges do not tilt it."""
    e = 0.06
    p = lambda t: V((math.sin(t), -math.cos(t), 0.0)) * _skirt_rmax(t, z, half)
    c = p(th)
    tg = (p(th + e) - p(th - e)).normalized()
    n = tg.cross(V((0, 0, 1))).normalized()
    if n.dot(c) < 0:
        n = -n
    return c + n * off + V((0, 0, z))


def _skirt_th(x, z, off, back=False, half=0.0):
    """Angle (wearer's left if x > 0) on the front / back half where the skirt point + off has the given x."""
    sx = 1 if x > 0 else -1
    best = None
    # x is monotonic in theta on [0, 90] and [90, 180] separately
    a, b = (RAD(0.5), RAD(95)) if not back else (RAD(85), RAD(179.5))
    for _ in range(40):
        m = 0.5 * (a + b)
        xm = _skirt_pt(m, off, z, half).x
        if (xm < abs(x)) == (not back):
            a = m
        else:
            b = m
        best = m
    return sx * best


def _hang_frame(th, z_top, z_bot, clr, half=0.0, vert=0.0):
    """Frame lying on the (flaring) skirt: origin `clr` outside the skirt at z_top, z axis running up
    the skirt slope, y axis = outward surface normal, x along increasing theta.  half = angular
    half width of the gear (it rests on the outermost cloth under it)."""
    z_mid = 0.5 * (z_top + z_bot)
    p_top = _skirt_pt(th, clr, z_top, half)
    p_bot = _skirt_pt(th, clr, z_bot, half)
    # the bottom of the gear must clear the skirt too: lean out until both ends clear it
    p_mid = _skirt_pt(th, clr, z_mid, half)
    lerp_mid = p_top.lerp(p_bot, 0.5)
    rad = V((p_mid.x, p_mid.y, 0.0)).normalized()
    short = (p_mid - lerp_mid).dot(rad)
    if short > 0.0:                      # skirt bulges between top and bottom: push out
        p_top, p_bot = p_top + rad * short, p_bot + rad * short
    up = (p_top - p_bot).normalized()
    if vert:                             # hangs closer to plumb: pivot about the bottom contact
        up = up.lerp(V((0, 0, 1)), vert).normalized()
        p_top = p_bot + up * ((p_top.z - p_bot.z) / up.z)
    t = _skirt_pt(th + 0.06, clr, z_top, half) - _skirt_pt(th - 0.06, clr, z_top, half)
    n = t.cross(up).normalized()
    t = up.cross(n).normalized()
    return _Frame(p_top, t, n, up)


# ============================================================================ shoulder armour
_A0 = spec.SHOULDER.copy()
_LU = (spec.ELBOW - spec.SHOULDER).length
_AD = (spec.ELBOW - spec.SHOULDER).normalized()             # down the upper arm
_AB = V((0.0, 1.0, 0.0))
_AB = (_AB - _AD * _AB.dot(_AD)).normalized()               # towards the back
_AO = _AD.cross(_AB).normalized()                           # outward (+X, a little up)

_ARM_TOP = spec.armhole_ring(16)[0]                         # top of the armhole seam
# under-plate crown: PL_Z_IN at the collar end, passing PL_CLR above the armhole seam's top point
_PL_KC = (PL_Z_IN - PL_SLOPE * (_ARM_TOP.x - PL_X0) - (_ARM_TOP.z + PL_CLR)) / (_ARM_TOP.x - PL_X0) ** 2
_P2_X1 = PL_X1 + PL_STEP


def _pl_zc(x):
    dx = max(x - PL_X0, 0.0)
    return PL_Z_IN - PL_SLOPE * dx - _PL_KC * dx * dx


def _pl_w(u):
    return _lerp(PL_W[0], PL_W[1], u)


def _pl_d(u):
    return _lerp(PL_D[0], PL_D[1], u ** 1.3)


_pl_amap, _PL_LV = _arc_map(lambda t: V((0.0, _pl_w(0.5) * _sp(math.sin((t - 0.5) * math.pi), 2 / PL_POW),
                                         -_pl_d(0.5) * (1 - abs(math.cos((t - 0.5) * math.pi)) ** (2 / PL_POW)))))


def _p2_under(u, v):
    """Underside of the under-plate.  u: collar -> outer end, v: front leg -> over the top -> back leg."""
    a = (_pl_amap(v) - 0.5) * math.pi
    e = 2.0 / PL_POW
    drop = _pl_d(u) * (1 - abs(math.cos(a)) ** e)
    xc = _lerp(PL_X0, _P2_X1, u)
    return V((xc - PL_LEAN * drop, PL_YC + _pl_w(u) * _sp(math.sin(a), e), _pl_zc(xc) - drop))


def _p2_under_z(x, y):
    """Approximate height of the under-plate underside above (x, y); None outside its footprint."""
    if x > _P2_X1 + 0.004:
        return None
    u = _clamp((x - PL_X0) / (_P2_X1 - PL_X0))
    t = abs(y - PL_YC) / (_pl_w(u) - 0.002)
    if t >= 1.0:
        return None
    return _pl_zc(x) - _pl_d(u) * (1 - (1 - t ** PL_POW) ** (1 / PL_POW))


def _normal_of(P, u, v, flip=False):
    e = 2e-3
    du = P(min(u + e, 1.0), v) - P(max(u - e, 0.0), v)
    dv = P(u, min(v + e, 1.0)) - P(u, max(v - e, 0.0))
    n = du.cross(dv).normalized()
    return -n if flip else n


_P1_U1 = (PL_X1 - PL_X0) / (_P2_X1 - PL_X0)
_P1_V0 = 0.05


def _p1_under(u, v):
    """Underside of the top plate = top surface of the under-plate, over a smaller footprint."""
    uu, vv = u * _P1_U1, _lerp(_P1_V0, 1 - _P1_V0, v)
    return _p2_under(uu, vv) + _normal_of(_p2_under, uu, vv) * (PL_T2 + 0.001)


# ---- upper-arm guard
def _gd_r(s, phi=0.0):
    b, x, a_sat, k = GD_ENV
    env = b + x * min(1.0, (abs(phi) / a_sat) ** k)
    return spec.arm_radius(max(s, 0.0) / _LU) + env + GD_CLR


def _gd_point(s, phi, extra=0.0):
    """Guard surface: a tube round the upper arm for s >= 0, continued above the shoulder joint
    (s < 0) as a spherical cap round spec.SHOULDER, so it can run up under the plates."""
    r = _gd_r(s, phi) + extra
    d = _AO * math.cos(phi) + _AB * math.sin(phi)
    if s >= 0.0:
        return _A0 + _AD * s + d * r
    psi = -s / r
    return _A0 + (d * math.cos(psi) - _AD * math.sin(psi)) * r


def _gd_s_range(phi):
    tau = GD_T + GD_TRIM[1]
    cap0 = _lerp(GD_TOP_Z[0], GD_TOP_Z[1], _smooth(RAD(30), RAD(75), abs(phi)))

    def above(s):                                        # > 0 while the edge is too high
        q = _gd_point(s, phi, tau)
        cap = cap0
        zu = _p2_under_z(q.x, q.y)
        if zu is not None:
            cap = min(cap, zu - GD_GAP)
        return q.z - cap
    lo, hi = -0.05, 0.12                                 # top edge tucks under the plates
    for _ in range(40):
        m = 0.5 * (lo + hi)
        if above(m) > 0:
            lo = m
        else:
            hi = m
    s_top = hi
    s_bot = 0.15
    for _ in range(6):                                   # bottom edge horizontal at pauldron_bottom_z
        q = _gd_point(s_bot, phi)
        s_bot += (q.z - C["pauldron_bottom_z"]) / -_AD.z
    return s_top, s_bot


_gd_pmap, _GD_LV = _arc_map(lambda t: V((math.cos(_lerp(*GD_PHI, t)), math.sin(_lerp(*GD_PHI, t)), 0.0))
                             * _gd_r(0.08, _lerp(*GD_PHI, t)))


def _gd_phi(v):
    return _lerp(*GD_PHI, _gd_pmap(v))


def _guard_P(u, v):
    phi = _gd_phi(v)
    s0, s1 = _gd_s_range(phi)
    return _gd_point(_lerp(s0, s1, u), phi)


def _pauldron_left():
    parts = []
    # --- under-plate: only its rim shows; underside kept as a 1-ring return along the edge
    vs, fs, _ = _plate(_p2_under, [0.5], [0.22, 0.38, 0.5, 0.62, 0.78],
                       _P2_X1 - PL_X0, _PL_LV, PL_T2, ch=0.0015, keep_under=1)
    parts.append(lib.mesh_object("pauldron_underplate", vs, fs, topo="rigid"))
    # --- top plate with raised border frame and two rows of three big dome rivets
    vs, fs, surf = _plate(_p1_under, [0.3, 0.5, 0.7], [0.2, 0.31, 0.42, 0.5, 0.58, 0.69, 0.8],
                          PL_X1 - PL_X0, _PL_LV * (1 - 2 * _P1_V0), PL_T1, PL_TRIM[0], PL_TRIM[1],
                          keep_under=1)
    parts.append(lib.mesh_object("pauldron_plate", vs, fs, topo="rigid"))
    for v in (0.135, 0.865):
        for u in (0.22, 0.47, 0.72):
            p, n = surf(u, v)
            parts.append(_rivet("rivet", p, n, r=0.0095, h=0.0068))
    p, n = surf(0.93, 0.5)                      # small stud on the outer end (ref side view)
    parts.append(_rivet("rivet", p, n, r=0.0065, h=0.004, n=6))
    # --- upper-arm guard
    lu = 0.15
    vs, fs, gsurf = _plate(_guard_P, [0.22, 0.36, 0.5, 0.64, 0.78],
                           [0.17, 0.27, 0.38, 0.5, 0.62, 0.73, 0.83],
                           lu, _GD_LV * 1.1, GD_T, GD_TRIM[0], GD_TRIM[1], keep_under=1)
    parts.append(lib.mesh_object("pauldron_guard", vs, fs, topo="rigid"))
    # rolled rim along the bottom edge
    pts, nrm = [], []
    for k in range(15):
        v = 0.015 + 0.97 * k / 14
        p, n = gsurf(0.955, v)
        pts.append(p)
        nrm.append(n)
    parts.append(lib.strip_along("guard_rim", pts, nrm, width=0.014, thickness=0.0045, sink=0.0015,
                                 profile="round", topo="rigid"))
    # ornate dagger-cross on the outer face, bent onto the curved guard
    vc = 0.5 - (0.5 * (GD_PHI[0] + GD_PHI[1])) / (GD_PHI[1] - GD_PHI[0])   # v where phi = 0 ...
    vc = _clamp(vc + 0.06)                                                 # ... nudged a bit back
    phi_c = _gd_phi(vc)
    s0, s1 = _gd_s_range(phi_c)
    s_c = _lerp(s0, s1, 0.43)
    em = lib.emblem_ornate("guard_emblem", size=0.035, thickness=0.0035, normal=(0, 0, 1), up=(0, 1, 0))

    def wrap(x, y, z, phi_c=phi_c, s_c=s_c):
        s = s_c - y
        r = _gd_r(s, phi_c) + GD_T - 0.0008
        return _gd_point(s, phi_c + x / r, GD_T - 0.0008 + z)
    parts.append(_bend(em, wrap))
    # small four-point star on the front upper corner, rosette rivets at the bottom corners
    p, n = gsurf(0.20, 0.13)
    parts.append(lib.emblem_cross("guard_star", size=0.010, thickness=0.0025, loc=p - n * 0.0005, normal=n,
                                  up=-_AD, long_tail=1.0))
    for v in (0.12, 0.88):
        p, n = gsurf(0.80, v)
        parts.append(_rivet("guard_rivet", p, n, r=0.0055, h=0.0035, n=6))
    return lib.join(parts, "pauldron_L", topo="rigid")


# ============================================================================ belt
def _belt_band():
    ch = 0.0018
    ri, ro, rm, z0, z1 = BELT_RI, BELT_RO, BELT_RO + BELT_RIM, BELT_Z0, BELT_Z1
    prof = [(ri, z0), (rm - ch, z0), (rm, z0 + ch), (rm, z0 + 0.008), (ro, z0 + 0.0105),
            (ro, z1 - 0.0105), (rm, z1 - 0.008), (rm, z1 - ch), (rm - ch, z1), (ri, z1)]
    rings = []
    for i in range(BELT_N):
        th = math.tau * i / BELT_N
        rings.append([_belt_pt(th, off, z) for off, z in prof])
    return lib.mesh_object("belt_band", *_torus(rings), topo="rigid", sharp_angle=35)


def _strap_section(th, pts2d, w, T, P=None):
    """4-vertex rings of a strap running through belt-section points pts2d = [(off, z, nx, nz)]
    (centre line + 2D outward normal), width w along the belt tangent at angle th."""
    P = P or (lambda off, z: _belt_pt(th, off, z))
    tang = _ell(spec.torso_section(BELT_ZM), th)[2]
    rings = []
    for off, z, nx, nz in pts2d:
        a = P(off - nx * T / 2, z - nz * T / 2)
        b = P(off + nx * T / 2, z + nz * T / 2)
        rings.append([a - tang * w / 2, b - tang * w / 2, b + tang * w / 2, a + tang * w / 2])
    return rings


def _belt_top(th, w=0.0):
    """Offset of the outermost belt layer (rim, tongue or side sleeve) under a strap of width w."""
    r = spec.torso_section(BELT_ZM)[1] + BELT_RO
    half = w / 2 / r
    top = BELT_RO + BELT_RIM
    t0, t1 = sorted((_belt_th(BUCKLE_X - BUCKLE_HW + 0.012), _belt_th(X_TONGUE_TIP)))
    if th + half > t0 and th - half < t1:
        top = max(top, BELT_RO + TONGUE_T)
    sh = SLEEVE_W / 2 / r
    for c in (SLEEVE_TH, -SLEEVE_TH):
        if abs(th - c) < sh + half:
            top = max(top, SLEEVE_IN + KEEPER_T)
    return top


def _keeper(name, th, w=0.016, over=None, hang=0.0, wrap=True):
    """Leather keeper wrapped over the belt's top edge and down its face.  over = the outermost
    thing it has to clear (default: whatever _belt_top finds under it).  hang > 0: carries on down
    the skirt by `hang` (pouch / canister hanger) instead of tucking under the bottom rim;
    wrap=False: ends flush with the belt's bottom edge."""
    T, g = KEEPER_T, 0.0008
    top = _belt_top(th, w) if over is None else over
    rf = top + g + T / 2
    z0, z1 = BELT_Z0, BELT_Z1
    zt = z1 + g + T / 2
    if over is None and top >= SLEEVE_IN + KEEPER_T - 1e-6:
        zt += T + g                      # crosses a side sleeve: lies over its top wrap too
    d = 0.7071
    pts = [(BELT_RI + 0.004, zt, 0, 1), (rf - 0.004, zt, 0, 1), (rf - 0.0012, zt - 0.0012, d, d),
           (rf, z1 - 0.004, 1, 0), (rf, BELT_ZM, 1, 0), (rf, z0 + 0.004, 1, 0)]
    rings = _strap_section(th, pts, w, T)
    if hang:
        def P(off, z):
            return _skirt_pt(th, off, z, 0.12)
        lo = [(0.004 + T / 2, z0 - hang * k, 1, 0) for k in (0.45, 1.0)]
        rings += _strap_section(th, lo, w, T, P)
    elif wrap:
        zb = z0 - g - T / 2
        rings += _strap_section(th, [(rf - 0.0012, zb + 0.0012, d, -d), (BELT_RO - 0.001, zb, 0, -1)],
                                w, T)
    else:
        rings += _strap_section(th, [(rf, z0 + 0.0005, 1, 0)], w, T)
    return lib.lofted(name, rings, cap_start=True, cap_end=True, topo="rigid", sharp_angle=35)


def _tongue():
    """Free end of the belt: lies on the belt field from under the buckle to a rounded tip."""
    th0 = _belt_th(BUCKLE_X - BUCKLE_HW + 0.012)
    th1 = _belt_th(X_TONGUE_TIP)
    n = 12
    rings = []
    for k in range(n + 1):
        t = k / n
        th = _lerp(th0, th1, t)
        hw = TONGUE_W / 2 * (1.0 if t < 0.84 else math.sqrt(max(1 - ((t - 0.84) / 0.16) ** 2, 0.0)) * 0.75 + 0.25)
        a0, a1 = BELT_RO - 0.0008, BELT_RO + TONGUE_T
        rings.append([_belt_pt(th, a0, BELT_ZM - hw), _belt_pt(th, a1 - 0.001, BELT_ZM - hw),
                      _belt_pt(th, a1, BELT_ZM - hw + 0.0015), _belt_pt(th, a1, BELT_ZM + hw - 0.0015),
                      _belt_pt(th, a1 - 0.001, BELT_ZM + hw), _belt_pt(th, a0, BELT_ZM + hw)])
    return lib.lofted("belt_tongue", rings, cap_start=True, cap_end=True, topo="rigid", sharp_angle=35)


def _dring(name, th, zc=BELT_ZM + 0.004):
    """D-ring hanging on the belt face from a short leather tab."""
    base = _belt_top(th, 0.026)
    p0, n0, t0 = _ell(spec.torso_section(zc), th, base + 0.004, zc)
    up = V((0, 0, 1))
    hw, depth, tube = 0.009, 0.022, 0.0026
    path = [V((-hw, 0.0)), V((-hw * 0.5, 0.0)), V((hw * 0.5, 0.0)), V((hw, 0.0))]
    for k in range(1, 8):
        a = math.pi * k / 8
        path.append(V((hw * math.cos(a), -depth * math.sin(a) ** 0.9)))
    rings = []
    m = len(path)
    for i in range(m):
        q = path[i]
        tg = (path[(i + 1) % m] - path[i - 1]).normalized()
        nr = V((tg.y, -tg.x))
        c = p0 + t0 * q.x + up * q.y
        side = t0 * nr.x + up * nr.y
        rings.append([c + side * tube * math.cos(math.tau * j / 5) + n0 * tube * math.sin(math.tau * j / 5)
                      for j in range(5)])
    ring = lib.mesh_object(name + "_ring", *_torus(rings), topo="rigid", sharp_angle=60)
    tab = _strap_section(th, [(base + 0.0005, zc + 0.012, 1, 0),
                              (base + 0.0055, zc - 0.002, 1, 0)], 0.014, 0.003)
    # the tab is a short strap folded over the ring's straight bar
    tab_ob = lib.lofted(name + "_tab", tab, cap_start=True, cap_end=True, topo="rigid")
    return [ring, tab_ob]


def _belt():
    parts = [_belt_band(), _tongue()]
    for s in (1, -1):
        parts.append(_keeper("belt_sleeve", s * SLEEVE_TH, w=SLEEVE_W, over=SLEEVE_IN - 0.0008, wrap=False))
    for x in X_KEEPERS_FRONT:
        parts.append(_keeper("keeper", _belt_th(x), wrap=False))
    for x in X_KEEPERS_BACK:
        for s in (1, -1):
            parts.append(_keeper("keeper", _back(_belt_th(s * x))))
    for i, x in enumerate(X_EYELETS):
        th = _belt_th(x)
        p, n, t = _ell(spec.torso_section(BELT_ZM), th, BELT_RO + TONGUE_T, BELT_ZM)
        parts.append(_grommet("eyelet", p, n, t))
    for k, dx in enumerate((-0.030, 0.0, 0.030)):          # punched holes at the back centre
        th = _back(_belt_th(dx)) if dx else math.pi
        p, n, t = _ell(spec.torso_section(BELT_ZM), th, BELT_RO, BELT_ZM)
        parts.append(_grommet("eyelet", p, n, t) if dx else _rivet("belt_rivet", p, n, r=0.0055, h=0.0035, n=6))
    parts += _dring("dring_f", _belt_th(X_DRING_FRONT))
    for s in (1, -1):
        parts += _dring("dring_b", _back(_belt_th(s * X_DRING_BACK)))
    return lib.join(parts, "belt", topo="rigid")


def _buckle():
    """Large square double-bar frame buckle, wrapped onto the belt curve.  Its lower bar leans out
    over the skirt that starts right under the belt."""
    hw, hh, bw, d, c = BUCKLE_HW, BUCKLE_HH, 0.0065, 0.010, 0.0025
    base = BELT_RO + TONGUE_T + 0.001
    # how far the lower bar must lean out to clear the skirt below the belt
    zb = BELT_ZM - hh - bw
    sk = _skirt_r(0.0, zb) - (spec.torso_section(zb)[2] - spec.torso_section(zb)[0])
    need = max(0.0, sk + 0.003 - base)

    def W(x, depth, z):
        lean = need * _smooth(BELT_Z0 + 0.010, zb, z)
        th = _belt_th(BUCKLE_X + x)
        return _belt_pt(th, base + depth + lean, z)

    corners = [V((-hw, -hh)), V((hw, -hh)), V((hw, hh)), V((-hw, hh))]
    norms = []
    for k in range(4):
        e = corners[(k + 1) % 4] - corners[k]
        norms.append(V((e.y, -e.x)).normalized())
    prof = [(-bw, 0.0), (bw, 0.0), (bw, d - c), (bw - c, d), (-bw + c, d), (-bw, d - c)]
    rings = []
    for k in range(4):
        a, b = corners[k], corners[(k + 1) % 4]
        for f, m in ((0.0, norms[k - 1] + norms[k]), (1 / 3, norms[k]), (2 / 3, norms[k])):
            pos = a.lerp(b, f)
            rings.append([W(pos.x + m.x * o, dep, BELT_ZM + pos.y + m.y * o) for o, dep in prof])
    parts = [lib.mesh_object("buckle_frame", *_torus(rings), topo="rigid")]
    # second (centre) bar the tongue wraps round, with a roller sleeve
    xb = -hw * 0.30
    bar = [[W(xb + o * 0.8, dep * 0.9, BELT_ZM + zz) for o, dep in prof]
           for zz in (-hh + 0.002, -hh * 0.33, hh * 0.33, hh - 0.002)]
    parts.append(lib.lofted("buckle_bar", bar, cap_start=True, cap_end=True, topo="rigid"))
    # prong from the centre bar across the opening, tip resting on the right-hand bar
    pr = []
    for k, x in enumerate((xb, xb + 0.02, xb + 0.045, hw - 0.002)):
        h = 0.0032 if k < 3 else 0.0024
        z = BELT_ZM
        pr.append([W(x, d * 0.95 - h, z - h), W(x, d * 0.95 + 0.0015, z - h * 0.8),
                   W(x, d * 0.95 + 0.0015, z + h * 0.8), W(x, d * 0.95 - h, z + h)])
    parts.append(lib.lofted("buckle_prong", pr, cap_start=True, cap_end=True, topo="rigid"))
    return lib.join(parts, "belt_buckle", topo="rigid")


# ============================================================================ pouches
def _pouch(name, th, w, h, d, cross_clasp=True, drop=0.0):
    half = (w * 0.5 + 0.006) / max(_skirt_r(th, BELT_Z0 - h * 0.5), 0.1)
    z_top = BELT_Z0 - 0.002 - drop
    fr = _hang_frame(th, z_top, z_top - h, POUCH_CLR, half)
    parts = []
    n = 16

    def ring(z, dw, dd):
        return [fr(p.x, p.y, z) for p in lib.ring_z(0.0, w / 2 - dw, d / 2 - dd, n=n, cy=d / 2,
                                                    phase=math.pi / 4, power=5.0)]
    rings = [ring(-h, 0.010, 0.009), ring(-h + 0.003, 0.003, 0.003), ring(-h + 0.011, 0.0, 0.0),
             ring(-h * 0.55, -0.0015, -0.0003), ring(-0.016, 0.0, 0.0), ring(0.0, 0.003, 0.003)]
    parts.append(lib.lofted("pouch_body", rings, cap_start=True, cap_end=True, topo="rigid"))
    # flap lid: sheet from the back top edge, over the top, round the front corner and down the front
    g, r, ft = 0.0026, 0.008, 0.0045
    fw = w + 0.007
    top_len = d + g - r + 0.004
    arc_len = r * math.pi / 2
    fixed = top_len + arc_len
    ua = 0.56
    drop0 = 0.40 * h

    def flap_P(u, v):
        x = (v - 0.5) * fw
        if u <= ua:
            sl = u / ua * fixed
            if sl <= top_len:
                return fr(x, -0.004 + sl, g)
            a = (sl - top_len) / r
            return fr(x, d + g - r + r * math.sin(a), g - r + r * math.cos(a))
        drop = drop0 + 0.012 * (1 - abs(2 * v - 1) ** 1.6)
        return fr(x, d + g, g - r - drop * (u - ua) / (1 - ua))
    k = ua / fixed
    us_int = [k * top_len * 0.5, k * top_len, k * (top_len + arc_len * 0.5), ua, 0.5 * (ua + 1)]
    vs, fs, _ = _plate(flap_P, us_int, [0.25, 0.5, 0.75], 0.10, fw, ft, ch=0.0015)
    parts.append(lib.mesh_object("pouch_flap", vs, fs, topo="rigid"))
    # clasp: strap tab down over the flap edge, a cross bar on it and a stud
    zt = g - r - drop0 - 0.010
    yf = d + g + ft + 0.0012
    tab = lib.box("pouch_tab", (0.016, 0.004, 0.040), bevel_w=0.0012)
    parts.append(fr.place(tab, (0.0, yf + 0.0015, zt + 0.006)))
    if cross_clasp:
        bar = lib.box("pouch_bar", (0.034, 0.004, 0.008), bevel_w=0.0012)
        parts.append(fr.place(bar, (0.0, yf + 0.0035, zt + 0.004)))
    parts.append(_rivet("pouch_stud", fr(0.0, yf + 0.0045, zt - 0.008), fr.y, r=0.0050, h=0.0035, n=6))
    # hanger strap over the belt
    parts.append(_keeper("pouch_hanger", th, w=0.026, hang=0.020 + drop))
    return lib.join(parts, name, topo="rigid")


# ============================================================================ canister + hoses
CAN_AXIS_OFF = CAN_R + 0.006 + CAN_CLR      # axis distance outside the skirt (bands stand 0.006 proud)
_CAN_HALF = 0.30                            # angular half width the canister covers on the skirt (rad)


def _can_theta():
    zm = CAN_TOP_Z - CAN_L * 0.5
    return _skirt_th(CAN_X, zm, CAN_AXIS_OFF, half=_CAN_HALF)


_TH_CAN = None


def _can_frame():
    global _TH_CAN
    if _TH_CAN is None:
        _TH_CAN = _can_theta()
    return _hang_frame(_TH_CAN, CAN_TOP_Z, CAN_TOP_Z - CAN_L, 0.0, _CAN_HALF, vert=CAN_PLUMB)


def _can_axis(h):
    fr = _can_frame()
    return fr(0.0, CAN_AXIS_OFF, h)


def _perp(fr, d):
    d = V(d)
    d = d - fr.z * d.dot(fr.z)
    return d.normalized()


def _canister():
    fr = _can_frame()
    R, b, L = CAN_R, 0.006, CAN_L
    # lathe profile (radius, height from the top) read off the ref: cap, top band, slim neck section,
    # big mid band, long body, bottom band
    prof = [(0.016, 0.034), (0.020, 0.031), (0.020, 0.022), (0.016, 0.020), (0.016, 0.010),
            (0.024, 0.006), (R - 0.004, 0.002),
            (R, -0.002), (R + b, -0.004), (R + b, -0.026), (R, -0.028),
            (R - 0.008, -0.034), (R - 0.008, -0.082), (R, -0.088),
            (R + b, -0.090), (R + b, -0.116), (R, -0.118),
            (R, -0.150), (R, -0.205), (R, -0.222),
            (R + b, -0.224), (R + b, -0.244), (R + b * 0.5, -L + 0.001), (0.030, -L), (0.018, -L - 0.002)]
    n = 16
    rings = [[fr(p.x, CAN_AXIS_OFF + p.y, z) for p in lib.ring_z(0.0, r, n=n)] for r, z in prof]
    parts = [lib.lofted("canister_body", rings, cap_start=True, cap_end=True, topo="rigid", sharp_angle=35)]
    # two hanger straps from the belt down to the top band
    dth = 0.024 / (spec.torso_section(BELT_ZM)[1] + BELT_RO)
    for s in (-1, 1):
        parts.append(_keeper("canister_strap", _TH_CAN + s * dth, w=0.016,
                             hang=BELT_Z0 - CAN_TOP_Z + 0.022))
    return lib.join(parts, "canister", topo="rigid")


def _collar(loc, axis, r=0.0155, depth=0.014):
    col = lib.cylinder("hose_collar", r, depth, n=8)
    return lib.align_z_to(col, axis, loc)


def _ribbed(name, ctrl, subdiv, n_rib):
    path = lib.smooth_path(ctrl, subdiv=subdiv)
    # resample evenly by arc length so every rib is the same size
    cum = [0.0]
    for a, b in zip(path, path[1:]):
        cum.append(cum[-1] + (b - a).length)
    m = 2 * n_rib + 1
    pts = []
    for k in range(m):
        t = cum[-1] * k / (m - 1)
        i = max(0, min(len(path) - 2, bisect.bisect_right(cum, t) - 1))
        f = (t - cum[i]) / max(cum[i + 1] - cum[i], 1e-9)
        pts.append(path[i].lerp(path[i + 1], f))
    radii = [HOSE_R * (1.07 if k % 2 == 0 else 0.90) for k in range(m)]
    return lib.lofted(name, lib.tube_rings(pts, radii, n=8, x_hint=(0, 0, 1)),
                      cap_start=True, cap_end=True, topo="rigid", sharp_angle=60)


def _hoses():
    fr = _can_frame()
    up = fr.z
    d_o = _perp(fr, (-1.0, 0.0, 0.0))            # outer side (wearer's right)
    d_o = (d_o + fr.y * 0.25).normalized()       # turned a little off the skirt
    d_i = _perp(fr, (1.0, 0.0, 0.0))             # towards the front centre
    R = CAN_R
    parts = []
    # --- hose A: clear C on the outer side, from under the top band round to the bottom band
    h_s, h_e = -0.058, -0.212
    hc, sv, sh = 0.5 * (h_s + h_e), 0.5 * (h_s - h_e), 0.050
    base = R + 0.016
    ctrl = [_can_axis(h_s) + d_o * (R + 0.002), _can_axis(h_s) + d_o * base]
    for k in range(1, 12):
        a = math.pi / 2 - math.pi * k / 12
        ctrl.append(_can_axis(hc) + up * (sv * math.sin(a)) + d_o * (base + sh * math.cos(a)))
    ctrl += [_can_axis(h_e) + d_o * base, _can_axis(h_e) + d_o * (R + 0.002)]
    parts.append(_ribbed("hose_c", ctrl, 3, 17))
    parts.append(_collar(_can_axis(h_s) + d_o * (R + 0.006), d_o))
    parts.append(_collar(_can_axis(h_e) + d_o * (R + 0.006), d_o))
    # --- hose B: from the mid band on the inner side up to the belt's lower edge near the front
    h_b = -0.118
    zt = BELT_Z0 - 0.010
    th_t = _skirt_th(-0.050, zt, 0.0)
    top = _skirt_pt(th_t, HOSE_R + 0.010, zt)
    z1, z2 = 1.080, 1.125
    p1 = _skirt_pt(_skirt_th(-0.072, z1, 0.0), HOSE_R + 0.014, z1)
    p2 = _skirt_pt(_skirt_th(-0.054, z2, 0.0), HOSE_R + 0.012, z2)
    ctrl = [_can_axis(h_b) + d_i * (R + 0.008), _can_axis(h_b) + d_i * (R + 0.024) - up * 0.002,
            p1, p2, top + V((0, 0, -0.026)), top]
    parts.append(_ribbed("hose_b", ctrl, 4, 11))
    parts.append(_collar(_can_axis(h_b) + d_i * (R + 0.008), d_i))
    clip = lib.cylinder("hose_clip", HOSE_R + 0.004, 0.018, n=8)
    parts.append(lib.align_z_to(clip, (top - ctrl[-2]).normalized(), top - (top - ctrl[-2]).normalized() * 0.006))
    return lib.join(parts, "canister_hose", topo="rigid")


# ============================================================================ build
def build():
    objs = []
    pl = _pauldron_left()
    objs += [pl, lib.mirror_copy(pl)]
    objs.append(_belt())
    objs.append(_buckle())
    xf, wf, hf, df = POUCH_FRONT
    zf = BELT_Z0 - hf * 0.5
    objs.append(_pouch("pouch_front_L", _skirt_th(xf, zf, POUCH_CLR + df * 0.5, half=0.3), wf, hf, df))
    xb, wb, hb, db = POUCH_BACK
    zb = BELT_Z0 - hb * 0.5
    thb = _skirt_th(xb, zb, POUCH_CLR + db * 0.5, back=True, half=0.3)
    objs.append(_pouch("pouch_back_L", thb, wb, hb, db, drop=POUCH_BACK_DROP))
    objs.append(_pouch("pouch_back_R", -thb, wb, hb, db, drop=POUCH_BACK_DROP))
    objs.append(_canister())
    objs.append(_hoses())
    for o in objs:
        o["topo"] = "rigid"
    return objs
