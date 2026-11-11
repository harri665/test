"""Rigid hard-surface accessories: layered pauldrons, belt + buckle + keepers, pouches, hip canister + hose.

Everything is derived from spec.py.  Layering used here (see docs/PLAN.md):
    bare torso +0.028 coat  ->  +0.030..0.040 belt  ->  keepers / hangers +0.041..0.046
    skirt surface (torso+0.034 at z=1.07 flaring to 0.36 x 0.30 at the hem) -> pouches +0.014, canister +0.014
    sleeve (arm radius + up to 0.030) -> pauldron inner surface >= +0.012 outside it

Belt angle convention used in this file: theta = 0 at FRONT centre (-Y), +90 deg at the wearer's
LEFT (+X), 180 deg at the back.
"""
import bisect
import math

import bpy  # noqa: F401
from mathutils import Matrix, Vector as V

from .. import lib, spec

C = spec.CLOTH
RAD = math.radians


# ============================================================================ small helpers
def _sp(x, e):
    """signed power"""
    return math.copysign(abs(x) ** e, x)


def _clamp(x, a=0.0, b=1.0):
    return max(a, min(b, x))


def _lerp(a, b, t):
    return a + (b - a) * t


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


def _plate(P, us_int, vs_int, Lu, Lv, thick, trim_w=0.0, trim_h=0.0, ch=0.002, step=0.002, flip=False):
    """Closed, all-quad plate with real thickness built over the INNER surface P(u, v) (u, v in 0..1).

    Edge loops are placed at fixed physical insets from the rim (Lu / Lv = approximate side lengths):
    a chamfer loop at `ch`, and - when trim_w is set - a raised border `trim_h` high and `trim_w`
    wide that steps down to the field over `step`.  Returns (verts, faces, surf) where
    surf(u, v, h=None) -> (point on the outer field surface, outward normal)."""
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
            faces.append((B[i][j], B[i][j + 1], B[i + 1][j + 1], B[i + 1][j]))
    rim = ([(i, 0) for i in range(nu)] + [(nu, j) for j in range(nv)]
           + [(i, nv) for i in range(nu, 0, -1)] + [(0, j) for j in range(nv, 0, -1)])
    for k in range(len(rim)):
        (a, b), (c, d) = rim[k], rim[(k + 1) % len(rim)]
        faces.append((T[a][b], B[a][b], B[c][d], T[c][d]))
    return verts, faces, surf


def _rivet(name, loc, normal, r=0.006, h=0.0035, n=8):
    """Closed domed rivet (lib.stud leaves the base open)."""
    rs = [lib.ring_z(-0.0015, r, n=n), lib.ring_z(h * 0.55, r * 0.88, n=n), lib.ring_z(h, r * 0.5, n=n)]
    ob = lib.lofted(name, rs, cap_start=True, cap_end=True, topo="rigid")
    return lib.align_z_to(ob, normal, loc)


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
    """Point `off` outside the BARE torso at height z (belt outer surface = off 0.040)."""
    return _ell(spec.torso_section(z), th, off, z)[0]


SKIRT_TOP_Z = 1.07
SKIRT_TOP_OFF = 0.034


def _skirt_sec(z):
    """Outer surface of the coat skirt the hanging gear rests on (ellipse at height z)."""
    cy, rx, ry = spec.torso_section(SKIRT_TOP_Z)
    rx, ry = rx + SKIRT_TOP_OFF, ry + SKIRT_TOP_OFF
    if z >= SKIRT_TOP_Z:
        cy2, rx2, ry2 = spec.torso_section(z)
        return (cy2, rx2 + SKIRT_TOP_OFF, ry2 + SKIRT_TOP_OFF)
    k = (SKIRT_TOP_Z - z) / (SKIRT_TOP_Z - C["skirt_hem_z"])
    return (cy * (1 - k), _lerp(rx, C["skirt_hem_rx"], k), _lerp(ry, C["skirt_hem_ry"], k))


def _skirt_pt(th, off, z):
    return _ell(_skirt_sec(z), th, off, z)[0]


def _hang_frame(th, z_top, z_bot, clr):
    """Frame lying on the flaring skirt: origin `clr` outside the skirt at z_top, z axis running up
    the skirt slope, y axis = outward surface normal."""
    p_top = _skirt_pt(th, clr, z_top)
    p_bot = _skirt_pt(th, clr, z_bot)
    up = (p_top - p_bot).normalized()
    t = _ell(_skirt_sec(z_top), th)[2]
    n = t.cross(up).normalized()
    t = up.cross(n).normalized()
    return _Frame(p_top, t, n, up)


# ============================================================================ pauldron
# arm frame (left arm)
_A0 = spec.SHOULDER.copy()
_AD = (spec.ELBOW - spec.SHOULDER).normalized()              # down the upper arm
_AB = V((0.0, 1.0, 0.0))
_AB = (_AB - _AD * _AB.dot(_AD)).normalized()                # towards the back
_AO = _AD.cross(_AB).normalized()                            # outward (+X, a little up)

# upper slab: inner (under) surface.  u = collar -> outer end, v = front -> back over the top.
SL_X0, SL_X1 = 0.128, 0.335
SL_P = 2.8                    # superellipse power of the arch section (2 = round, bigger = boxier)
SL_T, SL_TRIM_H, SL_TRIM_W = 0.014, 0.005, 0.017


def _sl_zc(u):                # crown height of the under surface
    return 1.546 - 0.0108 * u - 0.0322 * u * u


def _sl_w(u):                 # half width (front-back) of the under surface
    return 0.099 + 0.019 * u


def _sl_d(u):                 # how far the front / back lips come down from the crown
    return 0.034 + 0.018 * u


_sl_amap, _SL_LV = _arc_map(lambda t: V((0.0, _sl_w(0.5) * _sp(math.sin((t - 0.5) * math.pi), 2 / SL_P),
                                         -_sl_d(0.5) * (1 - abs(math.cos((t - 0.5) * math.pi)) ** (2 / SL_P)))))


def _slab_P(u, v):
    a = (_sl_amap(v) - 0.5) * math.pi
    e = 2.0 / SL_P
    return V((_lerp(SL_X0, SL_X1, u), _sl_w(u) * _sp(math.sin(a), e),
              _sl_zc(u) - _sl_d(u) * (1 - abs(math.cos(a)) ** e)))


def _slab_under(x, y):
    """z of the slab's under surface above (x, y); below the lip once |y| leaves the slab."""
    u = _clamp((x - SL_X0) / (SL_X1 - SL_X0))
    t = abs(y) / (_sl_w(u) - 0.002)
    if t >= 1.0:
        return _sl_zc(u) - _sl_d(u) - 0.004
    return _sl_zc(u) - _sl_d(u) * (1 - (1 - t ** SL_P) ** (1 / SL_P))


# lower plate: U-channel around the upper arm.  u = top -> bottom, v = front leg -> outer face -> back leg.
LP_P = 3.2
LP_T, LP_TRIM_H, LP_TRIM_W = 0.010, 0.004, 0.017
LP_PHI = RAD(88)
LP_GAP = 0.004                # air between lower plate top edge and the slab's under surface


def _lp_r(s):                 # inner semi-axis; sleeve is <= arm_radius + 0.030 (0.088 -> 0.080)
    return 0.1015 - 0.0065 * _clamp(s / 0.20)


_lp_pmap, _LP_LV = _arc_map(lambda t: V((_sp(math.cos((t - 0.5) * 2 * LP_PHI), 2 / LP_P),
                                         _sp(math.sin((t - 0.5) * 2 * LP_PHI), 2 / LP_P), 0.0)) * 0.098)


def _lp_point(s, phi, extra=0.0):
    r = _lp_r(s) + extra
    e = 2.0 / LP_P
    return _A0 + _AD * s + _AO * (r * _sp(math.cos(phi), e)) + _AB * (r * _sp(math.sin(phi), e))


def _lp_s_range(phi):
    tau = LP_T + LP_TRIM_H
    s_top = 0.0
    for _ in range(8):                                   # top edge tucks under the slab
        q = _lp_point(s_top, phi, tau)
        s_top += (q.z - (_slab_under(q.x, q.y) - LP_GAP)) / -_AD.z
    s_bot = 0.2
    for _ in range(4):                                   # bottom edge horizontal at pauldron_bottom_z
        q = _lp_point(s_bot, phi)
        s_bot += (q.z - C["pauldron_bottom_z"]) / -_AD.z
    return s_top, s_bot


def _lower_P(u, v):
    phi = (_lp_pmap(v) - 0.5) * 2 * LP_PHI
    s0, s1 = _lp_s_range(phi)
    return _lp_point(_lerp(s0, s1, u), phi)


def _pauldron_left():
    parts = []
    # --- upper slab
    vs, fs, surf = _plate(_slab_P, [0.29, 0.5, 0.71], [0.20, 0.32, 0.44, 0.56, 0.68, 0.80],
                          SL_X1 - SL_X0, _SL_LV, SL_T, SL_TRIM_W, SL_TRIM_H)
    parts.append(lib.mesh_object("pauldron_slab", vs, fs, topo="rigid"))
    for u in (0.17, 0.83):
        for v in (0.145, 0.855):
            p, n = surf(u, v)
            parts.append(_rivet("rivet", p, n, r=0.0075, h=0.0045))
    # --- lower plate
    vs, fs, surf = _plate(_lower_P, [0.27, 0.43, 0.59, 0.75], [0.15, 0.265, 0.38, 0.5, 0.62, 0.735, 0.85],
                          0.20, _LP_LV * 1.04, LP_T, LP_TRIM_W, LP_TRIM_H)
    parts.append(lib.mesh_object("pauldron_lower", vs, fs, topo="rigid"))
    for v in (0.335, 0.665, 0.13, 0.87):
        p, n = surf(0.87, v)
        parts.append(_rivet("rivet", p, n, r=0.0065, h=0.004))
    p, n = surf(0.50, 0.5)
    parts.append(lib.emblem_cross("pauldron_emblem", size=0.040, thickness=0.005, loc=p - n * 0.0004,
                                  normal=n, up=-_AD, long_tail=1.7))
    return lib.join(parts, "pauldron_L", topo="rigid")


# ============================================================================ belt
BELT_Z0, BELT_Z1 = C["belt_z"]
BELT_ZM = 0.5 * (BELT_Z0 + BELT_Z1)
BELT_RO = C["belt_offset"]
BELT_RI = 0.030
BELT_N = 40

TH_KEEPERS = [RAD(a) for a in (17, -17, 104, -98, 164, -164)]
TH_RIVETS = [RAD(a) for a in (24.5, 30, -24.5, -30, 114, 122, -70, -78, -86, 174, 180, -174)]
TH_POUCH_HIP = (RAD(43), RAD(73))
TH_POUCH_BACK = (RAD(139), RAD(-139))
TH_CANISTER = RAD(-45)
TH_HOSE_END = RAD(-116)


def _belt_band():
    ch = 0.003
    ri, ro, z0, z1 = BELT_RI, BELT_RO, BELT_Z0, BELT_Z1
    prof = [(ri, z0 + ch), (ri + ch, z0), (ro - ch, z0), (ro, z0 + ch), (ro, z0 + 0.009),
            (ro - 0.0018, z0 + 0.011), (ro - 0.0018, BELT_ZM), (ro - 0.0018, z1 - 0.011),
            (ro, z1 - 0.009), (ro, z1 - ch), (ro - ch, z1), (ri + ch, z1), (ri, z1 - ch)]
    rings = []
    for i in range(BELT_N):
        th = math.tau * i / BELT_N
        rings.append([_belt_pt(th, off, z) for off, z in prof])
    return lib.mesh_object("belt_band", *_torus(rings), topo="rigid")


def _keeper(name, th, w=0.020, hang=0.0):
    """Leather loop wrapped over the belt (ends buried in the belt body).  hang > 0: instead of
    wrapping under the bottom rim the strap carries on down the skirt by `hang` (pouch / canister hanger)."""
    ro, z0, z1, g, t = BELT_RO, BELT_Z0, BELT_Z1, 0.001, 0.005
    top = [((ro - 0.004, z1 - 0.008), (ro - 0.009, z1 - 0.008)),
           ((ro - 0.004, z1 + g), (ro - 0.009, z1 + g + t)),
           ((ro - 0.001, z1 + g), (ro + g + t - 0.002, z1 + g + t)),
           ((ro + g, z1 - 0.002), (ro + g + t, z1 + g + t - 0.002)),
           ((ro + g, BELT_ZM), (ro + g + t, BELT_ZM))]
    tang = _ell(spec.torso_section(BELT_ZM), th)[2]
    st = [(_belt_pt(th, a[0], a[1]), _belt_pt(th, b[0], b[1])) for a, b in top]
    if hang:
        st.append((_belt_pt(th, ro + g, z0 + 0.004), _belt_pt(th, ro + g + t, z0 + 0.004)))
        for k in (0.5, 1.0):
            z = z0 - hang * k
            st.append((_skirt_pt(th, 0.0075, z), _skirt_pt(th, 0.0125, z)))
    else:
        # under the bottom rim the wrap stays shallow: the skirt starts right below the belt
        bot = [((ro + g, z0 + 0.002), (ro + g + t, z0 - g - t + 0.002)),
               ((ro - 0.001, z0 - g), (ro + g + t - 0.002, z0 - g - t)),
               ((ro - 0.003, z0 - g), (ro - 0.003, z0 - g - t))]
        st += [(_belt_pt(th, *a), _belt_pt(th, *b)) for a, b in bot]
    h = w / 2
    rings = [[a - tang * h, b - tang * h, b + tang * h, a + tang * h] for a, b in st]
    return lib.lofted(name, rings, cap_start=True, cap_end=True, topo="rigid")


def _belt():
    parts = [_belt_band()]
    for i, th in enumerate(TH_KEEPERS):
        parts.append(_keeper(f"keeper{i}", th))
    for th in TH_RIVETS:
        p, n, _ = _ell(spec.torso_section(BELT_ZM), th, BELT_RO - 0.0018, BELT_ZM)
        parts.append(_rivet("belt_rivet", p, n, r=0.0055, h=0.003))
    return lib.join(parts, "belt", topo="rigid")


def _buckle():
    cy, rx, ry = spec.torso_section(BELT_ZM)
    yf = cy - (ry + BELT_RO)                      # belt front surface (most negative y)
    rc = (rx + BELT_RO) ** 2 / (ry + BELT_RO)     # belt curvature radius at the front

    def W(x, depth, z):                           # buckle space -> world, wrapped onto the belt curve
        return V((x, yf - 0.0015 - depth + x * x / (2 * rc), z))

    hw, hh, bw, d, c = 0.047, 0.041, 0.0065, 0.011, 0.003
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
    frame = lib.mesh_object("buckle_frame", *_torus(rings), topo="rigid")
    sag = hw * hw / (2 * rc)
    prong = lib.box("buckle_prong", (2 * hw + 0.010, 0.0055, 0.0085), bevel_w=0.0015,
                    loc=(-0.003, yf - 0.0015 - d - 0.002 + sag, BELT_ZM))
    hinge = lib.cylinder("buckle_hinge", 0.0095, 0.018, n=8, bevel_w=0.0015,
                         loc=(hw, yf - 0.0015 - d * 0.5 + sag, BELT_ZM))
    return lib.join([frame, prong, hinge], "belt_buckle", topo="rigid")


# ============================================================================ pouches
def _pouch(name, th, w=0.078, h=0.098, d=0.043):
    fr = _hang_frame(th, BELT_Z0 + 0.004, BELT_Z0 + 0.004 - h, 0.016)
    parts = []
    # body: boxy superellipse rings, bevelled bottom, all-quad caps
    n = 16

    def ring(z, dw, dd):
        return [fr(p.x, p.y, z) for p in lib.ring_z(0.0, w / 2 - dw, d / 2 - dd, n=n, cy=d / 2,
                                                    phase=math.pi / 4, power=5.0)]
    rings = [ring(-h, 0.009, 0.008), ring(-h + 0.003, 0.003, 0.003), ring(-h + 0.010, 0.0, 0.0),
             ring(-h * 0.55, -0.0015, -0.0015), ring(-0.014, 0.0, 0.0), ring(0.0, 0.003, 0.003)]
    parts.append(lib.lofted("pouch_body", rings, cap_start=True, cap_end=True, topo="rigid"))
    # flap lid: sheet from the back top edge, over the top, round the front corner and down the front
    g, r, ft = 0.0018, 0.008, 0.0045
    fw = w + 0.007
    top_len = d + g - r + 0.004
    arc_len = r * math.pi / 2
    fixed = top_len + arc_len
    ua = 0.58

    def flap_P(u, v):
        x = (v - 0.5) * fw
        if u <= ua:
            sl = u / ua * fixed
            if sl <= top_len:
                return fr(x, -0.004 + sl, g)
            a = (sl - top_len) / r
            return fr(x, d + g - r + r * math.sin(a), g - r + r * math.cos(a))
        drop = 0.034 + 0.010 * (1 - abs(2 * v - 1) ** 1.6)
        return fr(x, d + g, g - r - drop * (u - ua) / (1 - ua))
    k = ua / fixed
    us_int = [k * top_len * 0.5, k * top_len, k * (top_len + arc_len * 0.5), ua, 0.5 * (ua + 1)]
    vs, fs, surf = _plate(flap_P, us_int, [0.25, 0.5, 0.75], 0.095, fw, ft, ch=0.0015, flip=True)
    parts.append(lib.mesh_object("pouch_flap", vs, fs, topo="rigid"))
    # closing tab + stud
    zt = g - r - 0.044
    tab = lib.box("pouch_tab", (0.017, 0.008, 0.030), bevel_w=0.002)
    parts.append(fr.place(tab, (0.0, d + g + ft * 0.5 + 0.001, zt + 0.004)))
    parts.append(_rivet("pouch_stud", fr(0.0, d + g + ft * 0.5 + 0.005, zt + 0.002), fr.y, r=0.0055, h=0.0035))
    # hanger strap over the belt
    parts.append(_keeper("pouch_hanger", th, w=0.030, hang=0.035))
    return lib.join(parts, name, topo="rigid")


# ============================================================================ canister + hose
CAN_R = 0.037
CAN_TOP_Z = 1.020
CAN_AXIS_OFF = 0.056          # axis distance outside the skirt surface


def _can_frame():
    return _hang_frame(TH_CANISTER, CAN_TOP_Z, CAN_TOP_Z - 0.20, 0.0)


def _canister():
    fr = _can_frame()
    R, b = CAN_R, 0.0038
    prof = [(0.020, -0.200), (0.031, -0.1985), (R, -0.192), (R, -0.182),
            (R + b, -0.180), (R + b, -0.150), (R, -0.148), (R, -0.119),
            (R + b, -0.117), (R + b, -0.106), (R, -0.104), (R, -0.075),
            (R + b, -0.073), (R + b, -0.062), (R, -0.060), (R, -0.044),
            (R + b, -0.042), (R + b, -0.014), (R, -0.012), (R, -0.005),
            (0.030, 0.001), (0.021, 0.007), (0.016, 0.009), (0.016, 0.021),
            (0.020, 0.022), (0.020, 0.032), (0.013, 0.035)]
    n = 16
    rings = [[fr(p.x, CAN_AXIS_OFF + p.y, z) for p in lib.ring_z(0.0, r, n=n)] for r, z in prof]
    parts = [lib.lofted("canister_body", rings, cap_start=True, cap_end=True, topo="rigid", sharp_angle=35)]
    # two hanger straps over the belt, running down behind the top band
    dth = 0.017 / (spec.torso_section(BELT_ZM)[1] + BELT_RO)
    for s in (-1, 1):
        parts.append(_keeper("canister_strap", TH_CANISTER + s * dth, w=0.018, hang=BELT_Z0 - CAN_TOP_Z + 0.035))
    return lib.join(parts, "canister", topo="rigid")


def _hose():
    fr = _can_frame()
    a = fr(-0.010, CAN_AXIS_OFF, 0.0145)
    ctrl = [a, fr(-0.036, CAN_AXIS_OFF + 0.002, 0.015), fr(-0.062, CAN_AXIS_OFF + 0.002, -0.010),
            _skirt_pt(RAD(-70), 0.050, 0.915), _skirt_pt(RAD(-80), 0.042, 0.850),
            _skirt_pt(RAD(-94), 0.036, 0.830), _skirt_pt(RAD(-108), 0.032, 0.870),
            _skirt_pt(TH_HOSE_END, 0.028, 0.950), _skirt_pt(TH_HOSE_END, 0.026, 1.020),
            _skirt_pt(TH_HOSE_END, 0.026, 1.062)]
    path = lib.smooth_path(ctrl, subdiv=6)
    radii = [0.0125 if i % 2 == 0 else 0.0108 for i in range(len(path))]
    hose = lib.lofted("hose_tube", lib.tube_rings(path, radii, n=8, x_hint=(0, 0, 1)),
                      cap_start=True, cap_end=True, topo="rigid", sharp_angle=60)
    parts = [hose]
    # collar where the hose leaves the canister neck
    col = lib.cylinder("hose_collar", 0.0155, 0.016, n=8, bevel_w=0.0015)
    lib.align_z_to(col, fr.x, fr(-0.024, CAN_AXIS_OFF, 0.0148))
    parts.append(col)
    # clip that holds the far end against the lower edge of the belt
    clip = lib.cylinder("hose_clip", 0.0165, 0.030, n=8, bevel_w=0.002)
    up = (ctrl[-1] - ctrl[-2]).normalized()
    lib.align_z_to(clip, up, ctrl[-1] - up * 0.014)
    parts.append(clip)
    return lib.join(parts, "canister_hose", topo="rigid")


# ============================================================================ build
def build():
    objs = []
    pl = _pauldron_left()
    objs += [pl, lib.mirror_copy(pl)]
    objs.append(_belt())
    objs.append(_buckle())
    objs.append(_pouch("pouch_hip_L1", TH_POUCH_HIP[0], w=0.074, h=0.094, d=0.042))
    objs.append(_pouch("pouch_hip_L2", TH_POUCH_HIP[1], w=0.074, h=0.094, d=0.042))
    objs.append(_pouch("pouch_back_L", TH_POUCH_BACK[0], w=0.086, h=0.104, d=0.045))
    objs.append(_pouch("pouch_back_R", TH_POUCH_BACK[1], w=0.086, h=0.104, d=0.045))
    objs.append(_canister())
    objs.append(_hose())
    for o in objs:
        o["topo"] = "rigid"
    return objs
