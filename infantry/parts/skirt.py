"""Coat skirt below the belt (round 5): narrow layered panels matched to the user's FBX game model.

Objects (left = +X built; "_R" = mirrored copy unless noted):
    skirt_front_L             wearer's left front panel = continuation of the coat's front wrap flap:
                              its inner edge carries the flap's diagonal on down, crosses the centre
                              and becomes the right edge of the tall inverted-V opening; hem rises
                              from a sharp point at the opening to the outer corner
    skirt_front_R             wearer's right front panel (built on +X, mirrored); its top runs on
                              under skirt_front_L, its inner edge is the opening's left edge
    skirt_side_front_L/_R     narrow front-side panel under the front panel, fanning out to a point
                              at the side
    skirt_side_L/_R           side panel, pointed at its back edge
    skirt_side_back_L/_R      shorter back-side panel over the side and back panels, pointed at its
                              inner edge
    skirt_back_L/_R           back panels split at centre by a deep vent, long point near the vent
    skirt_back_upper_L/_R     upper back overlay, V hem pointing down to the centre seam
    skirt_back_tab            pointed tab hanging from the belt at centre back (one object)
    skirt_back_tab_bar        thin raised bar on the tab continuing the coat's back-emblem stem,
                              ending in a small point (rigid)

Every panel is a closed slab on ONE shared cone-like surface `surf(angle, z, lift)`: bare torso +
offset under the belt, a straight flare below the hips.  Panels are stacked in layers (`LAYER`):
layer n sits n * step(z) outside layer 0, with a thin step inside the belt zone (everything <= bare +
0.036 there) that opens up below the belt.  Each slab: outer face with a narrow chamfer ring round
its outline (a loop BEV_W in from every edge, the edge itself pushed down) + rim + inner face.
No borders, piping, emblems, buttons, hip tier or under skirt (the FBX has none).
Angles follow lib.ring_z: 0 = +X (character's left), 90 = back (+Y), 270 (= -90) = front (-Y).
"""
import math

import bpy  # noqa: F401
from mathutils import Vector as V

from .. import lib, spec

D = math.radians
Z_TOP = 1.240                       # skirt panels start here, under the belt (I3)
BELT_Z = (1.195, 1.245)             # I1 belt band
BELT_CLEAR = 0.036                  # inside the belt zone nothing beyond bare + this
BELT_ROWS = (1.240, 1.226, 1.212, 1.198, 1.184, 1.165, 1.145, 1.125, 1.100, 1.075, 1.050, 1.020)
# ^ loops shared by every panel through the belt zone and the hip bend: stacked layers stay parallel
RAMP = (1.120, 0.880)               # layers open up between these heights (front: under the wrap flap)
OFF = (0.021, 0.023)                # layer-0 outer surface over the bare torso: belt zone / below
STEP = (0.0035, 0.0085)             # layer step: belt zone / below
THICK = (0.0022, 0.0055)            # slab thickness: belt zone / below
BEV_W, BEV_H = 0.006, 0.45          # chamfer: width along the face, depth as a share of the thickness
ROW_H = 0.036                       # target row spacing below the belt rows

# cone below the hips (measured on the FBX, re-based on our body): z, rx, ry_front, ry_back of layer 0
CONE = [(1.300, 0.112, 0.100, 0.100), (1.150, 0.135, 0.120, 0.120), (1.000, 0.172, 0.135, 0.140),
        (0.800, 0.228, 0.142, 0.172), (0.600, 0.290, 0.150, 0.212), (0.450, 0.330, 0.155, 0.245),
        (0.300, 0.355, 0.160, 0.270), (0.150, 0.375, 0.165, 0.285)]

# trouser outer envelope (z: max|x|, y-front, y-back) measured on the legs mesh; the inner face of
# layer 0 keeps TROUSER_CLR outside it
# I5 straight trousers (legs agent, measured on the real meshes)
TROUSER_ENV = [(1.20, 0.119, -0.103, 0.105), (1.15, 0.131, -0.108, 0.113), (1.10, 0.144, -0.113, 0.122),
               (1.05, 0.157, -0.119, 0.130), (1.00, 0.173, -0.117, 0.124), (0.95, 0.188, -0.115, 0.119),
               (0.90, 0.193, -0.106, 0.106), (0.85, 0.196, -0.097, 0.095), (0.80, 0.199, -0.092, 0.090),
               (0.75, 0.201, -0.087, 0.088), (0.70, 0.204, -0.083, 0.085), (0.65, 0.206, -0.080, 0.081),
               (0.60, 0.208, -0.078, 0.077), (0.55, 0.213, -0.076, 0.075), (0.50, 0.209, -0.073, 0.078),
               (0.45, 0.209, -0.065, 0.081), (0.40, 0.210, -0.058, 0.081), (0.35, 0.211, -0.059, 0.078),
               (0.30, 0.215, -0.052, 0.081), (0.25, 0.204, -0.037, 0.073)]
TROUSER_CLR = 0.007

# ---- panel layout (FBX "JacketMain" front panels + "Jacket wave" .. ".005"), left side -------------
LAYER = dict(back=0, side=1, side_front=2, side_back=2, back_upper=3, front_under=3, front_over=4, tab=4)
# front opening (I3): tall narrow inverted V, apex ~1.09, straight edges.  skirt_front_L's inner edge
# is the coat flap's edge line carried on down: straight from where the flap ends (OVER_EDGE[0]) to
# the panel's tip at the hem; above the flap bottom (hidden under the flap) it swings on to the left
# with UP_SLOPE.  FBX values; replace OVER_EDGE[0] with the coat flap's real bottom corner.
OVER_EDGE = ((1.130, -0.003), (0.380, 0.133))
UP_SLOPE = 0.57
UNDER_EDGE = ((1.085, 0.000), (0.392, -0.126))     # skirt_front_R's inner edge (true x) below the apex
UNDER_X_TOP = 0.025                 # ... above it, it runs on under skirt_front_L to this true x
FRONT_OUT = (0.149, 0.236)          # front panels' outer edge x = a + b * (1.0 - z)
FRONT_HEM = dict(over=(OVER_EDGE[1][0], 0.515), under=(UNDER_EDGE[1][0], 0.505))   # (tip at the opening, outer corner) z
# side front panel: inner edge under the front panel, outer edge fanning to the side, tip there
SF_IN, SF_OUT, SF_HEM = 0.035, ((1.24, -32.0), (0.46, 0.0)), (0.530, 0.460)
SF_KICK = (0.022, 0.56, 0.44, 0.45, 1.0)        # its point kicks out (front view: spikes at the sides)
# side panel: front edge under the side front panel, tip at its back edge
SD_EDGES, SD_HEM = (((1.24, -22.0), (0.40, -8.0)), ((1.24, 30.0), (0.40, 49.0))), (0.500, 0.400)
SD_KICK = (0.018, 0.58, 0.40, 0.55, 1.0)
# back-side panel: outer edge over the side panel, inner edge by x; tip at the inner edge
SB_OUT, SB_IN, SB_HEM = ((1.24, 31.0), (0.56, 37.0)), ((1.24, 0.045), (0.56, 0.112)), (0.650, 0.566)
# back panel: outer edge under the side panel; vent edge closed down to VENT_TOP, then opening
BK_OUT, BK_HEM = ((1.24, 26.0), (0.45, 42.0)), (0.450, 0.330, 0.370, 0.82)  # outer, tip, vent z; tip u
VENT_TOP, VENT_X0, VENT_X1 = 0.950, 0.0025, 0.030
# upper back overlay: V hem down to the centre seam
BU_OUT, BU_HEM = ((1.24, 24.0), (0.96, 19.0)), (0.960, 0.822)
BU_SEAM_X = 0.0025
# back tab (I4): hangs from the belt, pointed bottom; raised bar continues the coat's emblem stem
TAB_X, TAB_TOP, TAB_CORNER_Z, TAB_TIP_Z = 0.065, 1.215, 1.005, 0.950
BAR_W, BAR_H, BAR_TOP, BAR_TIP = 0.0045, 0.0030, 1.192, 0.972


def _sstep(e0, e1, x):
    t = min(1.0, max(0.0, (x - e0) / (e1 - e0)))
    return t * t * (3 - 2 * t)


def _smax(a, b, k=0.006):
    return 0.5 * (a + b + math.sqrt((a - b) ** 2 + k * k)) - 0.5 * k


def _lin(z, p0, p1):
    (z0, v0), (z1, v1) = p0, p1
    return v0 + (v1 - v0) * (z - z0) / (z1 - z0)


def _ramp(z):
    return _sstep(RAMP[0], RAMP[1], z)


def step(z):
    return STEP[0] + (STEP[1] - STEP[0]) * _ramp(z)


def thick(z):
    return THICK[0] + (THICK[1] - THICK[0]) * _ramp(z)


def lift(layer, z):
    return layer * step(z)


# ------------------------------------------------------------------ the shared surface
def _bare(z):
    return spec.torso_section(max(z, spec.TORSO[0][0]))


def _cone(z):
    C = CONE
    if z >= C[0][0]:
        return C[0][1:]
    if z <= C[-1][0]:
        return C[-1][1:]
    for i in range(len(C) - 1):
        if C[i + 1][0] <= z <= C[i][0]:
            p0, p1, p2, p3 = C[max(i - 1, 0)], C[i], C[i + 1], C[min(i + 2, len(C) - 1)]
            t = (C[i][0] - z) / (C[i][0] - C[i + 1][0])
            out = []
            for k in (1, 2, 3):                          # Catmull-Rom, uniform in the key index
                a, b, c, d = p0[k], p1[k], p2[k], p3[k]
                out.append(0.5 * ((2 * b) + (-a + c) * t + (2 * a - 5 * b + 4 * c - d) * t * t
                                  + (-a + 3 * b - 3 * c + d) * t ** 3))
            return tuple(out)


def _env(z):
    T = TROUSER_ENV
    if z >= T[0][0]:
        return T[0][1:]
    if z <= T[-1][0]:
        return T[-1][1:]
    for (z0, *a), (z1, *b) in zip(T, T[1:]):
        if z1 <= z <= z0:
            f = (z0 - z) / (z0 - z1)
            return tuple(p + (q - p) * f for p, q in zip(a, b))


_SEC, _HULL = {}, {}
HULL = (1.160, 0.250)               # below the hips each radius is the concave hull of its profile:


def _hull(k):
    """Upper concave hull (z, r) of radius k (1 rx, 2 ryf, 3 ryb) between HULL[0] and HULL[1]: where
    the trousers push the cloth out it bridges in straight lines (a clean cone, no bulge)."""
    if k not in _HULL:
        pts = []
        n = int(round((HULL[0] - HULL[1]) / 0.005))
        for i in range(n + 1):
            zi = HULL[0] - 0.005 * i
            pts.append((zi, _section_raw(zi)[k]))
        hull = []
        for p in pts:
            while len(hull) >= 2:
                (z1, r1), (z2, r2) = hull[-2], hull[-1]
                if (r2 - r1) * (p[0] - z1) - (p[1] - r1) * (z2 - z1) <= 0:
                    hull.pop()
                else:
                    break
            hull.append(p)
        _HULL[k] = hull
    return _HULL[k]


def _hull_r(k, z):
    H = _hull(k)
    for (z1, r1), (z2, r2) in zip(H, H[1:]):
        if z2 <= z <= z1:
            return r1 + (r2 - r1) * (z1 - z) / (z1 - z2)
    return None


def section(z):
    """(cy, rx, ry_front, ry_back) of the layer-0 outer surface at height z."""
    key = round(z, 6)
    if key in _SEC:
        return _SEC[key]
    s = list(_section_raw(z))
    if HULL[1] <= z <= HULL[0]:
        for k in (1, 2, 3):
            h = _hull_r(k, z)
            if h is not None and h > s[k]:
                s[k] = h
    _SEC[key] = tuple(s)
    return _SEC[key]


def _section_raw(z):
    cy, bx, by = _bare(z)
    off = OFF[0] + (OFF[1] - OFF[0]) * _ramp(z)
    w = _sstep(1.18, 1.10, z)                          # cone takes over below the belt
    out = []
    for base, cone in zip((bx + off, by + off, by + off), _cone(z)):
        out.append(base + (_smax(base, cone) - base) * w)
    rx, ryf, ryb = out
    if z < 1.17:                                       # layer 0's inner face clears the trousers
        mx, yf, yb = _env(z)
        need = TROUSER_CLR + thick(z)
        f = _sstep(1.17, 1.12, z)
        rx += (_smax(rx, mx + need) - rx) * f
        ryf += (_smax(ryf, -yf + cy + need) - ryf) * f
        ryb += (_smax(ryb, yb - cy + need) - ryb) * f
    return cy, rx, ryf, ryb


def surf(a, z, lift_=0.0):
    """Point on the skirt.  a: angle (rad); lift_: layer offset along the horizontal normal."""
    cy, rx, ryf, ryb = section(z)
    c, s = math.cos(a), math.sin(a)
    ry = ryb if s > 0 else ryf
    nx, ny = ry * c, rx * s
    nl = math.hypot(nx, ny)
    return V((rx * c + nx / nl * lift_, cy + ry * s + ny / nl * lift_, z))


def ang_x(x, z, back, lift_=0.0):
    """Angle on the front (back=False) or back half whose surface point (at lift_) has this x."""
    rx = section(z)[1]
    q = math.acos(max(-1.0, min(1.0, x / (rx + lift_))))
    a = q if back else -q
    for _ in range(5):
        err = x - surf(a, z, lift_).x
        a += err / (-(rx + lift_) * math.sin(a) or -1e-6)
    return a


# ------------------------------------------------------------------ panels
class Hem:
    """Straight-segment hem: z at u = 0, at the tip u_tip, at u = 1."""

    def __init__(self, z0, z_tip, z1, u_tip):
        self.z0, self.zt, self.z1, self.ut = z0, z_tip, z1, u_tip

    def __call__(self, u):
        if self.ut <= 0.0:
            return self.zt + (self.z1 - self.zt) * u
        if self.ut >= 1.0:
            return self.z0 + (self.zt - self.z0) * u
        if u <= self.ut:
            return self.z0 + (self.zt - self.z0) * u / self.ut
        return self.zt + (self.z1 - self.zt) * (u - self.ut) / (1 - self.ut)


class Panel:
    """A cloth slab: edges e0(z), e1(z) -> angle (rad, e0 < e1), hem(u) -> z, on layer `layer`."""

    def __init__(self, name, layer, e0, e1, hem, n_div, top=Z_TOP, rows=BELT_ROWS, flare=None):
        self.name, self.layer, self.e0, self.e1, self.hem = name, layer, e0, e1, hem
        self.n_div, self.top = n_div, top
        self.flare = flare                  # (amount, z from, z to, u from, u to): the free point kicks out
        z_low = max(hem(i / 16) for i in range(17)) + BEV_W * 3 + 0.02
        self.rows = [z for z in rows if z <= top + 1e-9 and z > z_low]
        if not self.rows or self.rows[0] < top - 1e-9:
            self.rows = [top] + self.rows

    def kick(self, a, z):
        """Extra outward lift near the free point (only where no higher panel lies over it)."""
        if not self.flare:
            return 0.0
        amt, z0, z1, u0, u1 = self.flare
        a0, a1 = self.e0(z), self.e1(z)
        u = (a - a0) / (a1 - a0)
        return amt * _sstep(z0, z1, z) * _sstep(u0, u1, u)

    def lift_at(self, a, z):
        return lift(self.layer, z) + self.kick(a, z)

    def P(self, a, z, extra=0.0):
        return surf(a, z, self.lift_at(a, z) + extra)

    def width(self, z):
        a0, a1 = self.e0(z), self.e1(z)
        pts = [self.P(a0 + (a1 - a0) * i / 6, z) for i in range(7)]
        return sum((q - p).length for p, q in zip(pts, pts[1:]))

    def covers(self, a, z):
        if z > self.top:
            return False
        a0, a1 = self.e0(z), self.e1(z)
        a = (a - a0 + math.pi) % math.tau - math.pi + a0
        if not a0 <= a <= a1:
            return False
        return z >= self.hem((a - a0) / (a1 - a0))

    def _us(self, eps):
        ut = self.hem.ut
        lo, hi = eps, 1.0 - eps
        n = self.n_div
        if 0.0 < ut < 1.0:
            nb = max(1, min(n - 1, round(n * (ut - lo) / (hi - lo))))
            inner = [lo + (ut - lo) * i / nb for i in range(nb)] + [ut + (hi - ut) * i / (n - nb) for i in range(n - nb + 1)]
        else:
            inner = [lo + (hi - lo) * i / n for i in range(n + 1)]
        return [0.0] + inner + [1.0]

    def grid(self):
        """-> rows of (angle, z) params.  Columns: edge, chamfer loop, interior, chamfer loop, edge;
        rows: belt rows, even rows, chamfer loop above the hem, hem."""
        hem = self.hem
        zb = sum(hem(i / 8) for i in range(9)) / 9
        w_hem = max(self.width(zb), 0.02)
        us_hem = self._us(min(BEV_W / w_hem, 0.2))

        def inset(u):                                  # vertical offset of the hem chamfer loop
            du = 1e-3
            s = []
            for u0, u1 in ((max(u - du, 0.0), u), (u, min(u + du, 1.0))):
                if u1 > u0:
                    s.append(abs(hem(u1) - hem(u0)) / ((u1 - u0) * w_hem))
            return BEV_W * math.sqrt(1.0 + max(s) ** 2)
        z_last = self.rows[-1]
        span = max(z_last - (hem(u) + inset(u)) for u in us_hem)
        n_free = max(1, round(span / ROW_H))
        nrow = len(self.rows) + n_free + 1          # top rows .. inset row, then hem
        G = []
        for k in range(nrow):
            # z per column for this row (columns indexed by their hem-row position)
            zs = []
            for u in us_hem:
                zi = hem(u) + inset(u)
                if k < len(self.rows):
                    z = self.rows[k]
                    if z < zi + 0.01:
                        z = zi + 0.01 + (len(self.rows) - k) * 0.002
                elif k < nrow - 1:
                    f = (k - len(self.rows) + 1) / n_free
                    z = z_last + (zi - z_last) * f
                else:
                    z = hem(u)
                zs.append(z)
            zm = sum(zs) / len(zs)
            w = max(self.width(zm), 0.02)
            us = self._us(min(BEV_W / w, 0.2)) if k < nrow - 1 else us_hem
            row = []
            for u, z in zip(us, zs):
                a0, a1 = self.e0(z), self.e1(z)
                row.append((a0 + (a1 - a0) * u, z))
            G.append(row)
        return G


def slab(panel, name=None, topo="main"):
    """Closed all-quad slab: outer face (edge loop pushed down = chamfer), rim, inner face."""
    G = panel.grid()
    nr, nc = len(G), len(G[0])
    verts, faces = [], []

    def boundary(k, j):
        return k in (0, nr - 1) or j in (0, nc - 1)
    for k in range(nr):
        for j in range(nc):
            a, z = G[k][j]
            verts.append(panel.P(a, z, -BEV_H * thick(z) if boundary(k, j) else 0.0))
    off = len(verts)
    for k in range(nr):
        for j in range(nc):
            a, z = G[k][j]
            verts.append(panel.P(a, z, -thick(z)))
    O = lambda k, j: k * nc + j  # noqa: E731
    for k in range(nr - 1):
        for j in range(nc - 1):
            f = (O(k, j), O(k + 1, j), O(k + 1, j + 1), O(k, j + 1))
            faces.append(f)
            faces.append(tuple(i + off for i in reversed(f)))
    loop = ([(0, j) for j in range(nc)] + [(k, nc - 1) for k in range(1, nr)]
            + [(nr - 1, j) for j in range(nc - 2, -1, -1)] + [(k, 0) for k in range(nr - 2, 0, -1)])
    for (k0, j0), (k1, j1) in zip(loop, loop[1:] + loop[:1]):
        a, b = O(k0, j0), O(k1, j1)
        faces.append((a, a + off, b + off, b))
    ob = lib.mesh_object(name or panel.name, verts, faces, topo=topo, merge=0.0, sharp_angle=30.0)
    me = ob.data
    if sum(p.normal.x * p.center.x + p.normal.y * (p.center.y - section(p.center.z)[0]) for p in me.polygons) < 0:
        me.flip_normals()
        me.update()
    return ob


# ------------------------------------------------------------------ the layout
def front_over_edge_x(z):
    """True x of skirt_front_L's inner edge (the coat flap's edge line carried down to the hem)."""
    (z0, x0), (z1, x1) = OVER_EDGE
    if z > z0:
        return x0 - UP_SLOPE * (z - z0)
    return x0 + (x1 - x0) * (z - z0) / (z1 - z0)


def front_under_edge_x(z):
    """True x of skirt_front_R's inner edge: < 0 (wearer's right) below the apex, running on under
    skirt_front_L to +UNDER_X_TOP above it."""
    (z0, x0), (z1, x1) = UNDER_EDGE
    slope = (x0 - x1) / (z0 - z1)
    return -_smax(-(x0 + slope * (z - z0)), -UNDER_X_TOP, 0.02)


def _front_out_x(z):
    return FRONT_OUT[0] + FRONT_OUT[1] * (1.0 - z)


def _edge_lin(p):
    return lambda z: D(_lin(z, p[0], p[1]))  # noqa: E731


_PANELS = {}


def panels():
    if _PANELS:
        return _PANELS
    L = LAYER
    P = {}
    lay = L["front_over"]
    P["front_over"] = Panel("skirt_front_L", lay,
                            lambda z: ang_x(front_over_edge_x(z), z, False, lift(lay, z)),
                            lambda z: ang_x(_front_out_x(z), z, False, lift(lay, z)),
                            Hem(FRONT_HEM["over"][0], FRONT_HEM["over"][0], FRONT_HEM["over"][1], 0.0), 6)
    lay = L["front_under"]
    P["front_under"] = Panel("skirt_front_R", lay,          # built on +X: build x = -true x
                             lambda z: ang_x(-front_under_edge_x(z), z, False, lift(lay, z)),
                             lambda z: ang_x(_front_out_x(z) + 0.004, z, False, lift(lay, z)),
                             Hem(FRONT_HEM["under"][0], FRONT_HEM["under"][0], FRONT_HEM["under"][1], 0.0), 6)
    lay = L["side_front"]
    P["side_front"] = Panel("skirt_side_front_L", lay,
                            lambda z: ang_x(_front_out_x(z) - SF_IN, z, False, lift(lay, z)),
                            _edge_lin(SF_OUT), Hem(SF_HEM[0], SF_HEM[1], SF_HEM[1], 1.0), 4, flare=SF_KICK)
    P["side"] = Panel("skirt_side_L", L["side"], _edge_lin(SD_EDGES[0]), _edge_lin(SD_EDGES[1]),
                      Hem(SD_HEM[0], SD_HEM[1], SD_HEM[1], 1.0), 6, flare=SD_KICK)
    lay = L["side_back"]
    P["side_back"] = Panel("skirt_side_back_L", lay, _edge_lin(SB_OUT),
                           lambda z: ang_x(_lin(z, *SB_IN), z, True, lift(lay, z)),
                           Hem(SB_HEM[0], SB_HEM[1], SB_HEM[1], 1.0), 5)
    lay = L["back"]

    def vent_x(z):
        return VENT_X0 + (VENT_X1 - VENT_X0) * max(0.0, (VENT_TOP - z) / (VENT_TOP - 0.33))
    P["back"] = Panel("skirt_back_L", lay, _edge_lin(BK_OUT), lambda z: ang_x(vent_x(z), z, True, lift(lay, z)),
                      Hem(BK_HEM[0], BK_HEM[1], BK_HEM[2], BK_HEM[3]), 6)
    lay = L["back_upper"]
    P["back_upper"] = Panel("skirt_back_upper_L", lay, _edge_lin(BU_OUT),
                            lambda z: ang_x(BU_SEAM_X, z, True, lift(lay, z)),
                            Hem(BU_HEM[0], BU_HEM[1], BU_HEM[1], 1.0), 6)
    lay = L["tab"]
    P["tab"] = Panel("skirt_back_tab", lay, lambda z: ang_x(TAB_X, z, True, lift(lay, z)),
                     lambda z: ang_x(-TAB_X, z, True, lift(lay, z)),
                     Hem(TAB_CORNER_Z, TAB_TIP_Z, TAB_CORNER_Z, 0.5), 6, top=TAB_TOP)
    _PANELS.update(P)
    return P


def tab_bar(name="skirt_back_tab_bar"):
    """Raised bar down the tab's centre line, from under the belt to a small spear point."""
    tab = panels()["tab"]
    a = D(90.0)
    zs = [BAR_TOP, 1.188, 1.170, 1.140, 1.100, 1.060, 1.025, 1.000, 0.992, 0.984, BAR_TIP]
    ws = [BAR_W, BAR_W, BAR_W, BAR_W, BAR_W * 0.95, BAR_W * 0.9, BAR_W * 0.9, BAR_W * 1.6, BAR_W * 1.4,
          BAR_W * 0.7, 0.0006]
    rings = []
    for z, w in zip(zs, ws):
        p = tab.P(a, z)
        n = V((0.0, 1.0, 0.0))
        h = BAR_H * (0.35 + 0.65 * _sstep(BAR_TOP - 0.002, BAR_TOP - 0.016, z))   # low under the belt
        x = V((1.0, 0.0, 0.0))
        base = p - n * 0.0008
        rings.append([base - x * w, base - x * w * 0.5 + n * (h + 0.0008), base + x * w * 0.5 + n * (h + 0.0008),
                      base + x * w])
    verts, faces = lib.loft(rings, closed=True)
    last = (len(rings) - 1) * 4
    faces += [(3, 2, 1, 0), (last, last + 1, last + 2, last + 3)]
    return lib.mesh_object(name, verts, faces, topo="rigid", sharp_angle=30)


# ------------------------------------------------------------------ queries for other authors
def outer_radius(a_deg, z):
    """Horizontal distance from the Z axis to the outermost skirt cloth at (angle, z), LEFT side
    (mirror the angle for the right).  The asymmetric front counts both front panels.  Where no
    panel covers the point (front opening, below the hems) it returns the layer-0 surface.
    Gear hangs on this."""
    a = D(a_deg)
    zq = min(z, Z_TOP)
    best = 0.0
    for p in panels().values():
        if p.covers(a, zq):
            best = max(best, p.lift_at(a, zq))
    q = surf(a, zq, best)
    return math.hypot(q.x, q.y)


# ------------------------------------------------------------------ build
def _remove(ob):
    me = ob.data
    bpy.data.objects.remove(ob)
    bpy.data.meshes.remove(me)


def build():
    _SEC.clear()
    _HULL.clear()
    _PANELS.clear()
    P = panels()
    objs = []
    objs.append(slab(P["front_over"]))
    u = slab(P["front_under"], "skirt_front_tmp")
    objs.append(lib.mirror_copy(u, "skirt_front_R"))
    _remove(u)
    for key in ("side_front", "side", "side_back", "back", "back_upper"):
        o = slab(P[key])
        objs += [o, lib.mirror_copy(o, o.name[:-2] + "_R")]
    objs.append(slab(P["tab"]))
    objs.append(tab_bar())
    return objs
