"""Upper half of the double-breasted great coat (round 2, spec v2): torso core with armholes,
overlapping front flap, big notched lapels on both sides, tall flared stand collar with a rolled
edge, 2 x 3 domed buttons, back yoke, converging piped seams and the dagger-cross on the back.

Topology of the torso core (32 around, all quads)
-------------------------------------------------
* rings 0..9      planar horizontal rings, coat_split_z + 1 cm -> just under the armpit
* row A           first free ring; its 5 side columns are the bottom of the armhole loop
* R1 / R0         one concentric 16-quad loop around each armhole; R0 == spec.armhole_ring(16, side)
* front / back    13-column sheets between the two armhole loops, joined along the shoulder seam,
                  with a 16-edge neck opening (hidden by the collar)
The only irregular vertices are two valence-5 poles per armhole (front + back lower corner of the
loop) and four valence-3 corners on the neck opening boundary.

Everything above ring 9 is positioned by a small thin-plate (bi-Laplacian, uniform weights) solve
with the planar rings, the armhole seam, the neck line, the shoulder seam and the yoke seam pinned,
and the bare torso + coat_offset as an obstacle.  So the core loops follow the yoke seam and the
whole shoulder re-flows when spec changes.

All outer panels (flap, lapels, seams, buttons, emblem) are laid out in a flat (U, z) "pattern"
space - read off the reference in world (x, z) - and wrapped onto the solved core by ray casting,
then offset along the smooth normal, so they follow the layering table in docs/PLAN.md.
Faces nobody can see (the undersides of panels lying on the coat) are not built.
"""
import math

import bpy
import numpy as np
from mathutils import Vector as V
from mathutils.bvhtree import BVHTree
from mathutils.interpolate import poly_3d_calc

from infantry import lib, spec

C = spec.CLOTH
OFF = C["coat_offset"]
BELT_LO, BELT_HI = C["belt_z"]
BELT_MAX = 0.036                         # DETAIL_PASS v2: inside the belt zone nothing exceeds body + 0.036
Z_BOTTOM = C["coat_split_z"] + 0.010     # 1.195: coat torso ends under the belt (DETAIL_PASS v2 seam)
ARMPIT_Z = spec.armhole_ring(16)[8].z
Z_TUBE_TOP = ARMPIT_Z - 0.035            # last planar ring, a little under the armpit
N_AROUND = 32
N_TUBE = 10                              # planar rings (~2.3 cm apart)
R_REF = spec.torso_section(spec.TORSO[4][0])[1] + OFF   # pattern space: U = R_REF * azimuth (chest)
NECK_Z = spec.TORSO[6][0]                # neck base

# ---------------------------------------------------------------- read off docs/reference.png
# (world metres, measured with a world-coordinate grid laid over the sheet's FRONT / BACK / SIDE views)
# front_torso.png
LAPEL_TOP_X = 0.050          # collar front edge meets the lapel's top edge here (both sides)
LAPEL_PEAK = (0.155, 1.500)  # outer point of the lapels, right under the shoulder armour
LAPEL_BEND = (0.077, 1.462)  # the lapel's lower edge turns steeper here
LAPEL_V = (0.000, 1.495)     # the two lapels' roll edges cross here (bottom of the neck V)
LAPEL_TIP = (-0.070, 1.345)  # bottom tip of the overlapping (wearer's left) lapel
FLAP_EDGE_X = LAPEL_TIP[0]   # the free front edge then runs straight down to the belt
FRONT_SEAM_X = 0.125         # princess seam lines on both fronts (the flap ends in the +X one)
BUTTONS = [(-0.062, 1.395), (0.075, 1.395), (-0.035, 1.333), (0.070, 1.333), (-0.035, 1.272), (0.070, 1.272)]
BUTTON_R = 0.015             # large domed buttons (DETAIL_PASS checklist)
PEAK_BUTTON_IN = 0.052       # the lapel-peak buttons sit this far in from the peak
# back_torso.png
YOKE_Z = 1.515               # yoke lower edge at the centre back ...
YOKE_DROP = (0.022, 0.150)   # ... and how far it falls by x = +-0.150 (nearly flat, drops at the shoulders)
YOKE_APEX = 0.004            # small point at the centre
BACK_SEAM_X = (0.135, 0.055) # double piped princess seams: x at the yoke -> x at the waist
EMBLEM_SIZE = 0.060          # dagger-cross: ~0.20 m tall (3.3 x size), top tip just under the yoke edge
# side_torso.png / callout_collar.png
COLLAR_OPEN_TOP = 68.0       # half-angle (deg from front) of the collar's front opening at the top
COLLAR_DROP = 0.040          # top edge falls from collar_top_z at the back to the front points (side view)
COLLAR_BASE_GROW = 0.030     # the collar stands on the shoulders: base +-0.123 wide (back view) ...
COLLAR_BASE_GROW_FRONT = 0.004   # ... narrowing to the lapel tops at the front
COLLAR_TOP_OUT = 0.010       # wall top outside collar_top_r: the sheet's collar is ~0.14 wide at the top
COLLAR_POINT_FWD = 0.068     # the front points flare forward ...
COLLAR_POINT_OUT = 0.008     # ... and out
COLLAR_THICK = 0.012         # wall thickness
COLLAR_RIM_R = 0.0085        # rolled / folded edge along the top and both front edges

# ---------------------------------------------------------------- layering (own constants)
FLAP_OFF = 0.008             # every overlapping panel steps out 8 mm (docs/PLAN.md)
STITCH = dict(width=0.0035, thickness=0.0012, sink=0.0006)     # thin top-stitch lines
PIPING = dict(width=0.0065, thickness=0.0025, sink=0.001)      # raised seam piping
PIPE_GAP = 0.011             # spacing of the double seam lines
STEP = 0.022                 # target quad / segment size on visible cloth


# ============================================================================ small helpers
def _coat_sec(z):
    cy, rx, ry = spec.torso_section(z)
    return cy, rx + OFF, ry + OFF


def _ellipse_ring(z, rx, ry, cy, n=N_AROUND, samples=1440):
    """Equal arc-length ellipse ring; index 0 = +X, CCW from above (same convention as lib.ring_z)."""
    pts = [(rx * math.cos(math.tau * i / samples), ry * math.sin(math.tau * i / samples)) for i in range(samples + 1)]
    acc = [0.0]
    for a, b in zip(pts, pts[1:]):
        acc.append(acc[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    out, k = [], 0
    for i in range(n):
        target = acc[-1] * i / n
        while acc[k + 1] < target:
            k += 1
        t = (target - acc[k]) / max(acc[k + 1] - acc[k], 1e-12)
        out.append(V((pts[k][0] + (pts[k + 1][0] - pts[k][0]) * t, cy + pts[k][1] + (pts[k + 1][1] - pts[k][1]) * t, z)))
    return out


def _neck_frame():
    """Neck line (collar seam) ellipse derived from the bare body + coat offset."""
    z_side = NECK_Z + 0.018                  # just above the neck base
    z_front = z_side - 0.036
    cy_s, rx_s, _ = _coat_sec(z_side)
    cy_f, _, ry_f = _coat_sec(z_front)
    cy_b, _, ry_b = _coat_sec(z_side)
    front, back = cy_f - ry_f, cy_b + ry_b
    return dict(z_side=z_side, drop=z_side - z_front, rx=rx_s, ry=(back - front) / 2, cy=(back + front) / 2)


NECK = _neck_frame()
COLLAR_OPEN_BASE = math.degrees(math.asin(LAPEL_TOP_X / (NECK["rx"] + COLLAR_BASE_GROW_FRONT)))


def _neckline(phi, grow=0.0, dz=0.0):
    """Point on the neck line. phi as in lib.ring_z (0 = +X, 90deg = back, 270deg = front)."""
    s, c = math.sin(phi), math.cos(phi)
    z = NECK["z_side"] - NECK["drop"] * max(0.0, -s) ** 1.5 + dz
    return V(((NECK["rx"] + grow) * c, NECK["cy"] + (NECK["ry"] + grow) * s, z))


def _yoke_z(x):
    t = min(abs(x) / YOKE_DROP[1], 1.6)
    return YOKE_Z - YOKE_DROP[0] * t ** 4 + YOKE_APEX * max(0.0, 1.0 - abs(x) / 0.03)


def _back_surface_y(x, z):
    cy, rx, ry = _coat_sec(z)
    return cy + ry * math.sqrt(max(0.0, 1.0 - (x / rx) ** 2))


# ============================================================================ torso core
_A_ALIAS = {30: (1, 6), 31: (1, 7), 0: (1, 8), 1: (1, 9), 2: (1, 10),
            18: (-1, 6), 17: (-1, 7), 16: (-1, 8), 15: (-1, 9), 14: (-1, 10)}


def _canon(k):
    """Resolve the aliases between the tube, the armhole loops and the front/back sheets."""
    kind = k[0]
    if kind == "F":
        _, r, c = k
        if r == 0:
            return _canon(("A", c))
        if c == 30:
            return ("R1", 1, 6 - r)
        if c == 18:
            return ("R1", -1, 6 - r)
        return k
    if kind == "B":
        _, r, c = k
        if r == 0:
            return _canon(("A", c))
        if c == 2:
            return ("R1", 1, (10 + r) % 16)
        if c == 14:
            return ("R1", -1, (10 + r) % 16)
        if r == 6:
            return _canon(("F", 6, 32 - c))
        return k
    if kind == "A":
        c = k[1] % N_AROUND
        return ("R1",) + _A_ALIAS[c] if c in _A_ALIAS else ("A", c)
    if kind in ("R1", "R0"):
        return (kind, k[1], k[2] % 16)
    return ("t", k[1], k[2] % N_AROUND)


def _core_topology():
    index, faces = {}, []

    def vid(*k):
        k = _canon(k)
        if k not in index:
            index[k] = len(index)
        return index[k]

    for j in range(N_TUBE - 1):
        for c in range(N_AROUND):
            faces.append((vid("t", j, c), vid("t", j, c + 1), vid("t", j + 1, c + 1), vid("t", j + 1, c)))
    for c in range(N_AROUND):
        faces.append((vid("t", N_TUBE - 1, c), vid("t", N_TUBE - 1, c + 1), vid("A", c + 1), vid("A", c)))
    strap_f = list(range(18, 22)) + list(range(26, 30))
    strap_b = list(range(2, 6)) + list(range(10, 14))
    for r in range(6):
        for c in (range(18, 30) if r < 4 else strap_f):
            faces.append((vid("F", r, c), vid("F", r, c + 1), vid("F", r + 1, c + 1), vid("F", r + 1, c)))
        for c in (range(2, 14) if r < 4 else strap_b):
            faces.append((vid("B", r, c), vid("B", r, c + 1), vid("B", r + 1, c + 1), vid("B", r + 1, c)))
    for side in (1, -1):
        for i in range(16):
            f = (vid("R1", side, i), vid("R1", side, i + 1), vid("R0", side, i + 1), vid("R0", side, i))
            faces.append(f)
    return index, faces


def _neck_keys():
    """The 16 neck-opening vertices in ring order starting at the centre front (phi = 270deg)."""
    ks = [("F", 4, 24), ("F", 4, 25), ("F", 4, 26), ("F", 5, 26), ("F", 6, 26), ("B", 5, 6), ("B", 4, 6),
          ("B", 4, 7), ("B", 4, 8), ("B", 4, 9), ("B", 4, 10), ("B", 5, 10), ("F", 6, 22), ("F", 5, 22),
          ("F", 4, 22), ("F", 4, 23)]
    return [_canon(k) for k in ks]


YOKE_ROW = 2                 # back sheet row that carries the yoke seam
OBSTACLE_W = 1.5
SEAM_R1 = 0.028              # width of the armhole loop on top of the shoulder
SEAM_ARCH = 0.006            # padded shoulder: the shoulder seam arches up this much


def _body_samples():
    """Dense samples (points + outward normals) of the bare torso, used as an obstacle for the cloth."""
    pts, nrm = [], []
    dz = 0.004
    z_lo, z_hi = Z_BOTTOM, spec.TORSO[-1][0]
    for z in np.arange(z_lo, z_hi + 1e-9, dz):
        z = float(z)
        cy, rx, ry = spec.torso_section(z)
        cy2, rx2, ry2 = spec.torso_section(min(z + dz, z_hi))
        cy1, rx1, ry1 = spec.torso_section(z - dz)
        for k in range(96):
            a = math.tau * k / 96
            ca, sa = math.cos(a), math.sin(a)
            pts.append((rx * ca, cy + ry * sa, z))
            du = V((-rx * sa, ry * ca, 0.0))
            dv = V(((rx2 - rx1) * ca, (cy2 - cy1) + (ry2 - ry1) * sa, 2 * dz if z + dz <= z_hi else dz))
            nrm.append(tuple(du.cross(dv).normalized()))
    return np.array(pts), np.array(nrm)


def _solve_core(extra_pins=None):
    index, faces = _core_topology()
    n = len(index)
    P = np.zeros((n, 3))
    pinned = np.zeros(n, bool)

    def pin(k, p):
        i = index[_canon(k)]
        P[i] = p
        pinned[i] = True

    for j in range(N_TUBE):
        z = Z_BOTTOM + (Z_TUBE_TOP - Z_BOTTOM) * j / (N_TUBE - 1)
        cy, rx, ry = _coat_sec(z)
        for c, p in enumerate(_ellipse_ring(z, rx, ry, cy)):
            pin(("t", j, c), p)
    for side in (1, -1):
        for i, p in enumerate(spec.armhole_ring(16, side)):
            pin(("R0", side, i), p)
    for i, k in enumerate(_neck_keys()):
        pin(k, _neckline(math.radians(270.0 + 22.5 * i)))
    # shoulder seam: gently arched straight line from the neck to the top of the armhole (padded shoulder)
    for side in (1, -1):
        a = _neckline(0.0 if side > 0 else math.pi)
        b = spec.armhole_ring(16, side)[0]
        r1 = b + (a - b).normalized() * SEAM_R1
        for k in range(1, 5):
            t = k / 4
            p = a.lerp(r1, t) + V((0, 0, SEAM_ARCH * math.sin(math.pi * t * 0.85)))
            pin(("F", 6, 26 + k) if side > 0 else ("F", 6, 22 - k), p)
    for k, p in (extra_pins or {}).items():
        pin(k, p)

    nbr = [set() for _ in range(n)]
    edge_faces = {}
    for f in faces:
        for a, b in zip(f, f[1:] + f[:1]):
            nbr[a].add(b)
            nbr[b].add(a)
            edge_faces[frozenset((a, b))] = edge_faces.get(frozenset((a, b)), 0) + 1
    boundary = np.zeros(n, bool)
    for e, cnt in edge_faces.items():
        if cnt == 1:
            for i in e:
                boundary[i] = True
    assert pinned[boundary].all(), "every open boundary of the core has to be pinned"

    free = np.where(~pinned)[0]
    col = {int(i): k for k, i in enumerate(free)}
    rows = [i for i in range(n) if not boundary[i] and (not pinned[i] or any(not pinned[j] for j in nbr[i]))]
    A = np.zeros((len(rows), len(free)))
    rhs = np.zeros((len(rows), 3))
    for r, i in enumerate(rows):
        w = 1.0 / len(nbr[i])
        for j, wj in [(i, 1.0)] + [(j, -w) for j in nbr[i]]:
            if pinned[j]:
                rhs[r] -= wj * P[j]
            else:
                A[r, col[j]] += wj
    # obstacle iterations: the cloth may never come closer to the bare body than the coat offset
    body_p, body_n = _body_samples()
    target = {}
    for _ in range(12):
        extra_a = np.zeros((len(target), len(free)))
        extra_b = np.zeros((len(target), 3))
        for r, (i, t) in enumerate(target.items()):
            extra_a[r, col[i]] = OBSTACLE_W
            extra_b[r] = OBSTACLE_W * t
        sol, *_ = np.linalg.lstsq(np.vstack([A, extra_a]), np.vstack([rhs, extra_b]), rcond=None)
        P[free] = sol
        for i in free:
            d = body_p - P[i]
            k = int(np.argmin((d * d).sum(1)))
            sd = float((P[i] - body_p[k]).dot(body_n[k]))
            if sd < OFF + 0.002 or int(i) in target:
                target[int(i)] = P[i] + (OFF + 0.002 - sd) * body_n[k] if sd < OFF + 0.002 else P[i].copy()
    verts = [V(p) for p in P]
    if extra_pins is None:
        return index, verts, faces
    # turned-in hem around the neck opening (added after the solve so it does not bend the shoulders)
    ring = [index[k] for k in _neck_keys()]
    for grow, dz in ((-0.010, -0.016), (-0.020, -0.034)):
        lip = []
        for i in range(16):
            lip.append(len(verts))
            verts.append(_neckline(math.radians(270.0 + 22.5 * i), grow=grow, dz=dz))
        for i in range(16):
            j = (i + 1) % 16
            faces.append((ring[i], ring[j], lip[j], lip[i]))
        ring = lip
    return index, verts, faces


def _yoke_pins():
    """Two-pass solve: pass 1 says where the yoke row lies; pass 2 pins that row onto the yoke seam curve."""
    index, verts, _ = _solve_core()
    pins = {}
    for c in range(3, 14):
        x = verts[index[_canon(("B", YOKE_ROW, c))]].x
        z = _yoke_z(x)
        pins[("B", YOKE_ROW, c)] = V((x, _back_surface_y(x, z), z))
    return pins


def _check_armholes(ob):
    """The armhole boundary loops must be exactly spec.armhole_ring(16, +-1)."""
    me = ob.data
    cnt = {}
    for p in me.polygons:
        vs = list(p.vertices)
        for a, b in zip(vs, vs[1:] + vs[:1]):
            cnt[frozenset((a, b))] = cnt.get(frozenset((a, b)), 0) + 1
    bverts = {i for e, c in cnt.items() if c == 1 for i in e}
    worst = 0.0
    reach = max(spec.ARMHOLE_R) + 0.004
    for side in (1, -1):
        ring = spec.armhole_ring(16, side)
        ctr = V((spec.ARMHOLE_CENTER.x * side, spec.ARMHOLE_CENTER.y, spec.ARMHOLE_CENTER.z))
        loop = [me.vertices[i].co for i in bverts if (me.vertices[i].co - ctr).length < reach]
        assert len(loop) == 16, f"armhole loop side {side} has {len(loop)} boundary vertices, expected 16"
        for p in ring:
            worst = max(worst, min((q - p).length for q in loop))
        for q in loop:
            worst = max(worst, min((q - p).length for p in ring))
    assert worst < 1e-4, f"armhole seam mismatch {worst:.2e} m"
    return worst


# ============================================================================ wrap surface
class Surface:
    """Ray-cast access to the solved core: pattern space (U, z) -> point + smooth normal."""

    def __init__(self, ob):
        me = ob.data
        self.co = [v.co.copy() for v in me.vertices]
        self.no = [v.normal.copy() for v in me.vertices]
        self.polys = [tuple(p.vertices) for p in me.polygons]
        if sum(1 for p, n in zip(self.co, self.no) if n.dot(V((p.x, p.y, 0))) > 0) < len(self.co) / 2:
            self.no = [-n for n in self.no]
        self.bvh = BVHTree.FromPolygons(self.co, self.polys)

    def at(self, u, z, back=False):
        psi = u / R_REF
        d = V((math.sin(psi), math.cos(psi) if back else -math.cos(psi), 0.0))
        loc, _, idx, _ = self.bvh.ray_cast(V((0, 0, z)) + d, -d)
        if loc is None:
            raise ValueError(f"pattern point (U={u:.3f}, z={z:.3f}, back={back}) misses the coat torso")
        ids = self.polys[idx]
        w = poly_3d_calc([self.co[i] for i in ids], loc)
        n = V((0, 0, 0))
        for i, wi in zip(ids, w):
            n += self.no[i] * wi
        return loc, n.normalized()

    def u_of_x(self, x, z, back=False):
        """Pattern U whose wrapped point has the given world x (what you read off a front/back view)."""
        lo, hi = -1.15 * R_REF, 1.15 * R_REF
        for _ in range(26):
            mid = (lo + hi) / 2
            if self.at(mid, z, back)[0].x < x:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2

    def uz(self, x, z, back=False):
        return V((self.u_of_x(x, z, back), z))

    def point(self, x, z, back=False, off=0.0):
        """World point on the coat at reference coordinates (x, z), pushed out by `off`, + normal."""
        p, n = self.at(self.u_of_x(x, z, back), z, back)
        return p + n * off, n


# ============================================================================ pattern meshing (2D)
class Pattern:
    """Welded 2D quad mesh in (U, z) pattern space."""

    def __init__(self):
        self.pts, self.faces = [], []

    def vid(self, p):
        p = V((p[0], p[1]))
        for i, q in enumerate(self.pts):
            if (q - p).length_squared < 1e-12:
                return i
        self.pts.append(p)
        return len(self.pts) - 1

    def add_grid(self, G):
        ids = [[self.vid(p) for p in row] for row in G]
        for j in range(len(ids) - 1):
            for i in range(len(ids[0]) - 1):
                self.faces.append((ids[j][i], ids[j][i + 1], ids[j + 1][i + 1], ids[j + 1][i]))
        return ids

    def relax(self, iters=60):
        """Uniform Laplacian smoothing of the interior points (boundary stays put)."""
        nbr = [set() for _ in self.pts]
        cnt = {}
        for f in self.faces:
            for a, b in zip(f, f[1:] + f[:1]):
                nbr[a].add(b)
                nbr[b].add(a)
                cnt[frozenset((a, b))] = cnt.get(frozenset((a, b)), 0) + 1
        fixed = {i for e, c in cnt.items() if c == 1 for i in e}
        for _ in range(iters):
            new = list(self.pts)
            for i, ns in enumerate(nbr):
                if i not in fixed:
                    s = V((0.0, 0.0))
                    for j in ns:
                        s += self.pts[j]
                    new[i] = s / len(ns)
            self.pts = new
        return self


def _line(a, b, n):
    return [a.lerp(b, i / n) for i in range(n + 1)]


def _poly_n(corners, n):
    """Poly line through `corners` with n segments in total, shared out by length (corners kept)."""
    lens = [(b - a).length for a, b in zip(corners, corners[1:])]
    tot = sum(lens)
    cnt = [max(1, round(n * l / tot)) for l in lens]
    while sum(cnt) > n:
        cnt[cnt.index(max(cnt))] -= 1
    while sum(cnt) < n:
        cnt[cnt.index(max(cnt))] += 1
    out = [corners[0]]
    for a, b, k in zip(corners, corners[1:], cnt):
        out += _line(a, b, k)[1:]
    return out


def _poly_step(corners, step=STEP):
    """Poly line through `corners`, each segment cut into pieces of about `step`."""
    out = [corners[0]]
    for a, b in zip(corners, corners[1:]):
        out += _line(a, b, max(1, round((b - a).length / step)))[1:]
    return out


def _inset(corners, d, closed=True):
    """Mitered parallel of a 2D poly line, `d` to the LEFT of the direction of travel
    (= inside for a counter-clockwise outline)."""
    n = len(corners)
    out = []
    for i, p in enumerate(corners):
        prv = corners[i - 1] if (closed or i > 0) else None
        nxt = corners[(i + 1) % n] if (closed or i < n - 1) else None
        ns = []
        for a, b in ((prv, p), (p, nxt)):
            if a is not None and b is not None:
                e = (b - a).normalized()
                ns.append(V((-e.y, e.x)))
        m = (ns[0] + ns[-1]).normalized()
        out.append(p + m * (d / max(m.dot(ns[0]), 0.3)))
    return out


def _coons(bot, right, top, left):
    n, m = len(bot) - 1, len(left) - 1
    assert len(top) == n + 1 and len(right) == m + 1
    G = []
    for j in range(m + 1):
        v = j / m
        row = []
        for i in range(n + 1):
            u = i / n
            row.append((1 - v) * bot[i] + v * top[i] + (1 - u) * left[j] + u * right[j]
                       - ((1 - u) * (1 - v) * bot[0] + u * (1 - v) * bot[n] + u * v * top[n] + (1 - u) * v * top[0]))
        G.append(row)
    return G


def _tri_patch(M, ab, bc, ca):
    """All-quad fill of a 3-sided region bounded by the poly lines ab (a->b), bc, ca:
    three Coons grids around one valence-3 centre."""
    n_ab, n_bc, n_ca = len(ab) - 1, len(bc) - 1, len(ca) - 1
    assert (n_ab + n_bc + n_ca) % 2 == 0
    p = (n_ab + n_bc - n_ca) // 2
    r, q = n_ab - p, n_bc - p
    assert min(p, q, r) >= 1, "edge counts violate the triangle inequality"
    s_ab, s_bc, s_ca = ab[p], bc[q], ca[r]
    o = (s_ab + s_bc + s_ca) / 3
    M.add_grid(_coons(ab[:p + 1], _line(s_ab, o, q), _line(s_ca, o, p), ca[r:][::-1]))
    M.add_grid(_coons(bc[:q + 1], _line(s_bc, o, r), _line(s_ab, o, q), ab[p:][::-1]))
    M.add_grid(_coons(ca[:r + 1], _line(s_ca, o, p), _line(s_bc, o, r), bc[q:][::-1]))
    return M


# ============================================================================ 3D builders
def _shell(outer, inner, faces, back=True):
    """Thick panel from an open sheet: outer faces, a quad rim, and (if back) the flipped inner faces.
    Without the back the inner copy only exists along the boundary (the underside lies on the coat)."""
    directed = set()
    for f in faces:
        for a, b in zip(f, f[1:] + f[:1]):
            directed.add((a, b))
    rim = [(a, b) for a, b in directed if (b, a) not in directed]
    n = len(outer)
    if back:
        inner_id = {i: n + i for i in range(n)}
        verts = list(outer) + list(inner)
    else:
        bv = sorted({i for e in rim for i in e})
        inner_id = {i: n + k for k, i in enumerate(bv)}
        verts = list(outer) + [inner[i] for i in bv]
    out = [tuple(f) for f in faces]
    if back:
        out += [tuple(inner_id[i] for i in reversed(f)) for f in faces]
    for a, b in rim:
        out.append((b, a, inner_id[a], inner_id[b]))
    return verts, out


def _panel(name, S, M, off, thick, back=False, topo="main", under=False):
    """Wrap a pattern mesh onto the coat: outer surface at +off, underside at +off-thick."""
    outer, inner = [], []
    for u, z in M.pts:
        p, n = S.at(u, z, back)
        outer.append(p + n * off)
        inner.append(p + n * (off - thick))
    return lib.mesh_object(name, *_shell(outer, inner, M.faces, back=under), topo=topo)


def _line_on(S, pts_xz, back=False, off=0.0):
    """Reference (x, z) poly line -> points on the coat (+off) and normals."""
    P, N = [], []
    for x, z in pts_xz:
        p, n = S.point(x, z, back, off)
        P.append(p)
        N.append(n)
    return P, N


def _strip(name, S, pts_xz, back=False, off=0.0, kind=PIPING, profile="flat"):
    P, N = _line_on(S, pts_xz, back, off)
    return lib.strip_along(name, P, N, profile=profile, topo="main", **kind)


def _double(name, S, corners, back=False, off=0.0, kind=PIPING, gap=PIPE_GAP, closed=False, profile="flat",
            first=0.0):
    """Two parallel seam lines `first` and `first + gap` to the left of the poly line."""
    objs = []
    for k, d in enumerate((first, first + gap)):
        cs = _inset(corners, d, closed) if d else list(corners)
        if closed:
            cs = cs + [cs[0]]
        objs.append(_strip(f"{name}_{k}", S, _poly_step(cs), back, off, kind, profile))
    return objs


def _join(objs, name, topo="main"):
    return lib.join(objs, name, topo=topo)


# ============================================================================ collar
def _collar(S):
    """Tall stand collar: base ring stands on the shoulders (ray-cast onto the coat), flares out to
    collar_top_r, open in a wide V at the front whose points flare forward and out."""
    n_col, n_row = 30, 6
    top_rx, top_ry = (r + COLLAR_TOP_OUT for r in C["collar_top_r"])
    top_z = C["collar_top_z"] - COLLAR_RIM_R
    a0, a1 = math.radians(COLLAR_OPEN_BASE), math.radians(COLLAR_OPEN_TOP)
    k_flare, k_base = 7, 6
    base_row, top_row = [], []
    for k in range(n_col + 1):
        edge = min(k, n_col - k)
        wb = max(0.0, 1.0 - edge / k_base)
        wb = wb * wb * (3 - 2 * wb)
        grow = COLLAR_BASE_GROW + (COLLAR_BASE_GROW_FRONT - COLLAR_BASE_GROW) * wb
        phb = 1.5 * math.pi + a0 + (math.tau - 2 * a0) * k / n_col
        b = _neckline(phb, grow=grow)
        hit = S.bvh.ray_cast(V((b.x, b.y, 2.5)), V((0, 0, -1)))[0]
        base_row.append(V((b.x, b.y, (hit.z if hit else b.z) - 0.014)))
        pht = 1.5 * math.pi + a1 + (math.tau - 2 * a1) * k / n_col
        w = max(0.0, 1.0 - edge / k_flare)
        w = w * w * (3 - 2 * w)
        ex, ey = top_rx * math.cos(pht), NECK["cy"] + top_ry * math.sin(pht)
        d_back = abs((math.degrees(pht) - 90.0 + 180.0) % 360.0 - 180.0)
        back = d_back / (180.0 - COLLAR_OPEN_TOP)                           # 0 at the back .. 1 at the points
        top_row.append(V((ex + math.copysign(COLLAR_POINT_OUT * w, ex), ey - COLLAR_POINT_FWD * w,
                          top_z - COLLAR_DROP * min(back, 1.0) ** 1.6)))
    grid = []
    for h in range(n_row + 1):
        t = h / n_row
        f = t ** 1.25                                     # a slightly flaring cone
        grid.append([V((bb.x + (tt.x - bb.x) * f, bb.y + (tt.y - bb.y) * f, bb.z + (tt.z - bb.z) * t))
                     for bb, tt in zip(base_row, top_row)])
    w_ = n_col + 1
    outer, inner, normals, faces = [], [], [], []
    for h in range(n_row + 1):
        for k in range(w_):
            p = grid[h][k]
            du = grid[h][min(k + 1, n_col)] - grid[h][max(k - 1, 0)]
            dv = grid[min(h + 1, n_row)][k] - grid[max(h - 1, 0)][k]
            n = du.cross(dv).normalized()
            if n.dot(V((p.x, p.y - NECK["cy"], 0))) < 0:
                n = -n
            outer.append(p)
            normals.append(n)
            inner.append(p - n * COLLAR_THICK)
    for h in range(n_row):
        for k in range(n_col):
            a = h * w_ + k
            faces.append((a, a + 1, a + 1 + w_, a + w_))
    wall = lib.mesh_object("coat_collar", *_shell(outer, inner, faces, back=True), topo="main")
    gi = lambda h, k: h * w_ + k
    mid = lambda h, k: (outer[gi(h, k)] + inner[gi(h, k)]) / 2
    # rolled / folded edge: up the left front edge, around the top, down the right front edge
    path = [mid(h, 0) for h in range(0, n_row)] + [mid(n_row, k) for k in range(w_)] + \
           [mid(h, n_col) for h in range(n_row - 1, -1, -1)]
    path = lib.smooth_path(path, 1)
    radii = []
    for i in range(len(path)):
        a, b, c = path[max(i - 1, 0)], path[i], path[min(i + 1, len(path) - 1)]
        turn = (b - a).angle(c - b, 0.0) if 0 < i < len(path) - 1 else 0.0
        radii.append(COLLAR_RIM_R / max(math.cos(turn / 2), 0.75))
    rim = lib.lofted("coat_collar_rim", lib.tube_rings(path, radii, n=6, x_hint=(0, 0, 1)), cap_start=True,
                     cap_end=True, topo="main")
    # top-stitching ~2 cm in from the folded edge, outside (fronts + top) and inside (top)
    def lerp_pt(src, h, k):
        h0, k0 = int(h), int(k)
        h1, k1 = min(h0 + 1, n_row), min(k0 + 1, n_col)
        fh, fk = h - h0, k - k0
        P = lambda hh, kk: src[gi(hh, kk)]
        return (P(h0, k0) * (1 - fh) + P(h1, k0) * fh) * (1 - fk) + (P(h0, k1) * (1 - fh) + P(h1, k1) * fh) * fk

    hs = n_row - 1.05
    seq = [(h, 0.95) for h in np.linspace(0.6, hs, 5)] + [(hs, k) for k in np.linspace(0.95, n_col - 0.95, 28)] + \
          [(h, n_col - 0.95) for h in np.linspace(hs, 0.6, 5)]
    seq = [s for i, s in enumerate(seq) if i == 0 or s != seq[i - 1]]
    o_pts = [lerp_pt(outer, h, k) for h, k in seq]
    o_nrm = [lerp_pt(normals, h, k) for h, k in seq]
    st_out = lib.strip_along("coat_collar_stitch_out", o_pts, o_nrm, topo="main", **STITCH)
    seq_in = [(hs, k) for k in np.linspace(1.5, n_col - 1.5, 26)]
    i_pts = [lerp_pt(inner, h, k) for h, k in seq_in]
    i_nrm = [-lerp_pt(normals, h, k) for h, k in seq_in]
    st_in = lib.strip_along("coat_collar_stitch_in", i_pts, i_nrm, topo="main", **STITCH)
    # collar stand seam: piping round the base of the collar, just above the coat's neck line
    band = lib.strip_along("coat_collar_band", [outer[gi(1, k)] for k in range(w_)],
                           [normals[gi(1, k)] for k in range(w_)], topo="main", width=0.012, thickness=0.003,
                           sink=0.001)
    # folded-back front edges: a flat facing band on the outside of both front edges (callout_collar)
    facings = []
    for kk in (0.42, n_col - 0.42):
        seqf = [(h, kk) for h in np.linspace(0.3, n_row - 0.35, 8)]
        f_pts = [lerp_pt(outer, h, k) for h, k in seqf]
        f_nrm = [lerp_pt(normals, h, k) for h, k in seqf]
        facings.append(lib.strip_along(f"coat_collar_facing_{kk:.0f}", f_pts, f_nrm, topo="main", width=0.014,
                                       thickness=0.0035, sink=0.001))
    objs = [wall, rim, _join([st_out, st_in, band] + facings, "coat_collar_trim")]
    # keep clear of the head ellipsoid (+ 1 cm face mask in front)
    hc, hr = spec.HEAD_CENTER, spec.HEAD_RADII
    worst = 9.0
    for ob in objs:
        for v in ob.data.vertices:
            p = v.co
            ry = hr.y + (0.010 if p.y < hc.y else 0.0)
            worst = min(worst, math.sqrt(((p.x - hc.x) / hr.x) ** 2 + ((p.y - hc.y) / ry) ** 2 + ((p.z - hc.z) / hr.z) ** 2))
    assert worst > 1.12, f"collar too close to the head (scaled distance {worst:.3f})"
    return objs, base_row[0] + V((0, 0, 0.014))


# ============================================================================ front
LAPEL_TRIM = dict(width=0.011, thickness=0.003, sink=0.0008)   # raised border band (front_torso)
LAPEL_TRIM_IN, LAPEL_STITCH_IN = 0.008, 0.020


def _lapel_border(name, S, corners, off):
    """Raised border band + a top-stitch line inside it along a closed (counter-clockwise) outline."""
    out = []
    for k, (d, kind) in enumerate(((LAPEL_TRIM_IN, LAPEL_TRIM), (LAPEL_STITCH_IN, STITCH))):
        cs = _inset(corners, d, closed=True)
        out.append(_strip(f"{name}_{k}", S, _poly_step(cs + [cs[0]]), off=off, kind=kind))
    return out


def _front(S, lapel_top_z):
    objs = []
    T = V((LAPEL_TOP_X, lapel_top_z))
    P, Mb, Vv, B = V(LAPEL_PEAK), V(LAPEL_BEND), V(LAPEL_V), V(LAPEL_TIP)
    mir = lambda p: V((-p.x, p.y))
    T2, P2, M2 = mir(T), mir(P), mir(Mb)
    z0 = Z_BOTTOM + 0.010
    uz = lambda p: S.uz(p.x, p.y)

    # ---- big overlapping lapel (wearer's left): T -> P -> (bend) -> B -> (V) -> T, on top of the flap
    ab = _poly_n([T, P], 6)
    bc = _poly_n([P, Mb, B], 12)
    ca = _poly_n([B, Vv, T], 12)
    Mp = _tri_patch(Pattern(), [uz(p) for p in ab], [uz(p) for p in bc], [uz(p) for p in ca]).relax(60)
    lapel_off = 2 * FLAP_OFF
    objs.append(_panel("coat_lapel_L", S, Mp, lapel_off, lapel_off - 0.002))
    objs += [_join(_lapel_border("coat_lapel_L_trim", S, [T, Vv, B, Mb, P], lapel_off), "coat_lapel_L_trim")]

    # ---- under lapel (wearer's right): visible part T2 -> P2 -> M2 -> X1, tucked 12 mm under the big one
    def isect(a, b, c, d):
        r, q = b - a, d - c
        den = r.x * q.y - r.y * q.x
        t = ((c.x - a.x) * q.y - (c.y - a.y) * q.x) / den
        return a + r * t
    X1 = isect(M2, mir(B), Vv, B)                       # its lower edge meets the big lapel's roll edge
    tuck = (B - Vv).normalized()
    tuck = V((tuck.y, -tuck.x)) * 0.012                 # perpendicular, pointing in under the big lapel
    if tuck.x < 0:
        tuck = -tuck
    X1e, Ve = X1 + tuck, Vv + tuck
    G = _coons(_poly_n([T2, P2], 5), _poly_n([P2, M2, X1e], 6), _poly_n([Ve, X1e], 5), _poly_n([T2, Ve], 6))
    Mr = Pattern()
    Mr.add_grid([[uz(p) for p in row] for row in G])
    Mr.relax(30)
    r_off = FLAP_OFF + 0.002
    objs.append(_panel("coat_lapel_R", S, Mr, r_off, r_off - 0.002))
    vis = [X1, M2, P2, T2, Vv]                          # clockwise -> inset to the right = inside
    trim_r = []
    for k, (d, kind) in enumerate(((LAPEL_TRIM_IN, LAPEL_TRIM), (LAPEL_STITCH_IN, STITCH))):
        cs = _inset(vis[::-1], d, closed=True)[::-1]
        cs = [cs[0] + (cs[1] - cs[0]).normalized() * 0.004] + cs[1:]
        trim_r.append(_strip(f"coat_lapel_R_trim_{k}", S, _poly_step(cs[:4] + [cs[3].lerp(cs[4], 0.85)]),
                             off=r_off, kind=kind))
    objs.append(_join(trim_r, "coat_lapel_R_trim"))

    def lapel_low_z(x):
        """Lower edge of the big lapel at x (the under lapel mirrors it for x < 0 near its peak)."""
        if x < 0 and x < B.x:
            x = -x
        if x >= Mb.x:
            return P.y + (Mb.y - P.y) * (P.x - x) / (P.x - Mb.x)
        return Mb.y + (B.y - Mb.y) * (Mb.x - x) / (Mb.x - B.x)

    # ---- overlapping front flap: rectangle under the lapel tip + triangle tucked under the lapel's lower edge
    under = 0.012
    lower = [p + V((0.0, under)) for p in (Mb, B)]
    x_r = FRONT_SEAM_X
    t = (P.x - x_r) / (P.x - Mb.x)
    TR = V((x_r, P.y + (Mb.y - P.y) * t + under))
    BLt = V((FLAP_EDGE_X, B.y))
    BRt = V((x_r, B.y))
    Mf = Pattern()
    G = _coons(_line(V((FLAP_EDGE_X, z0)), V((x_r, z0)), 9), _line(V((x_r, z0)), BRt, 6),
               _line(BLt, BRt, 9), _line(V((FLAP_EDGE_X, z0)), BLt, 6))
    Mf.add_grid([[uz(p) for p in row] for row in G])
    hyp = _poly_n([TR, lower[0], V((BLt.x + 0.004, BLt.y + under * 0.5))], 10)
    hyp[-1] = BLt
    _tri_patch(Mf, [uz(p) for p in _line(BLt, BRt, 9)], [uz(p) for p in _line(BRt, TR, 5)], [uz(p) for p in hyp])
    Mf.relax(40)
    objs.append(_panel("coat_front_flap", S, Mf, FLAP_OFF, FLAP_OFF))
    zs = BELT_HI + 0.004
    edge = [V((FLAP_EDGE_X, zs)), V((FLAP_EDGE_X, B.y + 0.002))]
    objs.append(_join(_double("coat_flap_stitch", S, [edge[1], edge[0]], off=FLAP_OFF, kind=STITCH, gap=0.006,
                              first=0.008), "coat_flap_stitch"))

    # ---- princess seams: the flap's right edge is one line on the wearer's left, piping beside it;
    #      a double piped seam on the wearer's right, both up to the lapels
    xr = x_r + PIPE_GAP
    seams = [_strip("coat_seam_front_L", S, _poly_step([V((xr, z0)), V((xr, lapel_low_z(xr) - 0.002))]))]
    for k, x in enumerate((-x_r + PIPE_GAP / 2, -x_r - PIPE_GAP / 2)):
        seams.append(_strip(f"coat_seam_front_R{k}", S, _poly_step([V((x, z0)), V((x, lapel_low_z(x) + 0.004))])))
    objs.append(_join(seams, "coat_seams_front"))

    # ---- buttons: 2 x 3 large domes + one on each lapel peak
    on_flap = lambda x, z: FLAP_EDGE_X < x < x_r and z < lapel_low_z(x) + under
    parts = []
    for k, (x, z) in enumerate(BUTTONS):
        off = FLAP_OFF if on_flap(x, z) else 0.0
        p, n = S.point(x, z, off=off - 0.001)
        parts.append(lib.rivet(f"btn{k}", radius=BUTTON_R, height=0.009, n=12, loc=p, normal=n))
    cen_l = (T + P + B) / 3
    for k, (pk, cen, off) in enumerate(((P, cen_l, lapel_off), (P2, (T2 + P2 + X1) / 3, r_off))):
        q = pk + (cen - pk).normalized() * PEAK_BUTTON_IN
        p, n = S.point(q.x, q.y, off=off - 0.001)
        parts.append(lib.rivet(f"btnp{k}", radius=0.011, height=0.007, n=12, loc=p, normal=n))
    objs.append(_join(parts, "coat_buttons", topo="rigid"))
    return objs


# ============================================================================ back
def _yoke(index, verts):
    """Back shoulder yoke: the core quads above the yoke row (+ the row over the shoulder seam), subdivided
    once and lifted 8 mm; the underside lies on the coat and is not built."""
    vid = lambda *k: index[_canon(k)]
    quads = []
    strap_b = list(range(2, 6)) + list(range(10, 14))
    strap_f = list(range(18, 22)) + list(range(26, 30))
    for r in range(YOKE_ROW, 6):
        for c in (range(2, 14) if r < 4 else strap_b):
            quads.append((vid("B", r, c), vid("B", r, c + 1), vid("B", r + 1, c + 1), vid("B", r + 1, c)))
    for c in strap_f:
        quads.append((vid("F", 5, c), vid("F", 5, c + 1), vid("F", 6, c + 1), vid("F", 6, c)))
    used = sorted({i for q in quads for i in q})
    remap = {i: k for k, i in enumerate(used)}
    tmp = lib.mesh_object("yoke_tmp", [verts[i] for i in used], [tuple(remap[i] for i in q) for q in quads])
    lib.subdivide(tmp, 1)
    me = tmp.data
    co = [v.co.copy() for v in me.vertices]
    no = [v.normal.copy() for v in me.vertices]
    centre = V((0.0, 0.0, ARMPIT_Z))
    if sum(1 for p, n in zip(co, no) if n.dot(p - centre) > 0) < len(co) / 2:
        no = [-n for n in no]
    polys = [tuple(p.vertices) for p in me.polygons]
    bpy.data.objects.remove(tmp)
    bpy.data.meshes.remove(me)
    outer = [p + n * FLAP_OFF for p, n in zip(co, no)]
    inner = [p + n * 0.001 for p, n in zip(co, no)]
    yoke = lib.mesh_object("coat_back_yoke", *_shell(outer, inner, polys, back=False), topo="main")
    # rolled edge along the yoke seam
    edge = [verts[vid("B", YOKE_ROW, c)] for c in range(2, 15)]
    edge = lib.smooth_path(edge, 2)
    S = Surface(yoke)
    path = []
    for p in edge:
        hit = S.bvh.find_nearest(p)
        n = hit[1] if hit[1].dot(V((0, 1, 0))) > 0 else -hit[1]
        path.append(p + n * (FLAP_OFF - 0.0035) - V((0, 0, 0.001)))
    roll = lib.lofted("coat_yoke_roll", lib.tube_rings(path, 0.0055, n=6, x_hint=(0, 1, 0)), cap_start=True,
                      cap_end=True, topo="main")
    return [yoke, roll]


def _back(S):
    objs = []
    z0 = Z_BOTTOM + 0.005
    # dagger-cross, bent onto the back; its top tip just under the yoke seam
    em = lib.emblem_ornate("coat_back_emblem", size=EMBLEM_SIZE, thickness=0.006, normal=(0, 1, 0),
                           up=(0, 0, 1), crescents=False)
    top = max(v.co.z for v in em.data.vertices)
    cz = _yoke_z(0.0) - 0.010 - top
    for v in em.data.vertices:
        x, h, z = v.co.x, v.co.y, v.co.z
        p, n = S.point(x, cz + z, back=True)
        v.co = p + n * (h + 0.0012)
    lib.finalize(em, sharp_angle=25)
    objs.append(em)
    tail = min(v.co.z for v in em.data.vertices)
    # centre back seam from the end of the dagger's tail down under the belt
    seams = [_strip("coat_seam_back_C", S, _poly_step([V((0.0, z0)), V((0.0, tail + 0.006))]), back=True,
                    profile="round")]
    # double piped princess seams converging to the waist
    for side in (1, -1):
        xt, xb = BACK_SEAM_X
        top_p = V((side * xt, _yoke_z(xt) - 0.004))
        bot_p = V((side * xb, z0))
        line = [bot_p, top_p] if side > 0 else [top_p, bot_p]
        seams += _double(f"coat_seam_back_{side}", S, line, back=True, first=-PIPE_GAP / 2, profile="round")
    objs.append(_join(seams, "coat_seams_back"))
    return objs


def _armhole_seams(S, index, verts):
    """Seam piping around the front / lower half of each armhole (the yoke completes the loop on top)."""
    parts = []
    for side in (1, -1):
        pts = [verts[index[("R1", side, i)]] for i in range(1, 13)]
        nrm = []
        for p in pts:
            k = min(range(len(S.co)), key=lambda j: (S.co[j] - p).length_squared)
            nrm.append(S.no[k])
        P = lib.smooth_path(pts, 2)
        N = [n.normalized() for n in lib.smooth_path(nrm, 2)]
        parts.append(lib.strip_along(f"coat_armhole_seam_{side}", P, N, topo="main", **PIPING))
    return _join(parts, "coat_armhole_seams")


# ============================================================================ build
def build():
    index, verts, faces = _solve_core(_yoke_pins())
    core = lib.mesh_object("coat_torso", verts, faces, topo="main")
    worst = _check_armholes(core)
    print(f"torso_coat: armhole seams match spec.armhole_ring(16, +-1) (max deviation {worst:.1e} m)")
    S = Surface(core)
    collar, front_base = _collar(S)
    objs = [core] + _yoke(index, verts) + collar
    objs += _front(S, front_base.z - 0.006)
    objs += _back(S)
    objs.append(_armhole_seams(S, index, verts))
    return objs
