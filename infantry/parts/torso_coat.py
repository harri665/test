"""Upper half of the double-breasted great coat: torso core with armholes, overlapping front flap,
lapels, tall stand collar, buttons, back yoke + seams and the back emblem.

Topology of the torso core (32 around, all quads)
-------------------------------------------------
* rings 0..9      planar horizontal rings, z 1.045 -> 1.315 (equal arc length ellipse sampling)
* row A           first free ring; its 5 side columns are the bottom of the armhole loop
* R1 / R0         one concentric 16-quad loop around each armhole; R0 == spec.armhole_ring(16, side)
* front / back    13-column sheets between the two armhole loops, joined along the shoulder seam,
                  with a 16-edge neck opening (hidden by the collar)
The only irregular vertices are two valence-5 poles per armhole (front + back lower corner of the
loop) and four valence-3 corners on the neck opening boundary.

Everything above ring 9 is positioned by a small thin-plate (bi-Laplacian, uniform weights) solve
with the planar rings, the armhole seam and the neck line pinned, which gives evenly sized quads and
a smooth padded shoulder without hand-placing a single vertex.

All outer panels (flap, lapels, seams, buttons, emblem) are laid out in a flat (U, z) "pattern"
space and wrapped onto the solved core by ray casting, then offset along the smooth normal, so
they follow the layering table in docs/PLAN.md exactly.
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
TH = C["coat_thickness"]
Z_BOTTOM = 1.045            # hidden under the belt
Z_TUBE_TOP = 1.315          # last planar ring
N_AROUND = 32
N_TUBE = 10                 # planar rings
R_REF = 0.17                # pattern space: U = R_REF * azimuth


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
    z_side = spec.TORSO[6][0] + 0.018                 # 1.538, just above the neck base
    z_front = z_side - 0.036
    cy_s, rx_s, _ = _coat_sec(z_side)
    cy_f, _, ry_f = _coat_sec(z_front)
    cy_b, _, ry_b = _coat_sec(z_side)
    front, back = cy_f - ry_f, cy_b + ry_b
    return dict(z_side=z_side, drop=z_side - z_front, rx=rx_s, ry=(back - front) / 2, cy=(back + front) / 2)


NECK = _neck_frame()


def _neckline(phi, grow=0.0, dz=0.0):
    """Point on the neck line. phi as in lib.ring_z (0 = +X, 90deg = back, 270deg = front)."""
    s, c = math.sin(phi), math.cos(phi)
    z = NECK["z_side"] - NECK["drop"] * max(0.0, -s) ** 1.5 + dz
    return V(((NECK["rx"] + grow) * c, NECK["cy"] + (NECK["ry"] + grow) * s, z))


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


OBSTACLE_W = 1.5
SEAM_R1 = 0.028             # width of the armhole loop on top of the shoulder
SEAM_ARCH = 0.006


def _body_samples():
    """Dense samples (points + outward normals) of the bare torso, used as an obstacle for the cloth."""
    pts, nrm = [], []
    dz = 0.004
    zs = np.arange(1.20, 1.60 + 1e-9, dz)
    for z in zs:
        cy, rx, ry = spec.torso_section(float(z))
        cy2, rx2, ry2 = spec.torso_section(float(min(z + dz, 1.60)))
        cy1, rx1, ry1 = spec.torso_section(float(z - dz))
        for k in range(96):
            a = math.tau * k / 96
            ca, sa = math.cos(a), math.sin(a)
            pts.append((rx * ca, cy + ry * sa, z))
            du = V((-rx * sa, ry * ca, 0.0))
            dv = V(((rx2 - rx1) * ca, (cy2 - cy1) + (ry2 - ry1) * sa, 2 * dz if z + dz <= 1.60 else dz))
            nrm.append(tuple(du.cross(dv).normalized()))
    return np.array(pts), np.array(nrm)


def _solve_core():
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
    return index, [V(p) for p in P], faces


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
    for side in (1, -1):
        ring = spec.armhole_ring(16, side)
        loop = [me.vertices[i].co for i in bverts if me.vertices[i].co.x * side > 0.15 and me.vertices[i].co.z > 1.2]
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
        for _ in range(28):
            mid = (lo + hi) / 2
            if self.at(mid, z, back)[0].x < x:
                lo = mid
            else:
                hi = mid
        return (lo + hi) / 2

    def uz(self, x, z, back=False):
        return V((self.u_of_x(x, z, back), z))


# ============================================================================ pattern meshing (2D)
class Pattern:
    """Welded 2D quad mesh in (U, z) pattern space."""

    def __init__(self):
        self.pts, self.faces, self._key = [], [], {}

    def vid(self, p):
        k = (round(p[0] * 1e5), round(p[1] * 1e5))
        if k not in self._key:
            self._key[k] = len(self.pts)
            self.pts.append(V((p[0], p[1])))
        return self._key[k]

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


def _tri_patch(M, a, b, c, n_ab, n_bc, n_ca):
    """All-quad fill of a triangle with given edge counts: three grids around one valence-3 centre."""
    assert (n_ab + n_bc + n_ca) % 2 == 0
    p = (n_ab + n_bc - n_ca) // 2
    r, q = n_ab - p, n_bc - p
    assert min(p, q, r) >= 1, "edge counts violate the triangle inequality"
    s_ab, s_bc, s_ca = a.lerp(b, p / n_ab), b.lerp(c, q / n_bc), c.lerp(a, r / n_ca)
    o = (s_ab + s_bc + s_ca) / 3
    M.add_grid(_coons(_line(a, s_ab, p), _line(s_ab, o, q), _line(s_ca, o, p), _line(a, s_ca, q)))
    M.add_grid(_coons(_line(b, s_bc, q), _line(s_bc, o, r), _line(s_ab, o, q), _line(b, s_ab, r)))
    M.add_grid(_coons(_line(c, s_ca, r), _line(s_ca, o, p), _line(s_bc, o, r), _line(c, s_bc, p)))
    return M


# ============================================================================ 3D builders
def _thicken(outer, inner, faces):
    """Closed shell from an open sheet: outer faces + flipped inner faces + quad rim."""
    n = len(outer)
    out = [tuple(f) for f in faces] + [tuple(i + n for i in reversed(f)) for f in faces]
    directed = set()
    for f in faces:
        for a, b in zip(f, f[1:] + f[:1]):
            directed.add((a, b))
    for a, b in directed:
        if (b, a) not in directed:
            out.append((b, a, a + n, b + n))
    return list(outer) + list(inner), out


def _panel(name, S, M, off, thick, back=False, topo="main"):
    outer, inner = [], []
    for u, z in M.pts:
        p, n = S.at(u, z, back)
        outer.append(p + n * off)
        inner.append(p + n * (off - thick))
    return lib.mesh_object(name, *_thicken(outer, inner, M.faces), topo=topo)


def _strip(name, S, path, back=False, width=0.012, height=0.0025, base=-0.001, topo="main"):
    """Raised seam welt following a pattern-space poly line; closed box section, all quads."""
    P = [S.at(u, z, back) for u, z in path]
    rings = []
    for i, (p, n) in enumerate(P):
        t = (P[min(i + 1, len(P) - 1)][0] - P[max(i - 1, 0)][0]).normalized()
        b = n.cross(t).normalized()
        rings.append([p + b * (-width / 2) + n * base, p + b * (-width / 2 + 0.002) + n * height,
                      p + b * (width / 2 - 0.002) + n * height, p + b * (width / 2) + n * base])
    return lib.lofted(name, rings, closed=True, cap_start=False, cap_end=False, topo=topo)


# ============================================================================ parts
def _collar():
    n_seg, n_row = 40, 7
    thick = 0.010
    top_rx, top_ry = C["collar_top_r"]
    top_z = C["collar_top_z"]
    a0, a1 = math.radians(25.0), math.radians(40.0)     # half angle of the front V at base / top
    grid = []
    for h in range(n_row + 1):
        t = h / n_row
        alpha = a0 + (a1 - a0) * t
        row = []
        for k in range(n_seg + 1):
            phi = 1.5 * math.pi + alpha + (math.tau - 2 * alpha) * k / n_seg
            base = _neckline(phi, grow=-0.002, dz=-0.012)
            d = abs(math.degrees(phi) - 270.0 - (360.0 if math.degrees(phi) > 450.0 else 0.0))
            d = min(d, 360.0 - d)
            w = min(max((80.0 - d) / 40.0, 0.0), 1.0)
            w = w * w * (3 - 2 * w)
            top = V((top_rx * math.cos(phi), top_ry * math.sin(phi), top_z - 0.030 * w))
            f = t ** 1.15
            row.append(V((base.x + (top.x - base.x) * f, base.y + (top.y - base.y) * f, base.z + (top.z - base.z) * t)))
        grid.append(row)
    outer, inner, faces = [], [], []
    w_ = n_seg + 1
    for h in range(n_row + 1):
        for k in range(w_):
            p = grid[h][k]
            du = grid[h][min(k + 1, n_seg)] - grid[h][max(k - 1, 0)]
            dv = grid[min(h + 1, n_row)][k] - grid[max(h - 1, 0)][k]
            n = du.cross(dv).normalized()
            if n.dot(V((p.x, p.y, 0))) < 0:
                n = -n
            outer.append(p)
            inner.append(p - n * thick)
    for h in range(n_row):
        for k in range(n_seg):
            a = h * w_ + k
            faces.append((a, a + 1, a + 1 + w_, a + w_))
    ob = lib.mesh_object("coat_collar", *_thicken(outer, inner, faces), topo="main")
    # keep clear of the head ellipsoid (+ 1 cm face mask in front)
    hc, hr = spec.HEAD_CENTER, spec.HEAD_RADII
    worst = 9.0
    for v in ob.data.vertices:
        p = v.co
        ry = hr.y + (0.010 if p.y < hc.y else 0.0)
        worst = min(worst, math.sqrt(((p.x - hc.x) / hr.x) ** 2 + ((p.y - hc.y) / ry) ** 2 + ((p.z - hc.z) / hr.z) ** 2))
    assert worst > 1.12, f"collar too close to the head (scaled distance {worst:.3f})"
    return ob


def build():
    index, verts, faces = _solve_core()
    core = lib.mesh_object("coat_torso", verts, faces, topo="main")
    _check_armholes(core)
    objs = [core]
    objs.append(_collar())
    return objs
