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
    verts = [V(p) for p in P]
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


def _tri_patch(M, a, b, c, n_ab, n_bc, n_ca, ab=None):
    """All-quad fill of a triangle with given edge counts: three grids around one valence-3 centre.
    `ab` optionally replaces the straight a->b side by a curve of n_ab + 1 points."""
    assert (n_ab + n_bc + n_ca) % 2 == 0
    p = (n_ab + n_bc - n_ca) // 2
    r, q = n_ab - p, n_bc - p
    assert min(p, q, r) >= 1, "edge counts violate the triangle inequality"
    ab = ab or _line(a, b, n_ab)
    s_ab, s_bc, s_ca = ab[p], b.lerp(c, q / n_bc), c.lerp(a, r / n_ca)
    o = (s_ab + s_bc + s_ca) / 3
    M.add_grid(_coons(ab[:p + 1], _line(s_ab, o, q), _line(s_ca, o, p), _line(a, s_ca, q)))
    M.add_grid(_coons(_line(b, s_bc, q), _line(s_bc, o, r), _line(s_ab, o, q), ab[p:][::-1]))
    M.add_grid(_coons(_line(c, s_ca, r), _line(s_ca, o, p), _line(s_bc, o, r), _line(c, s_bc, p)))
    return M


def _offset_polyline(pts, inset, sgn=1.0):
    """Mitered parallel curve of a 2D poly line (sgn picks the side)."""
    out = []
    for i, p in enumerate(pts):
        e1 = (p - pts[i - 1]).normalized() if i > 0 else None
        e2 = (pts[i + 1] - p).normalized() if i < len(pts) - 1 else None
        n1 = V((e1.y, -e1.x)) * sgn if e1 else None
        n2 = V((e2.y, -e2.x)) * sgn if e2 else None
        if n1 is None or n2 is None:
            out.append(p + (n1 or n2) * inset)
        else:
            m = (n1 + n2).normalized()
            out.append(p + m * (inset / max(m.dot(n1), 0.45)))
    return out


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


def _welt(name, samples, width=0.012, height=0.0025, base=-0.001, topo="support"):
    """Raised seam welt along (point, normal) samples; closed 4-quad section, all quads."""
    rings = []
    for i, (p, n) in enumerate(samples):
        t = (samples[min(i + 1, len(samples) - 1)][0] - samples[max(i - 1, 0)][0]).normalized()
        b = n.cross(t).normalized()
        rings.append([p + b * (-width / 2) + n * base, p + b * (-width / 2 + 0.002) + n * height,
                      p + b * (width / 2 - 0.002) + n * height, p + b * (width / 2) + n * base])
    return lib.lofted(name, rings, closed=True, cap_start=True, cap_end=True, topo=topo)


def _strip(name, S, path, back=False, off=0.0, **kw):
    """Seam welt following a pattern-space poly line on the wrapped surface."""
    samples = []
    for u, z in path:
        p, n = S.at(u, z, back)
        samples.append((p + n * off, n))
    return _welt(name, samples, **kw)


# ============================================================================ parts
def _collar():
    n_seg, n_row = 36, 8
    thick = 0.010
    rim_r = 0.0068
    top_rx, top_ry = (r - 0.002 for r in C["collar_top_r"])    # the rolled rim brings it out to collar_top_r
    top_z = C["collar_top_z"] - rim_r - 0.001                  # ... and up to collar_top_z
    a0, a1 = math.radians(22.0), math.radians(56.0)     # half angle of the front V at base / top
    grid = []
    for h in range(n_row + 1):
        t = h / n_row
        alpha = a0 + (a1 - a0) * t ** 1.3
        row = []
        for k in range(n_seg + 1):
            phi = 1.5 * math.pi + alpha + (math.tau - 2 * alpha) * k / n_seg
            base = _neckline(phi, grow=-0.002, dz=-0.012)
            d = abs(math.degrees(phi) - 270.0)
            d = min(d, 360.0 - d)
            w = min(max((100.0 - d) / 44.0, 0.0), 1.0)
            w = w * w * (3 - 2 * w)
            top = V((top_rx * math.cos(phi), top_ry * math.sin(phi), top_z - 0.028 * w))
            f = t ** 1.15
            row.append(V((base.x + (top.x - base.x) * f, base.y + (top.y - base.y) * f, base.z + (top.z - base.z) * t)))
        grid.append(row)
    outer, inner, normals, faces = [], [], [], []
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
            normals.append(n)
            inner.append(p - n * thick)
    for h in range(n_row):
        for k in range(n_seg):
            a = h * w_ + k
            faces.append((a, a + 1, a + 1 + w_, a + w_))
    wall = lib.mesh_object("coat_collar", *_thicken(outer, inner, faces), topo="main")
    mid = lambda h, k: (outer[h * w_ + k] + inner[h * w_ + k]) / 2
    # rolled rim: piping up the left front edge, around the top, down the right front edge
    path = [mid(h, 0) for h in range(2, n_row)] + [mid(n_row, k) for k in range(w_)] + [mid(h, n_seg) for h in range(n_row - 1, 1, -1)]
    radii = []
    for i in range(len(path)):
        a, b, c = path[max(i - 1, 0)], path[i], path[min(i + 1, len(path) - 1)]
        turn = (b - a).angle(c - b, 0.0) if 0 < i < len(path) - 1 else 0.0
        radii.append(rim_r / max(math.cos(turn / 2), 0.75))
    rim = lib.lofted("coat_collar_rim", lib.tube_rings(path, radii, n=6, x_hint=(1, 0, 0)), cap_start=True, cap_end=True, topo="main")
    # collar stand seam: welt around the base of the collar, just above the yoke
    band = _welt("coat_collar_band", [(outer[w_ + k], normals[w_ + k]) for k in range(w_)], width=0.014, height=0.003)
    # keep clear of the head ellipsoid (+ 1 cm face mask in front)
    hc, hr = spec.HEAD_CENTER, spec.HEAD_RADII
    worst = 9.0
    for ob in (wall, rim):
        for v in ob.data.vertices:
            p = v.co
            ry = hr.y + (0.010 if p.y < hc.y else 0.0)
            worst = min(worst, math.sqrt(((p.x - hc.x) / hr.x) ** 2 + ((p.y - hc.y) / ry) ** 2 + ((p.z - hc.z) / hr.z) ** 2))
    assert worst > 1.12, f"collar too close to the head (scaled distance {worst:.3f})"
    return [wall, rim, band]


def _isect(a, b, c, d):
    """Intersection of the 2D lines a-b and c-d."""
    r, q = b - a, d - c
    den = r.x * q.y - r.y * q.x
    t = ((c.x - a.x) * q.y - (c.y - a.y) * q.x) / den
    return a + r * t


# front pattern, read off the reference front view as (world x, z)
FLAP_EDGE_X = (-0.075, -0.085)      # free edge of the overlapping (wearer's left) front, waist -> lapel tip
FLAP_SEAM_X = 0.150                 # the flap ends in a princess seam on the wearer's left
LAPEL_TIP_Z = 1.265
LAPEL_TOP = (0.036, 1.497)
LAPEL_CTRL = (0.088, 1.492)         # corner of the lapel's top edge where it leaves the collar
LAPEL_PEAK = (0.185, 1.385)
BUTTON_X = (-0.045, 0.065)
BUTTON_Z = (1.145, 1.200, 1.255)
FLAP_OFF = 0.008                    # layering: every overlapping panel steps out 8 mm


def _front_panels(S):
    objs = []
    z0 = Z_BOTTOM + 0.015
    tip = S.uz(FLAP_EDGE_X[1], LAPEL_TIP_Z)
    # ---- overlapping flap: rectangle below the lapel tip + triangle running up under the lapel
    M = Pattern()
    n_col, n_row = 11, 7
    bl = S.uz(FLAP_EDGE_X[0], z0)
    grid = []
    for j in range(n_row + 1):
        z = z0 + (LAPEL_TIP_Z - z0) * j / n_row
        grid.append(_line(bl.lerp(tip, j / n_row), S.uz(FLAP_SEAM_X, z), n_col))
    M.add_grid(grid)
    top_r = S.uz(FLAP_SEAM_X, LAPEL_PEAK[1] + 0.007)
    _tri_patch(M, tip, grid[-1][-1], top_r, n_col, 4, 11)
    M.relax(40)
    objs.append(_panel("coat_front_flap", S, M, FLAP_OFF, FLAP_OFF - 0.001))
    # ---- left lapel (on top of the flap): neck / collar corner / peak / tip
    a, b = S.uz(*LAPEL_TOP), S.uz(*LAPEL_PEAK)
    k = S.uz(*LAPEL_CTRL)
    c = S.uz(FLAP_EDGE_X[1] - 0.004, LAPEL_TIP_Z - 0.006)
    top = _line(a, k, 2) + _line(k, b, 6)[1:]      # short run under the collar, then straight to the peak
    M = _tri_patch(Pattern(), a, b, c, 8, 13, 13, ab=top).relax(80)
    objs.append(_panel("coat_lapel_L", S, M, 2 * FLAP_OFF, 2 * FLAP_OFF - 0.002))
    trim = _offset_polyline(top + _line(b, c, 13)[1:], 0.014, 1.0)[2:-2]
    objs.append(_strip("coat_lapel_trim_L", S, trim, off=2 * FLAP_OFF, width=0.010))
    # ---- right lapel: mirrored outline, cut off 12 mm behind the left lapel's roll line
    a2, b2, c2 = V((-a.x, a.y)), V((-b.x, b.y)), V((-c.x, c.y))
    x_roll = _isect(a2, c2, a, c)
    x_edge = _isect(b2, c2, a, c)
    x_roll = x_roll + (c2 - a2).normalized() * 0.012
    x_edge = x_edge + (c2 - b2).normalized() * 0.012
    top2 = [V((-q.x, q.y)) for q in top]
    M = Pattern()
    M.add_grid(_coons(top2, _line(b2, x_edge, 7), _line(x_roll, x_edge, 8), _line(a2, x_roll, 7)))
    M.relax(40)
    objs.append(_panel("coat_lapel_R", S, M, FLAP_OFF + 0.002, FLAP_OFF))
    trim = _offset_polyline(top2 + _line(b2, x_edge, 7)[1:], 0.014, -1.0)[2:]
    objs.append(_strip("coat_lapel_trim_R", S, trim, off=FLAP_OFF + 0.002, width=0.010))
    # top-stitch welt along the free edge of the flap
    st = [V((p.x + 0.018, p.y)) for p in _line(bl, tip, 8)[1:]]
    objs.append(_strip("coat_flap_stitch", S, st, off=FLAP_OFF, width=0.008, height=0.0015))
    # ---- matching princess seam on the wearer's right
    zs = [z0 + (LAPEL_PEAK[1] - 0.008 - z0) * i / 14 for i in range(15)]
    objs.append(_strip("coat_seam_front_R", S, [S.uz(-FLAP_SEAM_X, z) for z in zs]))
    # ---- buttons: two columns on the flap + one on each lapel peak
    studs = []
    for x in BUTTON_X:
        for z in BUTTON_Z:
            studs.append((S.uz(x, z), FLAP_OFF))
    centroid = (a + b + c) / 3
    peak_btn = b + (centroid - b).normalized() * 0.040
    studs.append((peak_btn, 2 * FLAP_OFF))
    studs.append((V((-peak_btn.x, peak_btn.y)), FLAP_OFF + 0.002))
    parts = []
    for k, (uz, off) in enumerate(studs):
        pt, n = S.at(uz.x, uz.y)
        parts.append(lib.stud(f"btn{k}", radius=0.013, height=0.007, n=12, loc=pt + n * (off - 0.0008), normal=n))
    objs.append(lib.join(parts, "coat_buttons", topo="rigid"))
    return objs


def _yoke(index, verts, faces_unused):
    """Back shoulder yoke: the core quads above back row 2 (+ one row over the shoulder seam), subdivided
    once and lifted 8 mm, so its edge reads as the yoke seam and as the seam band around the armhole."""
    vid = lambda *k: index[_canon(k)]
    quads = []
    strap_b = list(range(2, 6)) + list(range(10, 14))
    strap_f = list(range(18, 22)) + list(range(26, 30))
    for r in range(2, 6):
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
    centre = V((0.0, 0.0, 1.40))
    if sum(1 for p, n in zip(co, no) if n.dot(p - centre) > 0) < len(co) / 2:
        no = [-n for n in no]
    polys = [tuple(p.vertices) for p in me.polygons]
    bpy.data.objects.remove(tmp)
    bpy.data.meshes.remove(me)
    outer = [p + n * FLAP_OFF for p, n in zip(co, no)]
    inner = [p + n * 0.001 for p, n in zip(co, no)]
    edge = sorted((verts[vid("B", 2, c)] for c in range(2, 15)), key=lambda p: p.x)
    return lib.mesh_object("coat_back_yoke", *_thicken(outer, inner, polys), topo="main"), edge


def _armhole_seams(S, index, verts):
    """Seam band around the front / lower half of each armhole (the yoke edge completes the loop on top)."""
    objs = []
    for side, name in ((1, "L"), (-1, "R")):
        pts = [verts[index[("R1", side, i)]] for i in range(1, 13)]
        nrm = []
        for p in pts:
            k = min(range(len(S.co)), key=lambda j: (S.co[j] - p).length_squared)
            nrm.append(S.no[k])
        P = lib.smooth_path(pts, 2)
        N = [n.normalized() for n in lib.smooth_path(nrm, 2)]
        objs.append(_welt(f"coat_armhole_seam_{name}", list(zip(P, N)), height=0.002))
    return objs


def _back_details(S, yoke_edge):
    def edge_z(x):
        for p, q in zip(yoke_edge, yoke_edge[1:]):
            if p.x <= x <= q.x:
                return p.z + (q.z - p.z) * (x - p.x) / max(q.x - p.x, 1e-9)
        return yoke_edge[0].z

    objs = []
    z0 = Z_BOTTOM + 0.015
    emblem_z = edge_z(0.0) - 0.085
    size = 0.070
    tail = emblem_z - size * 1.6
    # centre back seam (runs up under the emblem's tail)
    n = 8
    objs.append(_strip("coat_seam_back_C", S, [V((0.0, z0 + (tail + 0.012 - z0) * i / n)) for i in range(n + 1)], back=True))
    # princess seams converging towards the waist
    for side, name in ((1, "L"), (-1, "R")):
        top = S.uz(side * 0.135, edge_z(side * 0.135) + 0.006, back=True)
        bot = S.uz(side * 0.075, z0, back=True)
        objs.append(_strip(f"coat_seam_back_{name}", S, _line(bot, top, 17), back=True))
    # faction emblem, bent onto the back so it does not float at the tips
    em = lib.emblem_cross("coat_back_emblem", size=size, thickness=0.005, loc=(0, 0, 0), normal=(0, 1, 0), up=(0, 0, 1))
    for v in em.data.vertices:
        x, h, z = v.co.x, v.co.y, v.co.z
        p, nrm = S.at(S.u_of_x(x, emblem_z + z, back=True), emblem_z + z, back=True)
        v.co = p + nrm * (h + 0.0015)
    lib.finalize(em, sharp_angle=25)
    objs.append(em)
    return objs


def build():
    index, verts, faces = _solve_core()
    core = lib.mesh_object("coat_torso", verts, faces, topo="main")
    _check_armholes(core)
    S = Surface(core)
    yoke, yoke_edge = _yoke(index, verts, faces)
    objs = [core, yoke] + _collar()
    objs += _front_panels(S)
    objs += _back_details(S, yoke_edge)
    objs += _armhole_seams(S, index, verts)
    return objs
