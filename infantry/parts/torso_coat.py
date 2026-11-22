"""Upper half of the great coat (round 5: matched to the user's FBX game model): torso core with
armholes; a single-breasted asymmetric WRAP - the wearer's left front crosses diagonally over to the
right with a broad notched lapel, no buttons - whose flap carries on down over the belt (the belt
passes under it) to z ~1.13; a tall collar open at the front whose back rolls over and flops out and
down; two strap loops at the collar front; a big flat back yoke with two pointed V panels under it;
a small gothic cross whose stem runs down to the belt; a few crisp crumple facets above the belt.

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

All outer panels (wrap flap, lapels, V panels, seams, emblem) are laid out in a flat (U, z)
"pattern" space - read off the FBX in world (x, z) - and wrapped onto the solved core by ray
casting, then offset along the smooth normal, so they follow the layering table in docs/PLAN.md.
Below the core (z < 1.195) the wrap flap is wrapped onto a guide tube at the same coat offset.
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
BELT_HI = C["belt_z"][1]               # belt top: trims and stitches start above it
BELT_MAX = 0.036                         # DETAIL_PASS v2: inside the belt zone nothing exceeds body + 0.036
Z_BOTTOM = C["coat_split_z"] + 0.010     # 1.195: coat torso ends under the belt (DETAIL_PASS v2 seam)
ARMPIT_Z = spec.armhole_ring(16)[8].z
Z_TUBE_TOP = ARMPIT_Z - 0.035            # last planar ring, a little under the armpit
N_AROUND = 32
N_TUBE = 10                              # planar rings (~2.3 cm apart)
R_REF = spec.torso_section(spec.TORSO[4][0])[1] + OFF   # pattern space: U = R_REF * azimuth (chest)
NECK_Z = spec.TORSO[6][0]                # neck base

# ---------------------------------------------------------------- read off the FBX (tools/fbx_compare.py)
# (world metres, read with a world-coordinate grid over FRONT / BACK / SIDE renders of JacketMain)
# front: one wrap panel (wearer's left) crosses over to the right; broad notched lapels both sides
LAPEL_TOP_X = 0.050          # collar front edge meets the lapel's top edge here (both sides)
LAPEL_PEAK = (0.158, 1.518)  # outer point of the big lapel (FBX ~(0.155, 1.52))
LAPEL_NOTCH = (0.110, 1.508) # notch bottom in the lapel's top edge, between gorge and peak
LAPEL_BEND = (0.002, 1.418)  # the big lapel's lower edge runs straight from the peak to here ...
LAPEL_V = (0.000, 1.495)     # the two lapels' roll edges cross here (bottom of the neck V)
LAPEL_TIP = (-0.068, 1.335)  # ... then curves down to the break, where the wrap edge turns vertical
FLAP_EDGE_X = LAPEL_TIP[0]   # the free wrap edge then runs straight down to the belt ...
FLAP_EDGE_SLOPE = 0.24       # ... and below the belt slants back toward +X (dx per -dz, FBX skirt line)
FLAP_BOTTOM_Z = 1.130        # the flap ends here, over the skirt's front panel
FLAP_TONGUE_X = 0.080        # below the belt the flap is a tongue from the free edge to here
FRONT_SEAM_X = 0.125         # side-panel seam lines on both fronts (the flap ends in the +X one)
R_LAPEL_BEND = (-0.080, 1.455)   # under lapel (wearer's right): its lower edge turns here ...
R_LAPEL_AIM = (0.072, 1.338)     # ... and heads for this point (tucked under the big lapel)
# back: big flat yoke, two pointed V panels under its ends, small gothic cross
YOKE_Z = 1.490               # yoke lower edge at the centre back (FBX 1.485, nearly flat) ...
YOKE_DROP = (0.006, 0.150)   # ... and how far it falls by x = +-0.150
YOKE_APEX = 0.0              # no point at the centre any more
BACK_SEAM_X = (0.100, 0.070) # single seam lines: x at the yoke -> x at the belt top (centre panel narrows)
V_PANEL = ((0.104, 0.188), (0.114, 1.372))   # pointed V panel: top x range under the yoke, tip (x, z)
EMBLEM_ARM = 0.044           # small cross: side-arm length (FBX head ~0.11 wide), stem to the belt top
# collar (FBX: tall, open at the front, the back rolls over and flops out / down)
COL_FOLD_Z = (1.698, 1.686)  # fold line (top of the stand): back / front ends
COL_STAND_R = (0.099, 0.110) # stand radii (x, y about the neck centre) at the fold
COL_OPEN_TOP = 46.0          # half-angle (deg from front) of the front opening at the fold line
COL_ROLL = 0.006             # radius of the soft roll at the fold
COL_FALL = ((180.0, 1.632, 0.040), (95.0, 1.616, 0.034), (64.0, 1.545, 0.020))  # (deg from front,
                             # fall edge z, flop past the stand) - back, sides, collar end
COL_END = 54.0               # the fall is cut back between here and the front end (notch line)
COL_THICK = 0.006
COLLAR_BASE_GROW = 0.030     # the collar stands on the shoulders: base +-0.123 wide (back view) ...
COLLAR_BASE_GROW_FRONT = 0.004   # ... narrowing to the lapel tops at the front
COL_LOOP = dict(w=0.021, h=0.032, bar=0.0045, th=0.0045)   # C2 strap loops on the collar fronts
# crumples (C4): broad soft facets above the belt at the back and sides, (s, z) pattern space,
# s = R_REF * angle from the centre back toward +X
CRUMPLE_Z = (1.252, 1.372)

# ---------------------------------------------------------------- layering (own constants)
FLAP_OFF = 0.008             # every overlapping panel steps out 8 mm (docs/PLAN.md)
FLAP_BELT_OUT = 0.056        # over the belt zone and below it the wrap flap's outer face is bare + this
FLAP_THICK = 0.007           # ... and its inner face bare + 0.049 (belt outer face: bare + 0.040)
FLAP_RAMP = (1.252, 1.310)   # it lifts from the 8 mm step to that between these heights
STITCH = dict(width=0.0035, thickness=0.0012, sink=0.0006)     # thin top-stitch lines
PIPING = dict(width=0.0065, thickness=0.0025, sink=0.001)      # raised seam piping
PIPE_GAP = 0.011             # spacing of the double seam lines
STEP = 0.022                 # target quad / segment size on visible cloth
GUIDE_Z = 1.090              # the wrap guide tube runs down to here (flap tongue)


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
NECK_HEM_MIN_Z = 1.535       # the head's neck / gaiter reaches down to z 1.53 inside the collar
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
    # it runs in to the neck-top section (what the head's gaiter is built on) so no gap shows past it
    ring = [index[k] for k in _neck_keys()]
    ncy, nrx, nry = spec.torso_section(spec.TORSO[-1][0])
    for t, dz in ((0.45, -0.006), (1.0, -0.012)):
        lip = []
        for i in range(16):
            phi = math.radians(270.0 + 22.5 * i)
            a = _neckline(phi)
            b = V((nrx * math.cos(phi), ncy + nry * math.sin(phi), a.z))
            q = a.lerp(b, t)
            q.z = max(a.z + dz, NECK_HEM_MIN_Z) if t < 1 else max(a.z + dz, NECK_HEM_MIN_Z + 0.004)
            lip.append(len(verts))
            verts.append(q)
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


def _off(off, z):
    """Offsets are numbers, or functions of height (a panel that gets thinner where it rolls into the collar)."""
    return off(z) if callable(off) else off


def _panel(name, S, M, off, thick, back=False, topo="main", under=False):
    """Wrap a pattern mesh onto the coat: outer surface at +off, underside at +off-thick."""
    outer, inner = [], []
    for u, z in M.pts:
        p, n = S.at(u, z, back)
        o = _off(off, z)
        outer.append(p + n * o)
        inner.append(p + n * (o - thick))
    return lib.mesh_object(name, *_shell(outer, inner, M.faces, back=under), topo=topo)


def _line_on(S, pts_xz, back=False, off=0.0):
    """Reference (x, z) poly line -> points on the coat (+off) and normals."""
    P, N = [], []
    for x, z in pts_xz:
        p, n = S.point(x, z, back, _off(off, z))
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
def _smooth01(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3 - 2 * t)


def _fall_at(a_deg):
    """Fall edge height and flop (outward reach past the stand) at a column `a_deg` from the front."""
    keys = sorted(COL_FALL)
    if a_deg <= keys[0][0]:
        return keys[0][1:]
    for (a0, z0, f0), (a1, z1, f1) in zip(keys, keys[1:]):
        if a_deg <= a1:
            t = _smooth01((a_deg - a0) / (a1 - a0))
            return z0 + (z1 - z0) * t, f0 + (f1 - f0) * t
    return keys[-1][1:]


def _collar(S):
    """FBX collar: a tall stand rising from the shoulders to a fold line, open at the front; above the
    fold the cloth rolls over and the fall flops outward and down over the stand (deepest at the back
    and sides, cut back near the front so the stand's front edges and the strap loops show).
    One continuous sheet (columns around, rows stand -> roll -> fall) given thickness as a closed shell."""
    n_col = 30
    a0, a1 = math.radians(COLLAR_OPEN_BASE), math.radians(COL_OPEN_TOP)
    k_base = 6
    rows_stand, rows_fall = 5, 4
    fold = [180.0, 140.0, 95.0, 50.0, 0.0]               # roll: angle around the fold centre (deg)
    grid, base_row = [], []
    for k in range(n_col + 1):
        edge = min(k, n_col - k)
        wb = _smooth01(1.0 - edge / k_base)
        grow = COLLAR_BASE_GROW + (COLLAR_BASE_GROW_FRONT - COLLAR_BASE_GROW) * wb
        phb = 1.5 * math.pi + a0 + (math.tau - 2 * a0) * k / n_col
        b = _neckline(phb, grow=grow)
        hit = S.bvh.ray_cast(V((b.x, b.y, 2.5)), V((0, 0, -1)))[0]
        b = V((b.x, b.y, (hit.z if hit else b.z) - 0.014))
        base_row.append(b)
        pht = 1.5 * math.pi + a1 + (math.tau - 2 * a1) * k / n_col
        a_deg = 180.0 - abs((math.degrees(pht) - 90.0 + 180.0) % 360.0 - 180.0)    # 0 front .. 180 back
        ff = 1.0 - (a_deg - COL_OPEN_TOP) / (180.0 - COL_OPEN_TOP)                # 0 back .. 1 front end
        zf = COL_FOLD_Z[0] + (COL_FOLD_Z[1] - COL_FOLD_Z[0]) * ff ** 2
        # soft folded look: the roll line and the fall edge wander a little (a few broad waves)
        zf += 0.003 * math.sin(2.0 * pht + 0.6)
        rx, ry = COL_STAND_R
        c, s_ = math.cos(pht), math.sin(pht)
        F = V((rx * c, NECK["cy"] + ry * s_, zf))
        out = V((c / rx, s_ / ry, 0.0)).normalized()
        up = V((0, 0, 1))
        col = [b.lerp(F, i / rows_stand) for i in range(rows_stand)]
        # stand bows out a touch in the middle so it reads as soft cloth, not a cone
        col = [p + out * (0.004 * math.sin(math.pi * i / rows_stand)) for i, p in enumerate(col)]
        Cn = F + out * COL_ROLL
        col += [Cn + out * (COL_ROLL * math.cos(math.radians(t))) + up * (COL_ROLL * math.sin(math.radians(t)))
                for t in fold]
        G = col[-1]
        ze, flop = _fall_at(a_deg)
        ze += 0.006 * math.sin(3.0 * pht + 1.1) + 0.004 * math.sin(5.0 * pht + 0.3)
        flop += 0.004 * math.sin(4.0 * pht + 2.0)
        # cut back toward the front end: the fall shortens to nothing over COL_END -> COL_OPEN_TOP
        cut = _smooth01((a_deg - COL_OPEN_TOP) / (COL_END - COL_OPEN_TOP))
        ze = (G.z - 0.014) + (ze - (G.z - 0.014)) * cut       # a short turned-down lip remains at the end
        flop = 0.003 + (flop - 0.003) * cut
        E = V((G.x, G.y, ze)) + out * flop
        for i in range(1, rows_fall + 1):
            t = i / rows_fall
            p = G.lerp(E, t) + out * (0.005 * math.sin(math.pi * t) * cut)
            col.append(p)
        # the fall lies over the shoulders: never below the coat (+ lapel near the front)
        margin = 0.010 + 0.012 * _smooth01(ff * 1.6)
        for i in range(rows_stand + 1, len(col)):
            p = col[i]
            h = S.bvh.ray_cast(V((p.x, p.y, 2.5)), V((0, 0, -1)))[0]
            if h is not None and p.z < h.z + margin:
                col[i] = V((p.x, p.y, h.z + margin))
        grid.append(col)
    n_row = len(grid[0]) - 1
    w_ = n_col + 1
    P = lambda h, k: grid[k][h]
    outer, inner, normals, faces = [], [], [], []
    for h in range(n_row + 1):
        for k in range(w_):
            du = P(h, min(k + 1, n_col)) - P(h, max(k - 1, 0))
            dv = P(min(h + 1, n_row), k) - P(max(h - 1, 0), k)
            normals.append(du.cross(dv).normalized())
    # orient: on the stand the sheet normal points away from the neck
    k_mid, h_mid = n_col // 2, rows_stand // 2
    pm = P(h_mid, k_mid)
    if normals[h_mid * w_ + k_mid].dot(V((pm.x, pm.y - NECK["cy"], 0))) < 0:
        normals = [-n for n in normals]
    for h in range(n_row + 1):
        for k in range(w_):
            p = P(h, k)
            outer.append(p)
            inner.append(p - normals[h * w_ + k] * COL_THICK)
    for h in range(n_row):
        for k in range(n_col):
            a = h * w_ + k
            faces.append((a, a + 1, a + 1 + w_, a + w_))
    wall = lib.mesh_object("coat_collar", *_shell(outer, inner, faces, back=True), topo="main", sharp_angle=55)
    # C2: two small rectangular strap loops on the stand's front, just inside the fall's cut-back end
    loops = []
    for kk in (0.9, n_col - 0.9):
        hh = 1.9
        def at(src, h, k):
            h0, k0 = int(h), int(k)
            h1, k1 = min(h0 + 1, n_row), min(k0 + 1, n_col)
            fh, fk = h - h0, k - k0
            Q = lambda a, b: src[a * w_ + b]
            return (Q(h0, k0) * (1 - fh) + Q(h1, k0) * fh) * (1 - fk) + (Q(h0, k1) * (1 - fh) + Q(h1, k1) * fh) * fk
        c = at(outer, hh, kk)
        n = at(normals, hh, kk).normalized()
        ay = (at(outer, hh + 0.5, kk) - at(outer, hh - 0.5, kk))
        ay = (ay - n * ay.dot(n)).normalized()
        ax = ay.cross(n).normalized()
        loops.append(_strap_loop(f"coat_collar_loop_{'L' if c.x > 0 else 'R'}", c, ax, ay, n, **COL_LOOP))
    objs = [wall] + loops
    # keep clear of the head ellipsoid (+ 1 cm face mask in front)
    hc, hr = spec.HEAD_CENTER, spec.HEAD_RADII
    worst = 9.0
    for ob in objs:
        for v in ob.data.vertices:
            p = v.co
            ry = hr.y + (0.010 if p.y < hc.y else 0.0)
            worst = min(worst, math.sqrt(((p.x - hc.x) / hr.x) ** 2 + ((p.y - hc.y) / ry) ** 2 + ((p.z - hc.z) / hr.z) ** 2))
    assert worst > 1.08, f"collar too close to the head (scaled distance {worst:.3f})"
    return objs, base_row[0] + V((0, 0, 0.014))


def _strap_loop(name, c, ax, ay, an, w, h, bar, th):
    """Rectangular buckle frame (16 quads, closed) lying on a surface, with the strap tab through it."""
    from mathutils import Matrix

    def rect(hw, hh, d):
        return [c + ax * (sx * hw) + ay * (sy * hh) + an * (d - 0.001) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    verts = rect(w / 2, h / 2, 0) + rect(w / 2, h / 2, th) + rect(w / 2 - bar, h / 2 - bar, 0) + \
        rect(w / 2 - bar, h / 2 - bar, th)
    faces = []
    for a in range(4):
        b = (a + 1) % 4
        faces += [(4 + a, 4 + b, 12 + b, 12 + a), (8 + a, 8 + b, b, a), (a, b, 4 + b, 4 + a), (12 + a, 12 + b, 8 + b, 8 + a)]
    frame = lib.mesh_object(name + "_frame", verts, faces, topo="rigid", sharp_angle=30)
    tab = lib.box(name + "_tab", (w - 2 * bar - 0.003, h + 0.004, 0.0032), segs=(1, 2, 1), topo="rigid")
    tab.data.transform(Matrix.Translation(c + an * 0.0006) @ Matrix((ax, ay, an)).transposed().to_4x4())
    tab.data.update()
    return lib.join([frame, tab], name, topo="rigid")


# ============================================================================ front
LAPEL_ROLL_R = 0.0085        # rolled edge round the lapels (~1.7 cm, review: lapels must read thick)
FACING = dict(width=0.005, thickness=0.0016, sink=0.0008)       # FBX: a facing line along the wrap edge


def _roll(name, S, corners_xz, off, closed=True, r=LAPEL_ROLL_R):
    """Rolled cloth edge: a tube lying just inside an outline, its top standing r*0.65 above the panel."""
    cs = _inset(corners_xz, r * 0.75, closed)
    if closed:
        cs = cs + [cs[0]]
    pts = []
    for x, z in _poly_step(cs, 0.014):
        p, n = S.point(x, z, off=_off(off, z) - r * 0.35)
        pts.append(p)
    return lib.lofted(name, lib.tube_rings(pts, r, n=8, x_hint=(0, -1, 0)), cap_start=True, cap_end=True,
                      topo="main")


def flap_off(z):
    """Wrap flap offset over the coat: the 8 mm overlap step above the belt, lifted over the belt zone
    and below it so its outer face is bare + FLAP_BELT_OUT (the belt passes under it)."""
    t = _smooth01((FLAP_RAMP[1] - z) / (FLAP_RAMP[1] - FLAP_RAMP[0]))
    return FLAP_OFF + (FLAP_BELT_OUT - OFF - FLAP_OFF) * t


def flap_edge_x(z):
    """x of the wrap flap's free (front) edge at height z (pattern space = world x on the coat)."""
    return FLAP_EDGE_X + FLAP_EDGE_SLOPE * max(0.0, FLAP_RAMP[0] - z)


def _front(S, lapel_top_z):
    """Single-breasted asymmetric wrap (FBX): the wearer's left front crosses over to the right; its
    broad notched lapel runs from the collar to the break, the free edge drops straight to the belt and
    the flap carries on over the belt to FLAP_BOTTOM_Z.  The right front only shows its notched lapel."""
    objs = []
    T = V((LAPEL_TOP_X, lapel_top_z))
    P, K, Mb, Vv, B = V(LAPEL_PEAK), V(LAPEL_NOTCH), V(LAPEL_BEND), V(LAPEL_V), V(LAPEL_TIP)
    mir = lambda p: V((-p.x, p.y))
    T2, P2, K2, M2 = mir(T), mir(P), mir(K), V(R_LAPEL_BEND)
    z0 = Z_BOTTOM + 0.010
    uz = lambda p: S.uz(p.x, p.y)

    def big_low_z(x):
        """Lower edge of the big lapel at x (peak -> bend straight, then on to the break)."""
        if x >= Mb.x:
            return P.y + (Mb.y - P.y) * (P.x - x) / (P.x - Mb.x)
        return Mb.y + (B.y - Mb.y) * (Mb.x - x) / (Mb.x - B.x)

    # ---- big lapel (wearer's left): T -> notch -> peak -> bend -> break -> (V) -> T, on top of the flap
    ab = _poly_n([T, K, P], 7)
    bc = _poly_n([P, Mb, B], 12)
    ca = _poly_n([B, Vv, T], 11)
    Mp = _tri_patch(Pattern(), [uz(p) for p in ab], [uz(p) for p in bc], [uz(p) for p in ca]).relax(60)

    def lapel_off(z):
        """18 mm over the flap, thinning to 11 mm where the lapel rolls up into the collar."""
        t = min(max((z - LAPEL_BEND[1]) / (T.y - LAPEL_BEND[1]), 0.0), 1.0)
        return 0.018 + (0.011 - 0.018) * t
    objs.append(_panel("coat_lapel_L", S, Mp, lapel_off, 0.016))
    objs.append(_roll("coat_lapel_L_trim", S, [T, Vv, B, Mb, P, K], lapel_off))

    # ---- under lapel (wearer's right): visible part T2 -> notch -> P2 -> M2 -> X1, tucked under the big one
    def isect(a, b, c, d):
        r, q = b - a, d - c
        den = r.x * q.y - r.y * q.x
        t = ((c.x - a.x) * q.y - (c.y - a.y) * q.x) / den
        return a + r * t
    X1 = isect(M2, V(R_LAPEL_AIM), Vv, B)               # its lower edge meets the big lapel's roll edge
    tuck = (B - Vv).normalized()
    tuck = V((tuck.y, -tuck.x)) * 0.012                 # perpendicular, pointing in under the big lapel
    if tuck.x < 0:
        tuck = -tuck
    X1e, Ve = X1 + tuck, Vv + tuck
    G = _coons(_poly_n([T2, K2, P2], 6), _poly_n([P2, M2, X1e], 6), _poly_n([Ve, X1e], 6), _poly_n([T2, Ve], 6))
    Mr = Pattern()
    Mr.add_grid([[uz(p) for p in row] for row in G])
    Mr.relax(30)
    r_off = 0.012
    objs.append(_panel("coat_lapel_R", S, Mr, r_off, r_off))
    # its rolled edge along the visible outline (X1 -> M2 -> P2 -> notch -> T2 -> towards V), the rest is tucked
    vis = [X1, M2, P2, K2, T2, Vv]                      # clockwise
    ccw = vis[::-1]
    roll_cs = _inset(ccw, LAPEL_ROLL_R * 0.75, closed=True)[::-1]
    cs = [roll_cs[0] + (roll_cs[1] - roll_cs[0]).normalized() * 0.006] + roll_cs[1:5] + [roll_cs[4].lerp(roll_cs[5], 0.85)]
    pts = [S.point(x, z, off=r_off - LAPEL_ROLL_R * 0.35)[0] for x, z in _poly_step(cs, 0.014)]
    objs.append(lib.lofted("coat_lapel_R_trim", lib.tube_rings(pts, LAPEL_ROLL_R, n=8, x_hint=(0, -1, 0)),
                           cap_start=True, cap_end=True, topo="main"))

    # ---- the wrap flap: the big front below its lapel, down over the belt to FLAP_BOTTOM_Z
    under = 0.012
    lower = [p + V((0.0, under)) for p in (Mb, B)]
    x_r = FRONT_SEAM_X
    TR = V((x_r, big_low_z(x_r) + under))
    BLt, BRt = V((FLAP_EDGE_X, B.y)), V((x_r, B.y))
    zb, zr0, zr1 = FLAP_BOTTOM_Z, FLAP_RAMP[0], FLAP_RAMP[1]
    E1, E0 = V((FLAP_EDGE_X, zr0)), V((flap_edge_x(zb), zb))
    R0, R1, R2 = V((FLAP_TONGUE_X, zb)), V((FLAP_TONGUE_X, zr0 + 0.006)), V((x_r, zr1))
    Mf = Pattern()
    G = _coons(_line(E0, R0, 7), _poly_n([R0, R1, R2, BRt], 9), _line(BLt, BRt, 7), _poly_n([E0, E1, BLt], 9))
    Mf.add_grid([[uz(p) for p in row] for row in G])
    hyp = _poly_n([TR, lower[0], V((BLt.x + 0.004, BLt.y + under * 0.5))], 10)
    hyp[-1] = BLt
    _tri_patch(Mf, [uz(p) for p in _line(BLt, BRt, 7)], [uz(p) for p in _line(BRt, TR, 7)], [uz(p) for p in hyp])
    Mf.relax(40)
    objs.append(_panel("coat_front_flap", S, Mf, flap_off, FLAP_THICK, under=True))
    # facing line ~1 cm inside the free edge and the hem (FBX: the wrap edge reads doubled)
    edge = [V((FLAP_EDGE_X, B.y - 0.012)), E1, E0, R0 + V((-0.004, 0.0))]
    cs = _inset(edge, 0.010, closed=False)
    objs.append(_strip("coat_flap_facing", S, _poly_step(cs, 0.014), off=flap_off, kind=FACING))

    # ---- side-panel seam on the wearer's right front (FBX: a single line), up to the under lapel
    def r_low_z(x):
        return P2.y + (M2.y - P2.y) * (x - P2.x) / (M2.x - P2.x)
    xs = -x_r
    objs.append(_strip("coat_seams_front", S, _poly_step([V((xs, z0)), V((xs, r_low_z(xs) + 0.004))])))
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
    # small gothic cross (lib.emblem_gothic): head just under the yoke hem, the stem stretched down the
    # centre back to the belt top (the skirt's back tab carries it on below the belt)
    top_z, bot_z = _yoke_z(0.0) - 0.010, BELT_HI + 0.002
    h = EMBLEM_ARM / 0.21
    stem = 0.62
    em = lib.emblem_gothic("coat_back_emblem", height=h, thickness=0.0045, normal=(0, 1, 0), up=(0, 0, 1), stem=stem)
    zs = [v.co.z for v in em.data.vertices]
    z_hi, z_lo = max(zs), min(zs)
    bar = h * (stem - 0.5)
    z_keep = bar - 0.018                                 # head (cross-bar, barbs) keeps its size ...
    head_drop = z_hi - z_keep
    k = ((top_z - head_drop) - bot_z) / (z_keep - z_lo)  # ... the stem below it is stretched to the belt
    for v in em.data.vertices:
        x, hh, z = v.co.x, v.co.y, v.co.z
        wz = top_z - (z_hi - z) if z >= z_keep else bot_z + (z - z_lo) * k
        p, n = S.point(x, wz, back=True)
        v.co = p + n * (hh + 0.0012)
    lib.finalize(em, sharp_angle=25)
    objs.append(em)
    # pointed V panels under the yoke's ends (FBX): top tucked under the yoke, tip pointing down-in
    (xa, xb), (xt, zt) = V_PANEL
    for side in (1, -1):
        A = V((side * xa, _yoke_z(xa) + 0.014))
        Bp = V((side * xb, _yoke_z(xb) + 0.014))
        Tp = V((side * xt, zt))
        ab, bc, ca = _poly_n([A, Bp], 4), _poly_n([Bp, Tp], 6), _poly_n([Tp, A], 6)
        if side < 0:
            ab, bc, ca = [p for p in reversed(ab)], [p for p in reversed(ca)], [p for p in reversed(bc)]
        uzb = lambda p: S.uz(p.x, p.y, back=True)
        M = _tri_patch(Pattern(), [uzb(p) for p in ab], [uzb(p) for p in bc], [uzb(p) for p in ca]).relax(20)
        objs.append(_panel(f"coat_back_point_{'L' if side > 0 else 'R'}", S, M, 0.0055, 0.0045, back=True))
    # single seam lines bounding the narrowing centre-back panel (FBX), continued under the belt
    seams = []
    for side in (1, -1):
        xt_, xb_ = BACK_SEAM_X
        top_p = V((side * xt_, _yoke_z(xt_) - 0.004))
        belt_p = V((side * xb_, BELT_HI))
        bot_p = top_p + (belt_p - top_p) * ((top_p.y - z0) / (top_p.y - BELT_HI))
        seams.append(_strip(f"coat_seam_back_{side}", S, _poly_step([bot_p, top_p]), back=True, profile="round"))
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


# ============================================================================ crumples (C4)
CRUMPLES = [
    # back: a broad sag fold dipping toward the stem, a valley above it, a short one under it
    dict(pts=[(-0.17, 1.300), (-0.06, 1.284), (0.0, 1.279), (0.06, 1.286), (0.17, 1.303)], amp=0.0104, width=0.020, sharp=0.75),
    dict(pts=[(-0.13, 1.336), (-0.02, 1.322), (0.11, 1.338)], amp=-0.0072, width=0.016, sharp=0.6),
    dict(pts=[(0.03, 1.262), (0.14, 1.268)], amp=0.0072, width=0.016, sharp=0.7),
    # sides: diagonal drag folds pulled down to the belt
    dict(pts=[(0.20, 1.352), (0.27, 1.282)], amp=0.0096, width=0.018, sharp=0.7),
    dict(pts=[(-0.21, 1.346), (-0.28, 1.280)], amp=0.0096, width=0.018, sharp=0.7),
    dict(pts=[(0.30, 1.330), (0.34, 1.268)], amp=-0.0064, width=0.016, sharp=0.6),
    # the wearer's right front, beside the wrap flap
    dict(pts=[(-0.36, 1.338), (-0.42, 1.272)], amp=0.0080, width=0.016, sharp=0.7),
]


def _crumple(verts, index):
    """Displace the core's waist rings (radially) by a few crisp fold ridges; returns the moved ids."""
    moved = set()
    for k, i in index.items():
        if k[0] != "t":
            continue
        p = verts[i]
        if not (CRUMPLE_Z[0] <= p.z <= CRUMPLE_Z[1]):
            continue
        cy = spec.torso_section(p.z)[0]
        az = math.atan2(p.y - cy, p.x)
        s = R_REF * (((math.pi / 2 - az) + math.pi) % math.tau - math.pi)     # 0 centre back, + toward +X
        d = lib.fold_ridges(s, p.z, CRUMPLES)
        if abs(d) < 1e-5:
            continue
        r = V((p.x, p.y - cy, 0.0)).normalized()
        verts[i] = p + r * d
        moved.add(i)
    return moved


def _sharpen(ob, ids, angle=11.0):
    """Hard edges inside the crumple zone so the folds read as crisp facets (FBX), not soft bumps."""
    import bmesh
    me = ob.data
    bm = bmesh.new()
    bm.from_mesh(me)
    lim = math.radians(angle)
    for e in bm.edges:
        if all(v.index in ids for v in e.verts) and len(e.link_faces) == 2 and e.calc_face_angle(0.0) > lim:
            e.smooth = False
    bm.to_mesh(me)
    bm.free()
    me.update()


def _guide_surface(verts, faces):
    """Wrap guide: the core + a coat-offset tube below it down to GUIDE_Z (only the flap tongue uses it)."""
    vs, fs = list(verts), list(faces)
    rings = []
    for j in range(5):
        z = GUIDE_Z + (Z_BOTTOM - GUIDE_Z) * j / 4
        cy, rx, ry = _coat_sec(z)
        rings.append(_ellipse_ring(z, rx, ry, cy))
    v2, f2 = lib.loft(rings)
    base = len(vs)
    vs += v2
    fs += [tuple(i + base for i in f) for f in f2]
    ob = lib.mesh_object("coat_guide_tmp", vs, fs)
    S = Surface(ob)
    me = ob.data
    bpy.data.objects.remove(ob)
    bpy.data.meshes.remove(me)
    return S


# ============================================================================ build
def build():
    index, verts, faces = _solve_core(_yoke_pins())
    moved = _crumple(verts, index)
    core = lib.mesh_object("coat_torso", verts, faces, topo="main")
    _sharpen(core, moved)
    worst = _check_armholes(core)
    print(f"torso_coat: armhole seams match spec.armhole_ring(16, +-1) (max deviation {worst:.1e} m)")
    S = _guide_surface(verts, faces)
    collar, front_base = _collar(S)
    objs = [core] + _yoke(index, verts) + collar
    objs += _front(S, front_base.z - 0.006)
    objs += _back(S)
    objs.append(_armhole_seams(S, index, verts))
    return objs
