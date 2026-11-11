"""Arms: coat sleeves (from the armhole seam), forearm bracers with studs, gloved hands.

Everything is built for the LEFT (+X) arm and mirrored with lib.mirror_copy.

Objects per arm
    Sleeve_upper_L   main     armhole seam ring -> just above the elbow (16 around)
    Sleeve_elbow_L   support  3 tight loops at the elbow
    Sleeve_lower_L   main     elbow -> wrist, ends in a rolled, thick cuff
    Bracer_L         rigid    conical forearm cuff with thickness + raised rims
    Bracer_straps_L  rigid    2 strap bands + buckles
    Bracer_studs_L   rigid    two rows of rivets on the outer half
    Glove_cuff_L     main     thick flared glove cuff running up inside the sleeve cuff
    Wrist_L          support  3 tight loops at the wrist
    Hand_L           main     palm + 4 fingers + thumb, one welded all-quad mesh
    Hand_armor_L     rigid    knuckle studs + back-of-hand plate
"""
import bisect
import math

import bpy  # noqa: F401
from mathutils import Vector as V

from infantry import lib, spec

S = spec
CL = S.CLOTH
TAU = math.tau
NS = 16                                   # sleeve verts around
NB = 24                                   # bracer verts around
D1 = (S.ELBOW - S.SHOULDER).normalized()
D2 = (S.WRIST - S.ELBOW).normalized()
LU = (S.ELBOW - S.SHOULDER).length        # upper arm length
LF = (S.WRIST - S.ELBOW).length           # forearm length


# ------------------------------------------------------------------ small helpers
def keyf(x, keys, smooth=True):
    """Piecewise interpolation through (x, value) keys."""
    if x <= keys[0][0]:
        return keys[0][1]
    for (x0, v0), (x1, v1) in zip(keys, keys[1:]):
        if x <= x1:
            f = (x - x0) / (x1 - x0)
            if smooth:
                f = f * f * (3 - 2 * f)
            return v0 + (v1 - v0) * f
    return keys[-1][1]


def combine(parts):
    """[(verts, faces), ...] -> one (verts, faces)."""
    verts, faces = [], []
    for vs, fs in parts:
        o = len(verts)
        verts += list(vs)
        faces += [tuple(i + o for i in f) for f in fs]
    return verts, faces


def loop_loft(rings):
    """Loft a closed profile (last ring connects back to the first) -> torus-like closed shell."""
    return lib.loft(list(rings) + [rings[0]])


def obox(c, ax, ay, az, hx, hy, hz):
    """Oriented box, 6 quads."""
    vs = [c + ax * (sx * hx) + ay * (sy * hy) + az * (sz * hz)
          for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)]
    fs = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    return vs, fs


def frame_loop(c, ax, ay, az, ha, hb, bar, h0, h1):
    """Rectangular buckle frame (16 quads): outer half size ha x hb, bar width, height h0..h1 along az."""
    rings = []
    for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        o = c + ax * (sx * ha) + ay * (sy * hb)
        i = c + ax * (sx * (ha - bar)) + ay * (sy * (hb - bar))
        rings.append([o + az * h0, o + az * h1, i + az * h1, i + az * h0])
    return loop_loft(rings)


# ------------------------------------------------------------------ arm centre line
class ArmPath:
    """Arm centre line with a rounded elbow, addressed by arc length s (s=0 at SHOULDER)."""
    STEP = 0.001

    def __init__(self, rnd=0.028, s_min=-0.10, extra=0.06):
        a = S.ELBOW - D1 * rnd
        b = S.ELBOW + D2 * rnd
        start = S.SHOULDER + D1 * s_min
        end = S.WRIST + D2 * extra
        pts = []
        n1 = int(round((LU - rnd - s_min) / self.STEP))
        pts += [start.lerp(a, i / n1) for i in range(n1)]
        nb = int(round(2 * rnd / self.STEP))
        for i in range(nb):
            f = i / nb
            pts.append(a * (1 - f) ** 2 + S.ELBOW * (2 * f * (1 - f)) + b * (f * f))
        n2 = int(round((LF - rnd + extra) / self.STEP))
        pts += [b.lerp(end, i / n2) for i in range(n2 + 1)]
        self.pts = pts
        acc = [s_min]
        for i in range(1, len(pts)):
            acc.append(acc[-1] + (pts[i] - pts[i - 1]).length)
        # the rounded corner is a hair shorter than the poly line: rescale the forearm part so
        # that s = LU + LF still lands on the wrist
        self.acc = acc
        self.frames = lib.frames_along(pts, x_hint=(1, 0, 0))   # (tangent, outer, front)

    def at(self, s):
        acc = self.acc
        i = min(max(bisect.bisect_right(acc, s) - 1, 0), len(acc) - 2)
        f = (s - acc[i]) / max(acc[i + 1] - acc[i], 1e-9)
        f = min(max(f, 0.0), 1.0)
        c = self.pts[i].lerp(self.pts[i + 1], f)
        t0, x0, y0 = self.frames[i]
        t1, x1, y1 = self.frames[i + 1]
        return c, t0.lerp(t1, f).normalized(), x0.lerp(x1, f).normalized(), y0.lerp(y1, f).normalized()


def t_of(s):
    """arc length -> spec arm parameter (0 shoulder, 1 elbow, 2 wrist)."""
    return s / LU if s < LU else 1.0 + (s - LU) / LF


BAG = [(0.0, 0.004), (0.60, 0.004), (0.80, 0.007), (1.0, 0.009), (1.12, 0.006), (1.5, 0.003), (1.9, 0.0)]
FOLD = [(-0.084, 0.0), (-0.060, 0.006), (-0.036, -0.006), (-0.012, 0.006), (0.0, 0.007),
        (0.012, 0.004), (0.034, 0.0)]


def sleeve_R(s):
    """Smooth sleeve radius (no folds)."""
    t = min(max(t_of(s), 0.0), 2.0)
    return S.arm_radius(t) + CL["sleeve_offset"] + keyf(t, BAG)


def fold(ds, phi):
    """Extra radius for the cloth rolls around the elbow.  ds = s - LU, phi: 0 outer, 90deg front."""
    v = keyf(ds, FOLD)
    if v < 0:                                   # creases bite deeper in the elbow pit (front)
        v *= 1.0 + 0.7 * math.sin(phi)
    v += 0.004 * max(0.0, -math.sin(phi)) * math.exp(-(ds / 0.022) ** 2)   # elbow point (back)
    return v


def clamp_to_coat(p, weight=1.0, gap=0.004):
    """Keep the sleeve from sinking into the coat torso under the armpit."""
    if not (0.87 < p.z < 1.50):
        return p
    cy, rx, ry = S.torso_section(p.z)
    rx += CL["coat_offset"] + gap
    ry += CL["coat_offset"] + gap
    dy = (p.y - cy) / ry
    if abs(dy) >= 1.0:
        return p
    xlim = rx * math.sqrt(1.0 - dy * dy)
    if p.x < xlim:
        p = V((p.x + (xlim - p.x) * weight, p.y, p.z))
    return p


# ------------------------------------------------------------------ sleeve
def build_sleeve(path):
    def ring(s, extra=0.0, folds=True):
        c, t, e1, e2 = path.at(s)
        out = []
        for i in range(NS):
            phi = TAU * i / NS
            r = sleeve_R(s) + extra + (fold(s - LU, phi) if folds else 0.0)
            out.append(clamp_to_coat(c + (e1 * math.cos(phi) + e2 * math.sin(phi)) * r))
        return out

    # --- armhole seam ring in cylinder coordinates around the upper arm axis
    seam = S.armhole_ring(NS)
    _, _, e1, e2 = path.at(0.0)
    cyl = []
    for i, p in enumerate(seam):
        q = p - S.SHOULDER
        a = q.dot(D1)
        r = q - D1 * a
        phi = math.atan2(r.dot(e2), r.dot(e1))
        tgt = TAU * i / NS
        cyl.append((a, r.length, tgt + ((phi - tgt + math.pi) % TAU - math.pi)))

    KB = 7
    a_end = LU - 0.084
    upper = [[p.copy() for p in seam]]
    for k in range(1, KB + 1):
        x = k / KB
        wr = 1.0 - (1.0 - x) ** 3.0
        rg = []
        for i in range(NS):
            a0, rho0, phi0 = cyl[i]
            a = a0 + (a_end - a0) * x
            phi_t = TAU * i / NS
            R = sleeve_R(max(a, 0.0)) + fold(a - LU, phi_t)
            rho = rho0 + (R - rho0) * wr
            phi = phi0 + (phi_t - phi0) * wr
            c, t, f1, f2 = path.at(a)
            p = c + (f1 * math.cos(phi) + f2 * math.sin(phi)) * rho
            rg.append(clamp_to_coat(p, weight=min(1.0, k / 2.0)))
        upper.append(rg)
    upper += [ring(LU - 0.060), ring(LU - 0.036), ring(LU - 0.012)]

    elbow = [ring(LU + ds) for ds in (-0.012, -0.006, 0.0, 0.006, 0.012)]

    s_c0 = LU + LF - 0.018                      # cuff roll starts right below the bracer
    n_low = 10
    lower = [ring(LU + 0.012 + (s_c0 - LU - 0.012) * i / n_low) for i in range(n_low + 1)]
    for ds, off in ((0.0015, 0.0045), (0.005, 0.006), (0.0085, 0.0045), (0.010, 0.001),
                    (0.010, -0.004), (0.004, -0.005), (-0.006, -0.005)):
        lower.append(ring(s_c0 + ds, off, folds=False))

    return [lib.lofted("Sleeve_upper_L", upper, topo="main", sharp_angle=70),
            lib.lofted("Sleeve_elbow_L", elbow, topo="support", sharp_angle=70),
            lib.lofted("Sleeve_lower_L", lower, topo="main", sharp_angle=70)]


# ------------------------------------------------------------------ bracer
def build_bracer(path):
    t0, t1 = CL["bracer"]
    s0, s1 = LU + t0 * LF, LU + t1 * LF
    Lb = s1 - s0

    def Rb(x):                                   # outer body radius, x = distance from the top edge
        x = min(max(x, 0.0), Lb)
        return sleeve_R(s0 + x) + 0.010 + 0.010 * max(0.0, 1.0 - x / (0.55 * Lb)) ** 2

    def ring(x, off, n=NB):
        c, t, e1, e2 = path.at(s0 + x)
        r = Rb(x) + off
        return [c + (e1 * math.cos(TAU * j / n) + e2 * math.sin(TAU * j / n)) * r for j in range(n)]

    def surf(x, phi_deg, off=0.0):
        c, t, e1, e2 = path.at(s0 + x)
        nrm = e1 * math.cos(math.radians(phi_deg)) + e2 * math.sin(math.radians(phi_deg))
        return c + nrm * (Rb(x) + off), nrm, t

    TH, bv = 0.005, 0.0018
    h, wr = 0.007, 0.020                          # top rim: height, width
    h2, wr2 = 0.005, 0.016                        # bottom rim
    prof = [(0.0, -TH), (0.0, h - bv), (bv, h), (wr - bv, h), (wr, h - bv), (wr + 0.0008, 0.0)]
    nb = max(2, round((Lb - wr - wr2) / 0.022))
    prof += [(wr + (Lb - wr - wr2) * i / nb, 0.0) for i in range(1, nb)]
    prof += [(Lb - wr2 - 0.0008, 0.0), (Lb - wr2, h2 - bv), (Lb - wr2 + bv, h2), (Lb - bv, h2), (Lb, h2 - bv), (Lb, -TH)]
    prof += [(Lb * 0.75, -TH), (Lb * 0.5, -TH), (Lb * 0.25, -TH)]
    shell = lib.mesh_object("Bracer_L", *loop_loft([ring(x, o) for x, o in prof]), topo="rigid", sharp_angle=35)

    # --- straps + buckles
    parts = []
    sw = 0.020
    for fx in (0.40, 0.64):
        x = fx * Lb
        sp = [(x - sw / 2, -0.0008), (x - sw / 2, 0.0032), (x - sw / 2 + 0.0012, 0.0044),
              (x + sw / 2 - 0.0012, 0.0044), (x + sw / 2, 0.0032), (x + sw / 2, -0.0008)]
        parts.append(loop_loft([ring(xx, o) for xx, o in sp]))
        p, nrm, t = surf(x, 112.0)
        tg = t.cross(nrm).normalized()
        parts.append(frame_loop(p, t, tg, nrm, 0.0150, 0.0120, 0.0036, 0.003, 0.0080))
        parts.append(obox(p + nrm * 0.0068, t, tg, nrm, 0.0150, 0.0016, 0.0016))
    straps = lib.mesh_object("Bracer_straps_L", *combine(parts), topo="rigid", sharp_angle=35)

    # --- rivets: two rows on the outer half
    studs = []
    for fx, rad in ((0.235, 0.0105), (0.83, 0.0088)):
        for k, ang in enumerate((-90.0, -45.0, 0.0, 45.0, 90.0)):
            p, nrm, t = surf(fx * Lb, ang, -0.0008)
            studs.append(lib.stud(f"bracer_stud_{fx}_{k}", radius=rad, height=rad * 0.55, n=12, loc=p, normal=nrm))
    studs = lib.join(studs, "Bracer_studs_L", topo="rigid")
    return [shell, straps, studs]


# ------------------------------------------------------------------ hand
# Hand-local coordinates: x = u across the palm (+ towards the thumb / front of the character),
# y = v thickness (+ back of the hand, - palm), z = s along the hand (wrist -> finger tips).
FINGER_W = [0.0215, 0.0225, 0.0210, 0.0195]          # index, middle, ring, pinky
PALM_HW = sum(FINGER_W) / 2
_b = [PALM_HW]
for _w in FINGER_W:
    _b.append(_b[-1] - _w)
U_BOUND = _b                                          # 5 finger boundaries, thumb side first
U_CENT = [(U_BOUND[i] + U_BOUND[i + 1]) / 2 for i in range(4)]
CORNER = 0.90
XI = [CORNER]
for _i in range(4):
    XI.append(U_CENT[_i] / PALM_HW)
    XI.append(U_BOUND[_i + 1] / PALM_HW if _i < 3 else -CORNER)
# XI: normalised u of the 9 back-of-hand verts (ring index 1..9); the palm side mirrors them.

NP = 20                                               # verts around the palm / wrist
NR = 9                                                # palm rows (spans) wrist -> finger webs
S0 = 0.011                                            # first palm row
WEB = [(-1.0, 0.082), (-0.76, 0.087), (-0.28, 0.096), (0.23, 0.100), (0.75, 0.097), (1.0, 0.093)]
W_KEYS = [(0.0, 0.0310), (0.25, 0.0350), (0.6, 0.0405), (1.0, PALM_HW)]
T_KEYS = [(0.0, 0.0225), (0.3, 0.0180), (0.7, 0.0145), (1.0, 0.0112)]
P_KEYS = [(0.0, 2.3), (0.3, 3.0), (1.0, 3.6)]
FINGER_LEN = [0.080, 0.088, 0.082, 0.064]
FINGER_FAN = [4.0, 1.0, -2.5, -6.5]                   # degrees, + towards the thumb
FINGER_BEND = [(12, 22, 12), (15, 26, 14), (19, 30, 16), (23, 33, 18)]   # MCP, PIP, DIP curl (deg)
FINGER_RV = [0.0098, 0.0100, 0.0097, 0.0088]


def _sec(j, Wd, Td, p):
    """Point j (0..19) of a rounded-box cross section -> (u, v, xi)."""
    if j == 0:
        return Wd, 0.0, 1.0
    if j == 10:
        return -Wd, 0.0, -1.0
    xi, sg = (XI[j - 1], 1.0) if j < 10 else (XI[19 - j], -1.0)
    c = abs(xi) ** (p / 2)
    sn = math.sqrt(max(0.0, 1.0 - c * c))
    return Wd * xi, sg * Td * sn ** (2.0 / p), xi


def section(s, Wd, Td, p):
    return [V((u, v, s)) for u, v, _ in (_sec(j, Wd, Td, p) for j in range(NP))]


def back_v(u, s):
    """Height of the back-of-hand surface at (u, s)."""
    f = min(max((s - S0) / (0.095 - S0), 0.0), 1.0)
    Wd, Td, p = keyf(f, W_KEYS), keyf(f, T_KEYS), keyf(f, P_KEYS)
    xi = min(abs(u) / Wd, 0.999)
    return Td * (1.0 - xi ** p) ** (1.0 / p)


RING8 = [(0.76, 0.76), (0.0, 1.0), (-0.76, 0.76), (-1.0, 0.0), (-0.76, -0.76), (0.0, -1.0), (0.76, -0.76), (1.0, 0.0)]
# order: TL, TM, TR, RM, BR, BM, BL, LM   (T = back of hand, L = thumb side)


def digit(P0, dirs, Ls, fu_hint, fv_hint, ru, rv, sep=0.007, e=0.0032, taper=0.14):
    """Rings (after the base ring) + tip point of a 3-segment digit.
    dirs: direction of each segment; rings are 8 points in RING8 order."""
    joints = [V(P0)]
    for dd, L in zip(dirs, Ls):
        joints.append(joints[-1] + dd * L)
    cum = [0.0, Ls[0], Ls[0] + Ls[1], sum(Ls)]
    Ltot = cum[3]

    def frame(dd):
        fv = (fv_hint - dd * fv_hint.dot(dd)).normalized()
        fu = dd.cross(fv).normalized()
        if fu.dot(fu_hint) < 0:
            fu = -fu
        return fu, fv

    def at(x):
        k = 0 if x < cum[1] else (1 if x < cum[2] else 2)
        c = joints[k] + dirs[k] * (x - cum[k])
        dd = dirs[k]
        m = 1.7 * e
        for jn in (1, 2):
            if abs(x - cum[jn]) < m:
                wgt = (x - cum[jn] + m) / (2 * m)
                dd = dirs[jn - 1].lerp(dirs[jn], wgt).normalized()
        return c, dd

    L1, L2, L3 = Ls
    st = [(sep, 1.0)]
    span = L1 - e - sep
    n = max(1, round(span / 0.010))
    st += [(sep + span * i / n, 0.98) for i in range(1, n)]
    st += [(L1 - e, 1.04), (L1 + e, 1.04)]
    span = L2 - 2 * e
    n = max(1, round(span / 0.010))
    st += [(L1 + e + span * i / n, 0.97) for i in range(1, n)]
    st += [(L1 + L2 - e, 1.03), (L1 + L2 + e, 1.03)]
    st += [(L1 + L2 + e + (L3 - e) * 0.36, 1.0), (Ltot - 0.0100, 0.99), (Ltot - 0.0052, 0.90), (Ltot - 0.0018, 0.60)]

    rings = []
    for x, sc in st:
        c, dd = at(x)
        fu, fv = frame(dd)
        k = sc * (1.0 - taper * x / Ltot)
        rings.append([c + fu * (ru * k * cx) + fv * (rv * k * (cy if cy > 0 else cy * 0.92)) for cx, cy in RING8])
    c, dd = at(Ltot)
    fu, fv = frame(dd)
    return rings, c - fv * (rv * 0.12)


def build_hand():
    O = S.WRIST
    d = (S.HAND_TIP - S.WRIST).normalized()
    b = V((1.0, 0.0, 0.0))
    b = (b - d * b.dot(d)).normalized()          # back of the hand faces away from the thigh (+X)
    w = d.cross(b).normalized()                  # thumb side = character's front (-Y)

    def W(p):
        return O + w * p.x + b * p.y + d * p.z

    def Wd(p):
        return w * p.x + b * p.y + d * p.z

    verts, faces = [], []

    def add(p):
        verts.append(V(p))
        return len(verts) - 1

    # ---------------- palm grid
    grid = []
    for r in range(NR + 1):
        f = r / NR
        Wd_, Td_, p_ = keyf(f, W_KEYS), keyf(f, T_KEYS), keyf(f, P_KEYS)
        row = []
        for j in range(NP):
            u, v, xi = _sec(j, Wd_, Td_, p_)
            s = S0 + (keyf(xi, WEB) - S0) * f
            if v < 0:
                v *= 1.0 + 0.10 * math.sin(math.pi * f)          # fuller palm side
            if 0 < j < 10:
                kn = math.exp(-((r - (NR - 1.6)) / 0.9) ** 2)    # knuckle row on the back
                if j % 2 == 0:
                    v += 0.0016 * kn
                elif j not in (1, 9):
                    v -= 0.0010 * max(kn, 1.0 if r == NR else 0.0)
            if j not in (0, 10) and (j % 2 == 1) and j not in (1, 9, 11, 19) and r == NR:
                v *= 0.86                                         # valleys between the fingers
            row.append(V((u, v, s)))
        grid.append(row)

    # thenar mound (ball of the thumb)
    bump_c, bump_d = V((0.030, -0.014, 0.036)), V((0.55, -0.80, 0.0)).normalized()
    for r in range(NR + 1):
        for j in range(NP):
            p = grid[r][j]
            g = math.exp(-((p - bump_c).length / 0.020) ** 2)
            if r > 0:
                grid[r][j] = p + bump_d * (0.007 * g)

    # thumb root: a 2x2 patch on the thumb edge of the palm becomes the thumb's base ring
    R1 = 2
    hole_cells = {(R1, 19), (R1, 0), (R1 + 1, 19), (R1 + 1, 0)}          # (row, col) of removed quads
    h = [(R1, 1), (R1, 0), (R1, 19), (R1 + 1, 19), (R1 + 2, 19), (R1 + 2, 0), (R1 + 2, 1), (R1 + 1, 1)]
    th_c = V((0.0400, -0.0075, 0.0365))
    th_n0 = V((0.80, -0.30, 0.52)).normalized()
    th_dir = V((0.36, -0.34, 0.87)).normalized()
    vhint, uhint = V((0.0, 1.0, 0.0)), V((1.0, 0.0, -1.0))
    fv0 = (vhint - th_n0 * vhint.dot(th_n0)).normalized()
    fu0 = th_n0.cross(fv0).normalized()
    if fu0.dot(uhint) < 0:
        fu0 = -fu0
    th_base_cells = [h[0], h[7], h[6], h[5], h[4], h[3], h[2], h[1]]    # RING8 order
    for (r, j), (cx, cy) in zip(th_base_cells, RING8):
        grid[r][j] = th_c + fu0 * (0.0125 * cx) + fv0 * (0.0118 * cy)

    idx = {}
    for r in range(NR + 1):
        for j in range(NP):
            if (r, j) == (R1 + 1, 0):
                continue                                            # vertex inside the thumb hole
            idx[(r, j)] = add(grid[r][j])
    for r in range(NR):
        for j in range(NP):
            if (r, j) in hole_cells:
                continue
            j2 = (j + 1) % NP
            faces.append((idx[(r, j)], idx[(r, j2)], idx[(r + 1, j2)], idx[(r + 1, j)]))

    def add_digit(base, rings, tip):
        prev = base
        for rg in rings:
            cur = [add(p) for p in rg]
            for k in range(8):
                k2 = (k + 1) % 8
                faces.append((prev[k], prev[k2], cur[k2], cur[k]))
            prev = cur
        c = add(tip)
        for k in (1, 3, 5, 7):
            faces.append((c, prev[k], prev[(k + 1) % 8], prev[(k + 2) % 8]))

    # ---------------- four fingers, welded to the last palm row
    web = []
    for f in range(3):
        ub = U_BOUND[f + 1]
        web.append(add(V((ub, -0.0008, keyf(ub / PALM_HW, WEB) + 0.0040))))
    for f in range(4):
        base = [idx[(NR, 1 + 2 * f)], idx[(NR, 2 + 2 * f)], idx[(NR, 3 + 2 * f)],
                idx[(NR, 10)] if f == 3 else web[f],
                idx[(NR, 17 - 2 * f)], idx[(NR, 18 - 2 * f)], idx[(NR, 19 - 2 * f)],
                idx[(NR, 0)] if f == 0 else web[f - 1]]
        g = math.radians(FINGER_FAN[f])
        fd = V((math.sin(g), 0.0, math.cos(g)))
        fu = V((math.cos(g), 0.0, -math.sin(g)))
        fv = V((0.0, 1.0, 0.0))
        dirs, dd, vv = [], fd, fv
        for ang in FINGER_BEND[f]:
            a = math.radians(ang)
            dd, vv = (dd * math.cos(a) - vv * math.sin(a)).normalized(), (vv * math.cos(a) + dd * math.sin(a)).normalized()
            dirs.append(dd)
        L = FINGER_LEN[f]
        P0 = V((U_CENT[f], -0.0005, keyf(U_CENT[f] / PALM_HW, WEB)))
        rings, tip = digit(P0, dirs, (0.36 * L, 0.34 * L, 0.30 * L), fu, fv,
                           FINGER_W[f] / 2 - 0.0012, FINGER_RV[f])
        add_digit(base, rings, tip)

    # ---------------- thumb
    d0 = (th_n0 + th_dir).normalized()
    d2 = (th_dir + V((-0.20, -0.10, 0.04))).normalized()
    rings, tip = digit(th_c, [d0, th_dir, d2], (0.016, 0.034, 0.030), uhint, vhint, 0.0118, 0.0110, sep=0.006)
    add_digit([idx[c] for c in th_base_cells], rings, tip)

    hand = lib.mesh_object("Hand_L", [W(p) for p in verts], faces, topo="main", sharp_angle=180)

    # ---------------- wrist support loops
    Wd0, Td0, p0 = keyf(0.0, W_KEYS), keyf(0.0, T_KEYS), keyf(0.0, P_KEYS)
    wr_rings = [[W(p) for p in grid[0]]]
    for s, k in ((0.007, 0.970), (0.003, 0.955), (-0.001, 0.960)):
        wr_rings.append([W(p) for p in section(s, Wd0 * k, Td0 * k, p0)])
    wrist = lib.lofted("Wrist_L", wr_rings, topo="support", sharp_angle=180)

    # ---------------- glove cuff (thick, flares up inside the sleeve cuff)
    Wb, Tb = Wd0 * 0.96, Td0 * 0.96
    prof = [(0.000, 0.0008), (0.000, 0.0065), (-0.004, 0.0085), (-0.008, 0.0100), (-0.010, 0.0085),
            (-0.022, 0.0105), (-0.034, 0.0120), (-0.045, 0.0130), (-0.045, 0.0100), (-0.022, 0.0040)]
    cuff = lib.mesh_object("Glove_cuff_L", *loop_loft([[W(p) for p in section(s, Wb + g, Tb + g * 1.5, 2.3)]
                                                        for s, g in prof]), topo="main", sharp_angle=60)

    # ---------------- rigid bits: knuckle studs + back-of-hand plate
    armor = []
    nrm = Wd(V((0.0, 1.0, 0.22)).normalized())
    for f in range(4):
        p = (grid[NR - 2][2 + 2 * f] + grid[NR - 1][2 + 2 * f]) * 0.5 + V((0.0, -0.0010, 0.0))
        armor.append(lib.stud(f"knuckle_{f}", radius=0.0078 if f < 3 else 0.0068, height=0.0048, n=8,
                              loc=W(p), normal=nrm))
    pc_u, pc_s, phu, phs = -0.001, 0.047, 0.0175, 0.0165
    pr = []
    for shrink, lift in ((0.0, -0.0015), (0.0, 0.0028), (0.0022, 0.0040)):
        rg = []
        for k in range(12):
            a = TAU * k / 12
            cu = math.copysign(abs(math.cos(a)) ** 0.5, math.cos(a))
            cs = math.copysign(abs(math.sin(a)) ** 0.5, math.sin(a))
            u, s = pc_u + (phu - shrink) * cu, pc_s + (phs - shrink) * cs
            rg.append(W(V((u, back_v(u, s) + lift, s))))
        pr.append(rg)
    plate = lib.lofted("hand_plate", pr, cap_end=True, topo="rigid", sharp_angle=35)
    armor.append(plate)
    armor.append(lib.stud("hand_plate_stud", radius=0.0045, height=0.003, n=8,
                          loc=W(V((pc_u, back_v(pc_u, pc_s) + 0.0036, pc_s))), normal=Wd(V((0.0, 1.0, 0.08)).normalized())))
    armor = lib.join(armor, "Hand_armor_L", topo="rigid")
    return [cuff, wrist, hand, armor]


# ------------------------------------------------------------------ entry point
def build():
    path = ArmPath()
    left = build_sleeve(path) + build_bracer(path) + build_hand()
    right = [lib.mirror_copy(o) for o in left]
    return left + right
