"""Arms (round 2): coat sleeves from the armhole seam, flared gauntlet bracers, gloved hands.

Everything is built for the LEFT (+X) arm and mirrored with lib.mirror_copy.

Objects per arm
    Sleeve_upper_L   main     armhole seam ring -> elbow, with 3 soft fold rolls (16 around)
    Sleeve_seam_L    main     lengthwise sleeve seam (piping) on the outer-front of the sleeve
    Sleeve_elbow_L   support  3 tight loops at the elbow
    Sleeve_lower_L   main     elbow -> a little way inside the bracer (rest is hidden)
    Cuff_L           main     flared fabric turn-back cuff over the forearm (hem on top, thick fold at the wrist)
    Cuff_rivets_L    rigid    2 x 2 dome rivets on its front-outer face
    Glove_cuff_L     main     short flared glove cuff coming out of the bracer
    Wrist_L          support  3 tight loops at the wrist
    Hand_L           main     palm + 4 fingers + thumb, one welded all-quad mesh
    Hand_armor_L     rigid    4 small round knuckle rivets + a small star on the back of the hand

Sleeve "phi" angle around the arm: 0 = outer side (+X), 90 deg = front (-Y), 270 deg = back.
"""
import bisect
import math

import bpy  # noqa: F401
from mathutils import Quaternion, Vector as V

from infantry import lib, spec

S = spec
CL = S.CLOTH
TAU = math.tau
NS = 16                                   # sleeve verts around (PLAN: sleeve/arm 16)
NB = 32                                   # cuff verts around
# SIDE (side_arm, side_skirt): the forearm angles forward from the elbow more than spec v2 gives ->
# wrist and hand tip are moved forward together (the FRONT silhouette keeps them at spec x).
WRIST_OFFSET = V((0.0, -0.035, 0.0))
WRIST = S.WRIST + WRIST_OFFSET
HAND_TIP = S.HAND_TIP + WRIST_OFFSET
D1 = (S.ELBOW - S.SHOULDER).normalized()
D2 = (WRIST - S.ELBOW).normalized()
LU = (S.ELBOW - S.SHOULDER).length        # upper arm length
LF = (WRIST - S.ELBOW).length           # forearm length

# ---------------------------------------------------------------- read from the reference
# FRONT/BACK (docs/ref/front_arm_*.png): the sleeve hangs ~1.5-2 cm inside the straight
# SHOULDER -> ELBOW line at mid upper arm (sleeve spans x 0.18-0.30 at z 1.31) and tucks under the
# shoulder armour; it widens into stacked fold rolls just above the elbow (outer x 0.33-0.35).
UPPER_BOW = 0.017          # max inward bow of the upper-arm centre line (zero at shoulder + elbow)
UPPER_FLAT = 0.90          # lateral (front-view) radius factor of the upper sleeve
# callout_elbow.png: 3 gentle undulations above the elbow (the sleeves are otherwise smooth)
ROLL_CRESTS = (-0.098, -0.068, -0.038)    # crest positions, metres along the arm from the elbow
ROLL_AMP = (0.0030, 0.0036, 0.0042)       # roll height (radial)
ROLL_SAG = 0.008                          # front of each roll sits this much lower than the back
ROLL_SHAPE = ((0.0, 1.0), (0.008, 0.55), (0.0155, -0.12), (0.0225, 0.30))   # (offset from crest, height)
ELBOW_POINT = 0.0028                      # bony elbow bulge at the back
SEAM_PHI_INDEX = 2                        # lengthwise seam runs along sleeve vertex 2 (45 deg, outer-front)
# front_arm_*, side_arm, side_skirt: a flared FABRIC turn-back cuff over the forearm: thin hem at the
# top (edge rises toward the outer side), thick rounded fold at the wrist end, 2x2 rivets on the front
# face, no strap.  The cuff is cloth (topo main); only the rivets are rigid.
BRACER_R = (0.075, 0.055)                 # outer radius at the elbow end / wrist end
BRACER_TH = 0.005                         # double cloth layer (turned back)
BRACER_TOP_DROP = 0.0                     # 0: hem square to the forearm, so its outer side is already higher
RIM_TOP = (0.012, 0.0045)                 # top hem (width along the arm, bulge above the cuff)
RIM_BOT = (0.022, 0.0090)                 # thick rounded fold at the wrist end
CUFF_FOLD = 0.0016                        # soft vertical cloth folds, deeper toward the flared top
RIVET_R, RIVET_H = 0.0130, 0.0065         # 2 x 2 dome rivets
RIVET_PHI = (32.0, 82.0)                  # two columns on the front-outer face (0 outer, 90 front)
RIVET_X = (0.20, 0.76)                    # two rows along the cuff (0 elbow end, 1 wrist end)
HAND_SCALE = 1.15                         # checklist: larger, chunkier glove than round 1
FINGER_THICK = (1.10, 1.20)               # review: plain THICK gloved fingers (width, depth factors)
HAND_TURN = 25.0                          # front_arm_*: back of the hand turned toward the front (deg)
HAND_ALIGN = 0.7                          # how far the curled middle tip is swung onto WRIST->HAND_TIP
HAND_RISE = 0.012                         # glove emerges right under the bracer rim, tips end at z ~0.785


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
def bow(s):
    """Inward bow of the upper-arm centre line: 0 at the shoulder and at the elbow, small under the
    shoulder armour, largest at about two thirds of the upper arm (where the reference sleeve is)."""
    return UPPER_BOW * keyf(min(max(s / LU, 0.0), 1.0), [(0.0, 0.0), (0.42, 0.30), (0.70, 1.0), (1.0, 0.0)])


_OUT = (V((1.0, 0.0, 0.0)) - D1 * D1.x).normalized()     # lateral direction across the upper arm


class ArmPath:
    """Arm centre line (bowed upper arm, rounded elbow, straight forearm), addressed by the nominal
    parameter s: 0 at SHOULDER, LU at ELBOW, LU + LF at WRIST (distance along the spec bones)."""
    STEP = 0.002

    def __init__(self, rnd=0.028, s_min=-0.10, extra=0.06):
        def up(s):
            return S.SHOULDER + D1 * s - _OUT * bow(s)
        pts, ss = [], []
        n1 = int(round((LU - rnd - s_min) / self.STEP))
        for i in range(n1):
            s = s_min + (LU - rnd - s_min) * i / n1
            pts.append(up(s))
            ss.append(s)
        a, b = up(LU - rnd), S.ELBOW + D2 * rnd
        nb = int(round(2 * rnd / self.STEP))
        for i in range(nb):
            f = i / nb
            pts.append(a * (1 - f) ** 2 + S.ELBOW * (2 * f * (1 - f)) + b * (f * f))
            ss.append(LU - rnd + 2 * rnd * f)
        n2 = int(round((LF - rnd + extra) / self.STEP))
        for i in range(n2 + 1):
            f = i / n2
            pts.append(b.lerp(WRIST + D2 * extra, f))
            ss.append(LU + rnd + (LF - rnd + extra) * f)
        self.pts, self.ss = pts, ss
        self.frames = lib.frames_along(pts, x_hint=(1, 0, 0))   # (tangent, outer, front)

    def at(self, s):
        ss = self.ss
        i = min(max(bisect.bisect_right(ss, s) - 1, 0), len(ss) - 2)
        f = min(max((s - ss[i]) / max(ss[i + 1] - ss[i], 1e-9), 0.0), 1.0)
        c = self.pts[i].lerp(self.pts[i + 1], f)
        t0, x0, y0 = self.frames[i]
        t1, x1, y1 = self.frames[i + 1]
        return c, t0.lerp(t1, f).normalized(), x0.lerp(x1, f).normalized(), y0.lerp(y1, f).normalized()


def t_of(s):
    """nominal s -> spec arm parameter (0 shoulder, 1 elbow, 2 wrist)."""
    return s / LU if s < LU else 1.0 + (s - LU) / LF


BAG = [(0.0, 0.000), (0.55, 0.001), (0.85, 0.0025), (1.0, 0.003), (1.15, 0.002), (1.5, 0.0)]


def sleeve_R(s):
    """Smooth sleeve radius (no folds)."""
    t = min(max(t_of(s), 0.0), 2.0)
    return S.arm_radius(t) + CL["sleeve_offset"] + keyf(t, BAG)


def sleeve_k(s):
    """(lateral, front-back) radius factors: the upper sleeve is a little flatter seen from the front."""
    kx = keyf(t_of(s), [(0.0, UPPER_FLAT), (0.55, UPPER_FLAT), (0.95, 1.0)])
    return kx, 1.0 + (1.0 - kx) * 0.6


def clamp_to_coat(p, weight=1.0, gap=0.004):
    """Keep the sleeve from sinking into the coat torso under the armpit (soft clamp)."""
    if not (0.90 < p.z < 1.56):
        return p
    cy, rx, ry = S.torso_section(p.z)
    rx += CL["coat_offset"] + gap
    ry += CL["coat_offset"] + gap
    dy = (p.y - cy) / ry
    if abs(dy) >= 1.0:
        return p
    xlim = rx * math.sqrt(1.0 - dy * dy)
    m = 0.012
    if p.x < xlim + m:
        x_new = xlim + m * math.exp((p.x - xlim) / m - 1.0)
        p = V((p.x + (x_new - p.x) * weight, p.y, p.z))
    return p


# ------------------------------------------------------------------ sleeve
def build_sleeve(path):
    seam_pts, seam_nrm = [], []

    def ring(s, disp=0.0, tiltw=0.0, seam=True):
        """One 16-vert ring.  disp: radial fold displacement, tiltw: how much the loop follows the roll sag."""
        out = []
        for i in range(NS):
            phi = TAU * i / NS
            sp = math.sin(phi)
            si = s + ROLL_SAG * tiltw * sp
            c, t, e1, e2 = path.at(si)
            kx, ky = sleeve_k(si)
            R = sleeve_R(si)
            d = disp * (0.65 + 0.35 * max(0.0, sp) + 0.20 * max(0.0, -math.cos(phi)))   # bunches in the elbow pit
            if d < 0:
                d *= 1.0 + 0.35 * max(0.0, sp)                       # creases bite deeper in the elbow pit
            d += ELBOW_POINT * max(0.0, -sp) * math.exp(-((si - LU) / 0.020) ** 2)
            p = c + e1 * (math.cos(phi) * (R * kx + d)) + e2 * (sp * (R * ky + d))
            out.append(clamp_to_coat(p))
            if seam and i == SEAM_PHI_INDEX:
                seam_pts.append(out[-1])
                seam_nrm.append((p - c).normalized())
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

    # --- blend from the tilted seam ring into rings perpendicular to the arm
    KB = 9
    a_end = LU + ROLL_CRESTS[0] - 0.014
    upper = [[p.copy() for p in seam]]
    for k in range(1, KB + 1):
        x = k / KB
        wr = 1.0 - (1.0 - x) ** 4.0             # reach the full sleeve radius early (rounded sleeve head)
        rg = []
        for i in range(NS):
            a0, rho0, phi0 = cyl[i]
            phi_t = TAU * i / NS
            a = a0 + (a_end - a0) * x ** (1.0 + 0.15 * (1.0 + math.cos(phi_t)))
            kx, ky = sleeve_k(a)
            R = sleeve_R(max(a, 0.0))
            rho_t = R * math.hypot(kx * math.cos(phi_t), ky * math.sin(phi_t))
            phi_e = math.atan2(ky * math.sin(phi_t), kx * math.cos(phi_t))
            phi_e = phi_t + ((phi_e - phi_t + math.pi) % TAU - math.pi)
            rho = rho0 + (rho_t - rho0) * wr
            phi = phi0 + (phi_e - phi0) * wr
            c, t, f1, f2 = path.at(a)
            p = c + (f1 * math.cos(phi) + f2 * math.sin(phi)) * rho
            rg.append(clamp_to_coat(p, weight=min(1.0, k / 2.0)))
            if k >= 2 and i == SEAM_PHI_INDEX:
                seam_pts.append(rg[-1])
                seam_nrm.append((p - c).normalized())
        upper.append(rg)

    # --- soft fold rolls: crest, lower flank, crease, rise into the next roll
    for c, amp in zip(ROLL_CRESTS, ROLL_AMP):
        upper += [ring(LU + c + dc, f * amp, 1.0) for dc, f in ROLL_SHAPE]

    # --- elbow support band (shares its boundary rings with the neighbours)
    elbow = [upper[-1]]
    for ds, disp, tw in ((-0.0075, 0.0022, 0.8), (0.0, 0.0032, 0.6), (0.0075, 0.0016, 0.4)):
        elbow.append(ring(LU + ds, disp, tw))
    elbow.append(ring(LU + 0.014, -0.001, 0.2))

    # --- lower sleeve: just far enough into the bracer to hide its end
    lower = [elbow[-1]] + [ring(LU + ds, 0.0, 0.0, seam=False) for ds in (0.028, 0.042, 0.056)]

    seam_obj = lib.strip_along("Sleeve_seam_L", seam_pts, seam_nrm, width=0.0065, thickness=0.0020,
                               sink=0.0008, profile="round", topo="main")
    return [lib.lofted("Sleeve_upper_L", upper, topo="main", sharp_angle=180), seam_obj,
            lib.lofted("Sleeve_elbow_L", elbow, topo="support", sharp_angle=180),
            lib.lofted("Sleeve_lower_L", lower, topo="main", sharp_angle=180)]


# ------------------------------------------------------------------ bracer
BT0, BT1 = CL["bracer"]
LB = (BT1 - BT0) * LF                                       # bracer length along the forearm
E1F = (V((1.0, 0.0, 0.0)) - D2 * D2.x).normalized()         # outer
E2F = D2.cross(E1F).normalized()                            # front


def bracer_R(x):
    """Outer cuff radius (no folds), x = distance from the elbow-end edge along the forearm."""
    return BRACER_R[0] + (BRACER_R[1] - BRACER_R[0]) * min(max(x / LB, 0.0), 1.0)


def bracer_shear(x, phi_deg):
    """Along-arm shift of the upper cuff: the outer side (phi 0) of the top hem rides higher."""
    return BRACER_TOP_DROP * 0.5 * (1.0 + math.cos(math.radians(phi_deg))) * max(0.0, 1.0 - x / (0.55 * LB))


def cuff_fold(x, phi_deg):
    """Soft vertical cloth folds round the cuff, deepest at the flared top."""
    v = 1.0 - min(max(x / LB, 0.0), 1.0)
    return lib.fold_wave(phi_deg / 360.0, v, amp=CUFF_FOLD, count=5.0, seed=0.3, grow=0.85)


def bracer_pt(x, phi_deg, off=0.0):
    """Point on the cuff (+off radially), cone normal, direction down the surface."""
    x = x + bracer_shear(x, phi_deg)
    c = S.ELBOW + D2 * (BT0 * LF + x)
    ph = math.radians(phi_deg)
    rad = E1F * math.cos(ph) + E2F * math.sin(ph)
    slope = (BRACER_R[1] - BRACER_R[0]) / LB
    r = bracer_R(x) + off + cuff_fold(x, phi_deg)
    return c + rad * r, (rad - D2 * slope).normalized(), (D2 + rad * slope).normalized()


def build_bracer():
    def ring(x, off):
        return [bracer_pt(x, 360.0 * j / NB, off)[0] for j in range(NB)]

    def arc(xc, w, h, a0, a1, n):
        """Rounded hem / fold profile: elliptic arc centred at xc, spanning -TH .. h radially."""
        cy, ay = (h - BRACER_TH) / 2, (h + BRACER_TH) / 2
        return [(xc + w / 2 * math.cos(math.radians(a)), cy + ay * math.sin(math.radians(a)))
                for a in (a0 + (a1 - a0) * i / n for i in range(n + 1))]

    wt, ht = RIM_TOP
    wb, hb = RIM_BOT
    prof = arc(wt / 2, wt, ht, 270, 0, 4)                     # inner layer -> over the top hem -> outside
    body0, body1 = wt + 0.0015, LB - wb - 0.0015
    nb = max(2, round((body1 - body0) / 0.021))
    prof += [(body0 + (body1 - body0) * i / nb, 0.0) for i in range(nb + 1)]
    prof += arc(LB - wb / 2, wb, hb, 180, -90, 6)             # outside -> round the wrist fold -> inner layer
    prof += [(LB * 0.62, -BRACER_TH), (LB * 0.30, -BRACER_TH)]
    cuff = lib.mesh_object("Cuff_L", *loop_loft([ring(x, o) for x, o in prof]), topo="main", sharp_angle=60)

    # --- 2 x 2 dome rivets on the front-outer face
    rivets = []
    for i, fx in enumerate(RIVET_X):
        for j, ph in enumerate(RIVET_PHI):
            p, nrm, _ = bracer_pt(fx * LB, ph, -0.0007)
            rivets.append(lib.rivet(f"cuff_rivet_{i}{j}", radius=RIVET_R, height=RIVET_H, n=16, loc=p, normal=nrm))
    rivets = lib.join(rivets, "Cuff_rivets_L", topo="rigid")
    return [cuff, rivets]


# ------------------------------------------------------------------ hand
# Hand-local coordinates (before HAND_SCALE): x = u across the palm (+ towards the thumb / front of
# the character), y = v thickness (+ back of the hand, - palm), z = s along the hand (wrist -> tips).
FINGER_W = [0.0220, 0.0230, 0.0215, 0.0200]          # index, middle, ring, pinky
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
W_KEYS = [(0.0, 0.0360), (0.25, 0.0385), (0.6, 0.0415), (1.0, PALM_HW)]
T_KEYS = [(0.0, 0.0285), (0.3, 0.0205), (0.7, 0.0155), (1.0, 0.0116)]
P_KEYS = [(0.0, 2.3), (0.3, 3.0), (1.0, 3.6)]
FINGER_LEN = [0.078, 0.086, 0.080, 0.062]
FINGER_FAN = [7.0, 1.5, -4.0, -9.5]                   # degrees, + towards the thumb (slightly spread)
FINGER_BEND = [(10, 21, 13), (12, 24, 14), (15, 27, 15), (18, 30, 16)]   # MCP, PIP, DIP curl (deg)
FINGER_RV = [0.0101, 0.0103, 0.0100, 0.0091]


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


class Digit:
    """3-segment digit: rings (after the base ring) with loop pairs at each knuckle + a tip point."""

    def __init__(self, P0, dirs, Ls, fu_hint, fv_hint, ru, rv, sep=0.007, e=0.0032, taper=0.14):
        self.dirs, self.Ls, self.fu_hint, self.fv_hint = dirs, Ls, fu_hint, fv_hint
        self.ru, self.rv, self.e, self.sep, self.taper = ru, rv, e, sep, taper
        self.joints = [V(P0)]
        for dd, L in zip(dirs, Ls):
            self.joints.append(self.joints[-1] + dd * L)
        self.cum = [0.0, Ls[0], Ls[0] + Ls[1], sum(Ls)]
        L1, L2, L3 = Ls
        Lt = self.cum[3]
        st = [(sep, 1.0)]
        span = L1 - e - sep
        n = max(1, round(span / 0.010))
        st += [(sep + span * i / n, 0.98) for i in range(1, n)]
        st += [(L1 - e, 1.04), (L1 + e, 1.04)]
        span = L2 - 2 * e
        n = max(1, round(span / 0.010))
        st += [(L1 + e + span * i / n, 0.97) for i in range(1, n)]
        st += [(L1 + L2 - e, 1.03), (L1 + L2 + e, 1.03)]
        xa, xb = L1 + L2 + e, Lt - 0.0100
        n = max(1, round((xb - xa) / 0.008))
        st += [(xa + (xb - xa) * i / n, 1.0) for i in range(1, n)]
        st += [(xb, 0.99), (Lt - 0.0052, 0.90), (Lt - 0.0018, 0.60)]
        self.rings = [self.ring_at(x, sc) for x, sc in st]
        c, dd = self.at(Lt)
        fu, fv = self.frame(dd)
        self.tip = c - fv * (rv * 0.12)

    def frame(self, dd):
        fv = (self.fv_hint - dd * self.fv_hint.dot(dd)).normalized()
        fu = dd.cross(fv).normalized()
        if fu.dot(self.fu_hint) < 0:
            fu = -fu
        return fu, fv

    def at(self, x):
        cum, dirs = self.cum, self.dirs
        k = 0 if x < cum[1] else (1 if x < cum[2] else 2)
        c = self.joints[k] + dirs[k] * (x - cum[k])
        dd = dirs[k]
        m = 1.7 * self.e
        for jn in (1, 2):
            if abs(x - cum[jn]) < m:
                wgt = (x - cum[jn] + m) / (2 * m)
                dd = dirs[jn - 1].lerp(dirs[jn], wgt).normalized()
        return c, dd

    def ring_at(self, x, sc=1.0):
        c, dd = self.at(x)
        fu, fv = self.frame(dd)
        k = sc * (1.0 - self.taper * x / self.cum[3])
        return [c + fu * (self.ru * k * cx) + fv * (self.rv * k * (cy if cy > 0 else cy * 0.92)) for cx, cy in RING8]


def build_hand():
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

    # thenar mound (ball of the thumb) + the web between thumb and index finger
    bump_c, bump_d = V((0.030, -0.018, 0.034)), V((0.45, -0.89, 0.0)).normalized()
    for r in range(1, NR + 1):
        for j in range(NP):
            p = grid[r][j]
            p = p + bump_d * (0.005 * math.exp(-((p - bump_c).length / 0.022) ** 2))
            grid[r][j] = p + V((-0.0045, 0.002, 0.0)) * math.exp(-((p - V((0.040, -0.006, 0.058))).length / 0.011) ** 2)

    # thumb root: a 2x2 patch on the palm-side thumb corner of the palm becomes the thumb's base ring
    R1 = 2
    hole_cells = {(R1, 18), (R1, 19), (R1 + 1, 18), (R1 + 1, 19)}
    hole_vert = (R1 + 1, 19)
    h = [(R1, 0), (R1, 19), (R1, 18), (R1 + 1, 18), (R1 + 2, 18), (R1 + 2, 19), (R1 + 2, 0), (R1 + 1, 0)]
    th_c = V((0.0365, -0.0125, 0.0360))
    th_n0 = V((0.84, -0.40, 0.36)).normalized()
    th_dir = V((0.50, -0.20, 0.84)).normalized()
    vhint, uhint = V((0.0, 1.0, 0.0)), V((1.0, 0.0, -1.0))
    fv0 = (vhint - th_n0 * vhint.dot(th_n0)).normalized()
    fu0 = th_n0.cross(fv0).normalized()
    if fu0.dot(uhint) < 0:
        fu0 = -fu0
    th_base_cells = [h[0], h[7], h[6], h[5], h[4], h[3], h[2], h[1]]    # RING8 order
    for (r, j), (cx, cy) in zip(th_base_cells, RING8):
        tgt = th_c + fu0 * (0.0130 * cx) + fv0 * (0.0122 * cy)
        grid[r][j] = grid[r][j].lerp(tgt, 0.75)
    th_c = sum((grid[r][j] for r, j in th_base_cells), V((0.0, 0.0, 0.0))) / 8.0

    idx = {}
    for r in range(NR + 1):
        for j in range(NP):
            if (r, j) == hole_vert:
                continue
            idx[(r, j)] = add(grid[r][j])
    for r in range(NR):
        for j in range(NP):
            if (r, j) in hole_cells:
                continue
            j2 = (j + 1) % NP
            faces.append((idx[(r, j)], idx[(r, j2)], idx[(r + 1, j2)], idx[(r + 1, j)]))

    def add_digit(base, dg):
        prev = base
        for rg in dg.rings:
            cur = [add(p) for p in rg]
            for k in range(8):
                k2 = (k + 1) % 8
                faces.append((prev[k], prev[k2], cur[k2], cur[k]))
            prev = cur
        c = add(dg.tip)
        for k in (1, 3, 5, 7):
            faces.append((c, prev[k], prev[(k + 1) % 8], prev[(k + 2) % 8]))

    # ---------------- four fingers, welded to the last palm row
    digits = []
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
        dg = Digit(P0, dirs, (0.36 * L, 0.34 * L, 0.30 * L), fu, fv, (FINGER_W[f] / 2 - 0.0012) * FINGER_THICK[0],
                   FINGER_RV[f] * FINGER_THICK[1])
        add_digit(base, dg)
        digits.append(dg)

    # ---------------- thumb
    d0 = (th_n0 * 0.8 + th_dir * 0.2).normalized()
    d2 = (th_dir + V((-0.24, -0.14, 0.06))).normalized()
    thumb = Digit(th_c, [d0, th_dir, d2], (0.020, 0.032, 0.029), uhint, vhint, 0.0118 * FINGER_THICK[0],
                  0.0110 * FINGER_THICK[1], sep=0.006)
    add_digit([idx[c] for c in th_base_cells], thumb)

    # ---------------- frames.  Back of the hand faces out (+X) turned HAND_TURN toward the front,
    # thumb to the front; then the whole hand is rotated about the wrist so that the relaxed (curled)
    # middle fingertip swings (HAND_ALIGN of the way) onto the spec.WRIST -> spec.HAND_TIP line.
    d0_ = (HAND_TIP - WRIST).normalized()
    O = WRIST - d0_ * HAND_RISE
    b = V((1.0, 0.0, 0.0))
    b = (b - d0_ * b.dot(d0_)).normalized()
    w = d0_.cross(b).normalized()
    d = d0_.copy()
    tr = math.radians(HAND_TURN)
    b, w = b * math.cos(tr) + w * math.sin(tr), w * math.cos(tr) - b * math.sin(tr)
    mt = digits[1].tip
    q = (w * mt.x + b * mt.y + d * mt.z).normalized().rotation_difference(d0_)
    q = Quaternion().slerp(q, HAND_ALIGN)
    w, b, d = q @ w, q @ b, q @ d
    HS = HAND_SCALE

    def W(p):
        return O + (w * p.x + b * p.y + d * p.z) * HS

    def Wd(p):
        return (w * p.x + b * p.y + d * p.z).normalized()

    bf = (V((1.0, 0.0, 0.0)) - D2 * D2.x).normalized()   # forearm-aligned frame for the glove cuff
    wf = D2.cross(bf).normalized()
    bf, wf = bf * math.cos(tr) + wf * math.sin(tr), wf * math.cos(tr) - bf * math.sin(tr)

    def Wf(p):
        return O + (wf * p.x + bf * p.y + D2 * p.z) * HS

    hand = lib.mesh_object("Hand_L", [W(p) for p in verts], faces, topo="main", sharp_angle=180)

    # ---------------- wrist support loops
    Wd0, Td0, p0 = keyf(0.0, W_KEYS), keyf(0.0, T_KEYS), keyf(0.0, P_KEYS)
    wr_rings = [[W(p) for p in grid[0]]]
    for s, k, hw in ((0.007, 0.970, 0.67), (0.003, 0.955, 0.33), (-0.001, 0.960, 0.0)):
        wr_rings.append([Wf(p).lerp(W(p), hw) for p in section(s, Wd0 * k, Td0 * k, p0)])
    wrist = lib.lofted("Wrist_L", wr_rings, topo="support", sharp_angle=180)

    # ---------------- short flared glove cuff: a rolled lip just above the wrist, flaring up into the bracer
    Wb, Tb = Wd0 * 0.96, Td0 * 0.96
    prof = [(0.0010, 0.0008), (0.0010, 0.0042), (-0.0015, 0.0056), (-0.0045, 0.0052), (-0.0080, 0.0050),
            (-0.0160, 0.0062), (-0.0300, 0.0068), (-0.0300, 0.0038), (-0.0120, 0.0016)]
    cuff = lib.mesh_object("Glove_cuff_L", *loop_loft([[Wf(p) for p in section(s, Wb + g, Tb + g * 1.4, 2.3)]
                                                        for s, g in prof]), topo="main", sharp_angle=60)

    # ---------------- rigid bits: small round rivets on the knuckles + a small star on the back of the hand
    armor = []
    kr = [(grid[NR - 2][j] + grid[NR - 1][j]) * 0.5 for j in range(1, 10)]
    for f in range(4):
        i = 1 + 2 * f
        tg = kr[i + 1] - kr[i - 1]
        n = tg.cross(V((0.0, 0.0, 1.0))).normalized()
        n = n if n.y > 0 else -n
        armor.append(lib.rivet(f"knuckle_rivet_{f}", radius=(0.0050 if f < 3 else 0.0044) * HS, height=0.0042,
                               n=12, loc=W(kr[i]) - Wd(n) * 0.0006, normal=Wd(n)))
    pc_u, pc_s = -0.001, 0.045
    armor.append(lib.emblem_cross("hand_star", size=0.0085, thickness=0.0018, long_tail=1.5,
                                  loc=W(V((pc_u, back_v(pc_u, pc_s), pc_s))) - Wd(V((0.0, 1.0, 0.0))) * 0.0004,
                                  normal=Wd(V((0.0, 1.0, 0.08))), up=-d))
    armor = lib.join(armor, "Hand_armor_L", topo="rigid")
    return [cuff, wrist, hand, armor]


# ------------------------------------------------------------------ entry point
def build():
    path = ArmPath()
    left = build_sleeve(path) + build_bracer() + build_hand()
    right = [lib.mirror_copy(o) for o in left]
    return left + right
