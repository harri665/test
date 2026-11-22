"""Forearm bracers (round 5, FBX item A2): angular hard-surface vambrace like the user's FBX model.

Everything is built for the LEFT (+X) forearm and mirrored with lib.mirror_copy.  The bracer sits on
exactly the forearm arms.py builds: the frame below is derived from arms.WRIST / spec.ELBOW /
spec.CLOTH["bracer"] the same way arms.bracer_pt does it.

Design read from the FBX "Arm Cuffs" (measured in its T-pose, then mapped onto our hanging forearm):
a 12-sided faceted shell -- a thick collar ring at the elbow end, a groove, a tapering body, a narrow
rib -- and, on the back of the forearm (outer/front face here, where the back of the hand points), a
raised plate standing well proud at the elbow end and sloping down toward the wrist.  The plate has a
raised border band all round; its top edge is a V (chevron) whose apex meets at the centre line of the
plate, high up against the collar ring; the sides waist in (hourglass) and the wrist end flares into
two small wing corners; a short spine ridge runs down the centre from the V's apex.  A separate wrist
ring closes the bracer over the glove cuff.  No rivets, no cloth folds.

Objects per arm (all hard surface -> topo "rigid")
    Cuff_L          collar ring + groove + faceted tapering body + rib, one all-quad shell (24 around:
                    12 flat facets with chamfered corners), support loops at every hard edge
    Cuff_plate_L    the raised back plate: border band (bevelled step in and out), faceted field, spine
                    ridge; a "picture frame" quad grid with mitred corners, its walls sunk into the shell
    Cuff_band_L     separate wrist ring (closed all-quad ring, chamfered edges); the glove cuff tucks
                    into it

Bracer coordinates: x = distance along the forearm from C0 toward the wrist,
phi = angle round the forearm, 0 = outer side (+X), 90 deg = front (-Y), 270 = back.
"""
import math

import bpy  # noqa: F401
from mathutils import Vector as V

from infantry import lib, spec
from infantry.parts import arms as A

S = spec

# ---------------------------------------------------------------- forearm frame (same as arms.bracer_pt)
WRIST = A.WRIST
D2 = (WRIST - S.ELBOW).normalized()                      # down the forearm
LF = (WRIST - S.ELBOW).length
BT0, BT1 = S.CLOTH["bracer"]
LB = (BT1 - BT0) * LF                                    # nominal bracer length along the forearm (~0.202)
C0 = S.ELBOW + D2 * (BT0 * LF)                           # axis point of the nominal top edge
E1F = (V((1.0, 0.0, 0.0)) - D2 * D2.x).normalized()      # outer
E2F = D2.cross(E1F).normalized()                         # front

# ---------------------------------------------------------------- shell (12 facets, chamfered corners)
PHI_C = 30.0                # centre of the back plate: the back of the hand points outer, turned to the front
NF = 12                     # flat facets around; one facet is centred on PHI_C
BEV = 3.2                   # corner chamfer half angle (deg) -> 2 columns per corner, 24 around
NB = 2 * NF

# collar ring at the elbow end (the sleeve ends inside it)
TOP_X0, TOP_TILT, TOP_HI = 0.0045, 0.0060, 120.0    # top edge x = X0 - TILT*cos(phi - HI): highest on the
#                                                     thumb side (FBX), lowest at the back-outer side
X_RB = 0.050                # ring bottom edge
A_RI = 0.0640               # ring inner apothem (sleeve end: r <= 0.0604)
A_RO = 0.0735               # ring outer apothem
CH = 0.0020                 # chamfer size on every hard edge
GROOVE = 0.0022             # groove depth between ring / body, body / rib
# tapering body (apothem), slightly waisted like the FBX
A_B0, A_B1, X_B1 = 0.0612, 0.0535, 0.150
X_RIB0, X_RIB1, A_RIB = 0.153, 0.166, 0.0558        # narrow rib above the wrist ring
X_TUCK = 0.178              # shell ends tucked inside the wrist ring

# ---------------------------------------------------------------- wrist ring
X_W0, X_W1 = 0.1685, 0.2045
A_WO = (0.0590, 0.0580)     # outer apothem top / bottom
# the glove cuff (arms.Glove_cuff_L) is not concentric with the forearm axis: it measures r 0.041 (inner
# side) .. 0.050 (front) here, so the ring's inner wall is an ellipse ~4 mm outside it
BAND_IN_C = (0.0018, 0.0013)        # ellipse centre offset (outer, front)
BAND_IN_AB = (0.0490, 0.0535)       # ellipse semi axes (outer-inner, front-back)

# ---------------------------------------------------------------- back plate
P_XA = 0.032                # V apex (top edge at the plate centre line), just inside the collar ring
P_CHEV = 0.028              # the V's arms run this much further down the arm at the plate sides
P_XE = 0.160                # wrist end (centre) ...
P_WING = 0.012              # ... the corners reach this much further down (wing tips)
P_W = [(0.0, 54.0), (0.45, 41.0), (0.80, 48.0), (1.0, 62.0)]   # half width (deg) along the plate (hourglass)
P_R = [(0.0, 0.0925), (0.30, 0.0820), (0.70, 0.0670), (1.0, 0.0615)]  # field radius on the centre line
P_DROP = 0.0040             # field falls this much from the centre line to its edges (flat facets)
P_LIFT = 0.0065             # wing corners lift this much at the wrist end
P_HB = 0.0032               # border band height over the field
P_BW = 0.0068               # border band width
P_SINK = 0.0028             # walls sink this far under the shell surface
SP_H, SP_W, SP_TIP = 0.0036, 0.0070, 0.62   # spine: height, half width at the top, tip (fraction of field)
P_NF = 7                    # field rows


def keyf(t, keys):
    return A.keyf(t, keys, smooth=True) if hasattr(A, "keyf") else _keyf(t, keys)


def _keyf(t, keys):
    if t <= keys[0][0]:
        return keys[0][1]
    for (t0, v0), (t1, v1) in zip(keys, keys[1:]):
        if t <= t1:
            f = (t - t0) / (t1 - t0)
            f = f * f * (3 - 2 * f)
            return v0 + (v1 - v0) * f
    return keys[-1][1]


def axis_pt(x):
    return C0 + D2 * x


def radial(phi_deg):
    ph = math.radians(phi_deg)
    return E1F * math.cos(ph) + E2F * math.sin(ph)


def pt(x, phi_deg, r):
    return axis_pt(x) + radial(phi_deg) * r


def shell_cols():
    """24 column angles: every facet corner (PHI_C + 15 + 30k) split into two chamfer verts."""
    out = []
    for k in range(NF):
        c = PHI_C + 180.0 / NF + 360.0 * k / NF
        out += [c - BEV, c + BEV]
    return out


def facet_r(apothem, phi_deg):
    """Radius of the 12-gon of the given apothem at angle phi (on the facet planes)."""
    step = 360.0 / NF
    d = (phi_deg - PHI_C + step / 2) % step - step / 2
    return apothem / math.cos(math.radians(d))


def top_x(phi_deg):
    return TOP_X0 - TOP_TILT * math.cos(math.radians(phi_deg - TOP_HI))


def body_A(x):
    t = min(max((x - X_RB) / (X_B1 - X_RB), 0.0), 1.0)
    return A_B1 + (A_B0 - A_B1) * (1.0 - t) ** 1.12


def shell_A(x):
    """Outer apothem of the shell (for sinking the plate)."""
    if x < X_RB - 0.001:
        return A_RO
    if X_RIB0 < x < X_RIB1:
        return A_RIB
    if x >= X_RIB1:
        return A_WO[0] - 0.0015               # inside the wrist ring's wall (never into the glove)
    return body_A(x)


# ---------------------------------------------------------------- shell
def build_shell():
    # rows: (x(phi), apothem); x may follow the tilted top edge
    tx = top_x
    xb = X_RB
    rows = [
        (lambda p: 0.044, A_RI),                                  # inner wall (hides the sleeve end)
        (lambda p: tx(p) + CH, A_RI),
        (lambda p: tx(p), A_RI + CH),                             # flat top face
        (lambda p: tx(p), A_RO - CH),
        (lambda p: tx(p) + CH, A_RO),                             # outer top edge
        (lambda p: (tx(p) + xb) / 2, A_RO + 0.0004),
        (lambda p: xb - CH, A_RO),                                # ring lower edge
        (lambda p: xb, A_RO - CH),
        (lambda p: xb, body_A(xb) - GROOVE),                      # groove floor
        (lambda p: xb + 0.0030, body_A(xb) - GROOVE),
        (lambda p: xb + 0.0030 + CH, body_A(xb + 0.005)),          # body top edge
    ]
    nb = 4
    x0, x1 = xb + 0.0030 + CH + 0.004, X_RIB0 - 0.0045
    rows += [((lambda xx: (lambda p: xx))(x0 + (x1 - x0) * i / nb), body_A(x0 + (x1 - x0) * i / nb))
             for i in range(nb + 1)]
    rows += [
        (lambda p: X_RIB0 - 0.0015, body_A(X_RIB0) - GROOVE * 0.6),           # small groove ...
        (lambda p: X_RIB0, A_RIB - CH * 0.6),                                 # ... up onto the rib
        (lambda p: X_RIB0 + CH * 0.6, A_RIB),
        (lambda p: X_RIB1 - CH * 0.6, A_RIB),
        (lambda p: X_RIB1, A_RIB - CH * 0.6),
        (lambda p: X_RIB1 + 0.001, A_RIB - 0.0045),                           # under the wrist ring
        (lambda p: X_TUCK, A_RIB - 0.0050),
    ]
    cols = shell_cols()
    rings = [[pt(xf(ph), ph, facet_r(a, ph)) for ph in cols] for xf, a in rows]
    return lib.lofted("Cuff_L", rings, closed=True, topo="rigid", sharp_angle=12)


# ---------------------------------------------------------------- wrist ring
def build_wrist_ring():
    cols = shell_cols()
    x0, x1 = X_W0, X_W1

    def ao(x):
        return A_WO[0] + (A_WO[1] - A_WO[0]) * (x - x0) / (x1 - x0)

    def inner(x, ph, d=0.0):
        a = math.radians(ph)
        return axis_pt(x) + E1F * (BAND_IN_C[0] + (BAND_IN_AB[0] + d) * math.cos(a)) \
            + E2F * (BAND_IN_C[1] + (BAND_IN_AB[1] + d) * math.sin(a))

    prof = [   # closed profile: inner wall -> bottom face -> outer face -> top face
        ("i", x0 + CH, 0.0), ("i", x1 - CH * 0.7, 0.0), ("i", x1, CH * 0.7),
        ("o", x1, -CH), ("o", x1 - CH, 0.0), ("o", x1 - 0.0055, 0.0),
        ("o", (x0 + x1) / 2, 0.0006), ("o", x0 + 0.0055, 0.0), ("o", x0 + CH, 0.0), ("o", x0, -CH),
        ("i", x0, CH),
    ]
    rings = []
    for kind, x, d in prof:
        if kind == "i":
            rings.append([inner(x, ph, d) for ph in cols])
        else:
            rings.append([pt(x, ph, facet_r(ao(x) + d, ph)) for ph in cols])
    rings.append(rings[0])                       # close the profile -> welded by finalize
    return lib.lofted("Cuff_band_L", rings, closed=True, topo="rigid", sharp_angle=12)


# ---------------------------------------------------------------- back plate
def p_top(u):
    return P_XA + P_CHEV * abs(u)


def p_end(u):
    return P_XE + P_WING * abs(u) ** 3


def p_halfw(x):
    return keyf((x - P_XA) / (P_XE - P_XA), P_W)


def p_field(x, u):
    """Field radius at (x, u): centre-line radius minus the facet drop, plus the wing lift."""
    f = (x - P_XA) / (P_XE + P_WING - P_XA)
    r = keyf(min(max(f, 0.0), 1.0), P_R)
    au = min(abs(u), 1.0)
    r -= P_DROP * au ** 1.6
    w = min(max((x - (P_XE - 0.030)) / 0.030, 0.0), 1.0)
    r += P_LIFT * w * w * au ** 2
    return r


def build_plate():
    # rows: fraction v of the way from the top edge to the wrist end, with the border-level of the row
    tr = [(0.0, 0), (0.0006, 1), (0.0017, 2), (P_BW, 3), (P_BW + 0.0009, 4)]    # distance in from the edge
    f0 = P_BW + 0.0026

    def row_x(i, u):
        """x of plate row i (0 .. NR-1) at column u."""
        xt, xe = p_top(u), p_end(u)
        if i < 5:
            return xt + tr[i][0]
        if i >= NR - 5:
            return xe - tr[NR - 1 - i][0]
        k = i - 5                                   # field rows: 0 .. P_NF-1 (first/last = step bottoms)
        return xt + f0 + (xe - f0 - (xt + f0)) * k / (P_NF - 1)

    NR = 10 + P_NF
    row_lvl = [min(i, NR - 1 - i, 5) for i in range(NR)]

    # columns: (u, level, spine kind); u in -1..1 across the plate, border width converted per row
    def cols_at(x):
        wr = math.radians(p_halfw(x)) * p_field(x, 0.9)              # half width in metres
        bw = P_BW / wr
        one = [(1.0, 0), (1.0 - 0.0006 / wr, 1), (1.0 - 0.0017 / wr, 2), (1.0 - bw, 3),
               (1.0 - bw - 0.0009 / wr, 4), (1.0 - bw - 0.0026 / wr, 5), (0.50, 5)]
        return one

    v_tip = SP_TIP

    def spine(k):
        """(half width scale, height) of the spine at field row k."""
        v = k / (P_NF - 1)
        if v >= v_tip:
            return 0.30, 0.0
        s = v / v_tip
        return 1.0 - 0.70 * s ** 1.5, SP_H * (1.0 - s ** 2.2)

    grid = []
    for i in range(NR):
        x_mid = row_x(i, 0.0)
        side = cols_at(x_mid)
        k = i - 5
        sw, sh = spine(min(max(k, 0), P_NF - 1)) if 5 <= i < NR - 5 else (1.0, 0.0)
        if i < 5:
            sw, sh = spine(0)
        uw = SP_W / (math.radians(p_halfw(x_mid)) * p_field(x_mid, 0.0))
        spc = [(uw * sw, 5, 0.0), (uw * sw * 0.30, 5, 0.88), (0.0, 5, 1.0)]
        cl = [(-u, l, 0.0) for u, l in side] + [(-u, l, s) for u, l, s in spc] + \
             [(u, l, s) for u, l, s in reversed(spc[:-1])] + [(u, l, 0.0) for u, l in reversed(side)]
        row = []
        for u, cl_lvl, s in cl:
            x = row_x(i, u)
            lvl = min(row_lvl[i], cl_lvl)
            phi = PHI_C + u * p_halfw(x)
            if lvl == 0:
                r = facet_r(shell_A(x), phi) - P_SINK
                r = min(r, shell_A(x) - P_SINK)
            else:
                ue = max(min(abs(u), 1.0 - (P_BW + 0.0026) / (math.radians(p_halfw(x)) * 0.07)), 0.0)
                edge = p_field(x, ue) if lvl < 5 else p_field(x, u)
                if lvl in (2, 3):
                    r = edge + P_HB
                elif lvl in (1, 4):
                    r = edge + P_HB - 0.0010
                else:
                    r = edge
                    if s and 5 <= i < NR - 5 or (s and i < 5):
                        r += sh * s
            row.append(pt(x, phi, r))
        grid.append(row)
    return lib.lofted("Cuff_plate_L", grid, closed=False, topo="rigid", sharp_angle=24)


# ---------------------------------------------------------------- entry point
def build():
    left = [build_shell(), build_plate(), build_wrist_ring()]
    right = [lib.mirror_copy(o) for o in left]
    return left + right
