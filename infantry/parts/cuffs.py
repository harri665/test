"""Forearm turn-back cuffs (round 4; moved out of arms.py).

Everything is built for the LEFT (+X) forearm and mirrored with lib.mirror_copy.  The cuff sits on
exactly the forearm arms.py builds: the frame below is derived from arms.WRIST / spec.ELBOW /
spec.CLOTH["bracer"] the same way arms.bracer_pt does it.

Objects per arm
    Cuff_L          main   flared turn-back cloth cuff: rolled top lip + hem band (top edge rides higher
                           on the outer side), soft vertical folds, stitch line under the hem; its
                           lower end tucks under the wrist band.  Inner wall only as deep as it can be seen.
    Cuff_band_L     main   thick rolled band (~2.3 cm) at the wrist end, standing proud of the cuff body,
                           edge loops top and bottom, crisp seam where it meets the body, stitch line above it
    Cuff_rivets_L   rigid  2 x 2 large decorated buttons (raised rim + embossed 8-petal rosette)

Cuff coordinates: x = distance along the forearm from the (unsheared) top edge toward the wrist,
phi = angle round the forearm, 0 = outer side (+X), 90 deg = front (-Y), 270 = back.
"""
import math

import bpy  # noqa: F401
from mathutils import Matrix, Vector as V

from infantry import lib, spec
from infantry.parts import arms as A

S = spec
TAU = math.tau

# ---------------------------------------------------------------- forearm frame (same as arms.bracer_pt)
WRIST = A.WRIST
D2 = (WRIST - S.ELBOW).normalized()                      # down the forearm
LF = (WRIST - S.ELBOW).length
BT0, BT1 = S.CLOTH["bracer"]
LB = (BT1 - BT0) * LF                                    # cuff length along the forearm (~0.202)
C0 = S.ELBOW + D2 * (BT0 * LF)                           # axis point of the top edge
E1F = (V((1.0, 0.0, 0.0)) - D2 * D2.x).normalized()      # outer
E2F = D2.cross(E1F).normalized()                         # front

# ---------------------------------------------------------------- shape (read from the reference)
NB = 24                     # verts around (~1.5-2 cm columns like the reference wireframe)
R_TOP, R_LOW = 0.0790, 0.0585   # body radius at the top edge / just above the wrist band
FB_FLARE = 0.06             # extra front/back opening at the top edge
FLARE = 1.35                # >1: the flare opens faster toward the top (slightly bell-shaped)
TH = 0.0050                 # double cloth layer
LIP = 0.0032                # rolled top lip + hem band stand this much proud of the body
HEM_W = 0.0125              # hem band width (top edge -> small step onto the body)
TOP_LIFT = 0.030            # top edge rides up the arm by up to this much ...
TOP_LIFT_PHI = 285.0        # ... centred on the outer-back side -> outer side clearly higher
TOP_DROP = 0.0060           # whole top edge moved this much down the arm (keeps top at z ~1.21)
INNER_DEPTH = 0.032         # inner wall depth below the top edge (sleeve hides the rest)

BAND_H = 0.0225             # wrist band height along the arm
BAND_DROP = 0.0025          # band's rolled lower edge reaches this far past LB
BAND_PROUD = 0.0055         # band outer surface above the body
BAND_IN = 0.0540            # nominal band inner radius (profile reference)
# the glove cuff (arms.Glove_cuff_L) is not concentric with the forearm axis: it measures r 0.041 (inner
# side) .. 0.049 (front) here, so the band's inner wall is an ellipse ~4.5 mm outside it -> no dark gap
BAND_IN_C = (0.0018, 0.0013)        # ellipse centre offset (outer, front)
BAND_IN_AB = (0.0482, 0.0528)       # ellipse semi axes (outer-inner, front-back)

RIVET_R, RIVET_H = 0.0140, 0.0062
RIVET_PHI = (50.0, -6.0)    # columns: front-outer and outer (side crop: both visible from the side)
RIVET_X = (0.048, 0.146)    # rows: under the hem / just above the wrist band
RIVET_N = 16

STITCH_W, STITCH_T = 0.0022, 0.0008

# soft vertical folds (s = arc length round the cuff at radius RS, l = x), kept clear of the rivets
RS = 0.070
PERIOD = TAU * RS


def _s(phi_deg):
    return math.radians(phi_deg % 360.0) * RS


FOLDS = [   # crests sit on / near column lines (every 15 deg) so they do not alias on the 24-column grid
    dict(pts=[(_s(24), 0.026), (_s(21), 0.090), (_s(25), 0.158)], amp=0.0030, width=0.016, sharp=0.6),
    dict(pts=[(_s(102), 0.016), (_s(107), 0.095), (_s(101), 0.170)], amp=0.0042, width=0.018, sharp=0.75),
    dict(pts=[(_s(152), 0.016), (_s(147), 0.110), (_s(151), 0.168)], amp=0.0038, width=0.018, sharp=0.75),
    dict(pts=[(_s(208), 0.018), (_s(213), 0.100), (_s(207), 0.168)], amp=0.0038, width=0.018, sharp=0.75),
    dict(pts=[(_s(258), 0.016), (_s(253), 0.090), (_s(259), 0.168)], amp=0.0042, width=0.018, sharp=0.75),
    dict(pts=[(_s(316), 0.016), (_s(313), 0.075), (_s(316), 0.135)], amp=0.0030, width=0.016, sharp=0.6),
    dict(pts=[(_s(127), 0.026), (_s(129), 0.095)], amp=-0.0025, width=0.015, sharp=0.5),     # creases
    dict(pts=[(_s(232), 0.026), (_s(230), 0.105)], amp=-0.0025, width=0.015, sharp=0.5),
    dict(pts=[(_s(75), 0.024), (_s(77), 0.080)], amp=-0.0020, width=0.014, sharp=0.5),
]


# ---------------------------------------------------------------- surface
def body_R(x, phi_deg=90.0):
    """Body radius (no folds) at distance x down the forearm; the top also opens a little more
    front/back than sideways (side reference: the cuff mouth is wide seen from the side)."""
    lo0 = LB - BAND_H
    fb = 1.0 + FB_FLARE * math.sin(math.radians(phi_deg)) ** 2 * max(0.0, 1.0 - x / (0.75 * lo0)) ** 1.5
    return fb * _body_R(x)


def _body_R(x):
    lo = LB - BAND_H
    t = min(max(x / lo, -0.25), 1.0)
    if t >= 0:
        return R_LOW + (R_TOP - R_LOW) * (1.0 - t) ** FLARE
    return R_TOP + (R_TOP - R_LOW) * FLARE / lo * (-t * lo) * 0.6        # gentle continuation above


def lift(phi_deg):
    """Along-arm shift of the top edge (negative = higher up the arm)."""
    return TOP_DROP - TOP_LIFT * 0.5 * (1.0 + math.cos(math.radians(phi_deg - TOP_LIFT_PHI)))


def shear(x, phi_deg):
    """Top-edge lift, full on the lip/hem, fading out down the body."""
    f = 1.0 - min(max((x - HEM_W) / 0.10, 0.0), 1.0)
    f = f * f * (3 - 2 * f)
    return lift(phi_deg) * f


def fold(x, phi_deg):
    return lib.fold_ridges(_s(phi_deg), x, FOLDS, period=PERIOD)


def axis_pt(x):
    return C0 + D2 * x


def radial(phi_deg):
    ph = math.radians(phi_deg)
    return E1F * math.cos(ph) + E2F * math.sin(ph)


def body_pt(x, phi_deg, off=0.0, folds=True):
    """Point on the cuff body; x is the unsheared parameter, off is radial offset."""
    xs = x + shear(x, phi_deg)
    r = body_R(xs, phi_deg) + off + (fold(x, phi_deg) if folds else 0.0)
    return axis_pt(xs) + radial(phi_deg) * r


def body_normal(x, phi_deg, off=0.0):
    e = 0.002
    p = body_pt(x, phi_deg, off)
    tx = body_pt(x + e, phi_deg, off) - body_pt(x - e, phi_deg, off)
    tp = body_pt(x, phi_deg + 1.5, off) - body_pt(x, phi_deg - 1.5, off)
    n = tp.cross(tx).normalized()
    return p, (n if n.dot(radial(phi_deg)) > 0 else -n)


def phis(n=NB, phase=0.0):
    return [phase + 360.0 * j / n for j in range(n)]


# ---------------------------------------------------------------- cuff body
STITCH_TOP_X = 0.0075                     # on the hem band
STITCH_LOW_X = LB - BAND_H - 0.0065       # just above the wrist band


def build_body():
    # profile (x, radial offset, use folds): inner wall -> rolled lip over the top -> hem band ->
    # step onto the body -> body rows -> tucked under the wrist band
    a = TH + LIP
    lip = [(0.0035 - 0.0055 * math.sin(math.radians(t)), -TH + a * (1 - math.cos(math.radians(t))) / 2)
           for t in (0, 60, 120, 180)]
    prof = [(INNER_DEPTH, -TH)] + lip
    prof += [(HEM_W - 0.0012, LIP * 0.85), (HEM_W, 0.0006)]
    lo = STITCH_LOW_X
    x0 = HEM_W + 0.006
    nrow = 8
    prof += [(x0 + (lo - x0) * i / nrow, 0.0) for i in range(nrow + 1)]
    prof += [(LB - 0.006, -0.0035)]                        # tucked under the wrist band
    rings = []
    for x, off in prof:
        rings.append([body_pt(x, ph, off, folds=x > HEM_W) for ph in phis()])
    ob = lib.lofted("cuff_body_tmp", rings, closed=True, topo="main", sharp_angle=66)
    return ob


# ---------------------------------------------------------------- wrist band
def build_band():
    top = LB - BAND_H
    bot = LB + BAND_DROP
    rb = body_R(top) + BAND_PROUD                 # outer radius at the top of the band
    rw = body_R(LB) + BAND_PROUD - 0.0005         # a touch narrower at the wrist end

    def rout(x):
        return rb + (rw - rb) * (x - top) / BAND_H

    # (x, radius) profile: inner wall -> rolled lower edge -> outer face with edge loops -> rounded top
    # edge -> seam just under the body surface
    prof = [
        lambda: (LB - 0.010, BAND_IN),                                  # inner wall (glove cuff covers it)
        lambda: (bot - 0.0009, BAND_IN),
        lambda: (bot, BAND_IN + 0.0024),                                # flat underside
        lambda: (bot, rout(bot) - 0.0026),
        lambda: (bot - 0.0026, rout(bot - 0.0026)),                     # lower edge loop (45 deg bevel)
        lambda: (LB - 0.0105, rout(LB - 0.0105) + 0.0005),              # outer face, very slightly full
        lambda: (top + 0.0024, rout(top + 0.0024)),                     # upper edge loop
        lambda: (top, rout(top) - 0.0024),                              # top face
        lambda: (top - 0.0003, None),                                   # crisp seam into the body
    ]
    rings = []
    for k, f in enumerate(prof):
        x, r = f()
        ring = []
        for ph in phis():
            if k < 3:        # inner wall + inner edge of the underside: ellipse hugging the glove cuff
                d = r - BAND_IN
                a = math.radians(ph)
                ring.append(axis_pt(x) + E1F * (BAND_IN_C[0] + (BAND_IN_AB[0] + d) * math.cos(a))
                            + E2F * (BAND_IN_C[1] + (BAND_IN_AB[1] + d) * math.sin(a)))
                continue
            rr = r if r is not None else body_R(x) - 0.0016
            ring.append(axis_pt(x) + radial(ph) * rr)
        rings.append(ring)
    return lib.lofted("cuff_band_tmp", rings, closed=True, topo="main", sharp_angle=55)


# ---------------------------------------------------------------- stitches
def build_stitches():
    out = []
    for x, off, nm in ((STITCH_TOP_X, LIP * 0.92, "top"), (STITCH_LOW_X, 0.0, "low")):
        pts, nrm = [], []
        for ph in phis():
            p, n = body_normal(x, ph, off)
            if x < HEM_W:
                p = body_pt(x, ph, off, folds=False)
            pts.append(p)
            nrm.append(n)
        out.append(lib.strip_along(f"cuff_stitch_{nm}", pts, nrm, width=STITCH_W, thickness=STITCH_T,
                                   sink=0.0005, closed=True, topo="main"))
    return out


# ---------------------------------------------------------------- decorated rivets
def rivet_deco(name, loc, normal, up):
    """Large button: bevelled raised rim, a groove, an embossed 8-petal rosette on a domed boss.
    Built on local Z then aligned; all quads (Coons cap), base sunk into the cloth."""
    n = RIVET_N
    R, H = RIVET_R, RIVET_H
    prof = [  # (radius factor, height factor, petal amount)
        (1.00, -0.12, 0.0),
        (0.97, 0.52, 0.0),
        (0.84, 0.72, 0.0),       # rim top
        (0.70, 0.42, 0.0),       # groove inside the rim
        (0.52, 0.80, 0.21),      # rosette petals (radius + height modulated)
    ]
    rings = []
    for rf, hf, pet in prof:
        ring = []
        for j in range(n):
            a = TAU * j / n
            m = math.cos(8 * a)                  # n=16 -> alternating petal tip / notch
            rr = R * rf * (1.0 + pet * m)
            zz = H * hf + (H * 0.10 * m if pet else 0.0)
            ring.append(V((rr * math.cos(a), rr * math.sin(a), zz)))
        rings.append(ring)
    vs, fs = lib.loft(rings, closed=True, cap_end=True)
    # dome the rosette centre (cap interior verts)
    k0 = len(rings) * n
    for i in range(k0, len(vs)):
        d = math.hypot(vs[i].x, vs[i].y) / (R * 0.52)
        vs[i] = V((vs[i].x, vs[i].y, H * (0.80 + 0.22 * max(0.0, 1.0 - d * d))))
    ob = lib.mesh_object(name, vs, fs, topo="rigid", sharp_angle=40)
    nz = V(normal).normalized()
    u = V(up)
    u = (u - nz * u.dot(nz)).normalized()
    xax = u.cross(nz).normalized()
    ob.data.transform(Matrix.Translation(V(loc)) @ Matrix((xax, u, nz)).transposed().to_4x4())
    ob.data.update()
    return lib.finalize(ob, sharp_angle=40)


def build_rivets():
    objs = []
    for i, x in enumerate(RIVET_X):
        for j, ph in enumerate(RIVET_PHI):
            p, n = body_normal(x, ph)
            objs.append(rivet_deco(f"cuff_rivet_{i}{j}", p - n * 0.0004, n, -D2))
    return lib.join(objs, "Cuff_rivets_L", topo="rigid")


# ---------------------------------------------------------------- entry point
def build():
    body = build_body()
    stitches = build_stitches()
    band = build_band()
    # stitch lines are separate thin shells grouped into the objects they sit on (not welded)
    body = lib.join([body, stitches[0]], "Cuff_L", topo="main")
    band = lib.join([band, stitches[1]], "Cuff_band_L", topo="main")
    rivets = build_rivets()
    left = [lib.finalize(body, sharp_angle=66), lib.finalize(band, sharp_angle=55), rivets]
    right = [lib.mirror_copy(o) for o in left]
    return left + right
