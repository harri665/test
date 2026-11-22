"""Coat skirt below the belt (round 2): separate overlapping cloth panels matched to docs/ref.

Objects (left side built, right mirrored):
    skirt_front_L/_R          front panels: inverted-V opening from right under the buckle, straight
                              diagonal inner edges ending in pointed tips, hem rising to the side
    skirt_back_L/_R           back panels with the narrow centre vent and pointed tips
    skirt_side_L/_R           outer side overlap (layer A): pointed at the side, over the panel edges
    skirt_side_lower_L/_R     lower stepped side layer (layer B): under the panels, longer point
    skirt_under               box-pleated under skirt (front groove = slit), fills the opening + vent
    *_trim_*                  wide border (0.03) + piping line, wrapped round every visible edge
    skirt_emblem_*            lib.emblem_ornate dagger-crosses (rigid)

Every cloth surface hangs off ONE shared surface `surf(angle, z, lift)`: bare torso + offset under
the belt, then the straight A-line of the reference silhouette down to the hem ellipse, with shared
vertical folds (lib.fold_wave), so stacked layers stay parallel and never cross.
Angles follow lib.ring_z: 0 = +X (character's left), 90 = back (+Y), 270 = front (-Y).
Panels are single cloth sheets; their visible edges get real thickness from the wrapped trim
(the hidden inner faces of a solidify are not paid for).
"""
import math

import bpy  # noqa: F401
from mathutils import Matrix, Vector as V

from .. import lib, spec

D = math.radians
C = spec.CLOTH
OFF = C["coat_offset"]
Z_TOP = 1.240                       # DETAIL_PASS v2 seam: skirt panels start here, under the belt
BELT_LO = C["belt_z"][0]
BELT_CLEAR = 0.036                  # DETAIL_PASS v2 seam: inside the belt zone nothing beyond bare + this
Z_ROW1 = BELT_LO + 0.005            # first edge loop just under the belt
Z_HEM = C["skirt_hem_z"]
Z_TIP = C["skirt_tip_z"]
HEM_RX, HEM_RY, HEM_RYB = C["skirt_hem_rx"], C["skirt_hem_ry"], C["skirt_hem_ry_back"]
SLIT_TOP, SLIT_W = C["front_slit_top_z"], C["front_slit_half_w"]
VENT_TOP, VENT_W = C["back_vent_top_z"], C["back_vent_half_w"]
UNDER_HEM = C["under_skirt_hem_z"]

# ---- read from the reference (docs/ref/*_skirt.png, measured with tools/compare.py masks) -------
SIL_TOP = (BELT_LO, 0.133)          # FRONT/BACK outline is a straight A-line from the belt bottom ...
                                    # ... to (skirt_hem_z, skirt_hem_rx)
SEAM_X = 0.0035                     # front panels meet edge to edge above the slit (under the buckle)
BACK_TIP_Z = Z_TIP - 0.040          # back view: back tips hang ~4 cm lower than the front tips
FRONT_CORNER = (0.250, 0.510)       # front view: front panel outer corner (x, z) ...
FRONT_OUT_SLOPE, FRONT_OUT_MIN = 0.30, 0.070   # ... its outer edge climbs inward (x 0.11 at z 0.9)
BACK_OUT_X, BACK_CORNER_Z = 0.175, 0.410      # back view: back panels reach x 0.175, corner z 0.41
A_OVERLAP_F, A_BACK_X = 0.020, 0.150          # layer A laps 2.5 cm over the front panel; back edge x 0.15
SIDE_A = dict(tip=14.0, tip_z=0.420, front_z=0.600, back_z=0.580, top=Z_TOP)   # pointed at the side
SIDE_B = dict(front=322.0, back=58.0, tip=18.0, tip_z=0.335, front_z=0.470, back_z=0.460, top=0.720)
A_LIFT = (0.004, 0.012, 0.012)      # layer A over the panels: in the belt zone / below / extra flare
                                    # (front view: the side flap stands off the body below z 0.75)
B_LIFT = -0.012                     # layer B sits under the front / back panels
FOLD_AMP, FOLD_COUNT, FOLD_SEED = 0.008, 8.0, 0.37
# trim: wide border + piping (profile across the edge, s = inward from the cloth edge, h = off the cloth)
T_OVER, T_RISE, T_UNDER = 0.003, 0.003, 0.006
T_PROFILE = [(-T_OVER, -T_UNDER), (-T_OVER, T_RISE), (0.030, T_RISE), (0.034, 0.0009),
             (0.039, 0.0020), (0.044, 0.0008), (0.044, -0.0020), (0.014, -T_UNDER)]
# emblems: (x, z, half width, tilt deg) -- front ones lean toward the panel tip like the reference
EMB_FRONT = (0.168, 0.590, 0.045, -28.0)
EMB_BACK = (0.092, 0.500, 0.042, 0.0)
EMB_VENT = (0.0, VENT_TOP - 0.075, 0.028, 0.0)
# under skirt (box pleats)
U_OFF, U_GAP_X, U_GAP_Y = 0.018, 0.040, 0.028
U_UNIT_F, U_N_F = 10.0, 5            # pleat width (deg) and count from the front groove outward
U_UNIT_B, U_N_B = 11.0, 3            # pleats around the back vent (a face centred on the vent)
U_N_PLAIN = 5
U_GAP, U_DEPTH = 0.26, 0.016         # groove share of a pleat and groove depth at the hem
U_SLIT_X = 0.003

NR_FRONT, NC_FRONT = 27, 7
NR_BACK, NC_BACK = 28, 7
NR_A, NC_A_FRONT, NC_A_BACK = 25, 6, 8
NR_B, NC_B_FRONT, NC_B_BACK = 12, 8, 6
NR_UNDER = 14


def _sstep(e0, e1, x):
    t = min(1.0, max(0.0, (x - e0) / (e1 - e0)))
    return t * t * (3 - 2 * t)


def _smax(a, b, k=0.004):
    return 0.5 * (a + b + math.sqrt((a - b) ** 2 + k * k)) - 0.5 * k


def _angdist(a, b):
    return abs((a - b + math.pi) % math.tau - math.pi)


# ------------------------------------------------------------------ the shared surface
def _bare(z):
    return spec.torso_section(max(z, spec.TORSO[0][0]))


_TOP_B = _bare(Z_TOP)


def _a_lift(z, flare=True):
    return (A_LIFT[0] + (A_LIFT[1] - A_LIFT[0]) * _sstep(BELT_LO + 0.005, BELT_LO - 0.055, z)
            + (A_LIFT[2] * _sstep(1.0, 0.72, z) if flare else 0.0))


def _sil_rx(z):
    z0, r0 = SIL_TOP
    return r0 + (HEM_RX - r0) * (z0 - z) / (z0 - Z_HEM)


def section(z, kind="coat"):
    """(cy, rx, ry_front, ry_back) of the base coat surface (or the under skirt) at height z."""
    cy, bx, by = _bare(z)
    ze = max(z, Z_TOP - 1.06 * (Z_TOP - Z_HEM))          # stop flaring a little below the hem
    t = (Z_TOP - ze) / (Z_TOP - Z_HEM)
    off = OFF - 0.002 * _sstep(1.17, 1.08, z)            # 2 mm tighter over the hips (reference outline)
    tx, ty = _TOP_B[1] + OFF, _TOP_B[2] + OFF
    rx = _smax(bx + off, max(tx, _sil_rx(ze) - _a_lift(ze, False)))
    ryf = _smax(by + off, ty + (HEM_RY - ty) * t)
    ryb = _smax(by + off, ty + (HEM_RYB - ty) * t)
    if kind == "under":
        rx = _smax(bx + U_OFF, rx - U_GAP_X)
        ryf = _smax(by + U_OFF, ryf - U_GAP_Y)
        ryb = _smax(by + U_OFF, ryb - U_GAP_Y)
    return cy, rx, ryf, ryb


def _emblem_centres():
    out = []
    for x, z, size, _ in (EMB_FRONT, EMB_BACK, EMB_VENT):
        front = x == EMB_FRONT[0]
        out.append((_angle_for_x(x, z, front), z, size))
    return out


_EMB_AZ = None


def fold(a, z):
    """Shared vertical folds: none under the belt, deepening toward the hem, symmetric L/R,
    ironed flat under the rigid emblems."""
    global _EMB_AZ
    v = (Z_ROW1 - z) / (Z_ROW1 - 0.30)
    if v <= 0.0:
        return 0.0
    v = min(v, 1.0)
    u = ((a - D(270.0)) % math.tau) / math.tau
    if u > 0.5:
        u = 1.0 - u
    f = lib.fold_wave(u, v, amp=FOLD_AMP, count=FOLD_COUNT, seed=FOLD_SEED, grow=1.0)
    if _EMB_AZ is None:
        _EMB_AZ = []                    # guard against recursion while the centres are solved
        _EMB_AZ = _emblem_centres()
    for ae, ze, size in _EMB_AZ:
        r = section(ze)[1]
        d = math.hypot(_angdist(a, ae) * r, (z - ze) * 0.8)
        f *= _sstep(size * 1.6, size * 3.0, d)
    return f


def surf(a, z, lift=0.0, kind="coat", extra=0.0):
    """Point on the skirt.  a: angle (rad); lift: layer offset along the horizontal normal."""
    cy, rx, ryf, ryb = section(z, kind)
    c, s = math.cos(a), math.sin(a)
    ry = ryb if s > 0 else ryf
    nx, ny = ry * c, rx * s
    nl = math.hypot(nx, ny)
    d = lift + extra + (fold(a, z) if kind == "coat" and _EMB_AZ != [] else 0.0)
    return V((rx * c + nx / nl * d, cy + ry * s + ny / nl * d, z))


def _angle_for_x(x, z, front, lift=0.0):
    """Angle on the front (or back) LEFT half whose surface point has this x."""
    rx = section(z)[1]
    q = math.acos(max(-1.0, min(1.0, x / rx)))
    a = math.tau - q if front else q
    for _ in range(4):
        err = x - surf(a, z, lift).x
        a += err / (-rx * math.sin(a))
    return a


# ------------------------------------------------------------------ panels
class Panel:
    """A cloth panel = grid of (angle, z) parameters on the shared surface at a layer lift."""

    def __init__(self, edge0, edge1, hem, us, nr, lift, z_top=Z_TOP, z1=Z_ROW1):
        self.edge0, self.edge1, self.hem, self.us, self.nr = edge0, edge1, hem, us, nr
        self.lift, self.z_top, self.z1 = lift, z_top, z1
        self.Q = []
        for k in range(nr + 1):
            row = []
            for u in us:
                zh = hem(u)
                if z1 is None:
                    z = z_top + (zh - z_top) * k / nr
                elif k == 0:
                    z = z_top
                else:
                    z = z1 + (zh - z1) * (k - 1) / (nr - 1)
                row.append((edge0(z) + (edge1(z) - edge0(z)) * u, z))
            self.Q.append(row)

    def P(self, a, z):
        return surf(a, z, self.lift(z) if callable(self.lift) else self.lift)

    def frame(self, a, z):
        """Point, d/da, d/dz and outward unit normal."""
        e = 1e-4
        p = self.P(a, z)
        pa = (self.P(a + e, z) - self.P(a - e, z)) / (2 * e)
        pz = (self.P(a, z + e) - self.P(a, z - e)) / (2 * e)
        n = pa.cross(pz).normalized()
        if n.x * p.x + n.y * p.y < 0:
            n = -n
        return p, pa, pz, n

    def points(self):
        return [[self.P(a, z) for a, z in row] for row in self.Q]


def _sheet(name, panel):
    ob = lib.lofted(name, panel.points(), closed=False, topo="main")
    me = ob.data
    if sum(p.normal.x * p.center.x + p.normal.y * p.center.y for p in me.polygons) < 0:
        me.flip_normals()
        me.update()
    return ob


def _trim(name, panel, path):
    """Wide border + piping wrapped round the panel edge along `path` (grid indices (k, j)).
    Inward offsets are solved in the panel's (angle, z) chart so the band hugs the cloth; corners
    are mitred."""
    Q = [panel.Q[k][j] for k, j in path]
    F = [panel.frame(a, z) for a, z in Q]
    # which side is inside: compare with an interior neighbour of a middle vertex
    km, jm = path[len(path) // 2]
    nb = (min(max(km, 1), panel.nr - 1), min(max(jm, 1), len(panel.us) - 2))
    inside = panel.P(*panel.Q[nb[0]][nb[1]]) - F[len(path) // 2][0]
    rings = []
    sign = None
    for i, (p, pa, pz, n) in enumerate(F):
        ws = []
        for j0, j1 in ((i - 1, i), (i, i + 1)):
            if 0 <= j0 and j1 < len(F):
                t = (F[j1][0] - F[j0][0]).normalized()
                ws.append(n.cross(t).normalized())
        if sign is None:
            mid = F[len(path) // 2]
            t = (F[min(len(path) // 2 + 1, len(F) - 1)][0] - F[max(len(path) // 2 - 1, 0)][0]).normalized()
            sign = 1.0 if mid[3].cross(t).dot(inside) > 0 else -1.0
        w = (ws[0] + ws[-1]).normalized() * sign
        cosh = max(abs(w.dot(ws[0])), 0.4)
        w = w / cosh
        # 3D direction -> (da, dz) per metre (least squares on the tangent plane)
        g11, g12, g22 = pa.dot(pa), pa.dot(pz), pz.dot(pz)
        b1, b2 = pa.dot(w), pz.dot(w)
        det = g11 * g22 - g12 * g12
        da, dz = (g22 * b1 - g12 * b2) / det, (g11 * b2 - g12 * b1) / det
        a0, z0 = Q[i]
        ring = []
        for s, h in T_PROFILE:
            q, _, _, nq = panel.frame(a0 + da * s, z0 + dz * s)
            ring.append(q + nq * h)
        rings.append(ring)
    return lib.lofted(name, rings, closed=True, cap_start=True, cap_end=True, topo="main")


def _cols(n_before, u_tip, n_after):
    us = [u_tip * i / n_before for i in range(n_before + 1)]
    return us + [u_tip + (1.0 - u_tip) * i / n_after for i in range(1, n_after + 1)]


def _front_edge_x(z):
    return _smax(SEAM_X, SEAM_X + (SLIT_TOP - z) * (SLIT_W - SEAM_X) / (SLIT_TOP - Z_TIP), 0.004)


def _back_edge_x(z):
    return 0.0015 + max(0.0, (VENT_TOP - z) * (VENT_W - 0.0015) / (VENT_TOP - Z_HEM))


def _front_out_x(z):
    return _smax(FRONT_OUT_MIN, FRONT_CORNER[0] + FRONT_OUT_SLOPE * (FRONT_CORNER[1] - z), 0.01)


def _back_out_x(z, x=BACK_OUT_X):
    return min(x, 0.8 * section(z)[1])


def front_panel():
    return Panel(lambda z: _angle_for_x(_front_edge_x(z), z, True),
                 lambda z: _angle_for_x(_front_out_x(z), z, True),
                 lambda u: Z_TIP + (FRONT_CORNER[1] - Z_TIP) * u,
                 [i / NC_FRONT for i in range(NC_FRONT + 1)], NR_FRONT, 0.0)


def back_panel():
    return Panel(lambda z: _angle_for_x(_back_edge_x(z), z, False),
                 lambda z: _angle_for_x(_back_out_x(z), z, False),
                 lambda u: BACK_TIP_Z + (BACK_CORNER_Z - BACK_TIP_Z) * u,
                 [i / NC_BACK for i in range(NC_BACK + 1)], NR_BACK, 0.0)


def _side_hem(L, e0, e1):
    """V hem with the tip at angle L['tip'] -> (u_tip, hem(u))."""
    zt = L["tip_z"]
    ut = (D(L["tip"]) - e0(zt)) / (e1(zt) - e0(zt))

    def hem(u):
        if u <= ut:
            return L["front_z"] + (L["tip_z"] - L["front_z"]) * u / ut
        return L["tip_z"] + (L["back_z"] - L["tip_z"]) * (u - ut) / (1.0 - ut)
    return ut, hem


def _a_edges():
    e0 = lambda z: _angle_for_x(_front_out_x(z) - A_OVERLAP_F, z, True, _a_lift(z)) - math.tau  # noqa: E731
    e1 = lambda z: _angle_for_x(_back_out_x(z, A_BACK_X), z, False, _a_lift(z))  # noqa: E731
    return e0, e1


def side_a():
    e0, e1 = _a_edges()
    ut, hem = _side_hem(SIDE_A, e0, e1)
    return Panel(e0, e1, hem, _cols(NC_A_FRONT, ut, NC_A_BACK), NR_A, _a_lift)


def side_b():
    e0, e1 = (lambda z: D(SIDE_B["front"] - 360.0)), (lambda z: D(SIDE_B["back"]))
    ut, hem = _side_hem(SIDE_B, e0, e1)
    return Panel(e0, e1, hem,
                 _cols(NC_B_FRONT, ut, NC_B_BACK), NR_B, B_LIFT, z_top=SIDE_B["top"], z1=None)


def _rows_up_to(panel, j, z_max):
    """Last row index (from the hem upward) whose z at column j is still <= z_max."""
    k = panel.nr
    while k > 1 and panel.Q[k - 1][j][1] <= z_max:
        k -= 1
    return k


def _path_main(panel, k_outer):
    """Inner edge down (every row), hem out, outer edge up to row k_outer."""
    nr, nc = panel.nr, len(panel.us) - 1
    return ([(k, 0) for k in range(nr + 1)] + [(nr, j) for j in range(1, nc + 1)]
            + [(k, nc) for k in range(nr - 1, k_outer - 1, -1)])


def _path_side(panel, step, k0_front=0, k0_back=0):
    nr, nc = panel.nr, len(panel.us) - 1
    down = list(range(k0_front, nr, step)) + [nr]
    up = [k for k in range(nr - 1, k0_back - 1, -1) if (nr - k) % step == 0 or k == k0_back]
    return [(k, 0) for k in down] + [(nr, j) for j in range(1, nc + 1)] + [(k, nc) for k in up]


# ------------------------------------------------------------------ under skirt (box pleats)
def _under_half():
    """Left half (front groove/slit -> back centre), rows of points.  Faces stay on the under
    surface, grooves sink inward; pleats are closed under the belt and open toward the hem."""
    zs = [Z_TOP, Z_ROW1] + [Z_ROW1 + (UNDER_HEM - Z_ROW1) * i / (NR_UNDER - 1) for i in range(1, NR_UNDER)]
    rows = []
    for z in zs:
        v = _sstep(0.0, 1.0, (Z_ROW1 - z) / (Z_ROW1 - UNDER_HEM))
        g = 0.10 + (U_GAP - 0.10) * v          # pleats stitched nearly closed under the belt
        dep = U_DEPTH * v
        rx = section(z, "under")[1]
        a_slit = math.tau - math.acos(U_SLIT_X / rx)
        cols = [(a_slit, -dep)]
        a = D(270.0)
        for i in range(U_N_F):
            a_s = a + D(U_UNIT_F * (i + g / 2))
            if i == 0:
                a_s = max(a_s, a_slit + D(1.0))
            cols += [(a_s, 0.0), (a + D(U_UNIT_F * (i + 1 - g / 2)), 0.0),
                     (a + D(U_UNIT_F * (i + 1)), -dep)]
        a_end_f = a + D(U_UNIT_F * U_N_F)
        a_b0 = D(450.0 - U_UNIT_B * (U_N_B - 0.5))
        for i in range(1, U_N_PLAIN + 1):
            cols.append((a_end_f + (a_b0 - a_end_f) * i / (U_N_PLAIN + 1), 0.0))
        for m in range(U_N_B):
            gb = a_b0 + D(U_UNIT_B * m)
            cols += [(gb, -dep), (gb + D(U_UNIT_B * g / 2), 0.0)]
            if m < U_N_B - 1:
                cols.append((gb + D(U_UNIT_B * (1 - g / 2)), 0.0))
        cols.append((D(450.0), 0.0))
        rows.append([surf(a, z, 0.0, "under", e) for a, e in cols])
    # folded hem: one loop turned up inside
    rows.append([p + V((0, 0, 0.022)) - V((p.x, p.y - section(UNDER_HEM)[0], 0)).normalized() * 0.004
                 for p in rows[-1]])
    return rows


# ------------------------------------------------------------------ emblems
def _emblem(name, panel, x, z, size, tilt, front, crescents=True, clear=-0.0012):
    a = _angle_for_x(x, z, front, panel.lift(z) if callable(panel.lift) else panel.lift)
    p0, pa, pz, n = panel.frame(a, z)
    up = (pz - n * pz.dot(n)).normalized()
    right = up.cross(n).normalized()
    if not front:
        tilt = -tilt
    up = (Matrix.Rotation(D(tilt), 3, n) @ up).normalized()
    right = up.cross(n).normalized()
    # sink so the centre touches and the tips stay close on the curved cloth
    h = -1.0
    for dx, dy in ((size, 0), (-size, 0), (0, 1.3 * size), (0, -2.0 * size), (0.6 * size, -0.6 * size),
                   (-0.6 * size, -0.6 * size), (0.3 * size, 0.3 * size)):
        q3 = p0 + right * dx + up * dy
        # project onto the surface along n: param step via the tangent plane
        g11, g12, g22 = pa.dot(pa), pa.dot(pz), pz.dot(pz)
        d = q3 - p0
        b1, b2 = pa.dot(d), pz.dot(d)
        det = g11 * g22 - g12 * g12
        da, dz = (g22 * b1 - g12 * b2) / det, (g11 * b2 - g12 * b1) / det
        q = panel.P(a + da, z + dz)
        h = max(h, (q - p0).dot(n))
    ob = lib.emblem_ornate(name, size=size, thickness=0.004, loc=p0 + n * (h + clear), normal=n, up=up,
                           crescents=crescents)
    for o in [o for o in bpy.data.objects if o.name == name and o is not ob]:
        o.name = name + "_tmp"
    ob.name = ob.data.name = name
    return ob


# ------------------------------------------------------------------ queries for other authors
def _in_layer_a(a, z):
    """Is (angle rad, z) covered by the outer side layer A?"""
    e0, e1 = _a_edges()
    a = (a + math.pi) % math.tau - math.pi
    lo, hi = e0(z), e1(z)
    if not lo <= a <= hi or z > Z_TOP:
        return False
    ut, hem = _side_hem(SIDE_A, e0, e1)
    return z >= hem((a - lo) / (hi - lo))


def outer_radius(a_deg, z):
    """Horizontal distance from the Z axis to the outermost skirt cloth at (angle, z), LEFT side
    (mirror the angle for the right).  Includes layer A and the trim rise.  Gear hangs on this."""
    a = D(a_deg)
    lift = _a_lift(z) if _in_layer_a(a, z) else 0.0
    p = surf(a, z, lift + T_RISE)
    return math.hypot(p.x, p.y)


# ------------------------------------------------------------------ build
def _remove(ob):
    me = ob.data
    bpy.data.objects.remove(ob)
    bpy.data.meshes.remove(me)


def build():
    global _EMB_AZ
    _EMB_AZ = None
    objs = []

    fp = front_panel()
    k_out = _rows_up_to(fp, len(fp.us) - 1, SIDE_A["front_z"] + 0.06)
    f_l = _sheet("skirt_front_L", fp)
    ft_l = _trim("skirt_front_trim_L", fp, _path_main(fp, k_out))
    ef = _emblem("skirt_emblem_front_L", fp, *EMB_FRONT, front=True)

    bp = back_panel()
    k_out = _rows_up_to(bp, len(bp.us) - 1, SIDE_A["back_z"] + 0.06)
    b_l = _sheet("skirt_back_L", bp)
    path = _path_main(bp, k_out)
    k_vent = next(k for k in range(bp.nr + 1) if bp.Q[k][0][1] < VENT_TOP + 0.04)
    path = [p for p in path if not (p[1] == 0 and 0 < p[0] < k_vent and p[0] % 2)]   # straight part: every 2nd
    bt_l = _trim("skirt_back_trim_L", bp, path)
    eb = _emblem("skirt_emblem_back_L", bp, *EMB_BACK, front=False)
    ev = _emblem("skirt_emblem_vent", bp, *EMB_VENT, front=False, crescents=False, clear=T_RISE + 0.001)

    pa = side_a()
    a_l = _sheet("skirt_side_L", pa)
    at_l = _trim("skirt_side_trim_L", pa, _path_side(pa, 2))

    pb = side_b()
    nr = pb.nr
    b2_l = _sheet("skirt_side_lower_L", pb)
    b2t_l = _trim("skirt_side_lower_trim_L", pb, _path_side(pb, 1, k0_front=nr - 2, k0_back=nr - 2))

    for o in (f_l, ft_l, ef, b_l, bt_l, eb, a_l, at_l, b2_l, b2t_l):
        objs += [o, lib.mirror_copy(o, o.name[:-2] + "_R")]
    objs.append(ev)

    half = lib.lofted("skirt_under_L", _under_half(), closed=False, topo="main")
    other = lib.mirror_copy(half, "skirt_under_R")
    under = lib.join([half, other], "skirt_under", topo="main")
    lib.finalize(under)
    objs.append(under)
    return objs
