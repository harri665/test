"""Coat skirt below the belt: separate overlapping cloth panels.

Objects (all quads):
    skirt_front_L / _R    front panels, closed + lapped (left over right) under the belt, inverted-V
                          opening from CLOTH["front_slit_top_z"], hem pointed at the inner corner
    skirt_back_L / _R     back panels, centre vent, pointed hems, slightly longer / fuller
    skirt_side_L / _R     shorter overlap panel one layer further out, bridges front and back
    skirt_under           inner pleated skirt, closed tube with a front slit
    *_trim_*              raised binding along hems and slit / vent edges (closed 4-sided strips)
    skirt_emblem_*        rigid faction emblems

Everything hangs off ONE shared surface `surf(angle, z)`: at the top it is the bare torso +
coat_offset, it flares linearly to the hem ellipse, and gets vertical fold undulation toward
the hem.  Angles follow lib.ring_z: 0 = +X (character's left), 270 deg = front (-Y).
Panels are (row, column) grids: rows are horizontal loops at the belt and shear gradually into
the slanted hem, columns run belt -> hem and fan out with the flare.
"""
import math

import bpy  # noqa: F401
from mathutils import Vector as V

from .. import lib, spec

C = spec.CLOTH
TH = C["coat_thickness"]
OFF = C["coat_offset"]
Z_TOP = 1.075                       # tucked under the belt (belt spans 1.055 - 1.115)
Z_HEM = C["skirt_hem_z"]
Z_TIP = C["skirt_tip_z"]
HEM_RX, HEM_RY = C["skirt_hem_rx"], C["skirt_hem_ry"]
SLIT_TOP, SLIT_W = C["front_slit_top_z"], C["front_slit_half_w"]
VENT_TOP, VENT_W = C["back_vent_top_z"], C["back_vent_half_w"]

# ---- tunables ---------------------------------------------------------------------------
BACK_FULL = 0.05                    # back half of the hem ellipse is this much deeper
FOLD_K = 10                         # fold lobes around the whole skirt (crest at the sides)
FOLD_AMP = 0.015                    # fold amplitude at the hem
FRONT_SIDE_A, BACK_SIDE_A = 355.0, 5.0      # side edges of front / back panels (deg)
FRONT_SIDE_Z, BACK_SIDE_Z = 0.475, 0.468    # hem height at those side edges
BACK_TIP_Z = Z_TIP - 0.012                  # back panels hang a little longer
FRONT_LAP = 0.008                   # front panels cross the centre line by this much when closed
LAP_LIFT = 0.015                    # left front panel rides this far above the right one
SIDE_A = 35.0                       # side overlap panel covers -35 .. +35 deg around +X
SIDE_LIFT = 0.013                   # one layer further out (cloth 0.008 + binding 0.004 + air)
SIDE_TOP_Z = 1.070
SIDE_TIP_Z, SIDE_EDGE_Z = 0.445, 0.560
UNDER_OFF, UNDER_TH = 0.018, 0.004
UNDER_HEM_Z, UNDER_SCALE = 0.360, 0.85
UNDER_PLEAT = 0.010
TRIM_W = 0.022                      # width of the hem / slit binding
TRIM_RISE, TRIM_UNDER, TRIM_OVER = 0.004, 0.002, 0.003
EMB_FRONT = (299.0, 0.372, 0.042)   # angle deg, z, half width
EMB_BACK = (73.5, 0.372, 0.040)
EMB_VENT = (90.0, VENT_TOP + 0.050, 0.030)

NR_MAIN, NC_FRONT, NC_BACK = 19, 13, 15
NR_SIDE, NC_SIDE = 14, 12
NC_UNDER = 24
UNDER_ZS = [Z_TOP, 1.005, 0.935] + [0.935 - (0.935 - UNDER_HEM_Z) * i / 9 for i in range(1, 10)]

D = math.radians


def _sstep(e0, e1, x):
    t = min(1.0, max(0.0, (x - e0) / (e1 - e0)))
    return t * t * (3 - 2 * t)


def _smax(a, b, k=0.004):
    """Smooth max that is exact (not biased) where a == b."""
    return 0.5 * (a + b + math.sqrt((a - b) ** 2 + k * k)) - 0.5 * k


def _angdist(a, b):
    return abs((a - b + math.pi) % math.tau - math.pi)


# ------------------------------------------------------------------ the shared surface
_TOP = spec.torso_section(Z_TOP)


def section(z, kind="coat"):
    """(cy, rx, ry_front, ry_back) of the skirt at height z.  kind: 'coat' | 'under'."""
    cy, bx, by = spec.torso_section(max(z, spec.TORSO[0][0]))
    if kind == "coat":
        off, t = OFF, (Z_TOP - z) / (Z_TOP - Z_HEM)
        ex, ey = HEM_RX, HEM_RY
    else:
        off, t = UNDER_OFF, (Z_TOP - z) / (Z_TOP - UNDER_HEM_Z)
        ex, ey = HEM_RX * UNDER_SCALE, HEM_RY * UNDER_SCALE
    tx, ty = _TOP[1] + off, _TOP[2] + off
    rx = _smax(bx + off, tx + (ex - tx) * t)
    ry = _smax(by + off, ty + (ey - ty) * t)
    return cy, rx, ry, ry * (1.0 + BACK_FULL * max(t, 0.0))


def _fold_mask(a):
    """Folds are ironed flat around the emblems so the rigid emblems sit on the cloth."""
    m = 1.0
    for ae in (EMB_FRONT[0], 540.0 - EMB_FRONT[0], EMB_BACK[0], 180.0 - EMB_BACK[0]):
        m *= _sstep(D(10.0), D(21.0), _angdist(a, D(ae)))
    return m


def fold(a, z):
    """Radial fold displacement: nothing over the hips, heavy vertical folds at the hem."""
    t = (0.95 - z) / (0.95 - Z_HEM)
    if t <= 0.0:
        return 0.0
    x = a - D(270.0)
    wave = math.cos(FOLD_K * x) + 0.35 * math.cos(1.6 * FOLD_K * x)      # symmetric left / right
    return -FOLD_AMP * min(t, 1.25) ** 1.4 * wave * _fold_mask(a)


def surf(a, z, extra=0.0, kind="coat", pleat=0.0):
    """Point on the skirt surface.  a: angle (rad), extra: outward layer offset (m)."""
    cy, rx, ryf, ryb = section(z, kind)
    c, s = math.cos(a), math.sin(a)
    ry = ryb if s > 0 else ryf
    nx, ny = ry * c, rx * s
    nl = math.hypot(nx, ny)
    d = extra + (fold(a, z) if kind == "coat" else pleat)
    return V((rx * c + nx / nl * d, cy + ry * s + ny / nl * d, z))


def _angle_for_x(x, z, front, extra=0.0):
    """Angle on the front (or back) left half whose surface point has the given x."""
    rx = section(z)[1]
    q = math.acos(max(-1.0, min(1.0, x / rx)))
    a = math.tau - q if front else q
    for _ in range(3):
        err = x - surf(a, z, extra).x
        a += err / (-rx * math.sin(a))
    return a


def _arc(z, a0, a1, extra=0.0, n=8):
    pts = [surf(a0 + (a1 - a0) * k / n, z, extra) for k in range(n + 1)]
    return sum((q - p).length for p, q in zip(pts, pts[1:]))


def _under_ramp(z):
    """1 below the belt, 0 under it: layers that sit further out collapse under the belt
    so nothing pokes through its outer surface (+0.040)."""
    return 1.0 - _sstep(1.020, 1.050, z)


# ------------------------------------------------------------------ panel grids
def _row_z(i, nr, z_top, z_hem, z_ref):
    """Height of row i (0 = top, nr = hem).  Row nr-1 is the inner edge of the hem binding.
    Rows start horizontal and shear progressively into the slanted hem."""
    if i == nr:
        return z_hem
    zb, zr = z_hem + TRIM_W * 1.12, z_ref + TRIM_W * 1.12
    s = i / (nr - 1)
    return z_top + (zr - z_top) * s + (zb - zr) * s ** 2.2


def _panel_grid(nr, nc, z_top, hem, edge0, edge1, extra, border0=True, border1=False):
    """(nr+1) x (nc+1) grid of surface points.
    hem(v): hem height across the panel (v = 0 at edge0 .. 1 at edge1)
    edge0(z), edge1(z): angle of the two vertical edges;  extra(a, z): layer offset
    border0/1: make the first / last column exactly TRIM_W wide (binding strip)."""
    z_ref = hem(0.5)
    P = []
    for i in range(nr + 1):
        row = []
        for j in range(nc + 1):
            z = _row_z(i, nr, z_top, hem(j / nc), z_ref)
            a0, a1 = edge0(z), edge1(z)
            w = TRIM_W / max(_arc(z, a0, a1), 1e-6)
            u0 = w if border0 else 0.0
            u1 = 1.0 - w if border1 else 1.0
            j0, j1 = (1 if border0 else 0), (nc - 1 if border1 else nc)
            if j < j0:
                u = 0.0
            elif j > j1:
                u = 1.0
            else:
                u = u0 + (u1 - u0) * (j - j0) / (j1 - j0)
            a = a0 + (a1 - a0) * u
            row.append(surf(a, z, extra(a, z)))
        P.append(row)
    return P


def _grid_normals(P):
    nr, nc = len(P), len(P[0])
    N = []
    for i in range(nr):
        row = []
        for j in range(nc):
            du = P[i][min(j + 1, nc - 1)] - P[i][max(j - 1, 0)]
            dv = P[min(i + 1, nr - 1)][j] - P[max(i - 1, 0)][j]
            n = du.cross(dv).normalized()
            if n.x * P[i][j].x + n.y * P[i][j].y < 0:
                n = -n
            row.append(n)
        N.append(row)
    return N


def _orient_outward(ob):
    me = ob.data
    tot = sum(p.normal.x * p.center.x + p.normal.y * p.center.y for p in me.polygons)
    if tot < 0:
        me.flip_normals()
        me.update()
    return ob


def _sheet(name, P, thickness=TH):
    """Open quad grid -> cloth with real thickness growing OUTWARD from the given surface."""
    ob = lib.lofted(name, P, closed=False, topo="main")
    _orient_outward(ob)
    return lib.solidify(ob, thickness, offset=1.0)


def _trim(name, P, N, cells, thickness=TH):
    """Binding wrapped round a panel edge.  `cells` are grid cells (i, j) (rows i..i+1, columns
    j..j+1) along the panel border; the binding is that band of cells as a closed all-quad shell:
    raised above the outer cloth face, wrapped under the inner face, overhanging the cloth edge."""
    nr, nc = len(P) - 1, len(P[0]) - 1
    cells = set(cells)
    verts, faces, vid = [], [], {}

    def vert(i, j, k):
        if (i, j, k) not in vid:
            out = V((0, 0, 0))
            if j == 0 and (0, 0) in cells:
                out += (P[i][0] - P[i][1]).normalized()
            if j == nc and (0, nc - 1) in cells:
                out += (P[i][nc] - P[i][nc - 1]).normalized()
            if i == nr:
                out += (P[nr][j] - P[nr - 1][j]).normalized()
            rise = 0.0005 + (TRIM_RISE - 0.0005) * _under_ramp(P[i][j].z)
            vid[(i, j, k)] = len(verts)
            verts.append(P[i][j] + out * TRIM_OVER + N[i][j] * ((thickness + rise) if k else -TRIM_UNDER))
        return vid[(i, j, k)]

    for (i, j) in sorted(cells):
        c = [(i, j), (i, j + 1), (i + 1, j + 1), (i + 1, j)]
        faces.append(tuple(vert(*q, 1) for q in c))
        faces.append(tuple(vert(*q, 0) for q in reversed(c)))
        for (a, b), nb in zip(zip(c, c[1:] + c[:1]), ((i - 1, j), (i, j + 1), (i + 1, j), (i, j - 1))):
            if nb not in cells:
                faces.append((vert(*a, 0), vert(*b, 0), vert(*b, 1), vert(*a, 1)))
    return lib.mesh_object(name, verts, faces, topo="main")


def _cells_edge_hem(nr, nc):
    """Inner edge column plus the hem row (they share the mitre cell at the tip)."""
    return [(i, 0) for i in range(nr)] + [(nr - 1, j) for j in range(1, nc)]


def _cells_u(nr, nc):
    """Both vertical edges plus the hem row (side overlap panel)."""
    return _cells_edge_hem(nr, nc) + [(i, nc - 1) for i in range(nr - 1)]


# ------------------------------------------------------------------ panel definitions
def _front_edge_x(z):
    """x of the inner edge of the LEFT front panel: lapped past the centre when closed, then the
    straight inverted-V line reaching front_slit_half_w at hem height."""
    return _smax(-FRONT_LAP, (SLIT_TOP - z) * SLIT_W / (SLIT_TOP - Z_HEM), 0.003)


def _back_edge_x(z):
    return 0.0015 + max(0.0, (VENT_TOP - z) * (VENT_W - 0.0015) / (VENT_TOP - Z_HEM))


def _front_grid(lifted):
    def extra(a, z):
        if not lifted:
            return 0.0
        x = section(z)[1] * math.cos(a)
        return LAP_LIFT * _sstep(0.800, 0.845, z) * _under_ramp(z) * (1.0 - _sstep(0.012, 0.058, x))

    def edge0(z):
        x = _front_edge_x(z)
        return _angle_for_x(x, z, True, extra(D(270.0), z) if x < 0.012 else 0.0)

    return _panel_grid(NR_MAIN, NC_FRONT, Z_TOP,
                       lambda v: Z_TIP + (FRONT_SIDE_Z - Z_TIP) * v ** 1.15,
                       edge0, lambda z: D(FRONT_SIDE_A), extra)


def _back_grid():
    return _panel_grid(NR_MAIN, NC_BACK, Z_TOP,
                       lambda v: BACK_TIP_Z + (BACK_SIDE_Z - BACK_TIP_Z) * v ** 1.15,
                       lambda z: _angle_for_x(_back_edge_x(z), z, False),
                       lambda z: D(BACK_SIDE_A), lambda a, z: 0.0)


def _side_extra(a, z):
    return 0.0005 + (SIDE_LIFT - 0.0005) * _under_ramp(z)


def _side_grid():
    return _panel_grid(NR_SIDE, NC_SIDE, SIDE_TOP_Z,
                       lambda v: SIDE_TIP_Z + (SIDE_EDGE_Z - SIDE_TIP_Z) * abs(2 * v - 1) ** 1.15,
                       lambda z: D(-SIDE_A), lambda z: D(SIDE_A), _side_extra,
                       border0=True, border1=True)


def _under_grid():
    P = []
    for z in UNDER_ZS:
        t = _sstep(0.0, 1.0, (0.95 - z) / (0.95 - UNDER_HEM_Z))
        gap = 0.0015 + 0.016 * _sstep(0.72, UNDER_HEM_Z, z)           # front slit half width
        g = math.asin(gap / section(z, "under")[1])
        row = []
        for j in range(NC_UNDER + 1):
            a = D(270.0) + g + (math.tau - 2 * g) * j / NC_UNDER
            sgn = 1 if j % 2 == 0 else -1
            p = surf(a, z, 0.0, "under", pleat=UNDER_PLEAT * t * sgn)
            if z == UNDER_ZS[-1]:
                p.z -= 0.008 * sgn                                    # pleat points on the hem
            row.append(p)
        P.append(row)
    return P


# ------------------------------------------------------------------ emblems
def _emblem(name, a_deg, z, size, extra=0.0, clear=0.0):
    """Rigid emblem lying on the outer cloth surface at (angle, z), aligned to the local normal.
    Sunk 1.5 mm at its centre so the tips (which lift off the convex cloth) stay close."""
    a = D(a_deg)
    off = extra + TH
    p0 = surf(a, z, off)
    ta = surf(a + 1e-3, z, off) - surf(a - 1e-3, z, off)
    tz = surf(a, z + 1e-3, off) - surf(a, z - 1e-3, off)
    n = ta.cross(tz).normalized()
    if n.x * p0.x + n.y * p0.y < 0:
        n = -n
    speed_a, speed_z = ta.length / 2e-3, tz.length / 2e-3
    h = 0.0
    for da, dz in ((size, 0), (-size, 0), (0, size), (0, -1.6 * size), (0.2 * size, 0.2 * size),
                   (-0.2 * size, -0.2 * size), (0, -0.8 * size)):
        q = surf(a + da / speed_a, z + dz / speed_z, off)
        h = max(h, (q - p0).dot(n))
    return lib.emblem_cross(name, size=size, thickness=0.004, loc=p0 + n * (h + clear - 0.0015),
                            normal=n, up=tz)


# ------------------------------------------------------------------ queries for other authors
def outer_radius(a_deg, z):
    """Horizontal distance from the Z axis to the outermost skirt cloth at (angle, z) on the LEFT
    side, including the side overlap panel and its binding where it covers that angle."""
    a = D(a_deg) % math.tau
    extra = TH
    v = _angdist(a, 0.0) / D(SIDE_A)
    if v <= 1.0 and SIDE_TIP_Z + (SIDE_EDGE_Z - SIDE_TIP_Z) * v ** 1.15 <= z <= SIDE_TOP_Z:
        extra = _side_extra(a, z) + TH + TRIM_RISE
    p = surf(a, z, extra)
    return math.hypot(p.x, p.y)


# ------------------------------------------------------------------ build
def build():
    objs = []

    # front panels: left laps over right, so they are two different grids
    Pl = _front_grid(True)
    Pr = _front_grid(False)
    path = _cells_edge_hem(NR_MAIN, NC_FRONT)
    front_l = _sheet("skirt_front_L", Pl)
    trim_l = _trim("skirt_front_trim_L", Pl, _grid_normals(Pl), path)
    tmp = _sheet("skirt_front_tmp", Pr)
    tmp_t = _trim("skirt_front_trim_tmp", Pr, _grid_normals(Pr), path)
    front_r = lib.mirror_copy(tmp, "skirt_front_R")
    trim_r = lib.mirror_copy(tmp_t, "skirt_front_trim_R")
    for o in (tmp, tmp_t):
        me = o.data
        bpy.data.objects.remove(o)
        bpy.data.meshes.remove(me)
    objs += [front_l, front_r, trim_l, trim_r]

    # back panels
    Pb = _back_grid()
    back_l = _sheet("skirt_back_L", Pb)
    btrim_l = _trim("skirt_back_trim_L", Pb, _grid_normals(Pb), _cells_edge_hem(NR_MAIN, NC_BACK))
    objs += [back_l, lib.mirror_copy(back_l, "skirt_back_R"),
             btrim_l, lib.mirror_copy(btrim_l, "skirt_back_trim_R")]

    # side overlap panels
    Ps = _side_grid()
    side_l = _sheet("skirt_side_L", Ps)
    strim_l = _trim("skirt_side_trim_L", Ps, _grid_normals(Ps), _cells_u(NR_SIDE, NC_SIDE))
    objs += [side_l, lib.mirror_copy(side_l, "skirt_side_R"),
             strim_l, lib.mirror_copy(strim_l, "skirt_side_trim_R")]

    # under skirt
    objs.append(_sheet("skirt_under", _under_grid(), UNDER_TH))

    # emblems
    ef = _emblem("skirt_emblem_front_L", *EMB_FRONT)
    eb = _emblem("skirt_emblem_back_L", *EMB_BACK)
    ev = _emblem("skirt_emblem_vent", *EMB_VENT, clear=TRIM_RISE + 0.002)
    objs += [ef, lib.mirror_copy(ef, "skirt_emblem_front_R"),
             eb, lib.mirror_copy(eb, "skirt_emblem_back_R"), ev]
    return objs
