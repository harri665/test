"""Single source of truth for proportions.  ALL parts must derive positions from here.

Conventions
-----------
* Units: metres.  Z up.  Feet on z = 0.
* Character FACES -Y.  Character's LEFT side is +X.
* Relaxed A-pose: arms hang ~20 deg away from the body, legs slightly apart.
* Paired parts are built for the LEFT (+X) side and mirrored with lib.mirror_copy.
* "BODY" numbers describe the bare mannequin.  Clothing sits OUTSIDE it by the
  offsets in CLOTH.  Never let an outer layer go inside an inner one.
"""
import bpy  # noqa: F401  (must load before mathutils)
from mathutils import Vector as V

# ---------------------------------------------------------------- torso (bare body)
# z : (centre_y, radius_x, radius_y)   -- elliptical cross sections
TORSO = [
    (0.86, 0.000, 0.150, 0.105),   # crotch
    (0.95, 0.005, 0.175, 0.120),   # hips
    (1.085, 0.000, 0.150, 0.105),  # waist (belt centre line)
    (1.22, 0.000, 0.170, 0.115),   # ribs
    (1.36, 0.000, 0.190, 0.125),   # chest
    (1.46, 0.005, 0.185, 0.110),   # shoulder line
    (1.52, 0.010, 0.075, 0.070),   # neck base
    (1.60, 0.005, 0.055, 0.058),   # neck top / jaw
]

HEAD_CENTER = V((0.0, -0.010, 1.690))
HEAD_RADII = V((0.092, 0.108, 0.120))        # x, y, z half extents (top of skull z ~1.81)

# ---------------------------------------------------------------- left arm joints
SHOULDER = V((0.215, 0.000, 1.455))
ELBOW = V((0.305, 0.010, 1.190))
WRIST = V((0.375, -0.030, 0.940))
HAND_TIP = V((0.415, -0.055, 0.750))         # tip of middle finger, relaxed
ARM_RADII = dict(shoulder=0.058, bicep=0.052, elbow=0.045, forearm=0.043, wrist=0.031)

# ---------------------------------------------------------------- left leg joints
HIP = V((0.095, 0.000, 0.900))
KNEE = V((0.110, -0.010, 0.500))
ANKLE = V((0.115, 0.005, 0.090))
TOE = V((0.125, -0.205, 0.000))              # front tip of boot sole
HEEL = V((0.112, 0.085, 0.000))              # back of boot sole
LEG_RADII = dict(thigh=0.088, knee=0.060, calf=0.058, ankle=0.042)

# ---------------------------------------------------------------- clothing layout
CLOTH = dict(
    coat_offset=0.028,        # coat surface sits this far outside the bare torso
    coat_thickness=0.008,     # visible cloth edge thickness (hems, lapels, collar)
    sleeve_offset=0.022,      # sleeve radius = arm radius + this (baggy, more at elbow)
    belt_z=(1.055, 1.115),    # belt bottom / top
    belt_offset=0.040,        # belt outer surface outside bare torso
    coat_split_z=1.055,       # coat torso ends here; skirt panels start under the belt
    skirt_hem_z=0.300,        # general hem height
    skirt_tip_z=0.215,        # pointed tips of the front / back panels
    skirt_hem_rx=0.360, skirt_hem_ry=0.300,   # ellipse of the skirt at hem height
    front_slit_top_z=0.840,   # front opening (inverted V) starts here ...
    front_slit_half_w=0.105,  # ... and is this wide (half) at the hem
    back_vent_top_z=0.800, back_vent_half_w=0.030,
    collar_top_z=1.665,       # tall stand-up collar reaches the cheeks
    collar_top_r=(0.125, 0.135),
    pauldron_bottom_z=1.290,  # shoulder armour covers upper arm down to here
    bracer=(0.12, 0.93),      # forearm bracer covers this param range elbow(0)->wrist(1)
    boot_top_z=0.335,
    trouser_blouse=(0.335, 0.540, 0.105),  # bloused trouser: z range and max radius
    hat_brim_z=1.760, hat_brim_r=(0.235, 0.245),
    hat_crown_r=(0.112, 0.122), hat_crown_top_z=1.895,
)

# quad budget per part file (whole character target: 24k - 31k quads)
BUDGET = dict(head_hat=3000, torso_coat=4500, arms=7500, armor_gear=5000, skirt=4500, legs=5000)


def _lerp(a, b, t):
    return a + (b - a) * t


def torso_section(z):
    """Bare-body torso ellipse at height z -> (cy, rx, ry). Smooth (Catmull-Rom) interpolation."""
    T = TORSO
    if z <= T[0][0]:
        return T[0][1:]
    if z >= T[-1][0]:
        return T[-1][1:]
    for i in range(len(T) - 1):
        if T[i][0] <= z <= T[i + 1][0]:
            p0 = T[max(i - 1, 0)]; p1 = T[i]; p2 = T[i + 1]; p3 = T[min(i + 2, len(T) - 1)]
            t = (z - p1[0]) / (p2[0] - p1[0])
            out = []
            for k in (1, 2, 3):
                a, b, c, d = p0[k], p1[k], p2[k], p3[k]
                m1 = (c - a) * 0.5 * (p2[0] - p1[0]) / max(p2[0] - p0[0], 1e-6) * 2
                m2 = (d - b) * 0.5 * (p2[0] - p1[0]) / max(p3[0] - p1[0], 1e-6) * 2
                t2, t3 = t * t, t * t * t
                out.append((2*t3 - 3*t2 + 1) * b + (t3 - 2*t2 + t) * m1 + (-2*t3 + 3*t2) * c + (t3 - t2) * m2)
            return tuple(out)


def arm_point(t):
    """Left arm centre line. t: 0 shoulder, 1 elbow, 2 wrist, 3 fingertip."""
    pts = [SHOULDER, ELBOW, WRIST, HAND_TIP]
    i = min(int(t), 2)
    return pts[i].lerp(pts[i + 1], t - i)


def arm_radius(t):
    """Bare arm radius. t as in arm_point (0..2)."""
    r = ARM_RADII
    keys = [(0.0, r["shoulder"]), (0.45, r["bicep"]), (1.0, r["elbow"]), (1.3, r["forearm"]), (2.0, r["wrist"])]
    for (t0, r0), (t1, r1) in zip(keys, keys[1:]):
        if t <= t1:
            return _lerp(r0, r1, max(0.0, (t - t0) / (t1 - t0)))
    return r["wrist"]


def leg_point(t):
    """Left leg centre line. t: 0 hip, 1 knee, 2 ankle."""
    pts = [HIP, KNEE, ANKLE]
    i = min(int(t), 1)
    return pts[i].lerp(pts[i + 1], t - i)


def leg_radius(t):
    r = LEG_RADII
    keys = [(0.0, r["thigh"]), (1.0, r["knee"]), (1.35, r["calf"]), (2.0, r["ankle"])]
    for (t0, r0), (t1, r1) in zip(keys, keys[1:]):
        if t <= t1:
            return _lerp(r0, r1, max(0.0, (t - t0) / (t1 - t0)))
    return r["ankle"]


# ---------------------------------------------------------------- shared seams
ARMHOLE_CENTER = V((0.235, 0.000, 1.440))
ARMHOLE_NORMAL = V((0.94, 0.0, -0.34)).normalized()     # points out along the upper arm
ARMHOLE_R = (0.072, 0.070)                              # (up/down radius, front/back radius)


def armhole_ring(n=16, side=1):
    """THE seam between coat torso and sleeve (left side; side=-1 mirrors to the right).
    torso_coat's armhole boundary and the sleeve's first ring both use exactly these points.
    index 0 = top of shoulder, n/4 = front (-Y), n/2 = armpit, 3n/4 = back (+Y)."""
    import math
    up = V((0.34, 0.0, 0.94)).normalized()
    fwd = V((0.0, -1.0, 0.0))
    pts = []
    for i in range(n):
        a = math.tau * i / n
        p = ARMHOLE_CENTER + up * (ARMHOLE_R[0] * math.cos(a)) + fwd * (ARMHOLE_R[1] * math.sin(a))
        pts.append(V((p.x * side, p.y, p.z)))
    return pts
