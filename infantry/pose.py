"""Posing stage: deform the BUILT (A-pose) objects into another pose, e.g. a T-pose.

    from infantry import pose
    rig = pose.apply_pose(objs, "tpose")          # objs: everything preview.build_parts returned

The A-pose part modules stay the source of truth; this module only moves vertices.  Nothing here
changes topology (same vertices, faces, order, sharp edges).

Skeleton (per side, built for the LEFT and mirrored):
    torso (static) -> clavicle -> upperarm -> forearm -> forearm_twist
                                                     -> hand
                   clavicle -> epaulette   (the strap of the shoulder armour)
                   upperarm -> pauldron    (the upper-arm guard of the shoulder armour)
Joint positions come from spec (SHOULDER, ELBOW) and arms.py (WRIST incl. WRIST_OFFSET, hand frame).

Weights are computed from vertex POSITION only (one smooth field per skin class, see `_weights`), so
vertices that coincide on two objects (armhole seam coat/sleeve, sleeve/cuff, cuff/glove ...) get the
same weights and stay welded.  Skinning is dual-quaternion (volume preserving, no candy wrapper);
the Blender armature modifier gets the same weights with `use_deform_preserve_volume`, and the pose
is applied by evaluating that modifier, so the exported file is a rig whose rest pose is the T-pose.

Skin classes
    arm     arms.py / cuffs.py objects: full arm chain.  Upper-arm weight rises from g(alpha) on the
            armhole seam ring to 1 a little way down the sleeve.
    torso   torso_coat objects: upper-arm weight g(alpha) on the armhole ring fading out into the coat
            (far further under the arm than over the shoulder), clavicle weight over the shoulders.
    static  everything else (head, skirt, belt, pouches, legs): 100 % torso bone.
Both classes use the same g(alpha) and the same clavicle field, so the seam ring gets identical weights.

Rigid objects (topo "rigid") never bend: every connected island gets uniform weights (the field at the
island's centroid) -> with dual-quaternion skinning that is an exact rigid transform.  The shoulder
armour is split: strap + buttons -> epaulette bone, guard shell + rims + emblem + rivets -> pauldron.
"""
import math

import bmesh
import bpy
import numpy as np
from mathutils import Matrix, Quaternion, Vector as V

from infantry import spec as S
from infantry.parts import arms as A

POSES = ("tpose",)
X, Y, Z = V((1.0, 0.0, 0.0)), V((0.0, 1.0, 0.0)), V((0.0, 0.0, 1.0))

# ---------------------------------------------------------------- joints (left side, A-pose)
J = S.SHOULDER.copy()
E = S.ELBOW.copy()
WR = A.WRIST.copy()                     # includes arms.WRIST_OFFSET (forearm angled forward)
D1 = (E - J).normalized()               # down the upper arm
D2 = (WR - E).normalized()              # down the forearm
LU, LF = (E - J).length, (WR - E).length
CLAV0 = V((0.040, 0.000, 1.520))        # sterno-clavicular pivot

# armhole seam ring frame (spec.armhole_ring)
RC = S.ARMHOLE_CENTER.copy()
R_UP = V((0.34, 0.0, 0.94)).normalized()
R_FW = V((0.0, -1.0, 0.0))
R_N = S.ARMHOLE_NORMAL.copy()
R0, R1 = S.ARMHOLE_R

# ---------------------------------------------------------------- tunables
P = dict(
    clav_deg=7.0,          # clavicle elevation (shoulder rises)
    g_top=0.12,            # upper-arm weight on the armhole ring: top of the shoulder ...
    g_pit=0.58,            # ... and in the armpit (cos blend in between)
    arm_len_top=0.13,      # sleeve: distance from the seam (along the arm) to full upper-arm weight, top
    arm_len_pit=0.050,     # ... armpit
    coat_len_top=0.045,    # coat: distance from the seam to zero upper-arm weight over the shoulder
    coat_len_pit=0.100,    # ... under the arm
    clav_x=(0.085, 0.215), # clavicle weight ramps in over this x range ...
    clav_z=(1.36, 1.47),   # ... and this z range
    elbow_w=0.035,         # half width of the elbow blend (across the bisector plane)
    wrist=(-0.004, 0.026), # hand blend along the wrist plane normal, from the wrist pivot
    twist=(0.10, 0.95),    # forearm twist ramp (forearm parameter)
    twist_share=0.65,      # share of the forearm->hand twist carried by forearm_twist
    strap_deg=10.0,        # extra tilt of the epaulette strap about its collar end
    strap_lift=0.004,
    guard_slide=0.028,     # guard slides this far out along the raised arm ...
    guard_lift=0.004,      # ... and up
    guard_deg=0.0,         # extra rotation of the guard about the posed shoulder (+ = tilts outer end up)
)


def _smooth(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def _swing(a, b):
    """Shortest rotation taking direction a onto direction b."""
    return a.normalized().rotation_difference(b.normalized())


def _rt(q, pivot_from, pivot_to):
    """4x4: rotate by q about pivot_from, then move pivot_from to pivot_to."""
    return Matrix.Translation(pivot_to) @ q.to_matrix().to_4x4() @ Matrix.Translation(-pivot_from)


# ---------------------------------------------------------------- hand frame (replicates arms.build_hand)
def hand_frame():
    """(O, w, b, d): wrist pivot, across the palm toward the thumb, back of the hand, along the hand."""
    f = 1                                           # middle finger tip, as arms.build_hand computes it
    g = math.radians(A.FINGER_FAN[f])
    fd = V((math.sin(g), 0.0, math.cos(g)))
    fu = V((math.cos(g), 0.0, -math.sin(g)))
    fv = V((0.0, 1.0, 0.0))
    dirs, dd, vv = [], fd, fv
    for ang in A.FINGER_BEND[f]:
        a = math.radians(ang)
        dd, vv = (dd * math.cos(a) - vv * math.sin(a)).normalized(), (vv * math.cos(a) + dd * math.sin(a)).normalized()
        dirs.append(dd)
    L = A.FINGER_LEN[f]
    P0 = V((A.U_CENT[f], -0.0005, A.keyf(A.U_CENT[f] / A.PALM_HW, A.WEB)))
    dg = A.Digit(P0, dirs, (0.36 * L, 0.34 * L, 0.30 * L), fu, fv, (A.FINGER_W[f] / 2 - 0.0012) * A.FINGER_THICK[0],
                 A.FINGER_RV[f] * A.FINGER_THICK[1])
    d0_ = (A.HAND_TIP - A.WRIST).normalized()
    O = A.WRIST - d0_ * A.HAND_RISE
    b = (X - d0_ * d0_.x).normalized()
    w = d0_.cross(b).normalized()
    d = d0_.copy()
    tr = math.radians(A.HAND_TURN)
    b, w = b * math.cos(tr) + w * math.sin(tr), w * math.cos(tr) - b * math.sin(tr)
    mt = dg.tip
    q = (w * mt.x + b * mt.y + d * mt.z).normalized().rotation_difference(d0_)
    q = Quaternion().slerp(q, A.HAND_ALIGN)
    return O, q @ w, q @ b, q @ d


HO, HW, HB, HD = hand_frame()


# ---------------------------------------------------------------- skeleton
class Bone:
    def __init__(self, name, head, tail, parent, roll_axis, M):
        self.name, self.head, self.tail, self.parent = name, V(head), V(tail), parent
        self.roll_axis, self.M = V(roll_axis), M        # M: rest -> posed, world space (4x4)


def _guard_axis():
    out = (X - D1 * D1.x).normalized()
    return J + out * 0.095 - D1 * 0.01, J + out * 0.095 + D1 * 0.17


def skeleton_left(pose="tpose"):
    """Bones of the left side with their world-space deform matrices for `pose`."""
    if pose not in POSES:
        raise ValueError(f"unknown pose {pose!r}; known: {POSES}")
    I4 = Matrix.Identity(4)
    Mc = _rt(Quaternion(-Y, math.radians(P["clav_deg"])), CLAV0, CLAV0)
    J2 = Mc @ J
    Ru = _swing(D1, X)                                    # upper arm straight out (+X)
    Mu = _rt(Ru, J, J2)
    E2 = Mu @ E
    Rf = _swing(Ru @ D2, X) @ Ru                          # elbow straight, no twist vs. the upper arm
    Mf = _rt(Rf, E, E2)
    # hand: along +X, back of the hand up (palm down), thumb forward
    src = Matrix((HW, HB, HD)).transposed()
    dst = Matrix((-Y, Z, X)).transposed()
    Rh = (dst @ src.transposed()).to_quaternion()
    Mh = _rt(Rh, HO, Mf @ HO)
    # twist part of the forearm -> hand rotation (about the posed forearm axis X)
    qr = Rh @ Rf.inverted()
    tau = 2.0 * math.atan2(qr.x, qr.w)
    Mt = _rt(Quaternion(X, P["twist_share"] * tau) @ Rf, E, E2)
    # shoulder armour
    ps = V((0.140, 0.004, 1.5938))                          # strap underside at the collar end
    Ms = Mc @ Matrix.Translation(Z * P["strap_lift"]) @ _rt(Quaternion(-Y, math.radians(P["strap_deg"])), ps, ps)
    Mp = (Matrix.Translation(X * P["guard_slide"] + Z * P["guard_lift"])
          @ _rt(Quaternion(-Y, math.radians(P["guard_deg"])), J2, J2) @ Mu)
    g0, g1 = _guard_axis()
    hand_tip = HO + HD * 0.10
    return [
        Bone("torso", (0, 0, 1.00), (0, 0, 1.50), None, -Y, I4),
        Bone("clavicle", CLAV0, J, "torso", Z, Mc),
        Bone("upperarm", J, E, "clavicle", -Y, Mu),
        Bone("forearm", E, WR, "upperarm", -Y, Mf),
        Bone("forearm_twist", E.lerp(WR, 0.5), WR, "forearm", -Y, Mt),
        Bone("hand", HO, hand_tip, "forearm", HB, Mh),
        Bone("epaulette", ps, V((0.268, 0.004, 1.575)), "clavicle", Z, Ms),
        Bone("pauldron", g0, g1, "upperarm", -Y, Mp),
    ], dict(tau=math.degrees(tau))


SIDE_BONES = ("clavicle", "upperarm", "forearm", "forearm_twist", "hand", "epaulette", "pauldron")
MIRROR = Matrix.Diagonal((-1.0, 1.0, 1.0, 1.0))


def skeleton(pose="tpose"):
    """Both sides.  Returns {bone name: Bone}; side bones are suffixed _L / _R."""
    left, info = skeleton_left(pose)
    out = {}
    for b in left:
        if b.name not in SIDE_BONES:
            out[b.name] = b
            continue
        for sfx, m in (("_L", False), ("_R", True)):
            par = b.parent + sfx if b.parent in SIDE_BONES else b.parent
            if not m:
                out[b.name + sfx] = Bone(b.name + sfx, b.head, b.tail, par, b.roll_axis, b.M)
            else:
                mir = lambda v: V((-v.x, v.y, v.z))
                out[b.name + sfx] = Bone(b.name + sfx, mir(b.head), mir(b.tail), par, mir(b.roll_axis),
                                         MIRROR @ b.M @ MIRROR)
    return out, info


# ---------------------------------------------------------------- weights
def _v(a):
    return np.array(a, dtype=np.float64)


def _ring_coords(p):
    """Armhole ring coordinates of left-side points p (N,3): angle alpha (0 top, pi armpit),
    a_ring (arm parameter of the ring at alpha), normalised radius rho, plane offset h."""
    q = p - _v(RC)
    u = q @ _v(R_UP) / R0
    v = q @ _v(R_FW) / R1
    alpha = np.arctan2(v, u)
    rho = np.hypot(u, v)
    h = q @ _v(R_N)
    a_ring = (_v(RC) - _v(J)) @ _v(D1) + (_v(R_UP) @ _v(D1)) * R0 * np.cos(alpha) \
        + (_v(R_FW) @ _v(D1)) * R1 * np.sin(alpha)
    return alpha, a_ring, rho, h


def _by_alpha(alpha, top, pit):
    return top + (pit - top) * 0.5 * (1.0 - np.cos(alpha))


def _weights(p, cls):
    """Left-side points p (N,3) -> {bone: weight (N,)} for skin class cls ('arm' | 'torso')."""
    n = len(p)
    alpha, a_ring, rho, h = _ring_coords(p)
    g = _by_alpha(alpha, P["g_top"], P["g_pit"])
    if cls == "arm":
        t = (p - _v(J)) @ _v(D1) - a_ring
        Lr = _by_alpha(alpha, P["arm_len_top"], P["arm_len_pit"])
        wu = g + (1.0 - g) * _smooth(t / Lr)
    else:
        rm = 0.5 * (R0 + R1)
        dr = np.maximum(rho - 1.0, 0.0) * rm
        d = np.sqrt(dr * dr + h * h)
        Dr = _by_alpha(alpha, P["coat_len_top"], P["coat_len_pit"])
        wu = g * (1.0 - _smooth(d / Dr))
    x0, x1 = P["clav_x"]
    z0, z1 = P["clav_z"]
    wc = _smooth((p[:, 0] - x0) / (x1 - x0)) * _smooth((p[:, 2] - z0) / (z1 - z0))
    out = {"torso": (1.0 - wu) * (1.0 - wc), "clavicle": (1.0 - wu) * wc}
    if cls == "arm":
        ne = (D1 + D2).normalized()
        fe = _smooth(((p - _v(E)) @ _v(ne) + P["elbow_w"]) / (2 * P["elbow_w"]))
        nw = (D2 + HD).normalized()
        w0, w1 = P["wrist"]
        fw = _smooth(((p - _v(HO)) @ _v(nw) - w0) / (w1 - w0))
        s = ((p - _v(E)) @ _v(D2)) / LF
        t0, t1 = P["twist"]
        ft = _smooth((s - t0) / (t1 - t0))
        lo = wu * fe
        out.update(upperarm=wu * (1.0 - fe), forearm=lo * (1.0 - fw) * (1.0 - ft),
                   forearm_twist=lo * (1.0 - fw) * ft, hand=lo * fw)
    else:
        out["upperarm"] = wu
    for k in out:
        out[k] = np.clip(out[k], 0.0, 1.0)
    return out


def _islands(me):
    """Connected components -> array of island ids per vertex."""
    nv = len(me.vertices)
    parent = np.arange(nv)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a
    ev = np.empty(len(me.edges) * 2, dtype=np.int64)
    me.edges.foreach_get("vertices", ev)
    for a, b in ev.reshape(-1, 2):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    return np.array([find(i) for i in range(nv)])


def _coords(ob):
    co = np.empty(len(ob.data.vertices) * 3)
    ob.data.vertices.foreach_get("co", co)
    co = co.reshape(-1, 3)
    mw = np.array(ob.matrix_world)
    return co @ mw[:3, :3].T + mw[:3, 3]


def skin_class(ob):
    part = ob.get("part", "")
    if part in ("arms", "cuffs") or ob.name.startswith("pauldron_"):
        return "arm"
    if part == "torso_coat":
        return "torso"
    return "static"


def object_weights(ob):
    """{bone name (with side suffix): weights (N,)} for one object, position based."""
    co = _coords(ob)
    n = len(co)
    cls = skin_class(ob)
    if cls == "static":
        return {"torso": np.ones(n)}
    side = np.where(co[:, 0] >= 0.0, 1.0, -1.0)
    pl = co.copy()
    pl[:, 0] = np.abs(pl[:, 0])
    rigid = ob.get("topo") == "rigid"
    isl = _islands(ob.data) if rigid else None
    if rigid:                                   # evaluate the field once per island, at its centroid
        ids, inv = np.unique(isl, return_inverse=True)
        cent = np.zeros((len(ids), 3))
        np.add.at(cent, inv, pl)
        cent /= np.bincount(inv)[:, None]
        cside = np.zeros(len(ids))
        np.add.at(cside, inv, co[:, 0])
        cside = np.where(cside >= 0.0, 1.0, -1.0)
        wl = _weights(cent, cls)
        wl = {k: v[inv] for k, v in wl.items()}
        side = cside[inv]
        if ob.name.startswith("pauldron_"):
            strap = cent[:, 2] > 1.55               # strap + its buttons sit on top of the shoulder
            bone = np.where(strap, 0, 1)[inv]
            wl = {"epaulette": (bone == 0).astype(float), "pauldron": (bone == 1).astype(float)}
    else:
        wl = _weights(pl, cls)
    out = {}
    for k, w in wl.items():
        if k == "torso":
            out["torso"] = out.get("torso", 0.0) + w
            continue
        out[k + "_L"] = np.where(side > 0, w, 0.0)
        out[k + "_R"] = np.where(side < 0, w, 0.0)
    tot = sum(out.values())
    return {k: v / np.maximum(tot, 1e-12) for k, v in out.items() if np.any(v > 1e-6)}


# ---------------------------------------------------------------- dual quaternion skinning (numpy)
def _dq(M):
    q = M.to_quaternion().normalized()
    if q.w < 0:
        q = -q
    t = M.to_translation()
    qd = Quaternion((0.0, t.x, t.y, t.z)) @ q
    return np.array([q.w, q.x, q.y, q.z, 0.5 * qd.w, 0.5 * qd.x, 0.5 * qd.y, 0.5 * qd.z])


def dqs(co, weights, bones):
    """Dual-quaternion skinning of points co (N,3) with {bone: weights}."""
    b = np.zeros((len(co), 8))
    for k, w in weights.items():
        b += w[:, None] * _dq(bones[k].M)[None, :]
    r, d = b[:, :4], b[:, 4:]
    nrm = np.linalg.norm(r, axis=1)[:, None]
    r, d = r / nrm, d / nrm
    w0, v0 = r[:, :1], r[:, 1:]
    wd, vd = d[:, :1], d[:, 1:]
    t = 2.0 * (w0 * vd - wd * v0 + np.cross(v0, vd))
    # rotate co by r
    uv = np.cross(v0, co)
    rot = co + 2.0 * (w0 * uv + np.cross(v0, uv))
    return rot + t


# ---------------------------------------------------------------- Blender armature
def build_armature(bones, name="trooper_rig"):
    arm = bpy.data.armatures.new(name)
    ob = bpy.data.objects.new(name, arm)
    bpy.context.scene.collection.objects.link(ob)
    bpy.context.view_layer.objects.active = ob
    ob.select_set(True)
    bpy.ops.object.mode_set(mode="EDIT")
    for b in bones.values():
        eb = arm.edit_bones.new(b.name)
        eb.head, eb.tail = b.head, b.tail
        eb.align_roll(b.roll_axis)
        eb.use_deform = True
    for b in bones.values():
        if b.parent:
            eb = arm.edit_bones[b.name]
            eb.parent = arm.edit_bones[b.parent]
            eb.use_connect = False
    bpy.ops.object.mode_set(mode="OBJECT")
    return ob


def _depth(bones, n):
    d = 0
    while bones[n].parent:
        n = bones[n].parent
        d += 1
    return d


def pose_armature(rig, bones):
    for n in sorted(bones, key=lambda n: _depth(bones, n)):
        pb = rig.pose.bones[n]
        pb.matrix = bones[n].M @ rig.data.bones[n].matrix_local
        bpy.context.view_layer.update()


def apply_pose_as_rest(rig):
    bpy.context.view_layer.objects.active = rig
    rig.select_set(True)
    bpy.ops.object.mode_set(mode="POSE")
    bpy.ops.pose.select_all(action="SELECT")
    bpy.ops.pose.armature_apply(selected=False)
    bpy.ops.object.mode_set(mode="OBJECT")


# ---------------------------------------------------------------- entry point
def apply_pose(objs, pose="tpose", armature=True, verbose=True):
    """Deform the built objects in place into `pose`.  With armature=True a rig (rest pose = the new
    pose) is created, every mesh gets vertex groups + an Armature modifier, and the deformation is
    taken from evaluating that modifier; otherwise the same dual-quaternion skinning runs in numpy.
    Returns the armature object (or None)."""
    bones, info = skeleton(pose)
    meshes = [o for o in objs if o.type == "MESH"]
    weights = {o.name: object_weights(o) for o in meshes}
    if verbose:
        print(f"pose {pose}: forearm->hand twist {info['tau']:.1f} deg "
              f"({P['twist_share']:.0%} on forearm_twist)")
    if not armature:
        for o in meshes:
            co = _coords(o)
            new = dqs(co, weights[o.name], bones)
            mi = np.array(o.matrix_world.inverted())
            new = new @ mi[:3, :3].T + mi[:3, 3]
            o.data.vertices.foreach_set("co", new.ravel())
            o.data.update()
        return None
    for o in bpy.context.selected_objects:
        o.select_set(False)
    rig = build_armature(bones)
    for o in meshes:
        n = len(o.data.vertices)
        idx = list(range(n))
        for bname, w in weights[o.name].items():
            vg = o.vertex_groups.get(bname) or o.vertex_groups.new(name=bname)
            if np.all(w == w[0]):
                if w[0] > 0:
                    vg.add(idx, float(w[0]), "REPLACE")
                continue
            for val in np.unique(np.round(w, 7)):
                if val <= 0:
                    continue
                sel = np.nonzero(np.abs(w - val) < 5e-8)[0].tolist()
                vg.add(sel, float(val), "REPLACE")
        mod = o.modifiers.new("Armature", "ARMATURE")
        mod.object = rig
        mod.use_vertex_groups = True
        mod.use_bone_envelopes = False
        mod.use_deform_preserve_volume = True
        o.parent = rig
    pose_armature(rig, bones)
    dg = bpy.context.evaluated_depsgraph_get()
    for o in meshes:
        ev = o.evaluated_get(dg)
        me = ev.to_mesh()
        co = np.empty(len(me.vertices) * 3)
        me.vertices.foreach_get("co", co)
        ev.to_mesh_clear()
        o.data.vertices.foreach_set("co", co)
        o.data.update()
    apply_pose_as_rest(rig)
    bpy.context.view_layer.update()
    rig["pose"] = pose
    return rig
