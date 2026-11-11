# Infantry Ranged Trooper - modelling plan

Reference sheet: `docs/reference.png` (front / left side / back topology turnaround).

## Approach

The character is modelled **procedurally in Blender Python** (`bpy` 5.x, headless). Every
surface is built by lofting rings / rows with matching vertex counts, so edge loops are
explicit and countable rather than the by-product of sculpt + retopo. That gives exactly
what the sheet asks for:

| Sheet requirement | How it is met |
|---|---|
| Quad-dominant deformation topology | `lib.loft` grids + all-quad caps; triangles only on tiny rigid bevel corners |
| Clean shoulder / armpit loops | torso and sleeve share one 16-vertex seam ring (`spec.armhole_ring`) |
| Elbow / wrist / finger / knee loops | 3 tight support loops at each joint, tagged `support` |
| Separate coat panels | torso core, front L/R, side overlaps, back L/R, under skirt are separate meshes |
| Rigid accessories as separate parts | pauldrons, belt, buckles, pouches, canister, boot hardware tagged `rigid` |
| 24k-31k quads / 48k-62k tris | per-part budgets in `spec.BUDGET`, checked by `tools/preview.py` |

## Layout

```
infantry/spec.py      proportions, joints, clothing offsets, shared seams   (read-only for part authors)
infantry/lib.py       ring / loft / tube / box / cylinder / stud / emblem helpers (read-only)
infantry/parts/*.py   one module per region, each exposes build() -> [objects]
tools/preview.py      build parts, print topology stats, render wireframe contact sheet
tools/build_all.py    assemble everything -> out/*.blend, *.glb, stats.json, renders/turnaround_*.png
```

Conventions: metres, Z up, feet on z=0, character **faces -Y**, character's **left is +X**,
relaxed A-pose. Paired parts are built on the left and mirrored with `lib.mirror_copy`.

## Part breakdown (one author each)

| Module | Contents | Quad budget |
|---|---|---|
| `head_hat` | head + neck, faceted face mask with vertical slits, wide-brim hat, hat band + emblem | 3000 |
| `torso_coat` | double-breasted coat torso with armholes, lapels, tall stand collar, buttons, back seams + back emblem | 4500 |
| `arms` | sleeves (from the armhole seam), forearm bracers with studs, cuffs, gloved hands with 5 fingers | 7500 |
| `armor_gear` | layered pauldrons with emblem, belt + buckle + loops, pouches, hip canister + hose | 5000 |
| `skirt` | front panels L/R, side overlaps L/R, back panels L/R, under skirt, hem trim, hem emblems | 4500 |
| `legs` | baggy trousers bloused into the boots, boots with soles, 2 straps + buckles each | 5000 |

## Layering (inside -> outside), so parts never intersect visibly

1. bare body (`spec.torso_section`, `arm_radius`, `leg_radius`) - not modelled, only a guide
2. trousers, head/neck
3. under skirt (+0.018 over the hips)
4. coat torso / skirt panels (+0.028 = `coat_offset`); overlapping panels step out a further 0.008 each
5. belt (+0.040), sits over the torso/skirt seam at z 1.055-1.115
6. sleeves (arm radius + 0.022), then bracers (+0.010 over sleeve), pauldrons (>= +0.012 over sleeve)
7. pouches / canister hang outside the belt

## Seams between authors

* **Armhole**: `spec.armhole_ring(16)` - torso boundary and first sleeve ring are the same points.
* **Waist**: torso coat ends at z=1.045, skirt panels start at z=1.075; the belt hides both.
* **Neck**: head/neck mesh runs down to z=1.50 inside the collar; collar belongs to `torso_coat`.
* **Wrist**: glove cuff overlaps the sleeve end; both belong to `arms`.
* **Boot top**: z=0.335; trouser blouse overhangs it; both belong to `legs`.

## Topology rules

* Quads only on anything that deforms. No n-gons anywhere. Triangles tolerated only on rigid bevel corners.
* Ring counts: torso 32 around, sleeve/arm 16, leg 16-20, fingers 8, neck 16, hat 32.
* Joints get 3 closely spaced loops (elbow, wrist, knee, knuckles) as separate `support`-tagged
  bands or simply denser spacing; shoulders/armpit keep even spacing around the seam ring.
* Even quad size (~2-3 cm on the body, ~1 cm on hands/face). No long slivers on deforming parts.
* Cloth edges that are visible (hems, lapels, collar, brim) have real thickness (`lib.solidify`).
* Each object carries `ob["topo"]` = `main` | `support` | `rigid` (legend colours in renders).
