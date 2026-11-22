# Round 5 - match the user's FBX game model

Goal (from the user): make our model match `docs/ref/fbx/infantry_model.fbx` much more closely,
**while keeping our topology standards and the lessons of the earlier rounds**. The difference list,
by ID, is `docs/FBX_COMPARISON.md`; side-by-sides are in `docs/ref/fbx/`. Render any region of both
models with the same camera and scale:

    python3 tools/fbx_compare.py --zoom cx,cy,cz,half_h [--wire] [--parts a,b] [--fbx-objects X,Y] -o <scratch>/x.png
    python3 tools/fbx_compare.py --only-fbx --zoom ... -o ...        # FBX alone (e.g. its T-posed arms)

FBX object names: Retopology (mask), Neck (head+ears+neck), Hair, Hat, JacketMain (coat incl. the
long front panels), JacketMain.002/.003 (small coat pieces), Jacket wave .. .005 (skirt panels),
Legs (trousers + boots), Hands, Arm Cuffs, Arm Pads. The FBX is in a T-POSE: compare torso, legs,
head and skirt directly; for arms compare the design, not the position.

## What "keep our topology and lessons" means

* Match the FBX's **design** (shapes, panel layout, which details exist, how edges read), not its
  low-poly triangulated mesh: stay all-quad on cloth/skin, zero n-gons, zero non-manifold, clean
  loops that follow the forms, crisp hard edges where the FBX has hard edges (support loops).
* Derive positions from `infantry/spec.py`; one file per agent; part budgets in `spec.BUDGET`.
* Seams that are shared stay shared (armhole ring, neck bottom ring at z 1.530, belt zone).
* Judge with renders against the FBX every round (solid and wire); report honestly.
* Keep object names stable where the object still exists (the T-pose stage in `infantry/pose.py`
  classifies objects by part and name). New objects: follow the existing naming style
  (`thing_L` / `thing_R`).
* Do not commit; the coordinator commits each region separately so any change can be reverted.

## Interfaces between regions (read the ones that touch your part)

**I1 Belt (armor_gear).** Thin continuous belt band **z 1.195-1.245** (it must still hide the
coat/skirt seam all the way round), outer face at bare torso + 0.040. At the front it passes UNDER
the coat's front wrap flap (I2). FBX details: rectangular loop buckles on side tabs at the front
sides (just outside the flap edges), two round buttons at the back, a long thin strap hanging from
each side tab down the hip. No big central buckle, no pouches, no canister, no hoses.

**I2 Coat front wrap (torso_coat).** Single-breasted asymmetric wrap like the FBX: one big front
panel crossing diagonally (match the FBX's direction), broad notched lapel, no buttons. The wrap
flap continues DOWN OVER THE BELT to about z 1.13, lying at >= bare + 0.048 over the belt zone so the
belt passes under it; its lower edge continues the diagonal so the skirt's front panel (I3) can carry
the same edge line on down. Everything else of the coat torso still ends at z 1.195 under the belt.

**I3 Skirt front (skirt).** Below the belt, skirt panels start at z 1.240 (inside belt zone
<= bare + 0.036). Under the wrap flap (I2) the skirt top must stay below the flap's inner surface.
The front panel that sits under the flap continues the flap's diagonal edge down to the hem, so it
reads as one long panel like the FBX. Tall narrow inverted-V front opening starting right under the
belt. No under skirt, no borders/piping, no hem emblems (the FBX has none).

**I4 Back tab (skirt) + back emblem (torso_coat).** The skirt owns a pointed tab hanging from the
belt at centre back (z ~1.19 down to ~0.95, pointed bottom, like the FBX). The coat torso's back
emblem becomes small, like the FBX, with its stem running down to the belt top; the skirt continues
the stem on the tab (thin raised bar ending in a small point).

**I5 Trousers (legs).** Straight, slightly loose trousers like the FBX (no bloused overhang),
crumple folds at the knee, scrunched folds at the ankle above the boot. The legs agent publishes the
new envelope (max|x| and y-front/back every 5 cm from z 1.20 to 0.25) to the coordinator early; the
skirt fits to it (there is no under skirt any more, so the trousers show through the front opening).

**I6 Sleeve / cuff (arms, cuffs).** The sleeve still ends inside the cuff. The cuff's new V-shaped
top edge may dip lower at the front: its lowest point must stay >= 1.5 cm above the sleeve end; check
against the real arms mesh. Armhole ring unchanged. Above z 1.36 the sleeve stays within arm radius +
0.022 (the arm pads sit outside it); the FBX's puffiness goes below that.

**I7 Shoulder (armor_gear vs torso_coat, arms).** FBX shoulder boards (flat rectangular epaulettes
overhanging the upper arm) and plain upper-arm pads with an angled edge band replace our strap +
emblem guard. They must clear the coat (including the new rolled collar) and the sleeves.

## Ownership

| File | Owner | Items (from FBX_COMPARISON.md) |
|---|---|---|
| head_hat.py | head agent (already running) | H1-H4 head, ears, hair, mask + H5-H7 hat |
| legs.py | feet agent (already running) | L2-L5 boots + L1 trousers (I5) |
| torso_coat.py | coat agent | C1-C5, coat side of C6 (I2, I4) |
| armor_gear.py | gear agent | C7, C8, W1-W4 (I1, I7) |
| skirt.py | skirt agent | S1-S7, back tab C6 (I3, I4, I5) |
| arms.py | sleeve/hand agent | A1, A3 (I6) |
| cuffs.py | cuff agent | A2 (I6) |
| pose.py + tools | T-pose agent (already running) | must keep working on the changed parts |

## Result

All seven part files and the T-pose stage landed (see the per-region commits "Round 5: ...").
Whole model: 27,144 quads / 54,288 triangles, all quads, 0 n-gons, 0 non-manifold.
Side-by-sides: `docs/ref/fbx/final_compare_tpose.png` (ours T-posed) and `final_compare_apose.png`.
Exports: `out/infantry_trooper.glb/.blend` (A-pose) and `out/infantry_trooper_tpose.glb/.blend`
(T-pose, rigged: 15-bone armature, vertex groups).

Known gaps vs the FBX: our arms are exactly horizontal (the FBX's slope slightly down); shoulder
boards and arm pads are bulkier and sit higher (the pad must clear the sleeve envelope); the collar
fall is a more even band than the FBX's flatter stepped fold; coat/knee crumples are softer than the
FBX's hard facets; hair is spikier; front opening apex at z ~1.01 (FBX ~1.09); hips ~1 cm wider
(trouser clearance); textures not applied (the FBX's texture files were not embedded).
